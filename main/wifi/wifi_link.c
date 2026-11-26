#include "wifi_link.h"
#include "can_manager.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "firmware_update.h"
#include "esp_app_desc.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "nvs.h"
#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static const char *TAG = "wifi_link";

#define RING_BYTES  (8 * 1024)
#define HEAP_KEPT   (12 * 1024)  // free heap a datagram may not take: BLE allocates as it goes (it failed near 0, not at 15 KB)
#define HEAP_NEEDED (48 * 1024)  // free heap Wi-Fi start-up needs (~45 KB with the buffers in sdkconfig.defaults)
#define LOG_MAX     300   // longer log lines are cut

// One ring for both: log lines (always, oldest dropped for new) and the frame lines (only while a client asks).
static RingbufHandle_t  s_ring;
static TaskHandle_t     s_control;   // applies on/off: Wi-Fi start-up is too heavy for the BLE host task
static volatile bool    s_on, s_client;
static esp_netif_t     *s_netif;
static char             s_ssid[33], s_pass[65], s_key[65];   // s_key: what "O" lines must carry to flash
static volatile int     s_seen = -1, s_rssi, s_channel, s_auth;  // the last scan: networks seen; ours, if among them
static volatile int     s_reason;    // why it isn't on the network: the last disconnect reason (wifi_err_reason_t), -1 start failed, 0 none

static volatile bool    s_open_line;  // the last log piece didn't end its line: the next one continues it

static int log_hook(const char *fmt, va_list args)
{
    va_list copy;
    va_copy(copy, args);
    int len = vsnprintf(NULL, 0, fmt, copy);
    va_end(copy);
    if (s_ring && len > 0) {
        if (len > LOG_MAX) len = LOG_MAX;
        void *item = NULL;
        for (int tries = 0; tries < 2 && xRingbufferSendAcquire(s_ring, &item, len + 3, 0) != pdTRUE; tries++) {
            size_t size;
            void *oldest = xRingbufferReceive(s_ring, &size, 0);
            if (oldest) vRingbufferReturnItem(s_ring, oldest);
            item = NULL;
        }
        if (item) {
            char *p = item;
            // the Wi-Fi driver writes a line in pieces: "g" continues the line before, "G" starts one
            p[0] = s_open_line ? 'g' : 'G'; p[1] = ',';
            va_copy(copy, args);
            vsnprintf(p + 2, len + 1, fmt, copy);
            va_end(copy);
            s_open_line = p[len + 1] != '\n';
            if (!s_open_line) p[len + 1] = ' ';  // the line's own newline: one line per item
            p[len + 2] = '\n';
            xRingbufferSendComplete(s_ring, item);
        }
    }
    return vprintf(fmt, args);  // the console as before
}

bool wifi_link_send(const char *data, size_t len)
{
    return s_ring && s_client && xRingbufferSend(s_ring, data, len, 0) == pdTRUE;
}

static bool send_all(int sock, const char *data, size_t len)
{
    while (len) {
        int sent = send(sock, data, len, 0);
        if (sent <= 0) return false;
        data += sent; len -= sent;
    }
    return true;
}

// Frames left from a client that went would reach the next one with new times: only the log stays.
static void drop_frames(void)
{
    static char keep[LOG_MAX + 3];
    UBaseType_t waiting = 0;
    vRingbufferGetInfo(s_ring, NULL, NULL, NULL, NULL, &waiting);
    while (waiting--) {
        size_t size;
        char *item = xRingbufferReceive(s_ring, &size, 0);
        if (!item) break;
        bool log = item[0] == 'G' && size <= sizeof(keep);
        if (log) memcpy(keep, item, size);
        vRingbufferReturnItem(s_ring, item);
        if (log) xRingbufferSend(s_ring, keep, size, 0);
    }
}

static int socket_read(void *context, char *buf, int max) { return recv(*(int *)context, buf, max, 0); }

/** "O,<size>,<key>" then the image ([first]: its bytes that came with the line): answers "O,ok" (then restarts) or "O,error <why>". */
static void flash(int sock, const char *line, const char *first, int first_size)
{
    char key[65] = { 0 }, answer[64];
    unsigned long size = 0;
    const char *why = sscanf(line, "O,%lu,%64s", &size, key) != 2 || !s_key[0] || strcmp(key, s_key) ? "wrong key" : NULL;
    struct timeval limit = { .tv_sec = 10 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &limit, sizeof(limit));
    if (!why) why = firmware_update(size, first, first_size, socket_read, &sock);
    int n = snprintf(answer, sizeof(answer), why ? "O,error %s\n" : "O,ok\n", why);
    send_all(sock, answer, n);
    ESP_LOGI(TAG, "Wi-Fi update: %s", why ? why : "done, restarting");
    if (!why) { vTaskDelay(pdMS_TO_TICKS(500)); esp_restart(); }
}

/**
 * One client at a time: its "L" lines in; the log and the metrics out on TCP, and once it sent "U,<port>" the frame
 * batches as UDP datagrams to that port, each "Q,<sequence>\n" and the batch. TCP waits for acknowledgements, which
 * modem sleep (kept for BLE) delays by hundreds of ms: too slow for both buses. A lost datagram shows as a gap.
 */
static void serve(int sock)
{
    static char datagram[1472];   // the most a UDP datagram carries unsplit
    char b[64], in[96], line[192];
    int il = 0, udp = -1;
    uint32_t sequence = 0, unsent = 0, unsent_told = 0;
    struct sockaddr_in to = { 0 };
    socklen_t to_size = sizeof(to);
    int64_t last_m = 0;
    bool asked = false;  // nothing goes out before the client's first line: a connection that only looks takes no log
    while (s_on) {
        fd_set readable;
        FD_ZERO(&readable);
        FD_SET(sock, &readable);
        struct timeval wait = { .tv_usec = 20000 };
        if (select(sock + 1, &readable, NULL, NULL, &wait) < 0) break;
        if (FD_ISSET(sock, &readable)) {
            int r = recv(sock, b, sizeof(b), 0);
            if (r <= 0) break;
            for (int i = 0; i < r; i++) {
                if (b[i] != '\n') { if (il < (int)sizeof(in) - 1) in[il++] = b[i]; continue; }
                in[il] = 0; if (il && in[il - 1] == '\r') in[il - 1] = 0;
                il = 0; s_client = asked = true;
                int port = 0;
                if (in[0] == 'O') { flash(sock, in, b + i + 1, r - i - 1); goto done; }
                if (sscanf(in, "U,%d", &port) == 1 && port > 0 && port < 65536 && udp < 0 && !getpeername(sock, (struct sockaddr *)&to, &to_size)) {
                    to.sin_port = htons(port);
                    udp = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
                } else can_manager_wifi_line(in);
            }
        }
        size_t size;
        char *item;
        bool ok = true;
        if (!asked) continue;
        while (ok && (item = xRingbufferReceive(s_ring, &size, 0))) {
            if (item[0] == 'B') {
                // packed frames: 'B', the sequence number (u32 LE), then the item after its 'B'. Below
                // HEAP_KEPT free it isn't sent: BLE needs that memory more (a gap the app counts)
                uint32_t number = sequence++;
                // Wi-Fi's few transmit buffers fill (BLE shares the radio): wait for one, up to 20 ms, rather than drop
                bool sent = false;
                if (udp >= 0 && size + 4 <= sizeof(datagram) && esp_get_free_heap_size() >= HEAP_KEPT) {
                    datagram[0] = 'B';
                    memcpy(datagram + 1, &number, 4);
                    memcpy(datagram + 5, item + 1, size - 1);
                    for (int tries = 0; !(sent = sendto(udp, datagram, size + 4, 0, (struct sockaddr *)&to, sizeof(to)) >= 0) && errno == ENOMEM && tries < 20; tries++) vTaskDelay(1);
                }
                if (!sent) unsent++;
            } else ok = send_all(sock, item, size);
            vRingbufferReturnItem(s_ring, item);
        }
        int64_t now = esp_timer_get_time();
        if (ok && now - last_m >= 1000000) {
            last_m = now;
            int rssi = 0;
            esp_wifi_sta_get_rssi(&rssi);
            int n = can_manager_metrics(line, sizeof(line) - 64);
            const esp_app_desc_t *app = esp_app_get_description();
            n += snprintf(line + n, sizeof(line) - n, ",%d,%u\nV,%s %s\n", rssi, (unsigned)(esp_get_free_heap_size() / 1024), app->date, app->time);
            ok = send_all(sock, line, n);
            if (unsent != unsent_told) ESP_LOGW(TAG, "%lu datagrams not sent (no memory or transmit buffer)", (unsigned long)(unsent_told = unsent));
        }
        if (!ok) break;
    }
done:
    s_client = false;
    if (udp >= 0) close(udp);
    vTaskDelay(pdMS_TO_TICKS(20));  // a batch the lanes task was queuing as the client went
    drop_frames();
}

static void server_task(void *arg)
{
    (void)arg;
    int listener = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    int yes = 1;
    setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    struct sockaddr_in addr = { .sin_family = AF_INET, .sin_port = htons(WIFI_LINK_PORT), .sin_addr.s_addr = htonl(INADDR_ANY) };
    if (listener < 0 || bind(listener, (struct sockaddr *)&addr, sizeof(addr)) || listen(listener, 1)) {
        ESP_LOGE(TAG, "can't listen on port %d", WIFI_LINK_PORT);
        vTaskDelete(NULL);
    }
    for (;;) {
        int sock = accept(listener, NULL, NULL);
        if (sock < 0) { vTaskDelay(pdMS_TO_TICKS(1000)); continue; }
        struct timeval limit = { .tv_sec = 2 };  // a client that stops reading is dropped, not waited on
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &limit, sizeof(limit));
        ESP_LOGI(TAG, "client connected");
        if (s_on) serve(sock);
        close(sock);
        ESP_LOGI(TAG, "client gone");
    }
}

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    if (base == WIFI_EVENT && (id == WIFI_EVENT_STA_START || id == WIFI_EVENT_STA_DISCONNECTED) && s_on) {
        if (id == WIFI_EVENT_STA_DISCONNECTED) {
            s_reason = ((wifi_event_sta_disconnected_t *)data)->reason;
            vTaskDelay(pdMS_TO_TICKS(2000));  // the hotspot is off: don't spin
            // not found: look at what is there (the result says why), then try again from the scan's end
            if (s_reason == WIFI_REASON_NO_AP_FOUND && esp_wifi_scan_start(NULL, false) == ESP_OK) return;
        }
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_SCAN_DONE && s_on) {
        uint16_t count = 20;
        wifi_ap_record_t *found = calloc(count, sizeof(wifi_ap_record_t));
        uint16_t total = 0;
        esp_wifi_scan_get_ap_num(&total);
        s_seen = total; s_rssi = 0; s_channel = 0; s_auth = -1;
        if (found && esp_wifi_scan_get_ap_records(&count, found) == ESP_OK) {
            for (int i = 0; i < count; i++) if (!strcmp((char *)found[i].ssid, s_ssid)) { s_rssi = found[i].rssi; s_channel = found[i].primary; s_auth = found[i].authmode; break; }
        } else esp_wifi_clear_ap_list();
        free(found);
        ESP_LOGI(TAG, "scan: %d networks, %s %s (ch %d, %d dBm, auth %d)", s_seen, s_ssid, s_auth >= 0 ? "seen" : "not seen", s_channel, s_rssi, s_auth);
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        s_reason = 0;
        ESP_LOGI(TAG, "on %s as " IPSTR ", port %d", s_ssid, IP2STR(&((ip_event_got_ip_t *)data)->ip_info.ip), WIFI_LINK_PORT);
    }
}

static void save(void)
{
    nvs_handle_t h;
    if (nvs_open("wifi_link", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8(h, "on", s_on);
    nvs_set_str(h, "ssid", s_ssid);
    nvs_set_str(h, "pass", s_pass);
    nvs_set_str(h, "key", s_key);
    nvs_commit(h);
    nvs_close(h);
}

static void apply(bool on)
{
    if (!s_netif && on) {  // first time on: bring up the stack once, and leave it (stopping Wi-Fi frees the radio)
        // Nothing of this before it is asked for: a DashKit with Wi-Fi off has the memory it always had. Too little
        // free is refused here rather than run out later in BLE.
        if (esp_get_free_heap_size() < HEAP_NEEDED + RING_BYTES) {
            ESP_LOGE(TAG, "not enough memory for Wi-Fi: %u KB free", (unsigned)(esp_get_free_heap_size() / 1024));
            s_reason = -1;
            return;
        }
        if (!s_ring && (s_ring = xRingbufferCreate(RING_BYTES, RINGBUF_TYPE_NOSPLIT))) esp_log_set_vprintf(log_hook);
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        esp_err_t err = esp_netif_init();
        if (err == ESP_OK) err = esp_event_loop_create_default();
        if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) s_netif = esp_netif_create_default_wifi_sta();
        if (!s_ring || !s_netif || esp_wifi_init(&cfg) != ESP_OK || esp_wifi_set_storage(WIFI_STORAGE_RAM) != ESP_OK ||
            esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK ||
            esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_event, NULL) != ESP_OK ||
            esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_event, NULL) != ESP_OK ||
            xTaskCreatePinnedToCore(server_task, "wifi_link", 4096, NULL, 2, NULL, 0) != pdPASS) {
            ESP_LOGE(TAG, "Wi-Fi start failed");
            s_reason = -1;
            return;
        }
    }
    if (!s_netif) return;
    s_on = false;  // no reconnect for the disconnect that stopping causes
    esp_wifi_stop();
    s_on = on;
    if (!on) { ESP_LOGI(TAG, "off"); return; }
    wifi_config_t config = { 0 };
    memcpy(config.sta.ssid, s_ssid, strlen(s_ssid));
    memcpy(config.sta.password, s_pass, strlen(s_pass));
    // No minimum security: the name and password pick the network (a Samsung dual-band hotspot's 2.4 GHz side
    // isn't found with a WPA2 threshold; the earlier discovery firmware joined it without one)
    config.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;  // WPA3-only hotspots too
    if (esp_wifi_set_config(WIFI_IF_STA, &config) != ESP_OK || esp_wifi_start() != ESP_OK) { ESP_LOGE(TAG, "can't join %s", s_ssid); s_reason = -1; }
    else ESP_LOGI(TAG, "joining %s", s_ssid);
}

static void control_task(void *arg)
{
    (void)arg;
    // Wi-Fi left on starts once the rest is up and the image is confirmed (main.c, 10 s): never before BLE gets its
    // memory, and a start that crashes can't hold the DashKit in a boot loop (see wifi_link_init).
    vTaskDelay(pdMS_TO_TICKS(15000));
    for (;;) {
        uint32_t want = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // 1 off, 2 on, 3 just save
        if (want != 3) apply(want == 2);
        save();
    }
}

void wifi_link_init(void)
{
    nvs_handle_t h;
    uint8_t on = 0;
    if (nvs_open("wifi_link", NVS_READONLY, &h) == ESP_OK) {
        size_t n = sizeof(s_ssid);
        nvs_get_str(h, "ssid", s_ssid, &n);
        n = sizeof(s_pass);
        nvs_get_str(h, "pass", s_pass, &n);
        n = sizeof(s_key);
        nvs_get_str(h, "key", s_key, &n);
        nvs_get_u8(h, "on", &on);
        nvs_close(h);
    }
    esp_reset_reason_t reset = esp_reset_reason();
    bool crashed = reset == ESP_RST_PANIC || reset == ESP_RST_INT_WDT || reset == ESP_RST_TASK_WDT || reset == ESP_RST_WDT;
    if (xTaskCreatePinnedToCore(control_task, "wifi_ctl", 4096, NULL, 2, &s_control, 0) == pdPASS && on && s_ssid[0]) {
        if (crashed) ESP_LOGW(TAG, "not switching Wi-Fi on: the last restart was a crash (switch it on again from the app)");
        else xTaskNotify(s_control, 2, eSetValueWithOverwrite);
    }
}

bool wifi_link_command(const char *line)
{
    uint32_t want = 3;
    if (!s_control) return false;
    if (!strncmp(line, "WS,", 3) && strlen(line + 3) < sizeof(s_ssid)) strcpy(s_ssid, line + 3);
    else if (!strncmp(line, "WP,", 3) && strlen(line + 3) < sizeof(s_pass)) strcpy(s_pass, line + 3);
    else if (!strncmp(line, "WK,", 3) && strlen(line + 3) >= 16 && strlen(line + 3) < sizeof(s_key)) strcpy(s_key, line + 3);
    else if ((!strcmp(line, "W,1") && s_ssid[0]) || !strcmp(line, "W,0")) want = line[2] == '1' ? 2 : 1;
    else return false;
    xTaskNotify(s_control, want, eSetValueWithOverwrite);
    return true;
}

int wifi_link_status(char *out, size_t size)
{
    esp_netif_ip_info_t ip = { 0 };
    if (s_netif && s_on) esp_netif_get_ip_info(s_netif, &ip);
    int n = snprintf(out, size, "W,%d," IPSTR ",%d,%u,%d,%d,%d,%d", s_on, IP2STR(&ip.ip), s_reason, (unsigned)(esp_get_free_heap_size() / 1024),
                     s_seen, s_auth, s_channel, s_rssi);
    return n < 0 ? 0 : (size_t)n < size ? n : (int)size - 1;
}
