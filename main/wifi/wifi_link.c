#include "wifi_link.h"
#include "can_manager.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "nvs.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

static const char *TAG = "wifi_link";

#define RING_BYTES  (16 * 1024)
#define HEAP_NEEDED (80 * 1024)  // free heap Wi-Fi start-up needs, with room left for BLE
#define LOG_MAX     300   // longer log lines are cut

// One ring for both: log lines (always, oldest dropped for new) and the frame lines (only while a client asks).
static RingbufHandle_t  s_ring;
static TaskHandle_t     s_control;   // applies on/off: Wi-Fi start-up is too heavy for the BLE host task
static volatile bool    s_on, s_client;
static esp_netif_t     *s_netif;
static char             s_ssid[33], s_pass[65];
static volatile int     s_reason;    // why it isn't on the network: the last disconnect reason (wifi_err_reason_t), -1 start failed, 0 none

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
            p[0] = 'G'; p[1] = ',';
            va_copy(copy, args);
            vsnprintf(p + 2, len + 1, fmt, copy);
            va_end(copy);
            if (p[len + 1] == '\n') p[len + 1] = ' ';  // the line's own newline: one line per call
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

// One client at a time: its "L" lines in, the ring and the metrics out, until it goes or Wi-Fi is switched off.
static void serve(int sock)
{
    char b[64], in[16], line[192];
    int il = 0;
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
                if (b[i] == '\n') { in[il] = 0; if (il && in[il - 1] == '\r') in[il - 1] = 0; can_manager_wifi_line(in); il = 0; s_client = asked = true; }
                else if (il < (int)sizeof(in) - 1) in[il++] = b[i];
            }
        }
        size_t size;
        char *item;
        bool ok = true;
        if (!asked) continue;
        while (ok && (item = xRingbufferReceive(s_ring, &size, 0))) {
            ok = send_all(sock, item, size);
            vRingbufferReturnItem(s_ring, item);
        }
        int64_t now = esp_timer_get_time();
        if (ok && now - last_m >= 1000000) {
            last_m = now;
            int rssi = 0;
            esp_wifi_sta_get_rssi(&rssi);
            int n = can_manager_metrics(line, sizeof(line) - 24);
            n += snprintf(line + n, sizeof(line) - n, ",%d,%u\n", rssi, (unsigned)(esp_get_free_heap_size() / 1024));
            ok = send_all(sock, line, n);
        }
        if (!ok) break;
    }
    s_client = false;
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
        }
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
    config.sta.threshold.authmode = s_pass[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
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
    else if ((!strcmp(line, "W,1") && s_ssid[0]) || !strcmp(line, "W,0")) want = line[2] == '1' ? 2 : 1;
    else return false;
    xTaskNotify(s_control, want, eSetValueWithOverwrite);
    return true;
}

int wifi_link_status(char *out, size_t size)
{
    esp_netif_ip_info_t ip = { 0 };
    if (s_netif && s_on) esp_netif_get_ip_info(s_netif, &ip);
    int n = snprintf(out, size, "W,%d," IPSTR ",%d,%u", s_on, IP2STR(&ip.ip), s_reason, (unsigned)(esp_get_free_heap_size() / 1024));
    return n < 0 ? 0 : (size_t)n < size ? n : (int)size - 1;
}
