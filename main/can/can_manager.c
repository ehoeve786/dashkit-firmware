#include "can_manager.h"
#include "can_filter.h"
#include "automation_manager.h"
#include "dbc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdbool.h>
#include <string.h>

#ifdef CONFIG_DASHKIT_CAN_ACCEPT_ALL
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "dbc_generated.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "nvs.h"
#include <stdio.h>

// Discovery: every frame goes to USB as "ms,bus,id,data" (on payload change
// only); only DBC IDs go on to BLE, so the BLE link (and OTA) see normal load.
// Bus 0=chassis, 1=vehicle. Per-bus tables -> no race between the RX tasks.
static uint64_t s_last[DBC_HW_FILTER_BUS_COUNT][2048];
static uint32_t s_seen[DBC_HW_FILTER_BUS_COUNT][2048 / 32];

// Optional Wi-Fi sink: the same lines, UDP-broadcast on the joined network.
static RingbufHandle_t s_udp_rb;

static void wifi_reconnect(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    // Registered for STA_START and STA_DISCONNECTED only. Back off on
    // disconnect so a missing hotspot doesn't hog the radio BLE shares.
    if (id == WIFI_EVENT_STA_DISCONNECTED) vTaskDelay(pdMS_TO_TICKS(3000));
    esp_wifi_connect();  // harmless ESP_ERR_WIFI_NOT_STARTED after a stop
}

static void udp_task(void *arg)
{
    int sock = socket(AF_INET, SOCK_DGRAM, 0), on = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on));
    struct sockaddr_in dst = {
        .sin_family = AF_INET,
        .sin_port = htons(CONFIG_DASHKIT_DISCOVERY_UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_BROADCAST),
    };
    size_t n;
    void *p;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(20));  // batch lines into ~1.4 KB datagrams
        while ((p = xRingbufferReceiveUpTo(s_udp_rb, &n, 0, 1400))) {
            sendto(sock, p, n, 0, (struct sockaddr *)&dst, sizeof(dst));
            vRingbufferReturnItem(s_udp_rb, p);
        }
    }
}

// Apply credentials: empty SSID = Wi-Fi off. The Wi-Fi driver copies the
// config, so nothing here outlives the call.
static void wifi_apply(const char *ssid, const char *pass)
{
    if (!ssid[0]) {
        esp_wifi_stop();
        return;
    }
    wifi_config_t wc = {0};
    strlcpy((char *)wc.sta.ssid, ssid, sizeof(wc.sta.ssid));
    strlcpy((char *)wc.sta.password, pass, sizeof(wc.sta.password));
    esp_wifi_set_config(WIFI_IF_STA, &wc);
    if (esp_wifi_start() == ESP_OK) esp_wifi_disconnect();  // reconnects with the new config
}

// Best effort: a Wi-Fi failure must never take the dashboard down.
// Credentials live in NVS ("disc"/"ssid","pass"), set by the app over BLE.
static void wifi_init(void)
{
    s_udp_rb = xRingbufferCreate(16384, RINGBUF_TYPE_BYTEBUF);
    if (!s_udp_rb) return;
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.nvs_enable = 0;  // credentials are ours; don't let the driver cache them
    if (esp_wifi_init(&cfg) != ESP_OK) return;
    esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_START, wifi_reconnect, NULL);
    esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, wifi_reconnect, NULL);
    esp_wifi_set_mode(WIFI_MODE_STA);
    xTaskCreate(udp_task, "disc_udp", 3072, NULL, 3, NULL);

    char ssid[33] = "", pass[64] = "";
    size_t n;
    nvs_handle_t h;
    if (nvs_open("disc", NVS_READONLY, &h) == ESP_OK) {
        n = sizeof(ssid); nvs_get_str(h, "ssid", ssid, &n);
        n = sizeof(pass); nvs_get_str(h, "pass", pass, &n);
        nvs_close(h);
    }
    wifi_apply(ssid, pass);
}

esp_err_t can_discovery_set_wifi(const char *ssid, const char *pass)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open("disc", NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    err = nvs_set_str(h, "ssid", ssid);
    if (err == ESP_OK) err = nvs_set_str(h, "pass", pass);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    if (err == ESP_OK) wifi_apply(ssid, pass);
    return err;
}

static bool in_dbc_set(uint8_t bus, uint32_t id)
{
    const dbc_hw_filter_set_t *fs = &g_dbc_hw_filters[bus];
    for (uint8_t i = 0; i < fs->count; i++) {
        if (fs->ids[i] == id) return true;
    }
    return false;
}

// Returns true if the frame should still be forwarded to BLE.
static bool discovery_log(const can_tagged_frame_t *f)
{
    uint8_t b = f->bus_id, n = f->frame.dlc;
    uint32_t id = f->frame.id;
    if (b >= DBC_HW_FILTER_BUS_COUNT || f->frame.extended || n > 8) return false;

    uint64_t v = 0;
    memcpy(&v, f->frame.data, n);
    uint32_t bit = 1u << (id & 31), *seen = &s_seen[b][id >> 5];
    if (!(*seen & bit) || s_last[b][id] != v) {
        *seen |= bit;
        s_last[b][id] = v;
        char line[40];
        int len = snprintf(line, sizeof(line), "%lu,%u,0x%lx,",
                           (unsigned long)(esp_timer_get_time() / 1000),
                           (unsigned)b, (unsigned long)id);
        for (uint8_t i = 0; i < n; i++) {
            len += snprintf(line + len, 3, "%02x", f->frame.data[i]);
        }
        line[len++] = '\n';
        usb_serial_jtag_write_bytes(line, len, 0);  // never block the RX task
        if (s_udp_rb) xRingbufferSend(s_udp_rb, line, len, 0);
    }
    return in_dbc_set(b, id);
}
#endif

static const char *TAG = "can_mgr";

// Sized to absorb a few hundred ms of CAN traffic if BLE briefly stalls.
// At ~335 fps (realistic Tesla rates) this is ~750ms of headroom.
#define RX_QUEUE_DEPTH  256

static can_interface_t *s_interfaces[CAN_MANAGER_MAX_INTERFACES];
static int              s_iface_count = 0;
static QueueHandle_t    s_rx_queue = NULL;

// Callback invoked by CAN drivers from their RX tasks
static void on_frame_received(const can_tagged_frame_t *frame, void *user_ctx)
{
    (void)user_ctx;
    if (!s_rx_queue) return;
    // Feed the DBC value cache first so automation on_frame handlers (which read
    // signals via can_get) observe this frame's current contents.
    dbc_observe_frame(frame);
    // Automations run before the BLE filter so they see all raw traffic.
    automation_manager_on_frame(frame);
    // Drop non-matching frames at the source so the bridge task's batching
    // timeout reflects "no more *matching* frames" rather than "no more
    // frames at all". This avoids holding onto a matching frame while we
    // wait through a flood of unrelated traffic.
#ifdef CONFIG_DASHKIT_CAN_ACCEPT_ALL
    if (!discovery_log(frame)) return;
#endif
    if (!can_filter_should_forward(frame->bus_id, frame->frame.id)) return;

    can_tagged_frame_t stamped = *frame;
    stamped.timestamp_us = (uint32_t)esp_timer_get_time();
    // Drop frame if queue is full rather than blocking the driver
    xQueueSend(s_rx_queue, &stamped, 0);
}

esp_err_t can_manager_init(void)
{
    s_rx_queue = xQueueCreate(RX_QUEUE_DEPTH, sizeof(can_tagged_frame_t));
#ifdef CONFIG_DASHKIT_CAN_ACCEPT_ALL
    usb_serial_jtag_driver_config_t ucfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ucfg.tx_buffer_size = 4096;
    if (usb_serial_jtag_driver_install(&ucfg) == ESP_OK) {
        usb_serial_jtag_vfs_use_driver();  // console logs share the driver, no interleaving
    }
    wifi_init();
#endif
    if (!s_rx_queue) {
        return ESP_ERR_NO_MEM;
    }
    s_iface_count = 0;
    ESP_LOGI(TAG, "CAN manager initialized (queue depth=%d)", RX_QUEUE_DEPTH);
    return ESP_OK;
}

esp_err_t can_manager_add_interface(can_interface_t *iface)
{
    if (s_iface_count >= CAN_MANAGER_MAX_INTERFACES) {
        return ESP_ERR_NO_MEM;
    }
    iface->set_rx_callback(iface, on_frame_received, NULL);
    s_interfaces[s_iface_count++] = iface;
    ESP_LOGI(TAG, "Added interface bus_id=%d (total: %d)", iface->bus_id, s_iface_count);
    return ESP_OK;
}

esp_err_t can_manager_start(void)
{
    for (int i = 0; i < s_iface_count; i++) {
        esp_err_t err = s_interfaces[i]->init(s_interfaces[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Init failed for bus %d: %s", s_interfaces[i]->bus_id, esp_err_to_name(err));
            return err;
        }
        err = s_interfaces[i]->start(s_interfaces[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Start failed for bus %d: %s", s_interfaces[i]->bus_id, esp_err_to_name(err));
            return err;
        }
    }
    return ESP_OK;
}

esp_err_t can_manager_stop(void)
{
    for (int i = 0; i < s_iface_count; i++) {
        s_interfaces[i]->stop(s_interfaces[i]);
    }
    return ESP_OK;
}

esp_err_t can_manager_receive(can_tagged_frame_t *frame, uint32_t timeout_ms)
{
    if (!s_rx_queue) return ESP_ERR_INVALID_STATE;
    if (xQueueReceive(s_rx_queue, frame, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t can_manager_send(uint8_t bus_id, const can_frame_t *frame)
{
    if (!frame) return ESP_ERR_INVALID_ARG;
    for (int i = 0; i < s_iface_count; i++) {
        if (s_interfaces[i]->bus_id == bus_id) {
            if (!s_interfaces[i]->send) return ESP_ERR_NOT_SUPPORTED;
            return s_interfaces[i]->send(s_interfaces[i], frame);
        }
    }
    ESP_LOGW(TAG, "send: no interface for bus_id=%d", bus_id);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t can_manager_inject(const can_tagged_frame_t *frame)
{
    if (!s_rx_queue) return ESP_ERR_INVALID_STATE;
    if (!can_filter_should_forward(frame->bus_id, frame->frame.id)) {
        return ESP_OK;  // Silently dropped by filter
    }
    can_tagged_frame_t stamped = *frame;
    stamped.timestamp_us = (uint32_t)esp_timer_get_time();
    if (xQueueSend(s_rx_queue, &stamped, 0) == pdTRUE) {
        return ESP_OK;
    }
    return ESP_ERR_NO_MEM;
}
