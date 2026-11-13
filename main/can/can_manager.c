#include "can_manager.h"
#include "can_filter.h"
#include "automation_manager.h"
#include "vehicle_control.h"
#include "dbc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

static const char *TAG = "can_mgr";

// Sized to absorb a few hundred ms of CAN traffic if BLE briefly stalls.
// At ~335 fps (realistic Tesla rates) this is ~750ms of headroom.
#define RX_QUEUE_DEPTH  256

static can_interface_t *s_interfaces[CAN_MANAGER_MAX_INTERFACES];
static int              s_iface_count = 0;
static QueueHandle_t    s_rx_queue = NULL;

// USB serial link with the comma 3X (USB-Serial-JTAG driver, not the log console: its no-driver path can wedge
// for good when the host drops mid-line). Writes never block and are dropped while no host reads.
//  out: chassis-bus (0) frames the C3X can't see, as "F,<id>,<hex>": DAS_lanes 0x239, DAS_carLog 0x5D9,
//       DAS_autopilotDebug 0x247, UI_gpsVehicleSpeed 0x2F8, APP_frontParkAssistData 0x34B (~35 frames/s). The CAN
//       RX path only queues them (never blocks; dropped if 16 are already waiting).
//  in:  "C,7f0,<16 hex>" = openpilot state (COMMA_status), injected for the app as bus 1 frame 0x7F0 (never sent
//       on CAN; only that id is accepted). No status for 1 s (C3X off or unplugged): one all-zero frame clears it.
#define COMMA_ID 0x7F0
static QueueHandle_t    s_c3x_q = NULL;

static void comma_inject(uint64_t v)
{
    can_tagged_frame_t t = { .bus_id = 1, .frame = { .id = COMMA_ID, .dlc = 8 } };
    for (int i = 0; i < 8; i++) t.frame.data[i] = v >> (56 - 8 * i);
    can_manager_inject(&t);
}

static void lanes_task(void *arg)
{
    can_frame_t f;
    char b[48], in[48];
    int il = 0;
    int64_t last_c = 0;
    for (;;) {
        int r;
        while ((r = usb_serial_jtag_read_bytes(b, sizeof(b), 0)) > 0) {
            for (int i = 0; i < r; i++) {
                if (b[i] != '\n') { if (il < (int)sizeof(in) - 1) in[il++] = b[i]; continue; }
                in[il] = 0;
                unsigned long id;
                unsigned long long v;
                if (sscanf(in, "C,%lx,%16llx", &id, &v) == 2 && id == COMMA_ID) {
                    comma_inject(v);
                    last_c = esp_timer_get_time();
                }
                il = 0;
            }
        }
        if (last_c && esp_timer_get_time() - last_c > 1000000) {
            comma_inject(0);
            last_c = 0;
        }
        if (xQueueReceive(s_c3x_q, &f, pdMS_TO_TICKS(100)) != pdTRUE) continue;
        int n = snprintf(b, sizeof(b), "F,%03lx,", (unsigned long)f.id);
        for (int i = 0; i < f.dlc && i < 8; i++) n += snprintf(b + n, sizeof(b) - n, "%02x", f.data[i]);
        b[n++] = '\n';
        usb_serial_jtag_write_bytes(b, n, 0);
    }
}

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
    vehicle_control_on_frame(frame);
    if (frame->bus_id == 0 && !frame->frame.extended) {
        switch (frame->frame.id) {
        case 0x239: case 0x5D9: case 0x247: case 0x2F8: case 0x34B:
            xQueueSend(s_c3x_q, &frame->frame, 0);
        }
    }
    // Drop non-matching frames at the source so the bridge task's batching
    // timeout reflects "no more *matching* frames" rather than "no more
    // frames at all". This avoids holding onto a matching frame while we
    // wait through a flood of unrelated traffic.
    if (!can_filter_should_forward(frame->bus_id, frame->frame.id)) return;

    can_tagged_frame_t stamped = *frame;
    stamped.timestamp_us = (uint32_t)esp_timer_get_time();
    // Drop frame if queue is full rather than blocking the driver
    xQueueSend(s_rx_queue, &stamped, 0);
}

esp_err_t can_manager_init(void)
{
    s_rx_queue = xQueueCreate(RX_QUEUE_DEPTH, sizeof(can_tagged_frame_t));
    if (!s_rx_queue) {
        return ESP_ERR_NO_MEM;
    }
    usb_serial_jtag_driver_config_t usj = { .tx_buffer_size = 1024, .rx_buffer_size = 256 };
    s_c3x_q = xQueueCreate(16, sizeof(can_frame_t));
    if (!s_c3x_q || usb_serial_jtag_driver_install(&usj) != ESP_OK || xTaskCreate(lanes_task, "lanes", 3072, NULL, 1, NULL) != pdPASS) {
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
