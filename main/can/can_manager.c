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
#include <stdlib.h>
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
//  in:  "C,<bus>,<id>,<hex>" frames for the app only (never sent on CAN): bus 1 id 0x7F0 = openpilot state
//       (COMMA_status), bus 2 = the C3X's party bus, bus 3 = its Autopilot side. Anything else, or a line that isn't
//       exactly one frame (a write cut short on the C3X), is dropped. No status for 2 s (C3X off or unplugged): one
//       all-zero COMMA_status clears it.
#define COMMA_ID 0x7F0
static QueueHandle_t    s_c3x_q = NULL;

static bool c3x_line(char *in, int len)
{
    unsigned bus;
    unsigned long id;
    char hex[17];
    int n = -1;
    if (len && in[len - 1] == '\r') in[--len] = 0;
    if (sscanf(in, "C,%u,%lx,%16[0-9a-fA-F]%n", &bus, &id, hex, &n) != 3 || n != len || id > 0x7FF ||
        !((bus == 1 && id == COMMA_ID) || bus == 2 || bus == 3)) return false;
    int hl = strlen(hex);
    if (hl % 2) return false;
    can_tagged_frame_t t = { .bus_id = bus, .frame = { .id = id, .dlc = hl / 2 } };
    for (int i = 0; i < hl / 2; i++) {
        char h[3] = { hex[2 * i], hex[2 * i + 1], 0 };
        t.frame.data[i] = strtoul(h, NULL, 16);
    }
    can_manager_inject(&t);
    return bus == 1;
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
                if (c3x_line(in, il)) last_c = esp_timer_get_time();
                il = 0;
            }
        }
        if (last_c && esp_timer_get_time() - last_c > 2000000) {
            char z[] = "C,1,7f0,0000000000000000";
            c3x_line(z, sizeof(z) - 1);
            last_c = 0;
        }
        if (xQueueReceive(s_c3x_q, &f, pdMS_TO_TICKS(10)) != pdTRUE) continue;  // 10 ms: ~350 C3X lines/s come in
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
    usb_serial_jtag_driver_config_t usj = { .tx_buffer_size = 1024, .rx_buffer_size = 4096 };
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

// Hardware filters per bus = the firmware's own IDs + the ones the app asked for (no 32-ID limit: the driver merges
// them into masks). Called from the BLE task whenever the app's filter changes.
void can_manager_apply_filters(void)
{
    static uint16_t ids[320];
    for (int i = 0; i < s_iface_count; i++) {
        can_interface_t *f = s_interfaces[i];
        if (!f->set_ids) continue;
        size_t n = 0;
        if (f->bus_id < DBC_HW_FILTER_BUS_COUNT) {
            const dbc_hw_filter_set_t *fs = &g_dbc_hw_filters[f->bus_id];
            for (; n < fs->count && n < 64; n++) ids[n] = fs->ids[n];
        }
        n += can_filter_ids(f->bus_id, ids + n, sizeof(ids) / sizeof(*ids) - n);
        f->set_ids(f, ids, n);
    }
}
