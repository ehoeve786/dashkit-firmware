#include "can_manager.h"
#include "can_filter.h"
#include "can_diag.h"
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
#include "sdkconfig.h"

#ifdef CONFIG_DASHKIT_CAN_STREAM_ALL
#include "can_route.h"
#endif

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
//       exactly one frame (a write cut short on the C3X), is dropped. No status for 1 s (C3X off or unplugged): one
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
        if (last_c && esp_timer_get_time() - last_c > 1000000) {
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

#ifdef CONFIG_DASHKIT_CAN_STREAM_ALL
// Stream-all build: the driver delivers every frame on the bus, so the software
// route decides per frame (one bit lookup each) whether the firmware uses it
// and whether the app gets it. Anything else is dropped here, before any work.
#define ROUTE_REPORT_US  (5 * 1000 * 1000)
static volatile uint32_t s_seen, s_core, s_forwarded, s_queue_full;
static uint32_t          s_rep_seen, s_rep_core, s_rep_forwarded, s_rep_queue_full;

static void route_report_cb(void *arg)
{
    (void)arg;
    uint32_t seen = s_seen, core = s_core, fwd = s_forwarded, full = s_queue_full;
    uint32_t secs = ROUTE_REPORT_US / 1000000;
    ESP_LOGI(TAG, "route: %lu fps on the buses, %lu fps used by firmware, %lu fps to the app, "
                  "queue-full drops %lu (+%lu)",
             (unsigned long)((seen - s_rep_seen) / secs),
             (unsigned long)((core - s_rep_core) / secs),
             (unsigned long)((fwd - s_rep_forwarded) / secs),
             (unsigned long)full, (unsigned long)(full - s_rep_queue_full));
    uint8_t d[8];
    can_diag_pack4(d, (seen - s_rep_seen) / secs, (core - s_rep_core) / secs,
                   (fwd - s_rep_forwarded) / secs, full);
    can_diag_emit(0, CAN_DIAG_ID_ROUTE, d);
    s_rep_seen = seen; s_rep_core = core; s_rep_forwarded = fwd; s_rep_queue_full = full;
}

// Callback invoked by CAN drivers from their RX tasks
static void on_frame_received(const can_tagged_frame_t *frame, void *user_ctx)
{
    (void)user_ctx;
    if (!s_rx_queue) return;
    s_seen++;
    if (frame->bus_id == 0 && !frame->frame.extended) {
        switch (frame->frame.id) {
        case 0x239: case 0x5D9: case 0x247: case 0x2F8: case 0x34B:
            xQueueSend(s_c3x_q, &frame->frame, 0);
        }
    }
    uint8_t route = can_route_classify(frame->bus_id, frame->frame.id, frame->frame.extended);
    if (!route) return;
    if (route & CAN_ROUTE_CORE) {
        // Same order and same frame set as the default build, where the
        // hardware filters only pass manifest ids.
        s_core++;
        dbc_observe_frame(frame);
        automation_manager_on_frame(frame);
        vehicle_control_on_frame(frame);
    }
    if (!(route & CAN_ROUTE_FORWARD)) return;

    can_tagged_frame_t stamped = *frame;
    stamped.timestamp_us = (uint32_t)esp_timer_get_time();
    // Drop frame if queue is full rather than blocking the driver
    if (xQueueSend(s_rx_queue, &stamped, 0) == pdTRUE) {
        s_forwarded++;
    } else {
        s_queue_full++;
    }
}
#else
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
#endif  // CONFIG_DASHKIT_CAN_STREAM_ALL

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
#ifdef CONFIG_DASHKIT_CAN_STREAM_ALL
    can_route_init();
    const esp_timer_create_args_t rep_args = { .callback = route_report_cb, .name = "can_route" };
    esp_timer_handle_t rep_timer;
    if (esp_timer_create(&rep_args, &rep_timer) == ESP_OK) {
        esp_timer_start_periodic(rep_timer, ROUTE_REPORT_US);
    }
#endif
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

esp_err_t can_manager_inject_unfiltered(const can_tagged_frame_t *frame)
{
    if (!s_rx_queue) return ESP_ERR_INVALID_STATE;
    can_tagged_frame_t stamped = *frame;
    stamped.timestamp_us = (uint32_t)esp_timer_get_time();
    return xQueueSend(s_rx_queue, &stamped, 0) == pdTRUE ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t can_manager_inject(const can_tagged_frame_t *frame)
{
    if (!s_rx_queue) return ESP_ERR_INVALID_STATE;
#ifdef CONFIG_DASHKIT_CAN_STREAM_ALL
    if (!(can_route_classify(frame->bus_id, frame->frame.id, frame->frame.extended) & CAN_ROUTE_FORWARD)) {
        return ESP_OK;  // Silently dropped by filter
    }
#else
    if (!can_filter_should_forward(frame->bus_id, frame->frame.id)) {
        return ESP_OK;  // Silently dropped by filter
    }
#endif
    can_tagged_frame_t stamped = *frame;
    stamped.timestamp_us = (uint32_t)esp_timer_get_time();
    if (xQueueSend(s_rx_queue, &stamped, 0) == pdTRUE) {
        return ESP_OK;
    }
    return ESP_ERR_NO_MEM;
}
