#include "can_manager.h"
#include "can_filter.h"
#include "can_diag.h"
#include "host_tx.h"
#include "automation_manager.h"
#include "vehicle_control.h"
#include "dbc.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
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

// Routing totals for the diagnostic records: plain counters bumped from the two
// CAN RX tasks. A lost increment only makes a reported rate slightly low.
#define ROUTE_REPORT_US  (5 * 1000 * 1000)
static volatile uint32_t   s_seen, s_used, s_forwarded, s_queue_full;
static uint32_t            s_rep_seen, s_rep_used, s_rep_forwarded;
static esp_timer_handle_t  s_route_timer;

static void route_report_cb(void *arg)
{
    (void)arg;
    uint32_t seen = s_seen, used = s_used, fwd = s_forwarded, full = s_queue_full;
    uint32_t secs = ROUTE_REPORT_US / 1000000;
    if (can_diag_enabled()) {
        uint8_t d[8];
        can_diag_pack4(d, (seen - s_rep_seen) / secs, (used - s_rep_used) / secs,
                       (fwd - s_rep_forwarded) / secs, full);
        can_diag_emit(0, CAN_DIAG_ID_ROUTE, d);
    }
    s_rep_seen = seen; s_rep_used = used; s_rep_forwarded = fwd;
}


// USB serial link with the comma 3X (USB-Serial-JTAG driver, not the log console: its no-driver path can wedge
// for good when the host drops mid-line). Writes never block; a line that doesn't fit is dropped whole.
//  out: "A,<bus>,<id>,<hex>" (id: 3 hex digits, 8 if extended), batched:
//       - always: the chassis-bus (0) frames the C3X can't see: DAS_lanes 0x239, DAS_carLog 0x5D9,
//         DAS_autopilotDebug 0x247, UI_gpsVehicleSpeed 0x2F8, APP_frontParkAssistData 0x34B (~35 frames/s);
//       - while the app takes a whole bus ("Log every frame") and the C3X is up: every frame of that bus (0 and 1),
//         the rest of it here instead of over BLE (which still gets the ids the app asked for by id). The C3X
//         records them.
//       Once a second while the C3X is up: "S,<queue drops>,<write drops>" (totals since boot).
//       The CAN RX path only queues (never blocks; dropped if the queue is full).
//  in:  "C,<bus>,<id>,<hex>" frames for the app only (never sent on CAN): bus 1 id 0x7F0 = openpilot state
//       (COMMA_status), bus 2 = the C3X's party bus, bus 3 = its Autopilot side. Anything else, or a line that isn't
//       exactly one frame (a write cut short on the C3X), is dropped. No status for 2 s (C3X off or unplugged): one
//       all-zero COMMA_status clears it, and whole buses go back to BLE.
//
// The same port serves the CanPilot app on a phone, plugged in instead of the C3X (its transmit lines can also
// come over BLE, see can_manager_host_line; then the C3X stays on this port and sees none of the "H" and "D" lines):
//  in:  "L,1" once a second: send every frame of both buses ("L,0", or 2 s without it: stop).
//       "X,1" once a second and "T,<bus>,<id>,<hex>": transmit a frame on CAN, under the rules in host_tx.h.
//  out: the "A" lines; "D,<bus>,<id>,<hex>" for the diagnostic records of can_diag.h (bus error counters among
//       them), which are switched on while the app is there; and once a second
//       "H,<armed>,<sent>,<refused>,<expired>,<queue drops>,<write drops>".
#define COMMA_ID 0x7F0
typedef struct { uint32_t id; uint8_t bus, dlc, ext, data[8]; } c3x_frame_t;
static QueueHandle_t    s_c3x_q = NULL;
static volatile bool    s_c3x_up;
static volatile bool    s_c3x_all[2];  // C3X up and a device asked for this whole bus
static volatile uint32_t s_c3x_qdrop, s_c3x_wdrop;
static volatile bool    s_host_all;    // the CanPilot app asked for every frame
static volatile bool    s_host_up;     // it is there: streaming or armed to transmit
static volatile bool    s_host_usb;    // it is on the USB port: the "H" and "D" lines go there
static volatile bool    s_host_armed;
typedef struct { char text[32]; } host_line_t;
static QueueHandle_t    s_host_line_q; // transmit lines from BLE, for the lanes task
static volatile uint8_t s_bus_err[2][3];   // TEC, REC, flags of the last CAN_DIAG_ID_BUS_ERR record
static volatile bool    s_bus_err_seen[2];

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

static int a_line(char *o, const c3x_frame_t *f)
{
    static const char hx[] = "0123456789abcdef";
    char *p = o;
    *p++ = f->bus & CAN_DIAG_BUS_FLAG ? 'D' : 'A'; *p++ = ','; *p++ = '0' + (f->bus & 0x0F); *p++ = ',';
    int nd = f->ext ? 8 : 3;
    for (int i = nd - 1; i >= 0; i--) p[i] = hx[(f->id >> (4 * (nd - 1 - i))) & 15];
    p += nd;
    *p++ = ',';
    for (int i = 0; i < f->dlc; i++) { *p++ = hx[f->data[i] >> 4]; *p++ = hx[f->data[i] & 15]; }
    *p++ = '\n';
    return p - o;
}

static void lanes_task(void *arg)
{
    static char out[1024];
    c3x_frame_t f;
    char b[48], in[48];
    int il = 0;
    int64_t last_c = 0, last_s = 0, last_a = 0, last_l = 0, last_h = 0, last_u = 0;
    host_line_t ble_line;
    bool diag_before = false;
    for (;;) {
        int r;
        while ((r = usb_serial_jtag_read_bytes(b, sizeof(b), 0)) > 0) {
            for (int i = 0; i < r; i++) {
                if (b[i] != '\n') { if (il < (int)sizeof(in) - 1) in[il++] = b[i]; continue; }
                in[il] = 0;
                uint32_t line_ms = (uint32_t)(esp_timer_get_time() / 1000);
                if (il && in[il - 1] == '\r') in[--il] = 0;
                if (!strcmp(in, "L,1")) last_u = last_l = esp_timer_get_time();
                else if (!strcmp(in, "L,0")) last_l = 0;
                else if (host_tx_line(in, line_ms)) last_u = esp_timer_get_time();
                else if (c3x_line(in, il)) { last_c = esp_timer_get_time(); s_c3x_up = true; }
                il = 0;
            }
        }
        int64_t now = esp_timer_get_time();
        if (last_c && now - last_c > 2000000) {
            char z[] = "C,1,7f0,0000000000000000";
            c3x_line(z, sizeof(z) - 1);
            last_c = 0;
            s_c3x_up = false;
        }
        if (now - last_a >= 100000) {  // can_filter_all takes the filter lock: sampled, not per frame
            last_a = now;
            s_host_all = last_l && now - last_l < 2000000;
            for (uint8_t bus = 0; bus < 2; bus++) s_c3x_all[bus] = s_host_all || (s_c3x_up && can_filter_all(bus));
            // The app is there: it also gets the diagnostic records, as "D" lines.
            s_host_armed = host_tx_armed((uint32_t)(now / 1000));
            s_host_usb = last_u && now - last_u < 2000000;
            bool up = s_host_all || s_host_armed;
            if (up && !s_host_up) { diag_before = can_diag_enabled(); can_diag_set_enabled(true); }
            if (!up && s_host_up) can_diag_set_enabled(diag_before);
            s_host_up = up;
        }
        while (xQueueReceive(s_host_line_q, &ble_line, 0) == pdTRUE) host_tx_line(ble_line.text, (uint32_t)(now / 1000));
        // Frames the app asked to have transmitted, each at a moment that is safe for it.
        uint8_t tx_bus;
        can_frame_t tx_frame;
        while (host_tx_due((uint32_t)(now / 1000), &tx_bus, &tx_frame)) host_tx_result(can_manager_send(tx_bus, &tx_frame) == ESP_OK);
        int n = 0, lines = 0;
        if (s_c3x_up && now - last_s >= 1000000) {
            last_s = now;
            n = snprintf(out, sizeof(out), "S,%lu,%lu\n", (unsigned long)s_c3x_qdrop, (unsigned long)s_c3x_wdrop);
        }
        if (s_host_usb && now - last_h >= 1000000) {
            last_h = now;
            const host_tx_stats_t *tx = host_tx_stats();
            n += snprintf(out + n, sizeof(out) - n, "H,%d,%lu,%lu,%lu,%lu,%lu\n", host_tx_armed((uint32_t)(now / 1000)), (unsigned long)tx->sent,
                          (unsigned long)tx->refused, (unsigned long)tx->expired, (unsigned long)s_c3x_qdrop, (unsigned long)s_c3x_wdrop);
        }
        // 10 ms wait when idle: ~350 C3X lines/s come in
        if (xQueueReceive(s_c3x_q, &f, n ? 0 : pdMS_TO_TICKS(10)) == pdTRUE) {
            do { n += a_line(out + n, &f); lines++; }
            while (n < (int)sizeof(out) - 32 && xQueueReceive(s_c3x_q, &f, 0) == pdTRUE);
        }
        if (n && usb_serial_jtag_write_bytes(out, n, 0) != n) s_c3x_wdrop += lines;
    }
}

static void c3x_queue(const can_tagged_frame_t *t)
{
    c3x_frame_t f = { .id = t->frame.id, .bus = t->bus_id, .dlc = t->frame.dlc > 8 ? 8 : t->frame.dlc,
                      .ext = t->frame.extended };
    memcpy(f.data, t->frame.data, f.dlc);
    if (xQueueSend(s_c3x_q, &f, 0) != pdTRUE) s_c3x_qdrop++;
}

// Chassis-bus frames the comma 3X always gets over USB.
static inline bool c3x_tap(const can_tagged_frame_t *frame)
{
    if (frame->bus_id != 0 || frame->frame.extended) return false;
    switch (frame->frame.id) {
    case 0x239: case 0x5D9: case 0x247: case 0x2F8: case 0x34B:
        c3x_queue(frame);
        return true;
    }
    return false;
}

// Callback invoked by CAN drivers from their RX tasks
static void on_frame_received(const can_tagged_frame_t *frame, void *user_ctx)
{
    (void)user_ctx;
    if (!s_rx_queue) return;
    s_seen++;
    // The controllers deliver every frame on the bus. The firmware's own work is only for the ids in
    // dbc/used_messages.txt, the same frames the hardware filters let through before; the rest of the bus
    // skips straight to the app's filter, so it costs two bit lookups per frame.
    if (!frame->frame.extended) host_tx_saw(frame->bus_id, frame->frame.id, (uint32_t)(esp_timer_get_time() / 1000));
    if (!frame->frame.extended && can_filter_is_used(frame->bus_id, frame->frame.id)) {
        s_used++;
        // Feed the DBC value cache first so automation on_frame handlers (which read
        // signals via can_get) observe this frame's current contents.
        dbc_observe_frame(frame);
        // Automations run before the BLE filter so they see these frames whatever the app asked for.
        automation_manager_on_frame(frame);
        vehicle_control_on_frame(frame);
    }
    // Whole bus with the C3X up: every frame of it goes to the C3X; BLE keeps the ids the devices asked for.
    if (!c3x_tap(frame) && frame->bus_id < 2 && s_c3x_all[frame->bus_id]) c3x_queue(frame);
    // Drop non-matching frames at the source so the bridge task's batching
    // timeout reflects "no more *matching* frames" rather than "no more
    // frames at all". This avoids holding onto a matching frame while we
    // wait through a flood of unrelated traffic.
    if (!can_filter_should_forward(frame->bus_id, frame->frame.id)) return;

    can_tagged_frame_t stamped = *frame;
    stamped.timestamp_us = (uint32_t)esp_timer_get_time();
    // Drop frame if queue is full rather than blocking the driver
    if (xQueueSend(s_rx_queue, &stamped, 0) == pdTRUE) s_forwarded++;
    else s_queue_full++;
}

esp_err_t can_manager_init(void)
{
    s_rx_queue = xQueueCreate(RX_QUEUE_DEPTH, sizeof(can_tagged_frame_t));
    if (!s_rx_queue) {
        return ESP_ERR_NO_MEM;
    }
    usb_serial_jtag_driver_config_t usj = { .tx_buffer_size = 8192, .rx_buffer_size = 4096 };
    s_c3x_q = xQueueCreate(512, sizeof(c3x_frame_t));
    s_host_line_q = xQueueCreate(16, sizeof(host_line_t));
    if (!s_c3x_q || !s_host_line_q || usb_serial_jtag_driver_install(&usj) != ESP_OK || xTaskCreate(lanes_task, "lanes", 3072, NULL, 1, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    s_iface_count = 0;
    const esp_timer_create_args_t route_args = { .callback = route_report_cb, .name = "can_route" };
    if (esp_timer_create(&route_args, &s_route_timer) == ESP_OK) {
        esp_timer_start_periodic(s_route_timer, ROUTE_REPORT_US);
    }
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
    if (frame->frame.id == CAN_DIAG_ID_BUS_ERR && (frame->bus_id & 0x0F) < 2) {
        uint8_t bus = frame->bus_id & 0x0F;
        for (int i = 0; i < 3; i++) s_bus_err[bus][i] = frame->frame.data[i];
        s_bus_err_seen[bus] = true;
    }
    if (s_host_usb && (frame->bus_id & CAN_DIAG_BUS_FLAG)) c3x_queue(frame);   // diagnostic records, to the CanPilot app on USB too
    can_tagged_frame_t stamped = *frame;
    stamped.timestamp_us = (uint32_t)esp_timer_get_time();
    return xQueueSend(s_rx_queue, &stamped, 0) == pdTRUE ? ESP_OK : ESP_ERR_NO_MEM;
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

bool can_manager_host_line(const char *line)
{
    host_line_t l;
    size_t len = strlen(line);
    if (!s_host_line_q || len >= sizeof(l.text) || (line[0] != 'X' && line[0] != 'T') || line[1] != ',') return false;
    memcpy(l.text, line, len + 1);
    return xQueueSend(s_host_line_q, &l, 0) == pdTRUE;
}

int can_manager_host_status(char *out, size_t size)
{
    const host_tx_stats_t *tx = host_tx_stats();
    int n = snprintf(out, size, "H,%d,%lu,%lu,%lu", s_host_armed, (unsigned long)tx->sent, (unsigned long)tx->refused, (unsigned long)tx->expired);
    for (int bus = 0; bus < 2 && n > 0 && (size_t)n < size; bus++) {
        if (!s_bus_err_seen[bus]) continue;
        n += snprintf(out + n, size - n, "\nD,%d,%08lx,%02x%02x%02x", bus, (unsigned long)CAN_DIAG_ID_BUS_ERR,
                      s_bus_err[bus][0], s_bus_err[bus][1], s_bus_err[bus][2]);
    }
    return n < 0 ? 0 : (size_t)n < size ? n : (int)size - 1;
}
