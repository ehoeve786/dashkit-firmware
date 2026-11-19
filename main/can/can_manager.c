#include "can_manager.h"
#include "can_filter.h"
#include "can_diag.h"
#include "can_route.h"
#include "automation_manager.h"
#include "vehicle_control.h"
#include "dbc.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "sdkconfig.h"

#include "can_route.h"

static const char *TAG = "can_mgr";

// Sized to absorb a few hundred ms of CAN traffic if BLE briefly stalls.
// At ~335 fps (realistic Tesla rates) this is ~750ms of headroom.
#define RX_QUEUE_DEPTH  256

static can_interface_t *s_interfaces[CAN_MANAGER_MAX_INTERFACES];
static int              s_iface_count = 0;
static QueueHandle_t    s_rx_queue = NULL;   // frames the app asked for by id, and diagnostic records
static QueueHandle_t    s_bulk_queue = NULL; // frames forwarded only by a whole-bus entry: sent when s_rx_queue is empty
static SemaphoreHandle_t s_rx_count = NULL;  // one count per frame in either queue

// Bulk frames when the link can't carry them all (a phone call leaves it about a quarter of its speed): instead of
// losing whatever arrives while the queue is full, each id is thinned to one frame per s_bulk_gap_ms, so slow
// messages survive whole and fast ones stay evenly sampled. The gap doubles (at most every 25 ms) while the queue is
// over 3/4 full and halves after 1 s with it under 1/4. 0 = no thinning, where a full-speed link stays.
#define BULK_BUSES 2
static uint16_t s_bulk_last[BULK_BUSES][2048];  // ms (low 16 bits) of the last frame queued per id
static volatile uint16_t s_bulk_gap_ms;

uint16_t can_manager_bulk_gap_ms(void) { return s_bulk_gap_ms; }

static bool bulk_due(const can_tagged_frame_t *f)
{
    static uint32_t raised, lowered;  // shared by the RX tasks; a lost update only delays one adjustment
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    uint16_t gap = s_bulk_gap_ms;
    if (now - raised >= 25) {
        UBaseType_t n = uxQueueMessagesWaiting(s_bulk_queue);
        if (n > RX_QUEUE_DEPTH * 3 / 4) {
            raised = lowered = now;
            s_bulk_gap_ms = gap = gap == 0 ? 50 : gap < 800 ? gap * 2 : 1000;  // 1 s at most: once-a-second ids stay whole
        } else if (n < RX_QUEUE_DEPTH / 4 && gap && now - lowered >= 1000) {
            lowered = now;
            s_bulk_gap_ms = gap = gap > 50 ? gap / 2 : 0;
        }
    }
    if (!gap || f->frame.extended || f->frame.id > 0x7FF || f->bus_id >= BULK_BUSES) return true;
    uint16_t *last = &s_bulk_last[f->bus_id][f->frame.id];
    if ((uint16_t)((uint16_t)now - *last) < gap) return false;
    *last = (uint16_t)now;
    return true;
}

// Queues a frame for the app (never blocks the caller). `route` holds CAN_ROUTE_BULK for the bulk queue.
static bool enqueue(const can_tagged_frame_t *frame, uint8_t route)
{
    if ((route & CAN_ROUTE_BULK) && !bulk_due(frame)) return false;
    can_tagged_frame_t stamped = *frame;
    stamped.timestamp_us = (uint32_t)esp_timer_get_time();
    if (xQueueSend(route & CAN_ROUTE_BULK ? s_bulk_queue : s_rx_queue, &stamped, 0) != pdTRUE) return false;
    xSemaphoreGive(s_rx_count);
    return true;
}

// The drivers deliver both buses in full. Every frame goes to the DBC value cache and the automations; the
// software route decides per frame (one bit lookup) whether the app gets it too.
#define ROUTE_REPORT_US  (5 * 1000 * 1000)
static volatile uint32_t s_seen, s_core, s_forwarded, s_queue_full;
static uint32_t          s_rep_seen, s_rep_core, s_rep_forwarded, s_rep_queue_full;

static void route_report_cb(void *arg)
{
    (void)arg;
    uint32_t seen = s_seen, core = s_core, fwd = s_forwarded, full = s_queue_full;
    uint32_t secs = ROUTE_REPORT_US / 1000000;
    ESP_LOGI(TAG, "route: %lu fps on the buses, %lu fps in the DBC, %lu fps to the app, "
                  "queue-full drops %lu (+%lu)",
             (unsigned long)((seen - s_rep_seen) / secs),
             (unsigned long)((core - s_rep_core) / secs),
             (unsigned long)((fwd - s_rep_forwarded) / secs),
             (unsigned long)full, (unsigned long)(full - s_rep_queue_full));
    if (dbc_used_changed()) can_manager_apply_filters();  // firmware code read a message for the first time
    uint8_t d[8];
    can_diag_pack4(d, (seen - s_rep_seen) / secs, (core - s_rep_core) / secs,
                   (fwd - s_rep_forwarded) / secs, full);
    can_diag_emit(0, CAN_DIAG_ID_ROUTE, d);
    s_rep_seen = seen; s_rep_core = core; s_rep_forwarded = fwd; s_rep_queue_full = full;
}

// USB serial link with the comma 3X (USB-Serial-JTAG driver, not the log console: its no-driver path can wedge
// for good when the host drops mid-line). Writes never block; a line that doesn't fit is dropped whole.
//  out: "A,<bus>,<id>,<hex>" (id: 3 hex digits, 8 if extended), batched:
//       - always: the chassis-bus (0) frames the C3X can't see: DAS_lanes 0x239, DAS_carLog 0x5D9,
//         DAS_autopilotDebug 0x247, UI_gpsVehicleSpeed 0x2F8, APP_frontParkAssistData 0x34B (~35 frames/s);
//       - while the app takes a whole bus ("Log every frame") and the C3X is up: that bus's frames (0 and 1, all of
//         them), here instead of over BLE (which keeps the ids the app asked for by id). The C3X records them.
//       Once a second while the C3X is up: "S,<queue drops>,<write drops>" (totals since boot).
//       The CAN RX path only queues (never blocks; dropped if the queue is full).
//  in:  "C,<bus>,<id>,<hex>" frames for the app only (never sent on CAN): bus 1 id 0x7F0 = openpilot state
//       (COMMA_status), bus 2 = the C3X's party bus, bus 3 = its Autopilot side. Anything else, or a line that isn't
//       exactly one frame (a write cut short on the C3X), is dropped. No status for 2 s (C3X off or unplugged): one
//       all-zero COMMA_status clears it, and whole buses go back to BLE.
#define COMMA_ID 0x7F0
typedef struct { uint32_t id; uint8_t bus, dlc, ext, data[8]; } c3x_frame_t;
static QueueHandle_t    s_c3x_q = NULL;
static volatile bool    s_c3x_up;
static volatile uint32_t s_c3x_qdrop, s_c3x_wdrop;

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
    *p++ = 'A'; *p++ = ','; *p++ = '0' + f->bus; *p++ = ',';
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
    int64_t last_c = 0, last_s = 0;
    for (;;) {
        int r;
        while ((r = usb_serial_jtag_read_bytes(b, sizeof(b), 0)) > 0) {
            for (int i = 0; i < r; i++) {
                if (b[i] != '\n') { if (il < (int)sizeof(in) - 1) in[il++] = b[i]; continue; }
                in[il] = 0;
                if (c3x_line(in, il)) { last_c = esp_timer_get_time(); s_c3x_up = true; }
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
        int n = 0, lines = 0;
        if (s_c3x_up && now - last_s >= 1000000) {
            last_s = now;
            n = snprintf(out, sizeof(out), "S,%lu,%lu\n", (unsigned long)s_c3x_qdrop, (unsigned long)s_c3x_wdrop);
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
    s_core += dbc_observe_frame(frame);
    automation_manager_on_frame(frame);
    vehicle_control_on_frame(frame);
    bool tapped = c3x_tap(frame);
    uint8_t route = can_route_classify(frame->bus_id, frame->frame.id, frame->frame.extended);
    if (!route) return;
    if ((route & CAN_ROUTE_BULK) && frame->bus_id < 2 && s_c3x_up) {  // whole bus: to the C3X instead of BLE
        if (!tapped) c3x_queue(frame);
        return;
    }

    // Drop frame if queue is full rather than blocking the driver
    if (enqueue(frame, route)) {
        s_forwarded++;
    } else {
        s_queue_full++;
    }
}

esp_err_t can_manager_init(void)
{
    s_rx_queue = xQueueCreate(RX_QUEUE_DEPTH, sizeof(can_tagged_frame_t));
    s_bulk_queue = xQueueCreate(RX_QUEUE_DEPTH, sizeof(can_tagged_frame_t));
    s_rx_count = xSemaphoreCreateCounting(2 * RX_QUEUE_DEPTH, 0);
    if (!s_rx_queue || !s_bulk_queue || !s_rx_count) {
        return ESP_ERR_NO_MEM;
    }
    usb_serial_jtag_driver_config_t usj = { .tx_buffer_size = 8192, .rx_buffer_size = 4096 };
    s_c3x_q = xQueueCreate(512, sizeof(c3x_frame_t));
    if (!s_c3x_q || usb_serial_jtag_driver_install(&usj) != ESP_OK || xTaskCreate(lanes_task, "lanes", 3072, NULL, 1, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    s_iface_count = 0;
    can_route_init();
    const esp_timer_create_args_t rep_args = { .callback = route_report_cb, .name = "can_route" };
    esp_timer_handle_t rep_timer;
    if (esp_timer_create(&rep_args, &rep_timer) == ESP_OK) {
        esp_timer_start_periodic(rep_timer, ROUTE_REPORT_US);
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
    if (xSemaphoreTake(s_rx_count, pdMS_TO_TICKS(timeout_ms)) == pdTRUE &&
        (xQueueReceive(s_rx_queue, frame, 0) == pdTRUE || xQueueReceive(s_bulk_queue, frame, 0) == pdTRUE)) {
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
    return enqueue(frame, 0) ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t can_manager_inject(const can_tagged_frame_t *frame)
{
    if (!s_rx_queue) return ESP_ERR_INVALID_STATE;
    uint8_t route = can_route_classify(frame->bus_id, frame->frame.id, frame->frame.extended);
    if (!(route & CAN_ROUTE_FORWARD)) {
        return ESP_OK;  // Silently dropped by filter
    }
    return enqueue(frame, route) ? ESP_OK : ESP_ERR_NO_MEM;
}

// Each bus is received in full; these ids get the driver's priority lane, which the rest of the bus cannot crowd
// out: first the messages firmware code reads, then the ids the app asked for, as far as the lane's filters go.
// Called from the BLE task when the app's list changes and from the report timer when the firmware's set grows.
void can_manager_apply_filters(void)
{
    s_bulk_gap_ms = 0;  // a new subscription starts unthinned; the gap only decays while bulk frames flow
    for (int i = 0; i < s_iface_count; i++) {
        can_interface_t *f = s_interfaces[i];
        if (!f->set_ids) continue;
        uint16_t ids[64];   // more than the lane holds: the driver skips duplicates and stops when it is full
        size_t n = dbc_used_ids(f->bus_id, ids, sizeof(ids) / sizeof(*ids) / 2);
        n += can_filter_ids(f->bus_id, ids + n, sizeof(ids) / sizeof(*ids) - n);
        f->set_ids(f, ids, n);
    }
}
