#include "can_manager.h"
#include "can_filter.h"
#include "can_diag.h"
#include "can_route.h"
#include "automation_manager.h"
#include "vehicle_control.h"
#include "dbc.h"
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

// The driver delivers more than was asked for (masked filters, or the whole bus), so the software route decides
// per frame (one bit lookup each) whether the firmware uses it and whether the app gets it. Anything else is
// dropped here, before any work.
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

// Each bus receives the firmware's own IDs + the ones the app asked for (no 32-ID limit: the driver merges them
// into masks), or all of it once the app asks for the whole bus. Called from the BLE task whenever the app's
// filter changes.
void can_manager_apply_filters(void)
{
    static uint16_t ids[320];
    s_bulk_gap_ms = 0;  // a new subscription starts unthinned; the gap only decays while bulk frames flow
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
