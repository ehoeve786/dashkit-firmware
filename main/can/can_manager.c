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
