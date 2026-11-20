#include "can_diag.h"
#include "can_manager.h"
#include "mcp251xfd_stream.h"

#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "can_diag";

static volatile bool s_enabled;

bool can_diag_enabled(void)
{
    return s_enabled;
}

void can_diag_emit(uint8_t bus, uint32_t id, const uint8_t data[8])
{
    if (!s_enabled) return;
    can_tagged_frame_t f;
    memset(&f, 0, sizeof(f));
    f.bus_id = (uint8_t)(CAN_DIAG_BUS_FLAG | (bus & 0x0F));
    f.frame.id = id;
    f.frame.extended = true;
    f.frame.dlc = 8;
    memcpy(f.frame.data, data, 8);
    can_manager_inject_unfiltered(&f);
}

void can_diag_set_enabled(bool enabled)
{
    if (enabled != s_enabled) {
        ESP_LOGI(TAG, "diagnostic records %s", enabled ? "on" : "off");
    }
    s_enabled = enabled;
    if (!enabled) return;

    // Also when it was already on: the app asks again after every connect.
    uint8_t d[8] = {0};
    d[0] = CAN_DIAG_FORMAT;
    d[1] = 1;
    d[2] = MCPFD_LANES;
    d[3] = MCPFD_LANE_DEPTH;
    can_diag_put32(d + 4, (uint32_t)(esp_timer_get_time() / 1000000));
    can_diag_emit(0, CAN_DIAG_ID_HELLO, d);
}
