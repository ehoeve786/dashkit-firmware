#include "can_filter.h"
#include "can_manager.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

#include "can_route.h"

static const char *TAG = "can_filter";

// Maximum number of (bus, addr) entries the filter can hold.
#define CAN_FILTER_MAX_ENTRIES  256

typedef struct {
    uint8_t  bus_id;
    uint32_t addr;
} can_filter_entry_t;

static can_filter_entry_t s_entries[CAN_FILTER_MAX_ENTRIES];
static size_t             s_count = 0;
static SemaphoreHandle_t  s_lock = NULL;

esp_err_t can_filter_init(void)
{
    if (s_lock) return ESP_OK;
    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) return ESP_ERR_NO_MEM;
    s_count = 0;
    return ESP_OK;
}

esp_err_t can_filter_set(const uint8_t *data, size_t len)
{
    if (!s_lock) return ESP_ERR_INVALID_STATE;

    // Parse directly into s_entries under the lock to avoid a large
    // stack-allocated scratch buffer (the BLE host task that calls this
    // has a relatively small stack). s_count is only advanced on success,
    // so readers see either the previous or the new table.
    xSemaphoreTake(s_lock, portMAX_DELAY);

    size_t new_count = 0;
    size_t offset = 0;
    while (offset < len) {
        if (offset + 2 > len) {
            xSemaphoreGive(s_lock);
            ESP_LOGW(TAG, "Truncated header at offset %u", (unsigned)offset);
            return ESP_ERR_INVALID_ARG;
        }
        uint8_t bus_id   = data[offset++];
        uint8_t num_addr = data[offset++];

        if (offset + (size_t)num_addr * 4 > len) {
            xSemaphoreGive(s_lock);
            ESP_LOGW(TAG, "Truncated address list for bus %u", bus_id);
            return ESP_ERR_INVALID_ARG;
        }

        for (uint8_t i = 0; i < num_addr; i++) {
            if (new_count >= CAN_FILTER_MAX_ENTRIES) {
                xSemaphoreGive(s_lock);
                ESP_LOGW(TAG, "Filter table overflow (%d entries max)",
                         CAN_FILTER_MAX_ENTRIES);
                return ESP_ERR_NO_MEM;
            }
            uint32_t addr = ((uint32_t)data[offset + 0]      ) |
                            ((uint32_t)data[offset + 1] <<  8) |
                            ((uint32_t)data[offset + 2] << 16) |
                            ((uint32_t)data[offset + 3] << 24);
            offset += 4;

            s_entries[new_count].bus_id = bus_id;
            s_entries[new_count].addr   = addr;
            new_count++;
        }
    }

    s_count = new_count;
    // The per-frame decision is made by can_route (lock-free bit lookups), so mirror the accepted list into it.
    // An empty list returns to the default of forwarding the manifest ids.
    can_route_forward_begin();
    for (size_t i = 0; i < new_count; i++) {
        can_route_forward_add(s_entries[i].bus_id, s_entries[i].addr);
    }
    can_route_forward_commit(new_count);
    xSemaphoreGive(s_lock);
    can_manager_apply_filters();

    ESP_LOGI(TAG, "Filter updated from %u-byte write: %u entries%s",
             (unsigned)len, (unsigned)new_count,
             new_count == 0 ? " (pass-through)" : "");
    for (size_t i = 0; i < new_count; i++) {
        // Safe to read s_entries here without the lock: only this task
        // (the BLE host task) ever writes to the table.
        ESP_LOGI(TAG, "  [%u] bus=%u addr=0x%03lX",
                 (unsigned)i, s_entries[i].bus_id,
                 (unsigned long)s_entries[i].addr);
    }
    return ESP_OK;
}

size_t can_filter_ids(uint8_t bus_id, uint16_t *out, size_t max)
{
    size_t k = 0;
    if (!s_lock) return 0;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (size_t i = 0; i < s_count && k < max; i++)
        if (s_entries[i].bus_id == bus_id && (s_entries[i].addr <= 0x7FF || s_entries[i].addr == CAN_ROUTE_ADDR_ALL))
            out[k++] = (uint16_t)s_entries[i].addr;  // ALL arrives as 0xFFFF
    xSemaphoreGive(s_lock);
    return k;
}
