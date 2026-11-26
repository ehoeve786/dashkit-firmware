#include "firmware_update.h"
#include "ble_ota.h"
#include "esp_ota_ops.h"

const char *firmware_update(unsigned long size, const char *first, int first_size, firmware_read_t read, void *context)
{
    static char chunk[1024];
    esp_ota_handle_t handle = 0;
    const esp_partition_t *slot = esp_ota_get_next_update_partition(NULL);
    if (ble_ota_is_in_progress()) return "a Bluetooth update is running";
    if (!slot || size == 0 || size > slot->size) return "no room for that image";
    if (esp_ota_begin(slot, size, &handle) != ESP_OK) return "can't start the update";
    const char *why = NULL;
    unsigned long got = first_size > 0 ? first_size : 0;
    if (got > size || (got && esp_ota_write(handle, first, got) != ESP_OK)) why = "write failed";
    while (!why && got < size) {
        int r = read(context, chunk, size - got < sizeof(chunk) ? (int)(size - got) : (int)sizeof(chunk));
        if (r <= 0) why = "the upload stopped";
        else if (esp_ota_write(handle, chunk, r) != ESP_OK) why = "write failed";
        else got += r;
    }
    if (why) { esp_ota_abort(handle); return why; }
    if (esp_ota_end(handle) != ESP_OK) return "the image isn't valid";
    if (esp_ota_set_boot_partition(slot) != ESP_OK) return "can't boot it";
    return NULL;
}
