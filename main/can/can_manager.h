#pragma once
#include <stddef.h>

#include "can_interface.h"
#include "esp_err.h"

#define CAN_MANAGER_MAX_INTERFACES  4

// Initialize the CAN manager (creates the shared frame queue)
esp_err_t can_manager_init(void);

// Register a CAN interface with the manager. Takes ownership of lifecycle.
esp_err_t can_manager_add_interface(can_interface_t *iface);

// Initialize and start all registered interfaces
esp_err_t can_manager_start(void);

// Stop all interfaces
esp_err_t can_manager_stop(void);

// Get the next received frame (blocks up to timeout_ms). Returns ESP_ERR_TIMEOUT if none.
esp_err_t can_manager_receive(can_tagged_frame_t *frame, uint32_t timeout_ms);

// Inject a frame into the RX queue (for simulator use). Timestamps automatically.
esp_err_t can_manager_inject(const can_tagged_frame_t *frame);

// Queue a frame for the app regardless of the CAN filter (diagnostic records,
// see can_diag.h). Timestamps automatically.
esp_err_t can_manager_inject_unfiltered(const can_tagged_frame_t *frame);

// Transmit a frame on the interface with the given bus_id.
// Returns ESP_ERR_NOT_FOUND if no interface matches bus_id.
esp_err_t can_manager_send(uint8_t bus_id, const can_frame_t *frame);

// A transmit line ("X,1", "X,0", "T,<bus>,<id>,<hex>", see host_tx.h) that came in over something other than the
// USB port (the BLE transmit characteristic). It is handed to the task that owns host_tx, within 10 ms. False if it
// is not such a line or too many are waiting. Safe from any task.
bool can_manager_host_line(const char *line);

// What a host reads back about transmitting: "H,<armed>,<sent>,<refused>,<expired>" and, per bus that reported
// its error state, "\nD,<bus>,1da50006,<TEC><REC><flags>" (hex bytes, as in can_diag.h). Returns the length.
int can_manager_host_status(char *out, size_t size);
