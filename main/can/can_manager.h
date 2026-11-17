#pragma once

#include "can_interface.h"
#include "esp_err.h"

#define CAN_MANAGER_MAX_INTERFACES  4

// Initialize the CAN manager (creates the shared frame queue)
esp_err_t can_manager_init(void);

// Gives every bus's priority lane to the messages the firmware reads plus the app's current filter.
void can_manager_apply_filters(void);

// Current thinning of whole-bus frames: at most one frame per id in this many ms (0 = none). See bulk_due().
uint16_t can_manager_bulk_gap_ms(void);

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
