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

// Something reads the USB port (the comma 3X or the app): it gets the firmware log and the metrics too.
bool can_manager_usb_listener(void);

// A transmit line ("X,1", "X,0", "T,<bus>,<id>,<hex>", see host_tx.h) that came in over something other than the
// USB port (the BLE transmit characteristic). It is handed to the task that owns host_tx, within 10 ms. False if it
// is not such a line or too many are waiting. Safe from any task.
bool can_manager_host_line(const char *line);

// What a host reads back about transmitting: "H,<armed>,<sent>,<refused>,<expired>" and, per bus that reported
// its error state, "\nD,<bus>,1da50006,<TEC><REC><flags>" (hex bytes, as in can_diag.h). Returns the length.
int can_manager_host_status(char *out, size_t size);

// "L,1" / "L,0" from the Wi-Fi link (wifi_link.c): the app there asks for every frame, as on USB. While it does, the
// lines the USB port's app would get ("A", "D", "H") go to the Wi-Fi link as well. False for anything else.
bool can_manager_wifi_line(const char *line);

// One "M" line of device metrics since the previous call (one caller: the Wi-Fi link, once a second):
// "M,<cpu0 %>,<cpu1 %>,<bus0 load %>,<bus1 load %>,<bus0 fps>,<bus1 fps>,<bus0 FIFO overflows>,<bus1 FIFO overflows>,
//  <capture queue drops>,<USB write drops>,<Wi-Fi drops>" (the last five are totals since boot; load counts no
// stuff bits), no newline: the caller adds its own fields. Returns the length.
int can_manager_metrics(char *out, size_t size);
