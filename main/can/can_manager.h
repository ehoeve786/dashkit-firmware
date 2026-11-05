#pragma once

#include "can_interface.h"
#include "esp_err.h"
#include "sdkconfig.h"

#define CAN_MANAGER_MAX_INTERFACES  4

// Initialize the CAN manager (creates the shared frame queue)
esp_err_t can_manager_init(void);

#ifdef CONFIG_DASHKIT_CAN_ACCEPT_ALL
// Persist hotspot credentials (NVS) and (re)connect; empty ssid = Wi-Fi off.
esp_err_t can_discovery_set_wifi(const char *ssid, const char *pass);
// Persist discovery mode (0 off, 1 USB + Wi-Fi, 2 USB only); reboots if it changed.
esp_err_t can_discovery_set_mode(uint8_t mode);
// Discovery mode active this boot (valid after can_manager_init()).
bool can_discovery_active(void);
#endif

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

// Transmit a frame on the interface with the given bus_id.
// Returns ESP_ERR_NOT_FOUND if no interface matches bus_id.
esp_err_t can_manager_send(uint8_t bus_id, const can_frame_t *frame);
