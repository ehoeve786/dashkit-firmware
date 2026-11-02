/*
 * Tesla BLE client — high-level vehicle-command facade + poll loop.
 *
 * Orchestrates connect → VCSEC handshake → persistent GET_STATUS poll (the
 * link stays up; matches esphome-tesla-ble). The low-level central GATT
 * transport lives in tesla_ble_adapter; this module owns the protocol session
 * (tesla_session_t), the response correlation, and the app-channel status
 * reporting. Gated by CONFIG_DASHKIT_TESLA_BLE so a feature-off build drops
 * it entirely.
 */

#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Starts the client task (spawns the poll loop). Safe to call once at boot
// when CONFIG_DASHKIT_TESLA_BLE is enabled; with no enrolled key/link it logs
// a canary and waits.
esp_err_t tesla_ble_client_init(void);

// Queue a Tesla Low Power mode change (CarServer setLowPowerModeAction over
// the infotainment domain). Sent by the poll loop on the open link; the car
// must be awake for the infotainment domain to answer. Latest request wins.
esp_err_t tesla_ble_client_set_low_power(bool on);

#ifdef __cplusplus
}
#endif
