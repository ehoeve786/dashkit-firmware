#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

// CAN frame filter (the (bus_id, addr) pairs the app subscribed to).
//
// Wire format consumed by can_filter_set():
//   [bus_id : 1 byte]
//   [num_addrs : 1 byte]
//   [addr1 : 4 bytes LE]
//   [addr2 : 4 bytes LE]
//   ...
// repeated for each bus.
//
// The address 0xFFFFFFFF (CAN_ROUTE_ADDR_ALL) matches every frame on its bus.
//
// The list is a subscription: the CAN controllers receive both buses in full, and only the listed ids go on to
// the app. Any standard id can be listed, with no limit from the controllers' 32 hardware filters; without a list the
// app gets every message in dbc/used_messages.txt. The per-frame decision is made by can_route.

esp_err_t can_filter_init(void);

// Replace the current filter with the rules encoded in `data`. Passing
// len == 0 clears the filter (back to the used_messages.txt ids).
esp_err_t can_filter_set(const uint8_t *data, size_t len);

// Copies up to `max` of the standard IDs the app asked for on `bus_id` into `out`; returns how many.
size_t can_filter_ids(uint8_t bus_id, uint16_t *out, size_t max);
