#pragma once

// Software frame routing.
//
// The CAN controllers' hardware filters are masks fitted around the wanted ids (or the whole bus while the app logs
// every frame), so more arrives from the driver than was asked for. Two questions are asked per frame, each
// answered by one bit lookup:
//
//   core:    does the firmware itself use this frame? True for every message in the DBC manifest
//            (dbc/used_messages.txt). Only these reach the signal cache, the automations and the control hooks.
//   forward: does the connected app want this frame? Either the list the app wrote to the CAN filter
//            characteristic, or, while the app has not written one, the manifest set.
//
// There is no limit on the number of ids: an app can subscribe to any standard id on either bus without a
// firmware change.
//
// Plain C with no RTOS dependencies; unit-tested on the host.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define CAN_ROUTE_BUSES      2
#define CAN_ROUTE_CORE       0x01
#define CAN_ROUTE_FORWARD    0x02
// With FORWARD: the app gets this frame only through a whole-bus entry ("Log every frame"), not by its id. Such
// frames wait behind the ones the app asked for by id, so a capture or a slow link never delays the dashboard.
#define CAN_ROUTE_BULK       0x04

// In the filter wire format (see can_filter.h) this address subscribes to every
// frame on its bus, extended ids included.
#define CAN_ROUTE_ADDR_ALL   0xFFFFFFFFu

// Build the core set from the generated manifest tables.
void can_route_init(void);

// Replace the app's forward list. Call begin, then add once per (bus, addr)
// entry, then commit(n) with the number of entries; commit(0) returns to the
// default (forward the manifest set). One writer at a time; readers never block
// and see either the old or the new list.
void can_route_forward_begin(void);
void can_route_forward_add(uint8_t bus, uint32_t addr);
void can_route_forward_commit(size_t entries);

// CAN_ROUTE_CORE | CAN_ROUTE_FORWARD bits for a received frame; 0 = drop.
uint8_t can_route_classify(uint8_t bus, uint32_t id, bool extended);

// The manifest ids on `bus` that the firmware decodes or injects (the driver's
// fast lane). Writes at most `max` ids and returns the number written.
size_t can_route_fast_ids(uint8_t bus, uint16_t *out, size_t max);
