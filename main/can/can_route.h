#pragma once

// Software frame routing.
//
// The CAN controllers receive both buses in full and the firmware reads whatever it needs from the DBC value
// cache. What goes on to the connected app is decided here, one bit lookup per frame: only the ids the app wrote
// to the CAN filter characteristic, any number of them on either bus, and nothing until it has written a list.
//
// Plain C with no RTOS dependencies; unit-tested on the host.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define CAN_ROUTE_BUSES      2
#define CAN_ROUTE_FORWARD    0x02
// With FORWARD: the app gets this frame only through a whole-bus entry ("Log every frame"), not by its id. Such
// frames wait behind the ones the app asked for by id, so a capture or a slow link never delays the dashboard.
#define CAN_ROUTE_BULK       0x04

// In the filter wire format (see can_filter.h) this address subscribes to every
// frame on its bus, extended ids included.
#define CAN_ROUTE_ADDR_ALL   0xFFFFFFFFu

void can_route_init(void);

// Replace the app's forward list. Call begin, then add once per (bus, addr)
// entry, then commit. One writer at a time; readers never block and see either
// the old or the new list.
void can_route_forward_begin(void);
void can_route_forward_add(uint8_t bus, uint32_t addr);
void can_route_forward_commit(void);

// CAN_ROUTE_FORWARD (| CAN_ROUTE_BULK) if the app gets this frame; 0 = it stays in the firmware.
uint8_t can_route_classify(uint8_t bus, uint32_t id, bool extended);
