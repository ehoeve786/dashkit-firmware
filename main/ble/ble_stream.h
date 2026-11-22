#pragma once

// CAN stream to the connected devices: packet building and the choice of the primary device. Plain C with no
// NimBLE or RTOS dependencies; unit-tested on the host (tools/test/test_ble_stream.c).

#include "can_interface.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BLE_COMPACT_MARK      0x80  // first byte of a compact packet; the original format starts with a count < 0x80

// Does this device get this frame?
typedef bool (*ble_stream_want_fn)(const can_tagged_frame_t *f, void *ctx);

// Packs the wanted frames of [*from, count) until the buffer is full and moves *from past the ones packed (or
// skipped). Returns the packet length, 0 if nothing was packed.
//   original: [count][timestamp_us LE32, bus, addr LE32, len, data...]...   18 bytes per 8-byte frame
//   compact:  [0x80][id | bus << 11 (LE16), len, data]...                  11 bytes, no timestamp
// A compact packet skips frames that don't fit the format (the firmware's diagnostic records); the caller then
// sends only those (rest = true) in the original format.
size_t ble_stream_build(const can_tagged_frame_t *frames, int count, int *from, uint8_t *buf, size_t buf_size,
                        bool compact, bool rest, ble_stream_want_fn want, void *ctx);

// One connected device, as the primary choice sees it.
typedef struct {
    bool     receiver;   // encrypted and subscribed to the CAN stream
    bool     writable;   // not in read-only mode
    uint32_t sub_seq;    // order of its last subscribe (higher = later)
} ble_stream_cand_t;

// The primary device: the writable receiver that subscribed last, which also drives the vehicle; with no writable
// one, the receiver that subscribed last (it gets the stream first, but drives nothing). -1 if none.
int ble_stream_pick_primary(const ble_stream_cand_t *c, int n);
