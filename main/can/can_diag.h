#pragma once

// Diagnostic records for the app's CAN log.
//
// While the app has diagnostics switched on (control opcode VC_CMD_DIAG_LOG),
// the firmware adds small records to the CAN stream it already sends over BLE:
// the receive statistics of each bus every 5 s, and the outcome of every door
// unlatch. They look like CAN frames so the existing packet format carries
// them, but they can never be mistaken for one: the bus byte has
// CAN_DIAG_BUS_FLAG set and the id is outside the 11-bit range. They bypass the
// CAN filter and are never transmitted on a CAN bus.
//
// Every record is 8 bytes, little-endian, counters saturating.

#include <stdbool.h>
#include <stdint.h>

#define CAN_DIAG_BUS_FLAG     0x10           // bus byte = flag | CAN bus (0 if not per bus)

// [0] record format (1)  [1] 1 = fast lane + bulk lanes (this driver)  [2] fast-lane FIFO depth
// [3] bulk FIFO depth * lanes / 4  [4..7] uptime, seconds
#define CAN_DIAG_ID_HELLO     0x1DA50000u
// Per bus, last 5 s unless noted:
// [0..1] fast-lane frames/s  [2..3] bulk frames/s
// [4..5] fast-lane overflow events since boot  [6..7] bulk overflow events since boot
#define CAN_DIAG_ID_STREAM_RX 0x1DA50001u
// [0..1] frames transmitted since boot  [2..3] sends refused, TX FIFO full, since boot
// [4..5] longest service step, us  [6..7] longest gap between steps while busy, us
#define CAN_DIAG_ID_STREAM_TX 0x1DA50002u
// Since boot: [0..1] stale head index  [2..3] layout fallbacks  [4..5] desyncs  [6..7] SPI errors
#define CAN_DIAG_ID_STREAM_ERR 0x1DA50003u
// Since boot: [0..1] fast-lane ids that arrived through a bulk lane
#define CAN_DIAG_ID_STREAM_MISROUTE 0x1DA50005u
// Both buses, last 5 s unless noted: [0..1] frames/s seen
// [2..3] frames/s used by the firmware  [4..5] frames/s queued for the app
// [6..7] frames dropped because the app queue was full, since boot
#define CAN_DIAG_ID_ROUTE     0x1DA50010u
// One per door unlatch command: [0] door 0..3, bit 7 set = refused by the
// gear/speed gate (the rest is 0)  [1] frames sent  [2] sends refused, TX FIFO
// full  [3] sends phase-locked to the car's own frame
// [4..5] average delay car frame -> ours, us  [6..7] maximum delay, us
#define CAN_DIAG_ID_UNLATCH   0x1DA50020u
// Bluetooth link to the app, every 5 s: [0..1] ATT MTU  [2..3] link-layer packet size (27 = not extended)
// [4] TX PHY, [5] RX PHY (1 = 1M, 2 = 2M)  [6..7] connection interval, 1.25 ms units
#define CAN_DIAG_ID_LINK      0x1DA50030u
// CAN packets to the app, last 5 s: [0..1] packets/s  [2..3] bytes/s / 16  [4..5] waits for a busy link
// [6..7] packets dropped since boot (link still busy after the wait, or another error)
#define CAN_DIAG_ID_LINK_TX   0x1DA50031u
// [0..1] keepalive pings received from the app since boot
// [2..3] thinning of whole-bus frames right now: one frame per id per this many ms (0 = none)
// [4..5] connection attempts from the DashKit to the car's own Bluetooth (vehicle key) since boot
// [6..7] that link's state: 0 idle, 1 connecting, 2 discovering, 3 subscribing, 4 ready, 5 disconnecting
#define CAN_DIAG_ID_LINK_RX   0x1DA50032u

static inline uint16_t can_diag_sat16(uint32_t v) { return v > 0xFFFFu ? 0xFFFFu : (uint16_t)v; }
static inline uint8_t  can_diag_sat8(uint32_t v)  { return v > 0xFFu ? 0xFFu : (uint8_t)v; }

static inline void can_diag_put16(uint8_t *p, uint32_t v)
{
    uint16_t s = can_diag_sat16(v);
    p[0] = (uint8_t)s;
    p[1] = (uint8_t)(s >> 8);
}

static inline void can_diag_put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

// Four saturating 16-bit fields.
static inline void can_diag_pack4(uint8_t out[8], uint32_t a, uint32_t b, uint32_t c, uint32_t d)
{
    can_diag_put16(out + 0, a);
    can_diag_put16(out + 2, b);
    can_diag_put16(out + 4, c);
    can_diag_put16(out + 6, d);
}

static inline void can_diag_pack_unlatch(uint8_t out[8], uint8_t door, bool refused_by_gate,
                                         uint32_t sent, uint32_t tx_full, uint32_t locked,
                                         uint32_t avg_us, uint32_t max_us)
{
    out[0] = (uint8_t)((door & 0x7F) | (refused_by_gate ? 0x80 : 0));
    out[1] = can_diag_sat8(sent);
    out[2] = can_diag_sat8(tx_full);
    out[3] = can_diag_sat8(locked);
    can_diag_put16(out + 4, avg_us);
    can_diag_put16(out + 6, max_us);
}

// Switch the records on or off. Switching on sends a HELLO record.
void can_diag_set_enabled(bool enabled);
bool can_diag_enabled(void);

// Queue one record for the app (no-op while disabled). `bus` is the CAN bus the
// record is about, 0 if it is not per bus. Safe from any task.
void can_diag_emit(uint8_t bus, uint32_t id, const uint8_t data[8]);
