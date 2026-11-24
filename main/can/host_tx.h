#pragma once
// Frames a USB host (the CanPilot app) asks the firmware to transmit on CAN.
//
// The risk in sending on a car's bus is sending an id one of the car's own
// modules also sends: two nodes that start the same id at the same moment
// collide, neither backs off, and the car's module can end up off the bus (on a
// Model Y this silenced the steering column module for a second and latched a
// lane-keeping fault). So nothing is sent blind:
//   - the host has to arm transmitting ("X,1") and repeat that every second;
//     2 s without it and nothing more is sent (the host is gone or closed);
//   - after arming, the bus is only watched for HOST_TX_WATCH_MS, to learn
//     which ids the car sends and how often;
//   - a frame whose id the car sends goes out right after the car's own frame
//     of that id, in the first half of its period, where that module has
//     nothing to send; if no such moment comes it is dropped, not forced;
//   - a frame whose id the car does not send goes out at once.
// Ids the car sends less often than once in HOST_TX_WATCH_MS can be missed
// right after arming; they are known once they were seen.
//
// No ESP-IDF in here: the module is tested on a PC (tools/test/test_host_tx.c).
// One task calls everything except host_tx_saw(), which the CAN receive path calls.
#include <stdbool.h>
#include <stdint.h>
#include "can_interface.h"

#define HOST_TX_ARM_HOLD_MS  2000   // armed this long after the last "X,1"
#define HOST_TX_WATCH_MS     2000   // after arming: watch only
#define HOST_TX_OWNED_MS     10000  // an id seen this recently is one the car sends
#define HOST_TX_EXPIRE_MS    2500   // a frame that found no safe moment for this long is dropped
#define HOST_TX_PENDING      8      // frames waiting for their moment

typedef struct {
    uint32_t sent;      // handed to the CAN driver
    uint32_t refused;   // not armed, still watching, malformed, or the waiting list was full
    uint32_t expired;   // no safe moment came
} host_tx_stats_t;

// One line from the host, without its line end. True if it was a transmit line
// ("X,0", "X,1", "T,<bus>,<id hex>,<data hex>"), whatever became of it.
bool host_tx_line(const char *line, uint32_t now_ms);

// True while the host has transmitting armed.
bool host_tx_armed(uint32_t now_ms);

// The CAN receive path: a standard frame of `id` arrived on `bus`. Cheap, and a no-op while not armed.
void host_tx_saw(uint8_t bus, uint32_t id, uint32_t now_ms);

// One frame that may be sent now, or false. Call until false, every few ms.
bool host_tx_due(uint32_t now_ms, uint8_t *bus, can_frame_t *frame);

// The driver took (true) or refused (false) a frame host_tx_due() gave out.
void host_tx_result(bool sent);

const host_tx_stats_t *host_tx_stats(void);
