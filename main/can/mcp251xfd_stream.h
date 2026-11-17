#pragma once

// MCP251xFD receive/transmit core.
//
// The chip has 32 hardware filters and 2 KB of message RAM. This core receives the whole bus with them, so
// neither limits which CAN messages the firmware or the app can use:
//
//   * Frames somebody is known to use (the "fast lane": messages firmware code reads, then the ids the app asked
//     for) get exact filters into their own FIFO, which is always drained first. A flood on the bulk lanes can
//     only overflow the bulk lanes, never the fast lane.
//   * Four more filters spread every other frame over four bulk FIFOs. What is forwarded to the app is decided
//     in software afterwards (can_route).
//   * Frames are read in chunks: one status read, one RAM read for up to MCPFD_CHUNK messages, one pointer
//     increment per message.
//
// The code here is plain C with no FreeRTOS or ESP-IDF driver dependencies:
// SPI goes through one full-duplex transfer callback and locking is left to the
// caller, so the exact same logic runs against a chip model in the host tests
// (tools/test/test_can_stream.c).

#include "can_interface.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// FIFO layout. Sizes are in messages of 16 bytes (8 header + 8 payload), and
// together they use the chip's 2048 bytes of RAM exactly:
//   20*16 (fast) + 4*16 (TX) + 4*26*16 (bulk) = 2048
// Their order in RAM follows the FIFO numbers.
#define MCPFD_FIFO_FAST       1      // RX, fast lane
#define MCPFD_FIFO_TX         2      // TX
#define MCPFD_FIFO_BULK0      3      // RX, bulk lanes: FIFO 3..6
#define MCPFD_BULK_LANES      4
#define MCPFD_FAST_DEPTH      20
#define MCPFD_TX_DEPTH        4
#define MCPFD_BULK_DEPTH      26
#define MCPFD_OBJ_SIZE        16
#define MCPFD_RAM_SIZE        2048

#define MCPFD_LANES           (1 + MCPFD_BULK_LANES)   // lane 0 = fast
#define MCPFD_LANE_FAST       0

// Messages moved per SPI read. Bounds how long one step holds the SPI lock, so
// a pending transmit waits for at most one chunk.
#define MCPFD_CHUNK           8

// Hardware filters: the first ones (28 at most) are exact matches into the fast lane, the ones after them feed the
// bulk lanes. The chip stores a frame through its lowest-numbered matching filter.
#define MCPFD_MAX_FAST_IDS    28
#define MCPFD_FILTERS         32

// Scratch buffer sizes the caller must provide.
#define MCPFD_REG_BUF_SIZE    24                                   // 2 + 16, padded
#define MCPFD_BLK_BUF_SIZE    (4 + MCPFD_CHUNK * MCPFD_OBJ_SIZE)   // 2 + chunk, padded

// One full-duplex SPI transfer of `len` bytes with chip select held for the
// whole transfer. `rx` may be NULL when the reply is not needed.
typedef esp_err_t (*mcpfd_xfer_fn)(void *io, const uint8_t *tx, uint8_t *rx, size_t len);

typedef struct {
    uint8_t  fifo;        // chip FIFO number
    uint8_t  depth;       // messages
    uint16_t base;        // RAM offset of message 0
    int16_t  expect_tail; // tail index we expect on the next poll, or -1
} mcpfd_lane_t;

typedef struct {
    uint32_t rx_frames[MCPFD_LANES];     // frames read per lane
    uint32_t rx_overflow[MCPFD_LANES];   // overflow events per lane (>= 1 lost frame each)
    uint32_t rx_fast_in_bulk;            // fast-lane ids that arrived through a bulk lane
    uint32_t rx_head_glitch;             // not-empty FIFO reporting head == tail without "full"
    uint32_t rx_layout_fallback;         // chunk read refused: FIFO address outside our layout
    uint32_t rx_desync;                  // tail did not move as far as we incremented it
    uint32_t tx_frames;                  // frames handed to the chip
    uint32_t tx_full;                    // sends refused: TX FIFO full
    uint32_t spi_errors;                 // failed SPI transfers
} mcpfd_stats_t;

typedef struct {
    mcpfd_xfer_fn xfer;
    void         *io;
    // Scratch buffers (DMA-capable on the target). reg_* are used for register
    // access and transmit, blk_* for chunked message reads. All of them are only
    // touched while the caller holds the device lock.
    uint8_t      *reg_tx, *reg_rx;       // MCPFD_REG_BUF_SIZE each
    uint8_t      *blk_tx, *blk_rx;       // MCPFD_BLK_BUF_SIZE each

    mcpfd_lane_t  lane[MCPFD_LANES];
    uint16_t      tx_base;
    uint8_t       next_bulk;             // round-robin cursor over the bulk lanes
    uint32_t      fast_ids[64];          // bitmap of 11-bit ids routed to the fast lane
    mcpfd_stats_t stats;
} mcpfd_stream_t;

// Hooks for mcpfd_stream_step. lock/unlock bracket every group of SPI transfers
// (one chunk at most); deliver runs with the lock released.
typedef struct {
    void (*lock)(void *ctx);
    void (*unlock)(void *ctx);
    void (*deliver)(void *ctx, const can_frame_t *frame, bool fast_lane);
    void *ctx;
} mcpfd_hooks_t;

// Fill in the lane table. xfer/io and the four scratch buffers must be set.
void mcpfd_stream_init(mcpfd_stream_t *s);

// Configuration mode only: size the FIFOs and enable their not-empty interrupt.
esp_err_t mcpfd_stream_config_fifos(mcpfd_stream_t *s);

// Configuration mode only: disable every filter, then route all frames into the
// fast FIFO (used for the loopback self-test, which reads FIFO 1).
esp_err_t mcpfd_stream_filters_accept_all_fast(mcpfd_stream_t *s);

// Program the real filters. Allowed in normal mode too: every filter is disabled before it is rewritten, at the
// cost of the frames that arrive during the register writes (and a frame already waiting in a bulk lane may be
// delivered after a newer one of the same id from the fast lane, once). `fast_ids` are 11-bit standard ids, each an exact
// filter into the fast lane (duplicates and anything beyond MCPFD_MAX_FAST_IDS are skipped). Every other frame on
// the bus goes to the bulk lanes.
esp_err_t mcpfd_stream_filters_program(mcpfd_stream_t *s, const uint16_t *fast_ids, size_t count);

// Drop everything waiting in the RX FIFOs.
esp_err_t mcpfd_stream_reset_rx(mcpfd_stream_t *s);

// One service step: check which lanes hold frames, read one chunk from the fast
// lane if it has any, otherwise one chunk from the next bulk lane, and deliver
// it. Returns the number of frames delivered; 0 means every lane was empty.
// Call it in a loop until it returns 0.
int mcpfd_stream_step(mcpfd_stream_t *s, const mcpfd_hooks_t *hooks);

// Queue one classic CAN frame (dlc <= 8) for transmission. The caller holds the
// device lock. ESP_ERR_NO_MEM: the TX FIFO is full, nothing was written.
esp_err_t mcpfd_stream_send(mcpfd_stream_t *s, const can_frame_t *frame);

// Decode a 16-byte RX message object.
void mcpfd_stream_parse(const uint8_t *obj, can_frame_t *out);
