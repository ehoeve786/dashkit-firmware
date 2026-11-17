// Host unit tests for the CAN path:
//   main/can/mcp251xfd_stream.c  chunked RX / TX core
//   main/can/can_route.c         software routing
//
// The driver core talks to a small MCP2518FD model over the same full-duplex
// SPI callback the firmware uses: registers, hardware filters, FIFO RAM, head /
// tail pointers, overflow and the transmit FIFO. Run with
// tools/test/run_can_stream_test.sh (plain gcc, no ESP-IDF).

#include "mcp251xfd_stream.h"
#include "can_route.h"
#include "can_diag.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures = 0;
#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);        \
            failures++;                                                   \
        }                                                                 \
    } while (0)

// ---------------------------------------------------------------------------
// MCP2518FD model
// ---------------------------------------------------------------------------
#define NFIFO        32
#define REG_RXIF     0x020
#define REG_FIFOCON(n)  (0x05C + ((n) - 1) * 12)
#define REG_FIFOSTA(n)  (0x060 + ((n) - 1) * 12)
#define REG_FIFOUA(n)   (0x064 + ((n) - 1) * 12)
#define REG_FLTCON(n)   (0x1D0 + (n) * 4)
#define REG_FLTOBJ(n)   (0x1F0 + (n) * 8)
#define REG_MASK(n)     (0x1F4 + (n) * 8)
#define RAM          0x400

typedef struct {
    uint8_t reg[0x400];
    uint8_t ram[2048];
    int head[NFIFO], tail[NFIFO], count[NFIFO];
    bool ovf[NFIFO];

    // Frames "sent on the bus" by the TX FIFO.
    uint8_t txlog[64][16];
    int tx_count;
    bool hold_tx;            // TXREQ does not drain the TX FIFO (bus busy)

    // Fault injection.
    int base_shift;          // RAM layout differs from what the driver expects
    int glitch_head;         // next N FIFOSTA reads of a non-empty RX FIFO report head == tail
    int fail_xfer;           // fail the Nth transfer from now (1 = next), 0 = never

    // Bookkeeping checked by the tests.
    int uinc_on_empty;       // driver released a message that was not there
    int xfers;               // transfers since the test last cleared it
    int max_xfer_len;
} chip_t;

static uint32_t reg32(const chip_t *c, int addr)
{
    return (uint32_t)c->reg[addr] | ((uint32_t)c->reg[addr + 1] << 8) |
           ((uint32_t)c->reg[addr + 2] << 16) | ((uint32_t)c->reg[addr + 3] << 24);
}
static int fifo_depth(const chip_t *c, int n) { return (int)((reg32(c, REG_FIFOCON(n)) >> 24) & 0x1F) + 1; }
static bool fifo_is_tx(const chip_t *c, int n) { return (reg32(c, REG_FIFOCON(n)) >> 7) & 1; }
static int fifo_base(const chip_t *c, int n)
{
    int base = c->base_shift;
    for (int i = 1; i < n; i++) base += fifo_depth(c, i) * 16;   // RAM follows the FIFO numbers
    return base;
}

// `consume`: this read is the one that uses up a pending head glitch (the SPI
// model reads a register byte by byte, so only one byte may consume it).
static uint32_t fifo_sta(chip_t *c, int n, bool consume)
{
    int depth = fifo_depth(c, n);
    uint32_t sta = 0;
    if (fifo_is_tx(c, n)) {
        if (c->count[n] < depth) sta |= 1u << 0;          // not full
        sta |= (uint32_t)c->tail[n] << 8;
    } else {
        if (c->count[n] > 0) sta |= 1u << 0;              // not empty
        if (c->count[n] >= (depth + 1) / 2) sta |= 1u << 1;
        if (c->count[n] == depth) sta |= 1u << 2;         // full
        if (c->ovf[n]) sta |= 1u << 3;
        int head = c->head[n];
        if (c->glitch_head > 0 && c->count[n] > 0 && c->count[n] < depth) {
            head = c->tail[n];                            // erratum: stale index
            if (consume) c->glitch_head--;
        }
        sta |= (uint32_t)head << 8;
    }
    return sta;
}

static uint32_t fifo_ua(const chip_t *c, int n)
{
    int idx = fifo_is_tx(c, n) ? c->head[n] : c->tail[n];
    return (uint32_t)(fifo_base(c, n) + idx * 16);
}

static uint32_t rxif(const chip_t *c)
{
    uint32_t v = 0;
    for (int n = 1; n < NFIFO; n++) {
        if (!fifo_is_tx(c, n) && c->count[n] > 0 && (reg32(c, REG_FIFOCON(n)) & 1)) v |= 1u << n;
    }
    return v;
}

static uint8_t chip_read(chip_t *c, int addr)
{
    if (addr >= RAM) return (addr - RAM) < 2048 ? c->ram[addr - RAM] : 0;
    if (addr >= REG_RXIF && addr < REG_RXIF + 4) return (uint8_t)(rxif(c) >> ((addr - REG_RXIF) * 8));
    for (int n = 1; n < NFIFO; n++) {
        if (addr >= REG_FIFOSTA(n) && addr < REG_FIFOSTA(n) + 4)
            return (uint8_t)(fifo_sta(c, n, addr == REG_FIFOSTA(n) + 1) >> ((addr - REG_FIFOSTA(n)) * 8));
        if (addr >= REG_FIFOUA(n) && addr < REG_FIFOUA(n) + 4)
            return (uint8_t)(fifo_ua(c, n) >> ((addr - REG_FIFOUA(n)) * 8));
    }
    return c->reg[addr];
}

static void chip_write32(chip_t *c, int addr, uint32_t v)
{
    for (int n = 1; n < NFIFO; n++) {
        if (addr == REG_FIFOCON(n)) {
            // UINC, TXREQ and FRESET act once and read back as 0.
            uint32_t keep = v & ~((1u << 8) | (1u << 9) | (1u << 10));
            for (int i = 0; i < 4; i++) c->reg[addr + i] = (uint8_t)(keep >> (i * 8));
            int depth = fifo_depth(c, n);
            if (v & (1u << 10)) { c->head[n] = c->tail[n] = c->count[n] = 0; c->ovf[n] = false; }
            if (v & (1u << 8)) {
                if (fifo_is_tx(c, n)) {
                    c->head[n] = (c->head[n] + 1) % depth;
                    c->count[n]++;
                } else if (c->count[n] > 0) {
                    c->tail[n] = (c->tail[n] + 1) % depth;
                    c->count[n]--;
                } else {
                    c->uinc_on_empty++;
                }
            }
            if ((v & (1u << 9)) && fifo_is_tx(c, n) && !c->hold_tx) {
                while (c->count[n] > 0) {
                    if (c->tx_count < 64)
                        memcpy(c->txlog[c->tx_count++], &c->ram[fifo_base(c, n) + c->tail[n] * 16], 16);
                    c->tail[n] = (c->tail[n] + 1) % depth;
                    c->count[n]--;
                }
            }
            return;
        }
        if (addr == REG_FIFOSTA(n)) {
            if (!(v & (1u << 3))) c->ovf[n] = false;      // only RXOVIF is clearable
            return;
        }
    }
    for (int i = 0; i < 4; i++) c->reg[addr + i] = (uint8_t)(v >> (i * 8));
}

static esp_err_t chip_xfer(void *io, const uint8_t *tx, uint8_t *rx, size_t len)
{
    chip_t *c = (chip_t *)io;
    c->xfers++;
    if ((int)len > c->max_xfer_len) c->max_xfer_len = (int)len;
    if (c->fail_xfer > 0 && --c->fail_xfer == 0) return ESP_FAIL;

    int op = tx[0] >> 4;
    int addr = ((tx[0] & 0x0F) << 8) | tx[1];
    size_t n = len - 2;
    if (op == 0x3) {                                       // READ
        if (rx) {
            rx[0] = rx[1] = 0;
            for (size_t i = 0; i < n; i++) rx[2 + i] = chip_read(c, addr + (int)i);
        }
    } else if (op == 0x2) {                                // WRITE
        if (addr >= RAM) {
            for (size_t i = 0; i < n; i++) c->ram[addr - RAM + i] = tx[2 + i];
        } else {
            CHECK(n == 4);                                 // the core only writes whole registers
            chip_write32(c, addr, (uint32_t)tx[2] | ((uint32_t)tx[3] << 8) |
                                  ((uint32_t)tx[4] << 16) | ((uint32_t)tx[5] << 24));
        }
    } else {
        CHECK(!"unexpected SPI opcode");
    }
    return ESP_OK;
}

// A frame arriving from the bus: stored through the lowest-numbered matching
// filter, or lost (with the overflow flag) when that FIFO is full. Returns the
// FIFO number, 0 if no filter matched, -1 on overflow.
static int chip_bus_rx(chip_t *c, uint32_t id, const uint8_t *data, int dlc)
{
    for (int f = 0; f < 32; f++) {
        uint8_t con = c->reg[REG_FLTCON(f / 4) + (f % 4)];
        if (!(con & 0x80)) continue;
        uint32_t obj = reg32(c, REG_FLTOBJ(f)) & 0x7FF, mask = reg32(c, REG_MASK(f)) & 0x7FF;
        if ((id & mask) != (obj & mask)) continue;
        int n = con & 0x1F;
        int depth = fifo_depth(c, n);
        if (c->count[n] == depth) { c->ovf[n] = true; return -1; }
        uint8_t *o = &c->ram[fifo_base(c, n) + c->head[n] * 16];
        memset(o, 0, 16);
        o[0] = (uint8_t)id; o[1] = (uint8_t)(id >> 8);
        o[4] = (uint8_t)dlc;
        memcpy(o + 8, data, (size_t)(dlc > 8 ? 8 : dlc));
        c->head[n] = (c->head[n] + 1) % depth;
        c->count[n]++;
        return n;
    }
    return 0;
}

// ---------------------------------------------------------------------------
// Test harness
// ---------------------------------------------------------------------------
static const uint16_t kFast[] = { 0x118, 0x1F9, 0x257, 0x273, 0x2F3, 0x3B3 };
#define NFAST  (sizeof(kFast) / sizeof(kFast[0]))

typedef struct {
    chip_t chip;
    mcpfd_stream_t s;
    uint8_t reg_tx[MCPFD_REG_BUF_SIZE], reg_rx[MCPFD_REG_BUF_SIZE];
    uint8_t blk_tx[MCPFD_BLK_BUF_SIZE], blk_rx[MCPFD_BLK_BUF_SIZE];
    mcpfd_hooks_t hooks;

    int lock_depth;
    int xfers_at_lock, max_xfers_per_lock;
    int tx_every;                 // send a frame on every Nth unlock (0 = never)
    int unlocks;

    // Per-id sequence check: each injected frame carries its per-id counter.
    uint32_t sent_seq[2048], got_seq[2048];
    long delivered, delivered_fast;
    int last_step_fast;           // frames of the last step that came through the fast lane
} rig_t;

static void rig_lock(void *ctx)
{
    rig_t *r = ctx;
    CHECK(r->lock_depth == 0);
    r->lock_depth++;
    r->xfers_at_lock = r->chip.xfers;
}

static void rig_unlock(void *ctx)
{
    rig_t *r = ctx;
    CHECK(r->lock_depth == 1);
    int used = r->chip.xfers - r->xfers_at_lock;
    if (used > r->max_xfers_per_lock) r->max_xfers_per_lock = used;
    r->lock_depth--;
    // A transmit squeezing in between two RX chunks, as the vehicle-control
    // task does on the device.
    if (r->tx_every && (++r->unlocks % r->tx_every) == 0) {
        can_frame_t f = { .id = 0x1F9, .dlc = 8, .data = { 0x60, 0, 0x10 } };
        r->lock_depth++;
        mcpfd_stream_send(&r->s, &f);
        r->lock_depth--;
    }
}

static void rig_deliver(void *ctx, const can_frame_t *f, bool fast)
{
    rig_t *r = ctx;
    CHECK(r->lock_depth == 0);                 // delivery never holds the SPI lock
    CHECK(f->id < 2048 && f->dlc == 8);
    uint32_t seq = (uint32_t)f->data[0] | ((uint32_t)f->data[1] << 8) |
                   ((uint32_t)f->data[2] << 16) | ((uint32_t)f->data[3] << 24);
    CHECK(seq == r->got_seq[f->id]);           // in order, no loss, no duplicate
    CHECK(f->data[7] == (uint8_t)(f->id ^ 0x5A));
    r->got_seq[f->id] = seq + 1;
    r->delivered++;
    if (fast) { r->delivered_fast++; r->last_step_fast++; }
}

static void rig_init(rig_t *r)
{
    memset(r, 0, sizeof(*r));
    r->s.xfer = chip_xfer;
    r->s.io = &r->chip;
    r->s.reg_tx = r->reg_tx; r->s.reg_rx = r->reg_rx;
    r->s.blk_tx = r->blk_tx; r->s.blk_rx = r->blk_rx;
    r->hooks = (mcpfd_hooks_t){ rig_lock, rig_unlock, rig_deliver, r };
    mcpfd_stream_init(&r->s);
    CHECK(mcpfd_stream_config_fifos(&r->s) == ESP_OK);
    CHECK(mcpfd_stream_filters_program(&r->s, kFast, NFAST) == ESP_OK);
    CHECK(mcpfd_stream_reset_rx(&r->s) == ESP_OK);
}

// Put one frame on the bus. Returns the FIFO it went to (-1 = overflow).
static int rig_inject(rig_t *r, uint32_t id)
{
    uint8_t d[8];
    uint32_t seq = r->sent_seq[id];
    d[0] = (uint8_t)seq; d[1] = (uint8_t)(seq >> 8); d[2] = (uint8_t)(seq >> 16); d[3] = (uint8_t)(seq >> 24);
    d[4] = d[5] = d[6] = 0;
    d[7] = (uint8_t)(id ^ 0x5A);
    int fifo = chip_bus_rx(&r->chip, id, d, 8);
    if (fifo > 0) r->sent_seq[id] = seq + 1;
    return fifo;
}

static int rig_step(rig_t *r)
{
    r->last_step_fast = 0;
    return mcpfd_stream_step(&r->s, &r->hooks);
}

static void rig_drain(rig_t *r)
{
    int guard = 100000;
    while (rig_step(r) > 0 && --guard) {}
    CHECK(guard > 0);
}

static void rig_check_all_delivered(rig_t *r)
{
    for (int id = 0; id < 2048; id++) CHECK(r->got_seq[id] == r->sent_seq[id]);
}

static void rig_check_clean(rig_t *r)
{
    const mcpfd_stats_t *st = &r->s.stats;
    CHECK(st->rx_head_glitch == 0 && st->rx_layout_fallback == 0 && st->rx_desync == 0);
    CHECK(st->rx_fast_in_bulk == 0 && st->spi_errors == 0);
    CHECK(r->chip.uinc_on_empty == 0);
}

static uint32_t rnd_state = 12345;
static uint32_t rnd(void) { rnd_state = rnd_state * 1664525u + 1013904223u; return rnd_state >> 8; }

// ---------------------------------------------------------------------------
// Driver core tests
// ---------------------------------------------------------------------------
static void layout_matches_the_chip(void)
{
    static rig_t r;
    rig_init(&r);
    // The core's precomputed RAM offsets are what the chip derives from the
    // FIFO sizes, and together they use the RAM exactly.
    CHECK(fifo_base(&r.chip, MCPFD_FIFO_FAST) == r.s.lane[0].base);
    CHECK(fifo_base(&r.chip, MCPFD_FIFO_TX) == r.s.tx_base);
    for (int i = 0; i < MCPFD_BULK_LANES; i++) {
        CHECK(fifo_base(&r.chip, MCPFD_FIFO_BULK0 + i) == r.s.lane[1 + i].base);
        CHECK(fifo_depth(&r.chip, MCPFD_FIFO_BULK0 + i) == MCPFD_BULK_DEPTH);
    }
    const mcpfd_lane_t *last = &r.s.lane[MCPFD_LANES - 1];
    CHECK(last->base + last->depth * MCPFD_OBJ_SIZE == MCPFD_RAM_SIZE);
    CHECK(fifo_depth(&r.chip, MCPFD_FIFO_FAST) == MCPFD_FAST_DEPTH);
    CHECK(fifo_is_tx(&r.chip, MCPFD_FIFO_TX) && fifo_depth(&r.chip, MCPFD_FIFO_TX) == MCPFD_TX_DEPTH);
    CHECK(MCPFD_BLK_BUF_SIZE >= 2 + MCPFD_CHUNK * MCPFD_OBJ_SIZE && MCPFD_REG_BUF_SIZE >= 2 + MCPFD_OBJ_SIZE);
}

static void filters_route_fast_and_bulk(void)
{
    static rig_t r;
    rig_init(&r);
    for (size_t i = 0; i < NFAST; i++) CHECK(rig_inject(&r, kFast[i]) == MCPFD_FIFO_FAST);
    // Every other id is accepted, by its two low bits.
    for (uint32_t id = 0; id < 2048; id++) {
        bool fast = false;
        for (size_t i = 0; i < NFAST; i++) fast |= (kFast[i] == id);
        if (fast) continue;
        int fifo = rig_inject(&r, id);
        CHECK(fifo == MCPFD_FIFO_BULK0 + (int)(id & 3) || fifo == -1);
        rig_drain(&r);
    }
    rig_check_all_delivered(&r);
    rig_check_clean(&r);
    CHECK(r.delivered_fast == (long)NFAST);
}

static void accept_all_fast_filter_for_the_self_test(void)
{
    static rig_t r;
    rig_init(&r);
    CHECK(mcpfd_stream_filters_accept_all_fast(&r.s) == ESP_OK);
    CHECK(rig_inject(&r, 0x123) == MCPFD_FIFO_FAST);
    CHECK(rig_inject(&r, 0x256) == MCPFD_FIFO_FAST);
    CHECK(rig_inject(&r, 0x7FF) == MCPFD_FIFO_FAST);
}

static void fast_ids_beyond_the_filter_budget_stay_on_bulk(void)
{
    static rig_t r;
    rig_init(&r);
    uint16_t ids[40];
    for (int i = 0; i < 40; i++) ids[i] = (uint16_t)(0x100 + i);
    ids[5] = ids[4];                                          // a duplicate uses no filter
    CHECK(mcpfd_stream_filters_program(&r.s, ids, 40) == ESP_OK);
    CHECK(rig_inject(&r, 0x100) == MCPFD_FIFO_FAST);
    CHECK(rig_inject(&r, 0x100 + MCPFD_MAX_FAST_IDS) == MCPFD_FIFO_FAST);          // the 28th distinct id
    CHECK(rig_inject(&r, 0x100 + MCPFD_MAX_FAST_IDS + 1) == MCPFD_FIFO_BULK0 + ((0x100 + MCPFD_MAX_FAST_IDS + 1) & 3));
    CHECK(rig_inject(&r, 0x100 + 39) == MCPFD_FIFO_BULK0 + ((0x100 + 39) & 3));   // still received
    rig_drain(&r);
    rig_check_all_delivered(&r);
}

static void no_priority_ids_still_receives_the_whole_bus(void)
{
    static rig_t r;
    rig_init(&r);
    CHECK(mcpfd_stream_filters_program(&r.s, NULL, 0) == ESP_OK);
    for (uint32_t id = 0; id < 2048; id += 97) {
        CHECK(rig_inject(&r, id) == MCPFD_FIFO_BULK0 + (int)(id & 3));
        rig_drain(&r);
    }
    rig_check_all_delivered(&r);
}

static void long_random_stream_is_complete_and_ordered(void)
{
    static rig_t r;
    rig_init(&r);
    r.tx_every = 7;      // transmits interleave with the drain throughout
    long injected = 0, lost = 0;
    for (int round = 0; round < 60000; round++) {
        int burst = (int)(rnd() % 6);
        for (int i = 0; i < burst; i++) {
            uint32_t id = (rnd() % 10 == 0) ? kFast[rnd() % NFAST] : (rnd() % 2048);
            int fifo = rig_inject(&r, id);
            if (fifo > 0) injected++; else lost++;
        }
        int steps = (int)(rnd() % 3);
        for (int i = 0; i < steps; i++) {
            int n = rig_step(&r);
            CHECK(n <= MCPFD_CHUNK);
        }
    }
    rig_drain(&r);
    CHECK(injected > 100000);
    CHECK(r.delivered == injected);
    rig_check_all_delivered(&r);
    rig_check_clean(&r);
    CHECK(lost == 0 || r.s.stats.rx_overflow[1] + r.s.stats.rx_overflow[2] +
                       r.s.stats.rx_overflow[3] + r.s.stats.rx_overflow[4] + r.s.stats.rx_overflow[0] > 0);
    // One lock never covers more than: RXIF + status + overflow clear + chunk
    // read + one release per message.
    CHECK(r.max_xfers_per_lock <= 4 + MCPFD_CHUNK);
    CHECK(r.chip.max_xfer_len <= 2 + MCPFD_CHUNK * MCPFD_OBJ_SIZE);
    CHECK(r.chip.tx_count > 0 && r.s.stats.tx_frames > 0);
}

static void fast_lane_is_served_before_a_backlog(void)
{
    static rig_t r;
    rig_init(&r);
    for (int i = 0; i < 60; i++) rig_inject(&r, 0x400 + (uint32_t)i);   // bulk backlog on all lanes
    rig_inject(&r, 0x1F9);
    CHECK(rig_step(&r) == 1);
    CHECK(r.last_step_fast == 1);                                       // the fast frame came first
    // A fast frame arriving mid-backlog is the very next thing delivered.
    CHECK(rig_step(&r) > 0 && r.last_step_fast == 0);
    rig_inject(&r, 0x118);
    CHECK(rig_step(&r) == 1 && r.last_step_fast == 1);
    rig_drain(&r);
    rig_check_all_delivered(&r);
    rig_check_clean(&r);
}

static void bulk_lanes_take_turns(void)
{
    static rig_t r;
    rig_init(&r);
    for (int i = 0; i < 20; i++) for (uint32_t k = 0; k < 4; k++) rig_inject(&r, 0x500 + k);
    // Four steps, four different lanes: no lane waits behind another's backlog.
    uint32_t before[4], seen = 0;
    for (int k = 0; k < 4; k++) before[k] = r.s.stats.rx_frames[1 + k];
    for (int i = 0; i < 4; i++) CHECK(rig_step(&r) == MCPFD_CHUNK);
    for (int k = 0; k < 4; k++) seen += (r.s.stats.rx_frames[1 + k] - before[k]) == MCPFD_CHUNK;
    CHECK(seen == 4);
    rig_drain(&r);
    rig_check_all_delivered(&r);
}

static void full_fifo_and_wraparound(void)
{
    static rig_t r;
    rig_init(&r);
    // Move the ring so the full FIFO wraps, then fill it exactly: head == tail
    // with the full flag set.
    for (int i = 0; i < 13; i++) rig_inject(&r, 0x1F9);
    rig_drain(&r);
    for (int i = 0; i < MCPFD_FAST_DEPTH; i++) CHECK(rig_inject(&r, 0x1F9) == MCPFD_FIFO_FAST);
    CHECK(r.chip.count[MCPFD_FIFO_FAST] == MCPFD_FAST_DEPTH);
    CHECK(rig_step(&r) == MCPFD_FAST_DEPTH - 13);   // tail 13: the run up to the end of the ring
    CHECK(rig_step(&r) == MCPFD_CHUNK);             // then full chunks from index 0
    rig_drain(&r);
    CHECK(r.delivered == 13 + MCPFD_FAST_DEPTH);
    rig_check_all_delivered(&r);
    rig_check_clean(&r);
    CHECK(r.s.stats.rx_overflow[0] == 0);
}

static void overflow_stays_on_the_flooded_lane(void)
{
    static rig_t r;
    rig_init(&r);
    int lost = 0;
    for (int i = 0; i < MCPFD_BULK_DEPTH + 9; i++) lost += rig_inject(&r, 0x402) < 0;   // lane 2 only
    CHECK(lost == 9);
    for (int i = 0; i < 5; i++) CHECK(rig_inject(&r, 0x1F9) == MCPFD_FIFO_FAST);
    rig_drain(&r);
    CHECK(r.s.stats.rx_overflow[1 + 2] == 1);                  // one event, flag cleared again
    CHECK(r.s.stats.rx_overflow[0] == 0 && r.s.stats.rx_overflow[1] == 0);
    CHECK(!r.chip.ovf[MCPFD_FIFO_BULK0 + 2]);
    rig_check_all_delivered(&r);                               // everything that was stored
    CHECK(r.delivered_fast == 5);
}

static void stale_head_index_costs_no_frames(void)
{
    static rig_t r;
    rig_init(&r);
    for (int i = 0; i < 5; i++) rig_inject(&r, 0x1F9);
    r.chip.glitch_head = 2;          // FIFOSTA reports head == tail though 5 frames wait
    rig_drain(&r);
    CHECK(r.s.stats.rx_head_glitch == 2);
    CHECK(r.delivered == 5);
    rig_check_all_delivered(&r);
    CHECK(r.chip.uinc_on_empty == 0);
}

static void unexpected_ram_layout_falls_back_to_single_reads(void)
{
    static rig_t r;
    rig_init(&r);
    r.chip.base_shift = 8;           // chip addresses no longer line up with the core's table
    for (int i = 0; i < 30; i++) rig_inject(&r, 0x1F9 + (uint32_t)(i % 3) * 0x100);
    int guard = 1000, n;
    while ((n = rig_step(&r)) > 0 && --guard) CHECK(n == 1);
    CHECK(r.delivered == 30);
    CHECK(r.s.stats.rx_layout_fallback == 30);
    rig_check_all_delivered(&r);
    CHECK(r.chip.uinc_on_empty == 0);
}

static void spi_error_on_the_chunk_read_loses_nothing(void)
{
    static rig_t r;
    rig_init(&r);
    for (int i = 0; i < 6; i++) rig_inject(&r, 0x1F9);
    r.chip.fail_xfer = 3;            // RXIF, status, then the RAM read fails
    CHECK(rig_step(&r) == 0);
    CHECK(r.s.stats.spi_errors == 1);
    rig_drain(&r);
    CHECK(r.delivered == 6);
    rig_check_all_delivered(&r);
    CHECK(r.chip.uinc_on_empty == 0);
}

static void transmit_encodes_frames(void)
{
    static rig_t r;
    rig_init(&r);
    can_frame_t f = { .id = 0x1F9, .dlc = 8, .data = { 0x60, 0x00, 0x10, 0, 0, 0, 0, 0xAB } };
    CHECK(mcpfd_stream_send(&r.s, &f) == ESP_OK);
    CHECK(r.chip.tx_count == 1);
    const uint8_t *o = r.chip.txlog[0];
    CHECK(o[0] == 0xF9 && o[1] == 0x01 && o[2] == 0 && o[3] == 0);
    CHECK(o[4] == 8 && o[5] == 0);
    CHECK(memcmp(o + 8, f.data, 8) == 0);

    // Short frame: unused payload bytes are zero, not left over from before.
    can_frame_t g = { .id = 0x273, .dlc = 3, .data = { 1, 2, 3, 0xEE, 0xEE } };
    CHECK(mcpfd_stream_send(&r.s, &g) == ESP_OK);
    o = r.chip.txlog[1];
    CHECK(o[0] == 0x73 && o[1] == 0x02 && o[4] == 3);
    CHECK(o[8] == 1 && o[9] == 2 && o[10] == 3 && o[11] == 0 && o[12] == 0);

    // Extended id: SID = id[28:18], EID = id[17:0], IDE set.
    can_frame_t e = { .id = 0x18DAF110, .dlc = 1, .extended = true, .data = { 0x3E } };
    CHECK(mcpfd_stream_send(&r.s, &e) == ESP_OK);
    o = r.chip.txlog[2];
    uint32_t t0 = (uint32_t)o[0] | ((uint32_t)o[1] << 8) | ((uint32_t)o[2] << 16) | ((uint32_t)o[3] << 24);
    CHECK((t0 & 0x7FF) == (0x18DAF110u >> 18) && ((t0 >> 11) & 0x3FFFF) == (0x18DAF110u & 0x3FFFF));
    CHECK(o[4] == (1 | (1 << 4)));
    can_frame_t back;
    mcpfd_stream_parse(o, &back);
    CHECK(back.id == 0x18DAF110 && back.extended && back.dlc == 1 && back.data[0] == 0x3E);

    can_frame_t fd = { .id = 0x100, .dlc = 9 };
    CHECK(mcpfd_stream_send(&r.s, &fd) == ESP_ERR_NOT_SUPPORTED);
    CHECK(r.s.stats.tx_frames == 3 && r.s.stats.tx_full == 0);
}

static void transmit_reports_a_full_fifo(void)
{
    static rig_t r;
    rig_init(&r);
    r.chip.hold_tx = true;           // the bus is busy: nothing leaves the TX FIFO
    can_frame_t f = { .id = 0x1F9, .dlc = 8 };
    for (int i = 0; i < MCPFD_TX_DEPTH; i++) CHECK(mcpfd_stream_send(&r.s, &f) == ESP_OK);
    CHECK(mcpfd_stream_send(&r.s, &f) == ESP_ERR_NO_MEM);
    CHECK(r.s.stats.tx_full == 1 && r.s.stats.tx_frames == MCPFD_TX_DEPTH);
    CHECK(r.chip.count[MCPFD_FIFO_TX] == MCPFD_TX_DEPTH);   // the refused frame wrote nothing
    // Receive is unaffected by a stuck transmitter.
    rig_inject(&r, 0x1F9);
    rig_drain(&r);
    CHECK(r.delivered == 1);
}

static void idle_step_is_one_transfer(void)
{
    static rig_t r;
    rig_init(&r);
    r.chip.xfers = 0;
    CHECK(rig_step(&r) == 0);
    CHECK(r.chip.xfers == 1);        // just the RXIF read
    // A single waiting frame: RXIF, status, read, release.
    rig_inject(&r, 0x1F9);
    r.chip.xfers = 0;
    CHECK(rig_step(&r) == 1);
    CHECK(r.chip.xfers == 4);
    // A full chunk: 3 + one release per frame.
    for (int i = 0; i < MCPFD_CHUNK; i++) rig_inject(&r, 0x400);
    r.chip.xfers = 0;
    CHECK(rig_step(&r) == MCPFD_CHUNK);
    CHECK(r.chip.xfers == 3 + MCPFD_CHUNK);
}

// ---------------------------------------------------------------------------
// Software routing tests
// ---------------------------------------------------------------------------
static void route_forwards_nothing_without_a_list(void)
{
    can_route_init();
    for (int bus = 0; bus < CAN_ROUTE_BUSES; bus++)
        for (uint32_t id = 0; id < 2048; id++) CHECK(can_route_classify((uint8_t)bus, id, false) == 0);
    CHECK(can_route_classify(0, 0x12345, true) == 0);
}

static void route_app_list_is_a_subscription(void)
{
    can_route_init();
    can_route_forward_begin();
    can_route_forward_add(1, 0x7E0);
    can_route_forward_add(0, 0x129);
    can_route_forward_add(7, 0x129);              // unknown bus: ignored
    can_route_forward_add(1, 0x12345678);         // extended id: ignored without the wildcard
    can_route_forward_commit();

    CHECK(can_route_classify(1, 0x7E0, false) == CAN_ROUTE_FORWARD);
    CHECK(can_route_classify(0, 0x7E0, false) == 0);                      // per bus
    CHECK(can_route_classify(0, 0x129, false) == CAN_ROUTE_FORWARD);
    CHECK(can_route_classify(1, 0x129, false) == 0);                      // not listed: stays in the firmware
    CHECK(can_route_classify(1, 0x7E0, true) == 0);                       // an extended frame is not that id
    CHECK(can_route_classify(1, 0x12345678, true) == 0);
    CHECK(can_route_classify(2, 0x129, false) == 0);

    // Wildcard: the whole bus, extended frames included.
    can_route_forward_begin();
    can_route_forward_add(1, CAN_ROUTE_ADDR_ALL);
    can_route_forward_commit();
    // Matched only by the wildcard: forwarded in the bulk queue.
    CHECK(can_route_classify(1, 0x7E0, false) == (CAN_ROUTE_FORWARD | CAN_ROUTE_BULK));
    CHECK(can_route_classify(1, 0x12345678, true) == (CAN_ROUTE_FORWARD | CAN_ROUTE_BULK));
    CHECK(can_route_classify(0, 0x7E0, false) == 0);
    // Listed by id as well as by the wildcard: not bulk.
    can_route_forward_begin();
    can_route_forward_add(1, CAN_ROUTE_ADDR_ALL);
    can_route_forward_add(1, 0x7E0);
    can_route_forward_commit();
    CHECK(can_route_classify(1, 0x7E0, false) == CAN_ROUTE_FORWARD);
    CHECK(can_route_classify(1, 0x129, false) == (CAN_ROUTE_FORWARD | CAN_ROUTE_BULK));

    // Clearing the list (the app disconnected): nothing is forwarded.
    can_route_forward_begin();
    can_route_forward_commit();
    CHECK(can_route_classify(1, 0x7E0, false) == 0);
}

static void diag_record_layout(void)
{
    uint8_t d[8];
    can_diag_pack4(d, 0x1234, 70000, 0, 0xFFFF);            // little-endian, saturating
    const uint8_t want4[8] = { 0x34, 0x12, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF };
    CHECK(memcmp(d, want4, 8) == 0);

    can_diag_pack_unlatch(d, 2, false, 12, 1, 9, 310, 70000);
    const uint8_t wantu[8] = { 2, 12, 1, 9, 0x36, 0x01, 0xFF, 0xFF };
    CHECK(memcmp(d, wantu, 8) == 0);
    can_diag_pack_unlatch(d, 3, true, 300, 0, 0, 0, 0);     // refused by the gate; counts saturate
    CHECK(d[0] == 0x83 && d[1] == 0xFF);

    // Record ids can never be a real (11-bit) CAN id, and the bus byte never a real bus.
    CHECK(CAN_DIAG_ID_HELLO > 0x7FF && CAN_DIAG_ID_UNLATCH > 0x7FF);
    CHECK((CAN_DIAG_BUS_FLAG | 1) >= CAN_ROUTE_BUSES);
}

int main(void)
{
    diag_record_layout();
    layout_matches_the_chip();
    filters_route_fast_and_bulk();
    accept_all_fast_filter_for_the_self_test();
    fast_ids_beyond_the_filter_budget_stay_on_bulk();
    no_priority_ids_still_receives_the_whole_bus();
    long_random_stream_is_complete_and_ordered();
    fast_lane_is_served_before_a_backlog();
    bulk_lanes_take_turns();
    full_fifo_and_wraparound();
    overflow_stays_on_the_flooded_lane();
    stale_head_index_costs_no_frames();
    unexpected_ram_layout_falls_back_to_single_reads();
    spi_error_on_the_chunk_read_loses_nothing();
    transmit_encodes_frames();
    transmit_reports_a_full_fifo();
    idle_step_is_one_transfer();
    route_forwards_nothing_without_a_list();
    route_app_list_is_a_subscription();

    if (failures) {
        printf("%d check(s) FAILED\n", failures);
        return 1;
    }
    printf("ALL TESTS PASSED\n");
    return 0;
}
