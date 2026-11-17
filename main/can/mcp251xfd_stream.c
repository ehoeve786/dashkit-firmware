#include "mcp251xfd_stream.h"

#include <string.h>

// ---------------------------------------------------------------------------
// Chip registers used here (see the MCP2518FD data sheet, DS20006027)
// ---------------------------------------------------------------------------
#define REG_C1RXIF          0x020   // 0x024 is C1TXIF
#define REG_C1FIFOCON(n)    (0x05C + ((n) - 1) * 12)
#define REG_C1FIFOSTA(n)    (0x060 + ((n) - 1) * 12)
#define REG_C1FLTCON(n)     (0x1D0 + (n) * 4)
#define REG_C1FLTOBJ(n)     (0x1F0 + (n) * 8)
#define REG_C1MASK(n)       (0x1F4 + (n) * 8)
#define RAM_BASE            0x400

#define SPI_CMD_READ(addr)  (0x3000 | ((addr) & 0xFFF))
#define SPI_CMD_WRITE(addr) (0x2000 | ((addr) & 0xFFF))

#define FIFOCON_TFNRFNIE    (1u << 0)    // RX: not-empty interrupt enable
#define FIFOCON_TXEN        (1u << 7)
#define FIFOCON_UINC        (1u << 8)
#define FIFOCON_TXREQ       (1u << 9)
#define FIFOCON_FRESET      (1u << 10)
#define FIFOCON_FSIZE_SHIFT 24

#define FIFOSTA_TFNRFNIF    (1u << 0)    // RX: not empty / TX: not full
#define FIFOSTA_TFERFFIF    (1u << 2)    // RX: full
#define FIFOSTA_RXOVIF      (1u << 3)    // RX: a frame was dropped (sticky)
#define FIFOSTA_FIFOCI(sta) (((sta) >> 8) & 0x1F)   // RX: index the next frame is stored at

// Filter control byte: enabled, storing into FIFO n.
#define FLT_ENABLE(fifo)    (0x80u | ((fifo) & 0x1Fu))
#define MASK_SID_ALL        0x7FFu
#define MASK_SID_LOW2       0x003u

// FIFOCON values we configure, so run-time writes need no read-modify-write.
#define RX_CON(depth)   ((((uint32_t)(depth) - 1) << FIFOCON_FSIZE_SHIFT) | FIFOCON_TFNRFNIE)
#define TX_CON          ((((uint32_t)MCPFD_TX_DEPTH - 1) << FIFOCON_FSIZE_SHIFT) | FIFOCON_TXEN)

// ---------------------------------------------------------------------------
// SPI helpers (caller holds the device lock)
// ---------------------------------------------------------------------------
static inline uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline void put_le32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

static esp_err_t xfer(mcpfd_stream_t *s, const uint8_t *tx, uint8_t *rx, size_t len)
{
    esp_err_t err = s->xfer(s->io, tx, rx, len);
    if (err != ESP_OK) s->stats.spi_errors++;
    return err;
}

// Read `len` bytes (<= 16) at `addr`; the data is left at s->reg_rx + 2.
static esp_err_t reg_read(mcpfd_stream_t *s, uint16_t addr, size_t len)
{
    uint16_t cmd = SPI_CMD_READ(addr);
    memset(s->reg_tx, 0, 2 + len);
    s->reg_tx[0] = (uint8_t)(cmd >> 8);
    s->reg_tx[1] = (uint8_t)cmd;
    return xfer(s, s->reg_tx, s->reg_rx, 2 + len);
}

static esp_err_t reg_read32(mcpfd_stream_t *s, uint16_t addr, uint32_t *val)
{
    esp_err_t err = reg_read(s, addr, 4);
    if (err == ESP_OK) *val = le32(s->reg_rx + 2);
    return err;
}

static esp_err_t reg_write32(mcpfd_stream_t *s, uint16_t addr, uint32_t val)
{
    uint16_t cmd = SPI_CMD_WRITE(addr);
    s->reg_tx[0] = (uint8_t)(cmd >> 8);
    s->reg_tx[1] = (uint8_t)cmd;
    put_le32(s->reg_tx + 2, val);
    return xfer(s, s->reg_tx, NULL, 6);
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void mcpfd_stream_init(mcpfd_stream_t *s)
{
    uint16_t base = 0;

    s->lane[MCPFD_LANE_FAST] = (mcpfd_lane_t){ MCPFD_FIFO_FAST, MCPFD_FAST_DEPTH, base, -1 };
    base += MCPFD_FAST_DEPTH * MCPFD_OBJ_SIZE;

    s->tx_base = base;
    base += MCPFD_TX_DEPTH * MCPFD_OBJ_SIZE;

    for (int i = 0; i < MCPFD_BULK_LANES; i++) {
        s->lane[1 + i] = (mcpfd_lane_t){ (uint8_t)(MCPFD_FIFO_BULK0 + i), MCPFD_BULK_DEPTH, base, -1 };
        base += MCPFD_BULK_DEPTH * MCPFD_OBJ_SIZE;
    }
    // base == MCPFD_RAM_SIZE here (checked by the host test).

    s->next_bulk = 0;
    memset(s->fast_ids, 0, sizeof(s->fast_ids));
    memset(&s->stats, 0, sizeof(s->stats));
}

esp_err_t mcpfd_stream_config_fifos(mcpfd_stream_t *s)
{
    esp_err_t err = reg_write32(s, REG_C1FIFOCON(MCPFD_FIFO_TX), TX_CON);
    for (int i = 0; i < MCPFD_LANES && err == ESP_OK; i++) {
        err = reg_write32(s, REG_C1FIFOCON(s->lane[i].fifo), RX_CON(s->lane[i].depth));
    }
    return err;
}

static esp_err_t filters_disable_all(mcpfd_stream_t *s)
{
    esp_err_t err = ESP_OK;
    for (int w = 0; w < MCPFD_FILTERS / 4 && err == ESP_OK; w++) {
        err = reg_write32(s, REG_C1FLTCON(w), 0);
    }
    return err;
}

esp_err_t mcpfd_stream_filters_accept_all_fast(mcpfd_stream_t *s)
{
    esp_err_t err = filters_disable_all(s);
    if (err == ESP_OK) err = reg_write32(s, REG_C1FLTOBJ(0), 0);
    if (err == ESP_OK) err = reg_write32(s, REG_C1MASK(0), 0);   // mask 0: every id matches
    if (err == ESP_OK) err = reg_write32(s, REG_C1FLTCON(0), FLT_ENABLE(MCPFD_FIFO_FAST));
    memset(s->fast_ids, 0, sizeof(s->fast_ids));
    return err;
}

esp_err_t mcpfd_stream_filters_program(mcpfd_stream_t *s, const uint16_t *fast_ids, size_t count,
                                       const uint32_t *obj, const uint32_t *mask, int n, size_t *used)
{
    uint32_t con[MCPFD_FILTERS / 4] = {0};
    int f = 0;

    // A filter object can only be written while its filter is disabled.
    esp_err_t err = filters_disable_all(s);
    memset(s->fast_ids, 0, sizeof(s->fast_ids));

    // First filters: exact id -> fast lane.
    for (size_t i = 0; i < count && f < MCPFD_MAX_FAST_IDS && err == ESP_OK; i++) {
        uint16_t id = fast_ids[i] & 0x7FF;
        if (s->fast_ids[id >> 5] & (1u << (id & 31))) continue;   // duplicate
        err = reg_write32(s, REG_C1FLTOBJ(f), id);
        if (err == ESP_OK) err = reg_write32(s, REG_C1MASK(f), MASK_SID_ALL);
        con[f / 4] |= FLT_ENABLE(MCPFD_FIFO_FAST) << ((f % 4) * 8);
        s->fast_ids[id >> 5] |= 1u << (id & 31);
        f++;
    }
    if (used) *used = (size_t)f;

    // Then the bulk lanes. Whole bus (also when the fitted set has no room, so nothing asked for is ever lost):
    // four filters on the two low id bits. Fitted: the given masked filters, one lane each in turn. Either way a
    // given id always lands in the same lane, so per-id ordering is preserved.
    bool all = n < 0 || n > MCPFD_FILTERS - f;
    if (all) n = MCPFD_BULK_LANES;
    for (int k = 0; k < n && err == ESP_OK; k++, f++) {
        err = reg_write32(s, REG_C1FLTOBJ(f), all ? (uint32_t)k : obj[k]);
        if (err == ESP_OK) err = reg_write32(s, REG_C1MASK(f), all ? MASK_SID_LOW2 : mask[k] & MASK_SID_ALL);
        con[f / 4] |= FLT_ENABLE(MCPFD_FIFO_BULK0 + k % MCPFD_BULK_LANES) << ((f % 4) * 8);
    }

    for (int w = 0; w < MCPFD_FILTERS / 4 && err == ESP_OK; w++) {
        err = reg_write32(s, REG_C1FLTCON(w), con[w]);
    }
    return err;
}

esp_err_t mcpfd_stream_reset_rx(mcpfd_stream_t *s)
{
    esp_err_t err = ESP_OK;
    for (int i = 0; i < MCPFD_LANES && err == ESP_OK; i++) {
        err = reg_write32(s, REG_C1FIFOCON(s->lane[i].fifo),
                          RX_CON(s->lane[i].depth) | FIFOCON_FRESET);
        s->lane[i].expect_tail = -1;
    }
    return err;
}

// ---------------------------------------------------------------------------
// Receive
// ---------------------------------------------------------------------------
void mcpfd_stream_parse(const uint8_t *obj, can_frame_t *out)
{
    uint32_t r0 = le32(obj);
    uint32_t r1 = le32(obj + 4);

    memset(out, 0, sizeof(*out));
    out->id = r0 & 0x7FF;
    out->extended = (r1 >> 4) & 1;
    if (out->extended) {
        out->id = (out->id << 18) | ((r0 >> 11) & 0x3FFFF);
    }
    out->dlc = r1 & 0x0F;
    out->fd = (r1 >> 7) & 1;
    out->brs = (r1 >> 6) & 1;

    // The FIFOs hold 8 payload bytes per message; a longer CAN FD frame is
    // delivered with its first 8.
    uint8_t len = out->dlc > 8 ? 8 : out->dlc;
    memcpy(out->data, obj + 8, len);
}

// Read up to `max` messages from one lane into s->blk_rx + 2 and release them
// on the chip. Returns the number read (0: lane empty or SPI error).
static int rx_chunk(mcpfd_stream_t *s, int li, int max)
{
    mcpfd_lane_t *lane = &s->lane[li];

    // FIFOSTA and FIFOUA are adjacent: one read gives the flags, the index the
    // chip stores the next frame at (head) and the RAM address of the oldest
    // unread frame (tail).
    if (reg_read(s, REG_C1FIFOSTA(lane->fifo), 8) != ESP_OK) return 0;
    uint32_t sta = le32(s->reg_rx + 2);
    uint32_t ua  = le32(s->reg_rx + 6);

    if (sta & FIFOSTA_RXOVIF) {
        s->stats.rx_overflow[li]++;
        // Sticky flag: cleared by writing it back as 0.
        reg_write32(s, REG_C1FIFOSTA(lane->fifo), sta & ~FIFOSTA_RXOVIF);
    }
    if (!(sta & FIFOSTA_TFNRFNIF)) {
        lane->expect_tail = -1;
        return 0;
    }

    int depth = lane->depth;
    int head = (int)FIFOSTA_FIFOCI(sta);
    int32_t off = (int32_t)ua - (int32_t)lane->base;
    int run;

    if (off < 0 || off >= depth * MCPFD_OBJ_SIZE || (off % MCPFD_OBJ_SIZE) != 0 || head >= depth) {
        // The chip's addresses do not match the layout we configured. Fall back
        // to what needs no layout at all: exactly one message at the address the
        // chip gave us.
        s->stats.rx_layout_fallback++;
        lane->expect_tail = -1;
        run = 1;
    } else {
        int tail = off / MCPFD_OBJ_SIZE;
        if (lane->expect_tail >= 0 && lane->expect_tail != tail) {
            s->stats.rx_desync++;
        }
        int count = (head - tail + depth) % depth;
        if (count == 0) {
            // Not empty with head == tail: full, unless the chip says otherwise
            // (the FIFOSTA index can read wrong, MCP2518FD erratum). One frame
            // is always there.
            if (sta & FIFOSTA_TFERFFIF) {
                count = depth;
            } else {
                count = 1;
                s->stats.rx_head_glitch++;
            }
        }
        run = count;
        if (run > depth - tail) run = depth - tail;   // up to the end of the ring
        if (run > max) run = max;
        lane->expect_tail = (int16_t)((tail + run) % depth);
    }

    size_t bytes = (size_t)run * MCPFD_OBJ_SIZE;
    uint16_t cmd = SPI_CMD_READ(RAM_BASE + ua);
    memset(s->blk_tx, 0, 2 + bytes);
    s->blk_tx[0] = (uint8_t)(cmd >> 8);
    s->blk_tx[1] = (uint8_t)cmd;
    if (xfer(s, s->blk_tx, s->blk_rx, 2 + bytes) != ESP_OK) {
        lane->expect_tail = -1;
        return 0;   // nothing released: the frames are read again next time
    }

    // Release the messages. A failed increment leaves the rest on the chip; the
    // next poll re-reads them (and notices through expect_tail).
    for (int i = 0; i < run; i++) {
        if (reg_write32(s, REG_C1FIFOCON(lane->fifo), RX_CON(depth) | FIFOCON_UINC) != ESP_OK) {
            break;
        }
    }
    s->stats.rx_frames[li] += (uint32_t)run;
    return run;
}

static void deliver_chunk(mcpfd_stream_t *s, const mcpfd_hooks_t *h, int li,
                          const uint8_t *objs, int n)
{
    can_frame_t f;
    for (int i = 0; i < n; i++) {
        mcpfd_stream_parse(objs + (size_t)i * MCPFD_OBJ_SIZE, &f);
        bool fast = (li == MCPFD_LANE_FAST);
        if (!fast && !f.extended && (s->fast_ids[f.id >> 5] & (1u << (f.id & 31)))) {
            s->stats.rx_fast_in_bulk++;
        }
        h->deliver(h->ctx, &f, fast);
    }
}

int mcpfd_stream_step(mcpfd_stream_t *s, const mcpfd_hooks_t *h)
{
    // The chunk is copied out of the shared SPI buffer before the lock is
    // released, because a transmit may run between the unlock and the delivery.
    uint8_t objs[MCPFD_CHUNK * MCPFD_OBJ_SIZE];
    uint32_t rxif = 0;
    int li = -1, n = 0;

    h->lock(h->ctx);
    if (reg_read32(s, REG_C1RXIF, &rxif) == ESP_OK) {
        if (rxif & (1u << MCPFD_FIFO_FAST)) {
            li = MCPFD_LANE_FAST;
        } else {
            for (int k = 0; k < MCPFD_BULK_LANES; k++) {
                int b = (s->next_bulk + k) % MCPFD_BULK_LANES;
                if (rxif & (1u << (MCPFD_FIFO_BULK0 + b))) {
                    li = 1 + b;
                    s->next_bulk = (uint8_t)((b + 1) % MCPFD_BULK_LANES);
                    break;
                }
            }
        }
        if (li >= 0) {
            n = rx_chunk(s, li, MCPFD_CHUNK);
            if (n > 0) memcpy(objs, s->blk_rx + 2, (size_t)n * MCPFD_OBJ_SIZE);
        }
    }
    h->unlock(h->ctx);

    if (n > 0) deliver_chunk(s, h, li, objs, n);
    return n;
}

// ---------------------------------------------------------------------------
// Transmit
// ---------------------------------------------------------------------------
esp_err_t mcpfd_stream_send(mcpfd_stream_t *s, const can_frame_t *frame)
{
    if (frame->dlc > 8) return ESP_ERR_NOT_SUPPORTED;

    // Flags + address of the next free TX slot in one read.
    esp_err_t err = reg_read(s, REG_C1FIFOSTA(MCPFD_FIFO_TX), 8);
    if (err != ESP_OK) return err;
    uint32_t sta = le32(s->reg_rx + 2);
    uint32_t ua  = le32(s->reg_rx + 6);
    if (!(sta & FIFOSTA_TFNRFNIF)) {
        s->stats.tx_full++;
        return ESP_ERR_NO_MEM;
    }

    uint32_t t0 = frame->id & 0x7FF;
    if (frame->extended) {
        t0 = ((frame->id >> 18) & 0x7FF) | ((frame->id & 0x3FFFF) << 11);
    }
    uint32_t t1 = frame->dlc & 0x0F;
    if (frame->extended) t1 |= (1u << 4);
    if (frame->fd)       t1 |= (1u << 7);
    if (frame->brs)      t1 |= (1u << 6);

    // Always the whole 16-byte slot, as the default driver does.
    uint16_t cmd = SPI_CMD_WRITE(RAM_BASE + ua);
    memset(s->reg_tx, 0, 2 + MCPFD_OBJ_SIZE);
    s->reg_tx[0] = (uint8_t)(cmd >> 8);
    s->reg_tx[1] = (uint8_t)cmd;
    put_le32(s->reg_tx + 2, t0);
    put_le32(s->reg_tx + 6, t1);
    memcpy(s->reg_tx + 10, frame->data, frame->dlc);
    err = xfer(s, s->reg_tx, NULL, 2 + MCPFD_OBJ_SIZE);
    if (err != ESP_OK) return err;

    err = reg_write32(s, REG_C1FIFOCON(MCPFD_FIFO_TX), TX_CON | FIFOCON_UINC | FIFOCON_TXREQ);
    if (err == ESP_OK) s->stats.tx_frames++;
    return err;
}
