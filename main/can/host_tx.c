#include "host_tx.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BUSES    2
#define IDS      0x800

// What the car was seen sending, per bus and id (24 KB).
typedef struct {
    uint32_t seen_ms[BUSES][IDS];   // when the last frame arrived; valid if seen[][] is set
    uint16_t period_ms[BUSES][IDS]; // the time between the last two; 0: one frame so far
    uint8_t  seen[BUSES][IDS / 8];
} watch_t;

typedef struct {
    can_frame_t frame;
    uint32_t    queued_ms;
    uint8_t     bus;
    bool        used;
} pending_t;

// Allocated at the first arming and kept: the receive path reads the pointer
// without a lock, so it is never freed under it.
static watch_t *volatile s_watch;
static volatile bool     s_watching;
static bool              s_armed;
static uint32_t          s_armed_at_ms, s_arm_seen_ms;
static pending_t         s_pending[HOST_TX_PENDING];
static host_tx_stats_t   s_stats;

static void disarm(void)
{
    s_armed = false;
    s_watching = false;
    memset(s_pending, 0, sizeof(s_pending));
}

static void arm(uint32_t now_ms)
{
    s_arm_seen_ms = now_ms;
    if (s_armed) return;
    watch_t *w = s_watch;
    if (w) memset(w, 0, sizeof(*w));          // what was learnt last time is stale
    else if (!(w = calloc(1, sizeof(*w)))) return;   // no memory: stay disarmed, every frame is refused
    s_watch = w;
    s_armed_at_ms = now_ms;
    s_armed = true;
    s_watching = true;
}

bool host_tx_armed(uint32_t now_ms)
{
    if (s_armed && now_ms - s_arm_seen_ms > HOST_TX_ARM_HOLD_MS) disarm();
    return s_armed;
}

void host_tx_saw(uint8_t bus, uint32_t id, uint32_t now_ms)
{
    if (!s_watching || bus >= BUSES || id >= IDS) return;
    watch_t *w = s_watch;
    if (!w) return;
    uint8_t bit = (uint8_t)(1u << (id & 7));
    if (w->seen[bus][id >> 3] & bit) {
        uint32_t gap = now_ms - w->seen_ms[bus][id];
        w->period_ms[bus][id] = gap > 0xFFFF ? 0xFFFF : (uint16_t)gap;
    } else {
        w->seen[bus][id >> 3] |= bit;
    }
    w->seen_ms[bus][id] = now_ms;
}

static int hex_digit(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    c |= 0x20;
    return c >= 'a' && c <= 'f' ? c - 'a' + 10 : -1;
}

bool host_tx_line(const char *line, uint32_t now_ms)
{
    if (!strcmp(line, "X,1")) { arm(now_ms); return true; }
    if (!strcmp(line, "X,0")) { if (s_armed) disarm(); return true; }
    if (line[0] != 'T' || line[1] != ',') return false;

    unsigned bus;
    unsigned long id;
    char hex[17];
    int n = -1;
    size_t len = strlen(line);
    if (sscanf(line, "T,%u,%lx,%16[0-9a-fA-F]%n", &bus, &id, hex, &n) != 3 || n != (int)len ||
        bus >= BUSES || id >= IDS || strlen(hex) % 2 ||
        !host_tx_armed(now_ms) || now_ms - s_armed_at_ms < HOST_TX_WATCH_MS) {
        s_stats.refused++;
        return true;
    }
    for (int i = 0; i < HOST_TX_PENDING; i++) {
        pending_t *p = &s_pending[i];
        if (p->used) continue;
        memset(p, 0, sizeof(*p));
        p->frame.id = id;
        p->frame.dlc = (uint8_t)(strlen(hex) / 2);
        for (int b = 0; b < p->frame.dlc; b++) p->frame.data[b] = (uint8_t)(hex_digit(hex[2 * b]) << 4 | hex_digit(hex[2 * b + 1]));
        p->bus = (uint8_t)bus;
        p->queued_ms = now_ms;
        p->used = true;
        return true;
    }
    s_stats.refused++;
    return true;
}

bool host_tx_due(uint32_t now_ms, uint8_t *bus, can_frame_t *frame)
{
    if (!host_tx_armed(now_ms)) return false;
    watch_t *w = s_watch;
    for (int i = 0; i < HOST_TX_PENDING; i++) {
        pending_t *p = &s_pending[i];
        if (!p->used) continue;
        uint32_t id = p->frame.id;
        bool seen = w->seen[p->bus][id >> 3] & (1u << (id & 7));
        uint32_t since_car = now_ms - w->seen_ms[p->bus][id];
        bool owned = seen && since_car < HOST_TX_OWNED_MS;
        // Owned: only in the first half of the period after the car's own frame.
        if (!owned || (w->period_ms[p->bus][id] && since_car < w->period_ms[p->bus][id] / 2u)) {
            *bus = p->bus;
            *frame = p->frame;
            p->used = false;
            return true;
        }
        if (now_ms - p->queued_ms > HOST_TX_EXPIRE_MS) {
            p->used = false;
            s_stats.expired++;
        }
    }
    return false;
}

void host_tx_result(bool sent)
{
    if (sent) s_stats.sent++;
    else s_stats.refused++;
}

const host_tx_stats_t *host_tx_stats(void)
{
    return &s_stats;
}
