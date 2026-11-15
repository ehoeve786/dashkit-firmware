#include "can_route.h"
#include "dbc_generated.h"

#include <string.h>

#define WORDS  64   // 2048 standard ids / 32

typedef struct {
    uint32_t bits[CAN_ROUTE_BUSES][WORDS];
    bool     all[CAN_ROUTE_BUSES];   // wildcard: every frame on the bus
    bool     active;                 // false: no app list, use the manifest set
} forward_set_t;

static uint32_t s_core[CAN_ROUTE_BUSES][WORDS];

// Two forward sets: readers use s_fwd[s_fwd_idx], the writer fills the other one
// and then flips the index.
static forward_set_t   s_fwd[2];
static volatile uint8_t s_fwd_idx;

static inline bool test(const uint32_t *bits, uint32_t id)
{
    return (bits[id >> 5] >> (id & 31)) & 1u;
}

void can_route_init(void)
{
    memset(s_core, 0, sizeof(s_core));
    memset(s_fwd, 0, sizeof(s_fwd));
    s_fwd_idx = 0;
    for (int b = 0; b < CAN_ROUTE_BUSES && b < DBC_HW_FILTER_BUS_COUNT; b++) {
        const dbc_hw_filter_set_t *fs = &g_dbc_hw_filters[b];
        for (int i = 0; i < fs->count; i++) {
            uint32_t id = fs->ids[i] & 0x7FF;
            s_core[b][id >> 5] |= 1u << (id & 31);
        }
    }
}

void can_route_forward_begin(void)
{
    memset(&s_fwd[s_fwd_idx ^ 1], 0, sizeof(forward_set_t));
}

void can_route_forward_add(uint8_t bus, uint32_t addr)
{
    forward_set_t *w = &s_fwd[s_fwd_idx ^ 1];
    if (bus >= CAN_ROUTE_BUSES) return;
    if (addr == CAN_ROUTE_ADDR_ALL) {
        w->all[bus] = true;
    } else if (addr <= 0x7FF) {
        w->bits[bus][addr >> 5] |= 1u << (addr & 31);
    }
    // Other extended ids are only reachable through the wildcard.
}

void can_route_forward_commit(size_t entries)
{
    s_fwd[s_fwd_idx ^ 1].active = entries > 0;
    s_fwd_idx ^= 1;
}

uint8_t can_route_classify(uint8_t bus, uint32_t id, bool extended)
{
    if (bus >= CAN_ROUTE_BUSES) return 0;
    const forward_set_t *f = &s_fwd[s_fwd_idx];
    bool std = !extended && id <= 0x7FF;
    bool core = std && test(s_core[bus], id);
    bool fwd;
    if (!f->active) {
        fwd = core;
    } else {
        fwd = f->all[bus] || (std && test(f->bits[bus], id));
    }
    return (uint8_t)((core ? CAN_ROUTE_CORE : 0) | (fwd ? CAN_ROUTE_FORWARD : 0));
}

size_t can_route_fast_ids(uint8_t bus, uint16_t *out, size_t max)
{
    size_t n = 0;
    for (int i = 0; i < DBC_MESSAGE_COUNT && n < max; i++) {
        if (g_dbc_messages[i].default_bus != bus) continue;
        uint16_t id = (uint16_t)(g_dbc_messages[i].id & 0x7FF);
        bool dup = false;
        for (size_t k = 0; k < n; k++) dup |= (out[k] == id);
        if (!dup) out[n++] = id;
    }
    return n;
}
