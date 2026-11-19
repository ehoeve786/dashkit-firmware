#include "can_route.h"

#include <string.h>

#define WORDS  64   // 2048 standard ids / 32

typedef struct {
    uint32_t bits[CAN_ROUTE_BUSES][WORDS];
    bool     all[CAN_ROUTE_BUSES];   // wildcard: every frame on the bus
} forward_set_t;

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
    memset(s_fwd, 0, sizeof(s_fwd));
    s_fwd_idx = 0;
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

void can_route_forward_commit(void)
{
    s_fwd_idx ^= 1;
}

bool can_route_all(uint8_t bus)
{
    return bus < CAN_ROUTE_BUSES && s_fwd[s_fwd_idx].all[bus];
}

uint8_t can_route_classify(uint8_t bus, uint32_t id, bool extended)
{
    if (bus >= CAN_ROUTE_BUSES) return 0;
    const forward_set_t *f = &s_fwd[s_fwd_idx];
    if (!extended && id <= 0x7FF && test(f->bits[bus], id)) return CAN_ROUTE_FORWARD;
    return f->all[bus] ? CAN_ROUTE_FORWARD | CAN_ROUTE_BULK : 0;
}
