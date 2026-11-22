#include "ble_stream.h"

#include <string.h>

// Compact frames carry an 11-bit id and a bus below 32. Others (the firmware's diagnostic records) keep the
// original format.
static bool fits_compact(const can_tagged_frame_t *f)
{
    return !f->frame.extended && f->frame.id <= 0x7FF && f->bus_id < 32;
}

size_t ble_stream_build(const can_tagged_frame_t *frames, int count, int *from, uint8_t *buf, size_t buf_size,
                        bool compact, bool rest, ble_stream_want_fn want, void *ctx)
{
    size_t pos = 1;
    uint8_t n = 0;
    for (; *from < count; (*from)++) {
        const can_tagged_frame_t *f = &frames[*from];
        if (compact ? !fits_compact(f) : rest && fits_compact(f)) continue;
        if (want && !want(f, ctx)) continue;
        uint8_t len = f->frame.dlc > 8 ? 8 : f->frame.dlc;  // DashPilot expects classic CAN (max 8)
        if (pos + (compact ? 3 : 10) + len > buf_size) break;
        if (compact) {
            uint16_t h = (uint16_t)(f->frame.id | f->bus_id << 11);
            buf[pos++] = (uint8_t)h;
            buf[pos++] = (uint8_t)(h >> 8);
        } else {
            uint32_t ts = f->timestamp_us, id = f->frame.id;
            buf[pos + 0] = (uint8_t)ts;
            buf[pos + 1] = (uint8_t)(ts >> 8);
            buf[pos + 2] = (uint8_t)(ts >> 16);
            buf[pos + 3] = (uint8_t)(ts >> 24);
            buf[pos + 4] = f->bus_id;
            buf[pos + 5] = (uint8_t)id;
            buf[pos + 6] = (uint8_t)(id >> 8);
            buf[pos + 7] = (uint8_t)(id >> 16);
            buf[pos + 8] = (uint8_t)(id >> 24);
            pos += 9;
        }
        buf[pos++] = len;
        memcpy(&buf[pos], f->frame.data, len);
        pos += len;
        n++;
    }
    buf[0] = compact ? BLE_COMPACT_MARK : n;
    return n ? pos : 0;
}

int ble_stream_pick_primary(const ble_stream_cand_t *c, int n)
{
    int best = -1;
    // Claimed writable receivers first, then writable ones, then any receiver.
    for (int pass = 0; pass < 3 && best < 0; pass++) {
        for (int i = 0; i < n; i++) {
            if (!c[i].receiver || (pass < 2 && !c[i].writable) || (pass == 0 && !c[i].claimed)) continue;
            if (best < 0 || c[i].sub_seq > c[best].sub_seq) best = i;
        }
    }
    return best;
}
