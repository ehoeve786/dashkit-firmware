// Host unit test for the CAN filter table (main/can/can_filter.c).
// Compiled with plain gcc against the ESP-IDF shims in tools/test/shim.
//
// Covers the atomic-update contract: a malformed or oversized BLE write must
// leave the previously installed filter fully intact.

#include "can_filter.h"

#include <stdio.h>
#include <string.h>

static int s_failures;

#define CHECK(cond, msg) do { \
    if (cond) { printf("ok:   %s\n", msg); } \
    else      { printf("FAIL: %s\n", msg); s_failures++; } \
} while (0)

// Wire format: [bus][num_addrs][addr_LE32]*
static size_t put_entries(uint8_t *buf, uint8_t bus, const uint32_t *addrs, uint8_t n)
{
    buf[0] = bus;
    buf[1] = n;
    for (uint8_t i = 0; i < n; i++) {
        buf[2 + i * 4 + 0] = (uint8_t)(addrs[i]);
        buf[2 + i * 4 + 1] = (uint8_t)(addrs[i] >> 8);
        buf[2 + i * 4 + 2] = (uint8_t)(addrs[i] >> 16);
        buf[2 + i * 4 + 3] = (uint8_t)(addrs[i] >> 24);
    }
    return 2 + (size_t)n * 4;
}

int main(void)
{
    static uint8_t buf[2 + 4 * 300];
    uint32_t addrs[2] = { 0x123, 0x456 };

    can_filter_init();

    // Baseline: install a two-entry filter.
    size_t len = put_entries(buf, 0, addrs, 1);
    len += put_entries(buf + len, 1, addrs + 1, 1);
    CHECK(can_filter_set(buf, len) == ESP_OK, "valid write accepted");
    CHECK(can_filter_should_forward(0, 0x123), "baseline forwards bus0 0x123");
    CHECK(can_filter_should_forward(1, 0x456), "baseline forwards bus1 0x456");
    CHECK(!can_filter_should_forward(0, 0x456), "baseline drops bus0 0x456");

    // Truncated address list: header claims 2 addresses, only 1 present.
    // Old code had already overwritten the table; new code must not.
    uint8_t trunc[] = { 0, 2, 0x99, 0x00, 0x00, 0x00 };
    CHECK(can_filter_set(trunc, sizeof(trunc)) == ESP_ERR_INVALID_ARG,
          "truncated write rejected");
    CHECK(can_filter_should_forward(0, 0x123), "truncated write keeps bus0 0x123");
    CHECK(!can_filter_should_forward(0, 0x099), "truncated write does not leak 0x099");

    // Overflow: one more entry than the table holds.
    static uint32_t many[257];
    for (uint32_t i = 0; i < 257; i++) many[i] = 0x700 + i;
    size_t big = 0;
    for (uint32_t i = 0; i < 257; i += 255) {
        uint8_t n = (uint8_t)((257 - i) > 255 ? 255 : (257 - i));
        big += put_entries(buf + big, 0, many + i, n);
    }
    CHECK(can_filter_set(buf, big) == ESP_ERR_NO_MEM, "overflow write rejected");
    CHECK(can_filter_should_forward(0, 0x123), "overflow write keeps bus0 0x123");
    CHECK(!can_filter_should_forward(0, 0x700), "overflow write does not leak 0x700");

    // Empty write clears the filter (pass-through).
    CHECK(can_filter_set(buf, 0) == ESP_OK, "empty write accepted");
    CHECK(can_filter_should_forward(0, 0x999), "empty write passes everything");

    printf(s_failures ? "\n%d FAILED\n" : "\nALL TESTS PASSED\n", s_failures);
    return s_failures ? 1 : 0;
}
