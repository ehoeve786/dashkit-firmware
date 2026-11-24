// Host test of the rules for frames a USB host asks to have transmitted (main/can/host_tx.c).
#include "host_tx.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int s_failed;
#define CHECK(cond) do { if (!(cond)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); s_failed++; } } while (0)

static uint8_t     s_bus;
static can_frame_t s_frame;

static bool due(uint32_t now) { return host_tx_due(now, &s_bus, &s_frame); }

// The car sends 0x249 on bus 1 every 50 ms and 0x129 every 10 ms, from `from` to `to`.
static void car(uint32_t from, uint32_t to)
{
    for (uint32_t t = from; t <= to; t++) {
        if (t % 50 == 0) host_tx_saw(1, 0x249, t);
        if (t % 10 == 0) host_tx_saw(1, 0x129, t);
    }
}

int main(void)
{
    const host_tx_stats_t *stats = host_tx_stats();

    // Lines of the other protocols are not ours.
    CHECK(!host_tx_line("C,1,7f0,00", 0));
    CHECK(!host_tx_line("L,1", 0));

    // Not armed: refused, and nothing ever comes due.
    CHECK(host_tx_line("T,1,249,00", 1000));
    CHECK(stats->refused == 1 && !due(1000));

    // Armed, but still watching the bus: refused.
    CHECK(host_tx_line("X,1", 10000) && host_tx_armed(10000));
    car(10000, 11000);
    CHECK(host_tx_line("T,1,555,01", 11000));
    CHECK(stats->refused == 2 && !due(11000));

    // Watching over. An id the car does not send goes out at once, as written.
    car(11001, 12010);
    CHECK(host_tx_line("X,1", 11500));
    CHECK(host_tx_line("T,0,555,0a0B0c", 12010));
    CHECK(due(12010) && s_bus == 0 && s_frame.id == 0x555 && s_frame.dlc == 3 && !s_frame.extended);
    CHECK(s_frame.data[0] == 0x0A && s_frame.data[1] == 0x0B && s_frame.data[2] == 0x0C);
    host_tx_result(true);
    CHECK(stats->sent == 1 && !due(12010));
    // The same id on the bus the car uses is still not the car's: ids are per bus.
    CHECK(host_tx_line("T,0,249,01", 12010) && due(12010));

    // An id the car sends: not in the second half of its period (car frame at 12000, next at 12050)...
    CHECK(host_tx_line("X,1", 12030));
    CHECK(host_tx_line("T,1,249,ce0f0000", 12030));
    CHECK(!due(12030) && !due(12049));
    // ...but right after the car's next frame, and only until half the period is over.
    car(12011, 12050);
    CHECK(due(12051) && s_bus == 1 && s_frame.id == 0x249 && s_frame.dlc == 4);
    CHECK(host_tx_line("T,1,249,ce0f0000", 12076) && !due(12076));
    car(12051, 12100);
    CHECK(due(12124));
    // A 100 Hz id has 5 ms.
    CHECK(host_tx_line("T,1,129,00", 12126) && !due(12126));
    car(12101, 12130);
    CHECK(due(12134));
    CHECK(host_tx_line("T,1,129,00", 12135) && !due(12135));
    car(12131, 12140);
    CHECK(due(12140));

    // The car's module went silent: the frame waits, then is dropped. Never forced.
    CHECK(host_tx_line("X,1", 13000));
    CHECK(host_tx_line("T,1,249,00", 13000));
    for (uint32_t t = 13000; t < 13000 + HOST_TX_EXPIRE_MS; t += 5) {
        if (t % 1000 == 0) host_tx_line("X,1", t);
        CHECK(!due(t));
    }
    host_tx_line("X,1", 15400);
    CHECK(!due(15600) && stats->expired == 1);

    // Malformed lines are refused and send nothing.
    uint32_t refused = stats->refused;
    host_tx_line("X,1", 16000);
    const char *bad[] = { "T,2,100,00", "T,1,800,00", "T,1,100,0", "T,1,100,001122334455667788", "T,1,100,zz", "T,1,100", "T,1,100,00,", "T,-1,100,00" };
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) CHECK(host_tx_line(bad[i], 16000));
    CHECK(stats->refused == refused + sizeof(bad) / sizeof(bad[0]) && !due(16000));

    // The waiting list holds HOST_TX_PENDING frames; the next is refused.
    car(16000, 16010);
    refused = stats->refused;
    for (int i = 0; i < HOST_TX_PENDING + 1; i++) host_tx_line("T,1,249,00", 16030);
    CHECK(stats->refused == refused + 1);

    // The host stopped saying it is armed: nothing more goes out, and what was waiting is gone.
    car(16011, 19000);
    CHECK(!host_tx_armed(16000 + HOST_TX_ARM_HOLD_MS + 1));
    CHECK(!due(19001));
    host_tx_line("X,1", 20000);
    car(20000, 21900);
    CHECK(host_tx_armed(21900) && !due(21901));

    // Disarming on request; arming again starts with watching again.
    CHECK(host_tx_line("X,0", 22200) && !host_tx_armed(22200));
    refused = stats->refused;
    host_tx_line("X,1", 22300);
    host_tx_line("T,1,555,00", 22400);
    CHECK(stats->refused == refused + 1);

    host_tx_result(false);
    CHECK(stats->refused == refused + 2);

    printf(s_failed ? "host_tx: %d checks FAILED\n" : "host_tx: all checks passed\n", s_failed);
    return s_failed != 0;
}
