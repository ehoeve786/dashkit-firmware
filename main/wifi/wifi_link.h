#pragma once
#include <stdbool.h>
#include <stddef.h>

// Optional Wi-Fi link to the CANLens app, for when nothing is on the USB port: the DashKit joins a network (the
// phone's hotspot) and serves one TCP client on port WIFI_LINK_PORT. Off until switched on over BLE; the choice and
// the network are kept in NVS. Nothing is sent before the client's first line.
//  in:  "L,1" once a second: every frame of both buses ("L,0", or 2 s without it: stop). Nothing else: transmitting
//       stays on USB and BLE, which a stranger on the network can't reach.
//  out: while asked, the lines the USB port's app gets ("A", "D", "H", see can_manager.c); always:
//       "G,<line>" the firmware log (the last ~32 KB of it from before the client came, then as it is written),
//       once a second "M,..." metrics (can_manager_metrics), then ",<RSSI dBm>,<free heap KB>".
#define WIFI_LINK_PORT 62402

// Starts capturing the log, and Wi-Fi if it was left on. Call right after nvs_flash_init.
void wifi_link_init(void);

// A command from the BLE transmit characteristic: "W,1" on, "W,0" off, "WS,<ssid>", "WP,<password>" (applied on the
// next "W,1"). False if it isn't one.
bool wifi_link_command(const char *line);

// "W,<on>,<ip or 0>" for the BLE status read. Returns the length.
int wifi_link_status(char *out, size_t size);

// Queues whole lines for the client; false (nothing queued) if there is no room. Never blocks.
bool wifi_link_send(const char *data, size_t len);
