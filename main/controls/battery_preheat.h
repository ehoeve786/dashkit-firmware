#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Whether the battery-preheat injection is currently enabled. Exposed so the
// multi-finger automation can toggle it. The automation itself (and its 0x35
// BLE config opcode) lives in battery_preheat.c as battery_preheat_automation.
bool battery_preheat_enabled(void);

// DASHKIT_status (the app-only frame battery_preheat.c sends) also tells the
// app whether sport kick-down is injecting Standard; reported at once.
void battery_preheat_report_kickdown(bool on);

#ifdef __cplusplus
}
#endif
