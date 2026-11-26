#pragma once
#include <stddef.h>

// Reads up to [max] bytes of the image into [buf]; the count, or <= 0 when the stream stopped.
typedef int (*firmware_read_t)(void *context, char *buf, int max);

// Writes an image of [size] bytes into the other OTA slot and makes it the one to boot, as BLE OTA does (main.c
// confirms it once it runs, else the bootloader goes back). [first] holds bytes of it that already came; the rest
// is read with [read]. NULL when done (the caller restarts), else why not.
const char *firmware_update(unsigned long size, const char *first, int first_size, firmware_read_t read, void *context);
