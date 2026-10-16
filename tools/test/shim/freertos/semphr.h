#pragma once
// Minimal FreeRTOS semaphore shim for host unit tests: single-threaded, so
// take/give always succeed and the mutex is a placeholder.
#include <stdint.h>

#define pdTRUE 1
#define portMAX_DELAY 0xFFFFFFFFu

typedef void *SemaphoreHandle_t;

static inline SemaphoreHandle_t xSemaphoreCreateMutex(void) { static int m; return &m; }
static inline int xSemaphoreTake(SemaphoreHandle_t h, uint32_t timeout) { (void)h; (void)timeout; return pdTRUE; }
static inline int xSemaphoreGive(SemaphoreHandle_t h) { (void)h; return pdTRUE; }
