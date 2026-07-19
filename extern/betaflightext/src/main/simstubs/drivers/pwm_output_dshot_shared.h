#pragma once
/*
 * propwash: sim shadow of drivers/pwm_output_dshot_shared.h. The stock
 * header is hardware DMA plumbing (motor timers, IRQ handlers); the only
 * thing a compiled TU consumes on SITL is the vendored cli.c's
 * `dshot telemetry_info` command reading the DMA IRQ cycle counters —
 * which the virtual ESC keeps at zero.
 */

#include <stdint.h>

#ifdef USE_DSHOT_TELEMETRY
typedef struct dshotDMAHandlerCycleCounters_s {
    uint32_t irqAt;
    uint32_t changeDirectionCompletedAt;
} dshotDMAHandlerCycleCounters_t;

extern dshotDMAHandlerCycleCounters_t dshotDMAHandlerCycleCounters;
#endif
