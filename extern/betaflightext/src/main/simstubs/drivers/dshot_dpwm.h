#pragma once
/*
 * propwash: sim shadow of drivers/dshot_dpwm.h (found first via simstubs/,
 * same pattern as build/atomic.h).
 *
 * The stock header embeds STM32 timer/DMA types (TIM_ICInitTypeDef, DMA
 * streams) inside motorDmaOutput_t — uncompilable off-target and
 * meaningless in the sim. The only cross-module contract actually consumed
 * on SITL is: dshot_command.c's allMotorsAreIdle() reading the last written
 * packet value through getMotorDmaOutput(), and motor.c's call to
 * dshotPwmDevInit() — both provided by the virtual DSHOT ESC in pw_sitl.c.
 */

#include "drivers/dshot.h"
#include "drivers/motor.h"

typedef struct motorDmaOutput_s {
    dshotProtocolControl_t protocolControl;
} motorDmaOutput_t;

motorDmaOutput_t *getMotorDmaOutput(uint8_t index);

motorDevice_t *dshotPwmDevInit(const struct motorDevConfig_s *motorConfig, uint16_t idlePulse, uint8_t motorCount, bool useUnsyncedPwm);
