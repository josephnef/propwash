/*
 * propwash target header — wraps Betaflight 4.5.2's stock SITL target.h and
 * re-enables the OSD, which stock SITL undefs. The OSD renders through a
 * fake max7456 displayport (displayport_fake.c) that captures the 16x30
 * character grid into osdScreen[] for the sim to stream to the client.
 *
 * Found ahead of the stock target.h on the include path; #include_next pulls
 * the stock one, then we re-enable what we need on top.
 */
#pragma once

#include_next "target.h"

// --- virtual DSHOT ESC ---
// common_pre.h gates USE_DSHOT on !SITL, so stock SITL never compiles the
// dshot stack. propwash provides a virtual ESC (pw_sitl.c dshotPwmDevInit):
// throttle maps onto the same pw_motors_pwm sink as the PWM device and
// SPIN_DIRECTION commands land in pw_motor_dir[] (crashflip). Deliberately
// NOT defined: USE_DSHOT_BITBANG (hardware-only). USE_DSHOT_TELEMETRY IS
// compiled (dshot.c's erpmToRpm needs its erpmToHz once USE_ESC_SENSOR is
// present, and the eRPM/RPM-filter work builds on it) but stays runtime-
// inactive until dshot_bidir is on AND the virtual ESC reports telemetry.
// Runtime opt-in: PROPWASH_DSHOT=1 skips the forced-PWM override in
// targetPreInit().
#define USE_DSHOT
#define USE_DSHOT_TELEMETRY
// On hardware USE_RPM_FILTER comes from the MCU platform headers
// (stm32/at32 platform_mcu.h), which SITL never includes; common_post.h
// keeps it only when USE_DSHOT_TELEMETRY is present (it is, above). The
// filter is runtime-inactive until the virtual ESC reports eRPM.
#define USE_RPM_FILTER

// --- re-enable the OSD stack (stock SITL undefs USE_OSD) ---
#define USE_OSD
#define USE_OSD_SD
#define USE_MAX7456                 // AUTO device -> fake max7456 displayport
#define USE_CMS
#define USE_MSP_DISPLAYPORT
#define USE_OSD_OVER_MSP_DISPLAYPORT

// OSD on by default so the FPV view has telemetry without extra config
#undef DEFAULT_FEATURES
#define DEFAULT_FEATURES (FEATURE_OSD | FEATURE_TELEMETRY)
