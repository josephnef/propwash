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
