/*
 * pw_stubs.c — macOS-only link stubs for propwash.
 *
 * On macOS the Betaflight static lib is linked WITHOUT dead-stripping (see the
 * APPLE branch in extern/CMakeLists.txt). -dead_strip was found to remove
 * pgRegistry_t records from the Mach-O section __DATA,__pg_registry on some
 * ld64 versions *despite* their __attribute__((used)) — the config variables
 * still exist (so the sim flies on defaults) but they drop out of the
 * parameter-group registry, so `save`/load silently stops persisting them
 * (diff_roundtrip / real_tune_hover fail on the CI runner while passing on a
 * newer local linker). Keeping every object is the robust cross-linker fix.
 *
 * The cost of not stripping: the one dead cross-reference that GNU
 * --gc-sections prunes on Linux is left undefined — sensors/gyro_init.c's
 * gyroReadRegister() tail-calls mpuGyroReadRegister(), which lives in the
 * STM32 driver accgyro_mpu.c that is excluded from the SITL source list. That
 * path is never reached with the virtual SITL gyro, so a no-op stub satisfies
 * the linker.
 *
 * The symbol is matched by name only (C, no mangling). We deliberately do NOT
 * include accgyro_mpu.h so the placeholder const void* parameter can't clash
 * with the real extDevice_t* prototype; the forward declaration here just
 * quiets -Wmissing-prototypes.
 */
#include <stdint.h>

uint8_t mpuGyroReadRegister(const void *dev, uint8_t reg);

uint8_t mpuGyroReadRegister(const void *dev, uint8_t reg)
{
    (void)dev;
    (void)reg;
    return 0;
}
