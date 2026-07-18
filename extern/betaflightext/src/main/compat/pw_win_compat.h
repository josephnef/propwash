#pragma once
/*
 * propwash Windows (MinGW-w64) compatibility.
 *
 * Force-included into the Betaflight static lib on Windows only (see the
 * WIN32 block in extern/CMakeLists.txt) to supply POSIX/GNU libc bits that
 * MinGW-w64 doesn't declare. Add entries here as the MinGW build surfaces
 * them, rather than vendoring large firmware files.
 */

/*
 * ffs(): GNU/POSIX "find first set". Betaflight calls it (fc/core.c, cli.c) to
 * turn the arming-disable bitmask into a bit index; MinGW-w64 declares it in no
 * header (implicit-declaration error). It's always available as a GCC builtin,
 * and Betaflight only ever uses ffs as a function call, so a macro is safe.
 */
#ifndef ffs
#define ffs __builtin_ffs
#endif
