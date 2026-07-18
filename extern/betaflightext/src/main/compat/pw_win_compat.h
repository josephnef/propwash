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
 * <string.h>: some firmware files (e.g. lib/main/google/olc/olc.c) call
 * strlen() but only #include <memory.h>, relying on glibc/libc++ pulling in
 * <string.h> transitively — which MinGW's <memory.h> does not. Harmless
 * everywhere, so it stays outside the _WIN32 guard.
 */
#include <string.h>

#if defined(_WIN32)

/*
 * strcasestr(): a GNU extension. Betaflight PROVIDES the implementation in
 * common/string_light.c, but MinGW's <string.h> doesn't DECLARE it, so cli.c
 * hits an implicit-declaration error. Just declare the prototype here; the
 * linker resolves the call to Betaflight's own definition. (POSIX libc already
 * declares it via <string.h> + _GNU_SOURCE, so this is Windows-only.)
 */
char *strcasestr(const char *haystack, const char *needle);

/*
 * ffs(): GNU/POSIX "find first set". Betaflight calls it (fc/core.c, cli.c) to
 * turn the arming-disable bitmask into a bit index; MinGW-w64 declares it in no
 * header. It's always available as a GCC builtin, and Betaflight only ever uses
 * ffs as a function call, so a macro is safe.
 */
#ifndef ffs
#define ffs __builtin_ffs
#endif

#endif /* _WIN32 */
