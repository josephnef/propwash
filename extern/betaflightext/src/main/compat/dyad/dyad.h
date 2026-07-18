#pragma once
/*
 * propwash Windows wrapper over lib/main/dyad/dyad.h.
 *
 * The stock dyad.h does `#include <windows.h>` on _WIN32 (for SOCKET), which
 * drags in winbase.h's COMMPROP baud-rate macros (BAUD_9600, BAUD_19200, ...).
 * Those collide with Betaflight's baudRate_e enum in io/serial.h in any TU that
 * includes both — e.g. io/serial.c pulls drivers/serial_tcp.h -> dyad.h ->
 * windows.h, then io/serial.h, and the enumerator BAUD_9600 hits a macro.
 *
 * This wrapper is placed ahead of lib/main/dyad on the include path on WIN32
 * (see extern/CMakeLists.txt), pulls the stock header via #include_next (the
 * same trick propwash's target.h uses), and undefs the colliding names. The
 * undefs are no-ops off Windows, so the file is harmless cross-platform.
 */
#include_next <dyad.h>

#ifdef _WIN32
#undef BAUD_9600
#undef BAUD_19200
#undef BAUD_38400
#undef BAUD_57600
#undef BAUD_115200
#endif
