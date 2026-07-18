/*
 * propwash Windows compatibility shim.
 *
 * Betaflight's drivers/serial_tcp.h does `#include <netinet/in.h>`, but only
 * vestigially — its tcpPort_t struct uses no netinet types (just dyad_Stream*,
 * pthread mutexes, and byte buffers), and dyad itself talks to Winsock through
 * its own guarded includes. MinGW-w64 has no <netinet/in.h>, so this shim just
 * satisfies the include.
 *
 * It is deliberately EMPTY: forwarding to <winsock2.h> would drag in
 * <windows.h> / <winbase.h>, whose COMMPROP baud macros (BAUD_9600, ...)
 * collide with Betaflight's baudRate_e enum in io/serial.h.
 *
 * This directory is on the bf_sitl include path only on WIN32
 * (see extern/CMakeLists.txt), so POSIX builds keep the real header.
 */
#pragma once
