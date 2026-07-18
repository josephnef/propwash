/*
 * propwash Windows compatibility shim.
 *
 * Betaflight's drivers/serial_tcp.h does `#include <netinet/in.h>`, but only
 * vestigially — its tcpPort_t struct uses no netinet types (just dyad_Stream*,
 * pthread mutexes, and byte buffers). MinGW-w64 has no <netinet/in.h>, so this
 * shim satisfies the include on Windows and forwards to Winsock for the
 * sockaddr_in family in case anything ever needs it.
 *
 * This directory is added to the bf_sitl include path only on WIN32
 * (see extern/CMakeLists.txt), so POSIX builds keep the real header.
 */
#pragma once

#include <winsock2.h>
#include <ws2tcpip.h>
