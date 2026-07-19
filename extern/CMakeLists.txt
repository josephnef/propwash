# Builds Betaflight 4.5.2 as an in-process static library (bf_sitl).
#
# The source list (bf_sources.txt) was harvested from an authoritative
# `make TARGET=SITL` build of the pinned submodule (object list of
# betaflight_SITL.elf), minus:
#   - main.c               (propwash-core provides its own main loop)
#   - target/SITL/sitl.c   (replaced by betaflightext/pw_sitl.c — deterministic
#                           time, direct state setters, no UDP threads)
#   - target/SITL/udplink.c (stock UDP fdm/rc transport, unused)
#
# Entries in bf_sources.txt are relative to src/main/, except lib/main/...
# which are relative to the repo root.

set(BF_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/betaflight")

file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/bf_sources.txt" BF_SOURCE_LIST)

set(BF_SOURCES "")
foreach(src ${BF_SOURCE_LIST})
  if(src MATCHES "^lib/main/")
    list(APPEND BF_SOURCES "${BF_ROOT}/${src}")
  else()
    list(APPEND BF_SOURCES "${BF_ROOT}/src/main/${src}")
  endif()
endforeach()

# propwash override layer (GPL, derived from betaflight's SITL target).
# scheduler.c is a patched copy for deterministic lockstep — see
# betaflightext/patches/scheduler-lockstep.diff (stock excluded from
# bf_sources.txt).
list(APPEND BF_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/betaflightext/src/main/target/SITL/pw_sitl.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/betaflightext/src/main/scheduler/scheduler.c"
  # cli.c: unconditional pgFind() null guard so diff/dump/save don't segfault
  # on values whose feature (and PG) is undef'd in SITL —
  # patches/cli-pgfind-null-guard.diff (stock excluded from bf_sources.txt).
  "${CMAKE_CURRENT_SOURCE_DIR}/betaflightext/src/main/cli/cli.c"
  # OSD: the target.h wrapper re-enables USE_OSD + USE_MAX7456; this fake
  # displayport captures the 16x30 grid into osdScreen[] (replaces the stock
  # SPI max7456 displayport, excluded from bf_sources.txt).
  "${CMAKE_CURRENT_SOURCE_DIR}/betaflightext/src/main/io/displayport_fake.c"
  # serial_tcp.c: stock TCP UART driver + dyad thread-safety. dyad is pumped
  # on pw_sitl.c's worker thread but called from the sim thread here too; the
  # unsynchronised race left the CLI/MSP listener unserviced on some hosts
  # (100% on macOS CI). This copy serialises dyad via pw_dyad_lock/unlock —
  # patches/serial-tcp-dyad-lock.diff (stock excluded from bf_sources.txt).
  "${CMAKE_CURRENT_SOURCE_DIR}/betaflightext/src/main/drivers/serial_tcp.c"
  # dyad.c: stock, with its mutable globals gathered into one exported struct
  # (pw_dyad_state) so the deterministic-reset snapshot can EXCLUDE dyad by
  # address range — dyad's SelectSet reallocates fd arrays as connections come
  # and go, so restoring boot-time pointers would be a use-after-free —
  # patches/dyad-globals-struct.diff (stock excluded from bf_sources.txt).
  "${CMAKE_CURRENT_SOURCE_DIR}/betaflightext/src/main/compat/dyad/dyad.c"
)

if(APPLE OR WIN32)
  # The dead reference GNU --gc-sections prunes on Linux (gyro_init.c's unused
  # gyroReadRegister -> mpuGyroReadRegister) survives on macOS (no dead-strip)
  # and on MinGW (--gc-sections doesn't prune it there), so provide a no-op
  # stub. See pw_stubs.c for the full rationale.
  list(APPEND BF_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/betaflightext/src/main/pw_stubs.c")
endif()

add_library(bf_sitl STATIC ${BF_SOURCES})

target_compile_definitions(bf_sitl PUBLIC
  SITL
  SIMULATOR_BUILD
  _GNU_SOURCE
  USE_STDPERIPH_DRIVER
  "__FORKNAME__=\"betaflight\""
  "__TARGET__=\"SITL\""
  "__REVISION__=\"propwash\""
  TARGET_PREINIT
)

target_include_directories(bf_sitl PUBLIC
  # propwash target/ MUST precede the stock target dir: its target.h wrapper
  # is found first and #include_next pulls the stock header beneath it
  "${CMAKE_CURRENT_SOURCE_DIR}/betaflightext/src/main/target/SITL"
  "${CMAKE_CURRENT_SOURCE_DIR}/betaflightext/src/main/io"
  # sim-safe header shadows (build/atomic.h: the stock one is inline ARM
  # assembly, pulled in once USE_DSHOT compiles dshot.c) — must precede
  # the stock src/main
  "${CMAKE_CURRENT_SOURCE_DIR}/betaflightext/src/main/simstubs"
  "${BF_ROOT}/src/main"
  "${BF_ROOT}/src/main/target/SITL"
  "${BF_ROOT}/lib/main/dyad"
  "${BF_ROOT}/lib/main/google/olc"
  # for the overridden scheduler.c / cli.c, whose same-directory includes
  # must resolve against the stock headers. (Not drivers/ — it contains a
  # time.h that would shadow the system <time.h>; the vendored serial_tcp.c
  # uses a "drivers/serial_tcp.h" path instead.)
  "${BF_ROOT}/src/main/scheduler"
  "${BF_ROOT}/src/main/cli"
)

if(WIN32)
  # MinGW-w64 has no <netinet/in.h>; serial_tcp.h includes it vestigially. This
  # shim dir satisfies the include (forwarding to Winsock) — Windows only, so
  # POSIX builds keep the real header.
  target_include_directories(bf_sitl PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/betaflightext/src/main/compat")
  # dyad.h wrapper: must precede the stock lib/main/dyad so "dyad.h" resolves
  # to it; it #include_next's the stock header then undefs the Windows baud
  # macros that clash with baudRate_e.
  target_include_directories(bf_sitl BEFORE PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/betaflightext/src/main/compat/dyad")
  # Force-include a compat header supplying POSIX/GNU libc bits MinGW lacks
  # (ffs, string.h, ...). SHELL: keeps the two tokens together, un-deduplicated.
  target_compile_options(bf_sitl PRIVATE
    "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/betaflightext/src/main/compat/pw_win_compat.h")
  # Winsock: dyad (in this lib) and the UDP server (in pw_sim, which links this
  # PUBLIC) call it. Declaring it as a PUBLIC dependency here — rather than a
  # bare -lws2_32 in BF_LINK_OPTIONS — lets CMake order it correctly after the
  # referencing objects (a raw flag can land before bf_sitl and get dropped).
  target_link_libraries(bf_sitl PUBLIC ws2_32)
endif()

target_compile_options(bf_sitl PRIVATE
  -std=gnu17
  -Wall -Wextra
  -Wno-error
  # GCC 14+ makes these hard errors; BF's CMS menus rely on the enum/uint8_t*
  # pun and on implicit int<->enum, so downgrade for the vendored firmware
  -Wno-error=incompatible-pointer-types
  -Wno-incompatible-pointer-types
  -Wno-unterminated-string-initialization
  -fno-common
  -ffunction-sections -fdata-sections
  -O2
)

# Consumers (executables) must link so the .pg_registry parameter-group
# section is laid out contiguously and its boundary symbols resolve.
if(APPLE)
  # macOS/ld64: no GNU linker scripts. Betaflight's pg.h already routes the
  # registry into the Mach-O section __DATA,__pg_registry and reads its bounds
  # via ld64's synthesized section$start/section$end symbols (the __APPLE__
  # branch). We must NOT force_load the archive: that would drag in objects
  # (gps_lap_timer, light_led, pidaudio, i2c...) whose dependencies live in
  # STM32 driver files excluded from the SITL source list, leaving undefined
  # symbols. Normal selective linking pulls only the referenced objects — the
  # same set Linux pulls — and the sim's init path transitively references
  # every PG-registered module, so the registry stays complete.
  #
  # Crucially we DON'T -dead_strip. It's the ld64 analogue of --gc-sections,
  # but some ld64 versions strip pgRegistry_t records out of
  # __DATA,__pg_registry despite their __attribute__((used)) — the config vars
  # survive (the sim flies on defaults) but drop out of the registry, so
  # `save`/load stops persisting them (diff_roundtrip / real_tune_hover fail on
  # the CI runner while passing on a newer local linker). Keeping every object
  # is the robust cross-linker choice; the price is the one dead cross-ref that
  # --gc-sections prunes on Linux, satisfied here by pw_stubs.c. No -lrt
  # (real-time calls live in libSystem here); pthreads are too.
  set(BF_LINK_OPTIONS
    -lm
    PARENT_SCOPE
  )
elseif(WIN32)
  # MinGW-w64 GNU ld: same pg.ld linker script as Linux (it already handles the
  # PE page-alignment case). winpthreads (-lpthread) for the tcp worker; no -lrt
  # (in the MinGW CRT). Winsock (ws2_32) is a PUBLIC dep of bf_sitl (above), so
  # it flows to consumers in the right link order.
  set(BF_LINK_OPTIONS
    "-T${BF_ROOT}/src/main/target/SITL/pg.ld"
    "-Wl,--gc-sections"
    -lm -lpthread
    PARENT_SCOPE
  )
else()
  # GNU ld: the pg.ld linker script collects .pg_registry and provides the
  # __pg_registry_start/end boundary symbols; --gc-sections + KEEP prune the
  # rest without dropping the registry.
  set(BF_LINK_OPTIONS
    "-T${BF_ROOT}/src/main/target/SITL/pg.ld"
    "-Wl,--gc-sections"
    -lm -lpthread -lrt
    PARENT_SCOPE
  )
endif()
