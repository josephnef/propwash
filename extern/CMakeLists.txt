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
)

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
  "${BF_ROOT}/src/main"
  "${BF_ROOT}/src/main/target/SITL"
  "${BF_ROOT}/lib/main/dyad"
  "${BF_ROOT}/lib/main/google/olc"
  # for the overridden scheduler.c / cli.c, whose same-directory includes
  # must resolve against the stock headers
  "${BF_ROOT}/src/main/scheduler"
  "${BF_ROOT}/src/main/cli"
)

target_compile_options(bf_sitl PRIVATE
  -std=gnu17
  -Wall -Wextra
  -Wno-error
  -Wno-unterminated-string-initialization
  -fno-common
  -ffunction-sections -fdata-sections
  -O2
)

# Consumers (executables) must link with the parameter-group linker script
# and these system libs. Modern linkers need --no-warn-rwx-segments quieted
# for the custom section layout.
set(BF_LINK_OPTIONS
  "-T${BF_ROOT}/src/main/target/SITL/pg.ld"
  "-Wl,--gc-sections"
  -lm -lpthread -lrt
  PARENT_SCOPE
)
