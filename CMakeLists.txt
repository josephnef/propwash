cmake_minimum_required(VERSION 3.20)
project(propwash LANGUAGES C CXX)

set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

add_subdirectory(extern)

# ---------------------------------------------------------------- pw_sim
# The simulation core: physics + betaflight glue. GPL-3.0.
add_library(pw_sim STATIC
  core/sim/bf.cpp
  core/sim/sim.cpp
  core/sim/physics.cpp
  core/sim/util/SimplexNoise.cpp
  core/net/server.cpp
  core/input/joystick.cpp
)
target_include_directories(pw_sim PUBLIC core core/sim protocol)
target_link_libraries(pw_sim PUBLIC bf_sitl)
target_compile_options(pw_sim PRIVATE -Wall -Wextra -Wno-unused-parameter)

# ---------------------------------------------------------- propwash-core
add_executable(propwash-core core/main.cpp)
target_link_libraries(propwash-core PRIVATE pw_sim ${BF_LINK_OPTIONS})

# ---------------------------------------------------------------- tester
add_executable(pw-tester tools/tester/main.cpp)
target_link_libraries(pw-tester PRIVATE pw_sim ${BF_LINK_OPTIONS})

enable_testing()
add_test(NAME headless_scenario COMMAND pw-tester)
add_test(NAME udp_e2e
  COMMAND bash ${CMAKE_SOURCE_DIR}/tools/tester/run_e2e.sh $<TARGET_FILE:propwash-core>)
set_tests_properties(udp_e2e PROPERTIES TIMEOUT 120)

# CLI/Configurator data path: apply the pilot's diff over TCP 5761, save,
# reboot in-process, verify the tune survives (the M3 core deliverable).
add_test(NAME diff_roundtrip
  COMMAND python3 ${CMAKE_SOURCE_DIR}/tools/bfcli/diff_roundtrip_check.py
    $<TARGET_FILE:propwash-core>
    ${CMAKE_SOURCE_DIR}/config/cinelog35v3.diff
    ${CMAKE_SOURCE_DIR}/config/sitl-overrides.txt)
set_tests_properties(diff_roundtrip PROPERTIES TIMEOUT 60)

# OSD: confirm the real Betaflight OSD renders through the fake displayport
# and streams to the client as PW_OSD.
add_test(NAME osd_render
  COMMAND python3 ${CMAKE_SOURCE_DIR}/tools/tester/osd_check.py $<TARGET_FILE:propwash-core>)
set_tests_properties(osd_render PROPERTIES TIMEOUT 60)

# The payoff: bake the pilot's REAL diff into an eeprom, then a fresh
# instance flies a hover with the pilot's actual PIDs/rates.
add_test(NAME real_tune_hover
  COMMAND python3 ${CMAKE_SOURCE_DIR}/tools/tester/real_config_hover_check.py
    $<TARGET_FILE:propwash-core>)
set_tests_properties(real_tune_hover PROPERTIES TIMEOUT 90)

# Godot client autotest (runs only if a godot 4 binary is available).
# PROPWASH_AUTOTEST=1 arms + hovers via the real GDScript client, headless.
find_program(GODOT_BIN NAMES godot godot4
  Godot_v4.7.1-stable_linux.x86_64 Godot_v4.3-stable_linux.x86_64
  PATHS /tmp)
if(GODOT_BIN)
  add_test(NAME godot_client
    COMMAND ${GODOT_BIN} --headless --path ${CMAKE_SOURCE_DIR}/client-godot)
  set_tests_properties(godot_client PROPERTIES
    TIMEOUT 120
    ENVIRONMENT "PROPWASH_AUTOTEST=1;PROPWASH_CORE=$<TARGET_FILE:propwash-core>")

  # Autonomous gate fly-through (angle mode): arms, cruises forward through
  # all three gates holding altitude, asserts it flew past the last gate.
  add_test(NAME flythrough
    COMMAND ${GODOT_BIN} --headless --path ${CMAKE_SOURCE_DIR}/client-godot)
  set_tests_properties(flythrough PROPERTIES
    TIMEOUT 120
    ENVIRONMENT "PROPWASH_DEMO=acro;PROPWASH_CORE=$<TARGET_FILE:propwash-core>")
endif()
