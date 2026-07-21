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
  core/sim/static_snapshot.cpp
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

# Contact solver gate: a disarmed quad dropped 0.5 m (level and inverted)
# must settle dead still on contact physics alone — and the inverted drop
# must stay inverted (crash outcomes are allowed to exist).
add_test(NAME contact_drop COMMAND pw-tester --drop-test)

# Crash lifecycle: rest never damages; a 10 m drop is a structural crash;
# a wrecked quad cannot hover (through physics, not scripting); REPAIR
# restores it and the same controller then hovers.
add_test(NAME crash_scenario COMMAND pw-tester --crash-test)
set_tests_properties(crash_scenario PROPERTIES TIMEOUT 120)

# Virtual DSHOT ESC (the default protocol now): boot grace clears under a DSHOT
# protocol, the quad arms + hovers through the virtual ESC's throttle
# mapping, and SPIN_DIRECTION commands sent through the stock
# dshot_command.c queue land in pw_motor_dir (the crashflip prerequisite).
add_test(NAME dshot_device COMMAND pw-tester --dshot-test)
set_tests_properties(dshot_device PROPERTIES TIMEOUT 120)

# Turtle mode end to end: settle inverted, arm with the crashflip box,
# reversed props pivot the quad over its duct edge (an emergent
# contact-solver maneuver), then a normal re-arm restores spin directions.
add_test(NAME turtle_flip COMMAND pw-tester --turtle-test)
set_tests_properties(turtle_flip PROPERTIES TIMEOUT 120)

# Bidirectional-DSHOT eRPM: the virtual ESC encodes physics-true rpm into
# real eRPM period frames, the stock stack decodes them, and the firmware's
# RPM filter runs — telemetry rpm must match physics rpm during a hover.
add_test(NAME rpm_filter COMMAND pw-tester --rpmfilter-test)
set_tests_properties(rpm_filter PROPERTIES TIMEOUT 120)

# Contact/damage over the wire: scripted drops onto the client-built ground
# manifold -> damage bands + crash latch + REPAIR read back from PW_STATE_OUT.
add_test(NAME collision_e2e
  COMMAND python3 ${CMAKE_SOURCE_DIR}/tools/tester/collision_e2e.py
    $<TARGET_FILE:propwash-core>)
set_tests_properties(collision_e2e PROPERTIES TIMEOUT 90)
add_test(NAME udp_e2e
  COMMAND bash ${CMAKE_SOURCE_DIR}/tools/tester/run_e2e.sh $<TARGET_FILE:propwash-core>)
set_tests_properties(udp_e2e PROPERTIES TIMEOUT 120)

# Codec parity (issue #22): the gym's standalone MIT wire codec and the GPL
# tools/tester codec must agree on the v2 format. Pure comparison, no core.
add_test(NAME codec_parity
  COMMAND python3 ${CMAKE_SOURCE_DIR}/tools/tester/codec_parity_check.py)
set_tests_properties(codec_parity PROPERTIES TIMEOUT 30)

# Determinism, both tiers — gating since the snapshot/restore reset made
# them green (BF::init() alone left ~20 subsystems of firmware statics
# carrying session history; Sim::reset now rewinds the process's writable
# statics to the post-first-boot snapshot, see core/sim/static_snapshot.h).
# On a mismatch both report the first divergent frame and field.
#
# Same-process: one core, the identical input tape twice around
# PW_CMD_RESET, byte-identical streams. Fails on any stray rand()/clock in
# the sim path and on anything leaking through the reset.
add_test(NAME reset_determinism
  COMMAND python3 ${CMAKE_SOURCE_DIR}/tools/tester/reset_determinism.py
    $<TARGET_FILE:propwash-core>)
set_tests_properties(reset_determinism PROPERTIES TIMEOUT 60)

# Cross-process: two fresh cores, identical inputs (one run with sends
# deliberately delayed past the recv timeout), byte-identical output — the
# project's headline claim. Diagnosis tools when it breaks:
#   determinism_check.py --runs N       trajectory-class histogram
#   PROPWASH_DUMP_STATE + state_diff.py bisect firmware residue to symbols
add_test(NAME cross_process_determinism
  COMMAND python3 ${CMAKE_SOURCE_DIR}/tools/tester/determinism_check.py
    $<TARGET_FILE:propwash-core>)
set_tests_properties(cross_process_determinism PROPERTIES TIMEOUT 180)

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

# Blackbox replay + system ID (stdlib only). Physics-only motor replay
# (PW_MOTOR_IN) is reproducible across cores, and the fitter recovers a known
# physics parameter from a synthetic reference. A real CineLog35 log drops into
# the same pipeline once the quad has flown.
add_test(NAME sysid_selfcheck
  COMMAND python3 ${CMAKE_SOURCE_DIR}/tools/sysid/sysid_check.py
    $<TARGET_FILE:propwash-core>)
set_tests_properties(sysid_selfcheck PROPERTIES TIMEOUT 180)

# Airframe asset. model/cinelog35_v3.scad is the source of truth and
# client-godot/assets/cinelog35_v3.glb is a committed build product — the same
# arrangement as extern/bf_sources.txt. See model/README.md.
#
# Always-on half: the committed GLB still satisfies the dimensional contract
# (wheelbase, duct bore, prop diameter, guard span, node/mesh/material names,
# normals + UVs, triangle budget). Pure stdlib python, no OpenSCAD, no Godot,
# so it runs on Windows/MSYS2 too.
add_test(NAME model_asset
  COMMAND python3 ${CMAKE_SOURCE_DIR}/model/validate_glb.py
    ${CMAKE_SOURCE_DIR}/client-godot/assets/cinelog35_v3.glb)
set_tests_properties(model_asset PROPERTIES TIMEOUT 30)

# Drift half: regenerating from the .scad must reproduce the committed GLB.
# Optional because it needs an OpenSCAD SNAPSHOT — the 2021.01 stable release
# has neither the manifold backend nor OBJ export, so it cannot build this at
# all (model/OPENSCAD_VERSION pins the exact build CI uses). Same
# absent-tool-skips pattern as GODOT_BIN and PW_GYM_PYTHON.
find_program(OPENSCAD_BIN NAMES openscad OpenSCAD
  PATHS /Applications/OpenSCAD.app/Contents/MacOS)
if(OPENSCAD_BIN)
  add_test(NAME model_regen
    COMMAND ${CMAKE_COMMAND}
      -DOPENSCAD_BIN=${OPENSCAD_BIN}
      -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
      -DWORK_DIR=${CMAKE_BINARY_DIR}/model
      -P ${CMAKE_SOURCE_DIR}/model/regen_check.cmake)
  set_tests_properties(model_regen PROPERTIES TIMEOUT 300)
  message(STATUS "model_regen test enabled (openscad: ${OPENSCAD_BIN})")
else()
  message(STATUS "model_regen test skipped (no openscad snapshot found)")
endif()

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
    ENVIRONMENT "PROPWASH_AUTOTEST=1;PROPWASH_PORT=9143;PROPWASH_CORE=$<TARGET_FILE:propwash-core>")

  # Quality-tier selection rule. Pure function, no rendering, no sim — the
  # cases that matter (240 Hz -> low) cannot be produced on a 60 Hz dev machine.
  add_test(NAME quality_tiers
    COMMAND ${GODOT_BIN} --headless --path ${CMAKE_SOURCE_DIR}/client-godot
            --script res://tests/quality_test.gd)
  set_tests_properties(quality_tiers PROPERTIES TIMEOUT 60)

  # FPV cull mask + prop->motor index map. Both are invariants a rewrite can
  # silently drop, and the cull mask already was once. PROPWASH_CORE points
  # nowhere on purpose: this builds the world but must not spawn a core.
  add_test(NAME fpv_cull
    COMMAND ${GODOT_BIN} --headless --path ${CMAKE_SOURCE_DIR}/client-godot
            --script res://tests/fpv_cull_test.gd)
  set_tests_properties(fpv_cull PROPERTIES
    TIMEOUT 60
    ENVIRONMENT "PROPWASH_PORT=9146;PROPWASH_CORE=/nonexistent/propwash-core")

  # Collision detection: hull vs analytic ground + engine-query gate tubes,
  # surface classification, slop depenetration, Godot->sim manifold
  # conversion. Detection only — the core's solver has its own ctests.
  add_test(NAME client_collision
    COMMAND ${GODOT_BIN} --headless --path ${CMAKE_SOURCE_DIR}/client-godot
            --script res://tests/collision_test.gd)
  set_tests_properties(client_collision PROPERTIES
    TIMEOUT 60
    ENVIRONMENT "PROPWASH_PORT=9147;PROPWASH_CORE=/nonexistent/propwash-core")

  # T-repair flow against a real core: crash from 10 m, repair in place,
  # assert damage/latch cleared, quad upright, and no stale-echo pose
  # flicker (the one-deep pipeline bug that made T "not work").
  add_test(NAME repair_flow
    COMMAND ${GODOT_BIN} --headless --path ${CMAKE_SOURCE_DIR}/client-godot
            --script res://tests/repair_flow_test.gd)
  set_tests_properties(repair_flow PROPERTIES
    TIMEOUT 90
    ENVIRONMENT "PROPWASH_NO_JS=1;PROPWASH_PORT=9145;PROPWASH_CORE=$<TARGET_FILE:propwash-core>")

  # Autonomous gate fly-through (angle mode): arms, cruises forward through
  # all three gates holding altitude, asserts it flew past the last gate.
  add_test(NAME flythrough
    COMMAND ${GODOT_BIN} --headless --path ${CMAKE_SOURCE_DIR}/client-godot)
  set_tests_properties(flythrough PROPERTIES
    TIMEOUT 120
    ENVIRONMENT "PROPWASH_DEMO=acro;PROPWASH_PORT=9144;PROPWASH_CORE=$<TARGET_FILE:propwash-core>")
endif()

# Python gym (M4). Runs only if an interpreter with numpy + gymnasium is found.
# Prefer the uv-managed venv (python/propwash_gym/.venv), else fall back to a
# system python3 that happens to have the deps. Mirrors the Godot-optional
# pattern: absent deps skip the test rather than fail the suite.
set(PW_GYM_DIR ${CMAKE_SOURCE_DIR}/python/propwash_gym)
find_program(PW_GYM_PYTHON
  NAMES python3
  PATHS ${PW_GYM_DIR}/.venv/bin
  NO_DEFAULT_PATH)
if(NOT PW_GYM_PYTHON)
  find_program(PW_GYM_PYTHON NAMES python3)
endif()
if(PW_GYM_PYTHON)
  execute_process(
    COMMAND ${PW_GYM_PYTHON} -c "import numpy, gymnasium"
    RESULT_VARIABLE PW_GYM_DEPS
    OUTPUT_QUIET ERROR_QUIET)
  if(PW_GYM_DEPS EQUAL 0)
    add_test(NAME gym_hover
      COMMAND ${PW_GYM_PYTHON} ${PW_GYM_DIR}/tests/gym_smoke.py
        $<TARGET_FILE:propwash-core>)
    set_tests_properties(gym_hover PROPERTIES TIMEOUT 120)
    message(STATUS "gym_hover test enabled (python: ${PW_GYM_PYTHON})")
  else()
    message(STATUS "gym_hover test skipped (numpy/gymnasium not in ${PW_GYM_PYTHON})")
  endif()
endif()
