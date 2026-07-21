# Driver for the model_regen ctest: regenerate the airframe from the .scad and
# diff it against the committed GLB.
#
# A script rather than two add_test()s because ctest has no notion of "run A
# then B", and the regen must land in the BUILD tree — a test that rewrote
# client-godot/assets/cinelog35_v3.glb would leave a dirty working tree behind
# on every `ctest` run.
#
# Expects: OPENSCAD_BIN, SOURCE_DIR, WORK_DIR.

set(regen "${WORK_DIR}/cinelog35_v3.glb")
set(committed "${SOURCE_DIR}/client-godot/assets/cinelog35_v3.glb")

file(MAKE_DIRECTORY "${WORK_DIR}")

execute_process(
  COMMAND python3 "${SOURCE_DIR}/model/build_asset.py"
    --openscad "${OPENSCAD_BIN}"
    --output "${regen}"
  RESULT_VARIABLE rc)
if(NOT rc EQUAL 0)
  message(FATAL_ERROR "build_asset.py failed (${rc}).")
endif()

execute_process(
  COMMAND python3 "${SOURCE_DIR}/model/validate_glb.py" "${regen}"
  RESULT_VARIABLE rc)
if(NOT rc EQUAL 0)
  message(FATAL_ERROR "the regenerated model does not satisfy validate_glb.py (${rc}).")
endif()

execute_process(
  COMMAND python3 "${SOURCE_DIR}/model/compare_glb.py" "${regen}" "${committed}"
  RESULT_VARIABLE rc)
if(NOT rc EQUAL 0)
  message(FATAL_ERROR "the committed GLB is stale (${rc}).")
endif()
