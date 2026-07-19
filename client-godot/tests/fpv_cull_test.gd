# Headless test for the FPV camera cull mask and the prop -> motor index map.
#
# Both are invariants that a rewrite can silently drop, and the cull mask already
# was: a rewrite of the drone model removed the layer assignment and the camera's
# mask together, and nothing noticed. Procedural scenes have no schema to violate,
# so only a test catches it.
#
#   godot --headless --path client-godot --script res://tests/fpv_cull_test.gd
extends SceneTree

const Main = preload("res://scripts/main.gd")

# Source of truth, restated here so the test fails if main.gd drifts from it:
#   core/sim/profile_cinelog35.h  motors are RR, FR, RL, FL   (sim: +z forward)
#   main.gd                       props  are FR, FL, RR, RL   (Godot: -z forward)
#   core/sim/physics.cpp:431      motor_dir = {-1, 1, 1, -1}  (sim order)
const SIM_MOTOR_DIR := [-1.0, 1.0, 1.0, -1.0]


var _main: Node3D
var _frames := 0


func _initialize() -> void:
	# _ready() on a node added to root is deferred, so the world does not exist
	# until a frame has been processed. Build here, assert in _process.
	_main = Node3D.new()
	_main.set_script(Main)
	root.add_child(_main)


func _process(_d: float) -> bool:
	_frames += 1
	if _frames < 2:
		return false
	_check()
	return true


func _check() -> void:
	var fails := 0
	var main := _main

	# --- the index map must be a permutation, not just plausible numbers
	var m: Array = Main.RPM_FOR_PROP
	if m.size() != 4:
		fails += 1
		print("FAIL RPM_FOR_PROP must have 4 entries, has %d" % m.size())
	else:
		var seen := {}
		for v in m:
			if v < 0 or v > 3 or seen.has(v):
				fails += 1
				print("FAIL RPM_FOR_PROP is not a permutation of 0..3: %s" % [m])
				break
			seen[v] = true

	# --- and it must agree with the firmware's spin directions. Permuting the
	# sim-order motor_dir by the map has to reproduce PROP_SPIN exactly; if
	# someone "fixes" one without the other, this catches it.
	if m.size() == 4:
		var derived := []
		for i in 4:
			derived.append(SIM_MOTOR_DIR[m[i]])
		if derived != Array(Main.PROP_SPIN):
			fails += 1
			print("FAIL PROP_SPIN %s disagrees with motor_dir permuted by "
					% [Main.PROP_SPIN] + "RPM_FOR_PROP, which gives %s" % [derived])

	# --- check the camera in the world built during _initialize
	var cam: Camera3D = _find_camera(main)
	if cam == null:
		fails += 1
		print("FAIL no Camera3D found in the built world")
	elif cam.get_cull_mask_value(Main.FPV_HIDDEN_LAYER):
		fails += 1
		print("FAIL FPV camera does not exclude layer %d" % Main.FPV_HIDDEN_LAYER)

	# --- something must actually be on the hidden layer, or the mask is a no-op
	var hidden := _count_on_layer(main, Main.FPV_HIDDEN_LAYER)
	if hidden == 0:
		fails += 1
		print("FAIL nothing is on layer %d — the cull mask protects nothing"
				% Main.FPV_HIDDEN_LAYER)

	# --- and the ducts/props must NOT be hidden: a real cinewhoop feed shows
	# them, and hiding them would be a silent authenticity regression
	var visible_parts := _count_on_layer(main, 1)
	if visible_parts < 4:
		fails += 1
		print("FAIL expected ducts/motors/props visible to the FPV camera, "
				+ "found %d instances on layer 1" % visible_parts)

	print("[fpv_cull_test] %s (%d failure(s), %d hidden, %d visible)"
			% ["PASS" if fails == 0 else "FAIL", fails, hidden, visible_parts])
	quit(1 if fails > 0 else 0)


func _find_camera(n: Node) -> Camera3D:
	if n is Camera3D:
		return n
	for c in n.get_children():
		var r := _find_camera(c)
		if r != null:
			return r
	return null


func _count_on_layer(n: Node, layer: int) -> int:
	var count := 0
	if n is VisualInstance3D and n.get_layer_mask_value(layer):
		count += 1
	for c in n.get_children():
		count += _count_on_layer(c, layer)
	return count
