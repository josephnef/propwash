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

	# --- check the FPV camera in the world built during _initialize.
	#
	# BY NAME, not "the first Camera3D a depth-first walk finds". That was an
	# identity only while there was exactly one camera; the demo director adds a
	# chase and an LOS camera, and both deliberately DO render the hidden layer,
	# so a traversal-order lookup could quietly start asserting against a camera
	# that is supposed to fail this check.
	var cam: Camera3D = main.find_child("FpvCamera", true, false) as Camera3D
	if cam == null:
		fails += 1
		print("FAIL no FpvCamera in the built world")
	elif cam.get_cull_mask_value(Main.FPV_HIDDEN_LAYER):
		fails += 1
		print("FAIL FPV camera does not exclude layer %d" % Main.FPV_HIDDEN_LAYER)

	# --- and the external cameras must exist and must NOT exclude it: seeing
	# the battery and camera pod from outside is the whole point of them
	for ext in ["ChaseCamera", "LosCamera"]:
		var e: Camera3D = main.find_child(ext, true, false) as Camera3D
		if e == null:
			fails += 1
			print("FAIL no %s in the built world" % ext)
		elif not e.get_cull_mask_value(Main.FPV_HIDDEN_LAYER):
			fails += 1
			print("FAIL %s excludes layer %d — it should show the whole airframe"
					% [ext, Main.FPV_HIDDEN_LAYER])

	# --- something must actually be on the hidden layer, or the mask is a no-op
	var hidden := _count_on_layer(main, Main.FPV_HIDDEN_LAYER)
	if hidden == 0:
		fails += 1
		print("FAIL nothing is on layer %d — the cull mask protects nothing"
				% Main.FPV_HIDDEN_LAYER)

	# --- and the ducts/props must NOT be hidden: a real cinewhoop feed shows
	# them, and hiding them would be a silent authenticity regression.
	#
	# Counted over the DRONE subtree, not the whole scene: the world (ground,
	# treeline, gates) puts dozens of instances on layer 1, so a scene-wide
	# count passed even when nothing on the airframe was visible at all.
	var drone: Node3D = main._drone
	var visible_parts := 0
	if drone == null:
		fails += 1
		print("FAIL the world has no drone node")
	else:
		visible_parts = _count_on_layer(drone, 1)
		if visible_parts < 4:
			fails += 1
			print("FAIL expected ducts/motors/props visible to the FPV camera, "
					+ "found %d airframe instances on layer 1" % visible_parts)

	# --- the model is generated from model/cinelog35_v3.scad and bound BY NAME
	# (main.gd MODEL_PROP_NODES / MODEL_FPV_HIDDEN). A regenerated GLB that
	# renamed or dropped a node would otherwise fail silently: props would stop
	# spinning, or interior parts would appear in the FPV feed.
	if drone != null:
		for name in Main.MODEL_PROP_NODES + Main.MODEL_FPV_HIDDEN:
			if drone.find_child(name, true, false) == null:
				fails += 1
				print("FAIL the drone model has no %s node — did the GLB get "
						% name + "regenerated with different node names?")

	# --- props must be bound, in the right count, and spinnable
	if main._props.size() != 4:
		fails += 1
		print("FAIL expected 4 bound props, got %d" % main._props.size())
	else:
		for i in 4:
			if main._props[i].name != Main.MODEL_PROP_NODES[i]:
				fails += 1
				print("FAIL prop %d is %s, expected %s — the prop order must "
						% [i, main._props[i].name, Main.MODEL_PROP_NODES[i]]
						+ "match MOTOR_OFFSETS (FR, FL, RR, RL)")

	# --- the airframe must actually BE in the FPV frame.
	#
	# Layer bits are not visibility. Every check above passes with the drone
	# entirely off screen, because they only ask which layer a mesh is on, never
	# whether the camera can see it — and a centimetre of camera height is all it
	# takes to drop a 21 mm-tall airframe out of the view.
	#
	# So project the hull corners and require real presence in the lower band:
	# some corners on screen, and the topmost no higher than mid frame (it must
	# stay BELOW the horizon — this is a chin-mounted FPV cam, not a chase view).
	# Anchored to PropGuards specifically, not to a count over every airframe
	# mesh: the duct cage IS what a cinewhoop pilot sees, and a total across all
	# meshes moves whenever the model gains or loses a part, which makes it fail
	# for reasons that have nothing to do with visibility.
	var guards := drone.find_child("PropGuards", true, false) as MeshInstance3D \
			if drone != null else null
	if cam != null and guards != null:
		var vp := cam.get_viewport().get_visible_rect().size
		_project_hull(guards, cam, vp)
		if _proj_in < 8:
			fails += 1
			print("FAIL only %d of %d sampled duct-cage vertices land in the FPV "
					% [_proj_in, _proj_total] + "frame — the drone is not on screen")
		elif _proj_top < vp.y * 0.5:
			fails += 1
			print("FAIL duct cage reaches %.0f%% down the frame — it should stay "
					% (100.0 * _proj_top / vp.y) + "in the lower band")
		else:
			print("[fpv_cull_test] duct cage: %d/%d sampled vertices on screen, "
					% [_proj_in, _proj_total]
					+ "topmost %.0f%% down" % (100.0 * _proj_top / vp.y))
	elif guards == null:
		fails += 1
		print("FAIL the drone model has no PropGuards node")

	print("[fpv_cull_test] %s (%d failure(s), %d hidden, %d visible)"
			% ["PASS" if fails == 0 else "FAIL", fails, hidden, visible_parts])
	quit(1 if fails > 0 else 0)


var _proj_in := 0
var _proj_total := 0
var _proj_top := INF

# Sample every Nth vertex of the cage. Deliberately real vertices rather than
# the AABB: the bounding box's corners are its extreme diagonals, which sit out
# past the guards entirely, so a corner test reports "off screen" while the
# front rims are plainly in the feed.
const GUARD_SAMPLE_STRIDE := 97   # coprime with the ring tessellation


func _project_hull(mi: MeshInstance3D, cam: Camera3D, vp: Vector2) -> void:
	var verts: PackedVector3Array = mi.mesh.surface_get_arrays(0)[Mesh.ARRAY_VERTEX]
	var i := 0
	while i < verts.size():
		var world: Vector3 = mi.global_transform * verts[i]
		_proj_total += 1
		i += GUARD_SAMPLE_STRIDE
		if cam.is_position_behind(world):
			continue
		var sp := cam.unproject_position(world)
		if sp.x >= 0 and sp.x < vp.x and sp.y >= 0 and sp.y < vp.y:
			_proj_in += 1
			_proj_top = minf(_proj_top, sp.y)


func _count_on_layer(n: Node, layer: int) -> int:
	var count := 0
	if n is VisualInstance3D and n.get_layer_mask_value(layer):
		count += 1
	for c in n.get_children():
		count += _count_on_layer(c, layer)
	return count
