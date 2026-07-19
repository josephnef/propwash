# Headless test for the client's collision *detection* (the core owns the
# response): the 5-sphere hull against the analytic ground plane and the
# engine-query obstacle path (gate tubes with pw_surface metadata), plus the
# Godot->sim manifold conversion rules.
#
# Like fpv_cull_test, this builds the real procedural world but must never
# spawn a core (PROPWASH_CORE points nowhere).
#
#   godot --headless --path client-godot --script res://tests/collision_test.gd
extends SceneTree

const Main = preload("res://scripts/main.gd")
const PwProtocol = preload("res://scripts/protocol.gd")

var _main: Node3D
var _frames := 0


func _initialize() -> void:
	_main = Node3D.new()
	_main.set_script(Main)
	root.add_child(_main)


func _process(_d: float) -> bool:
	_frames += 1
	# static bodies need a couple of physics frames to register in the space
	if _frames < 5:
		return false
	_check()
	return true


func _check() -> void:
	var fails := 0
	var main: Node3D = _main

	# --- A: resting pose slightly sunk into the ground -> ground contacts
	main._rot = Quaternion.IDENTITY
	main._pos = Vector3(0, Main.HULL_REST_H - 0.002, 0)
	main._prev_pos = main._pos
	main._detect_contacts()
	var ground: Array = main._pending_contacts
	if ground.size() < 4:
		fails += 1
		print("FAIL ground rest: expected >=4 hull contacts, got %d" % ground.size())
	for c in ground:
		if c.surface != PwProtocol.SURF_GROUND:
			fails += 1
			print("FAIL ground rest: surface %d != SURF_GROUND" % c.surface)
			break
		if not c.normal_world.is_equal_approx(Vector3.UP):
			fails += 1
			print("FAIL ground rest: normal %s != +y" % str(c.normal_world))
			break
	# 2 mm penetration is under the 4 mm slop: depenetration must NOT fire
	if absf(main._pos.y - (Main.HULL_REST_H - 0.002)) > 1e-6:
		fails += 1
		print("FAIL ground rest: depenetration fired inside the slop (y=%f)" % main._pos.y)

	# --- B: deep penetration -> depenetrate up to the slop residual
	main._pos = Vector3(0, Main.HULL_REST_H - 0.02, 0)
	main._prev_pos = main._pos
	main._detect_contacts()
	var rest_y: float = main._pos.y
	if absf(rest_y - (Main.HULL_REST_H - Main.CONTACT_SLOP)) > 1e-4:
		fails += 1
		print("FAIL depenetration: y=%f, want %f" % [
				rest_y, Main.HULL_REST_H - Main.CONTACT_SLOP])

	# --- C: overlap the first gate's top bar (its centre point is invariant
	# under the gate's random yaw): engine query must find a SURF_GATE contact
	main._pos = Vector3(0, 2.05, -6.0)
	main._prev_pos = main._pos
	main._detect_contacts()
	var gate_hits := 0
	for c in main._pending_contacts:
		if c.surface == PwProtocol.SURF_GATE:
			gate_hits += 1
			if absf(c.normal_world.length() - 1.0) > 1e-3:
				fails += 1
				print("FAIL gate contact: non-unit normal %s" % str(c.normal_world))
			if c.depth < 0.0 or c.depth > 0.06:
				fails += 1
				print("FAIL gate contact: depth %f out of range" % c.depth)
			# the contact point must be on the hull, not somewhere in the world
			if c.point_body.length() > 0.15:
				fails += 1
				print("FAIL gate contact: point_body %s not on the hull" % str(c.point_body))
	if gate_hits == 0:
		fails += 1
		print("FAIL no SURF_GATE contact when overlapping the top bar")

	# --- D: sim-frame conversion — a contact detected forward of the quad
	# (Godot -z) must convert to +z in the sim frame. Overlap the bar with the
	# hull just behind it so the bar is forward, and check the sign flip.
	main._pos = Vector3(0, 2.05, -5.93)   # bar axis ~7 cm forward (-z side)
	main._prev_pos = main._pos
	main._detect_contacts()
	for c in main._pending_contacts:
		if c.surface == PwProtocol.SURF_GATE:
			# godot z of the contact point is negative (forward); sim z must
			# be positive after the mirror
			if c.point_body.z < 0.0:
				fails += 1
				print("FAIL handedness: forward contact has sim z=%f (want > 0)"
						% c.point_body.z)
			break

	print("[collision_test] %s (%d failure(s), %d ground contacts, %d gate hits)"
			% ["PASS" if fails == 0 else "FAIL", fails, ground.size(), gate_hits])
	quit(1 if fails > 0 else 0)
