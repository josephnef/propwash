# A tree trunk must be solid where it is DRAWN, at every height.
#
# Regression test for a real bug: trunk colliders were capsules, and a
# capsule's bottom hemisphere tapers to a point at ground level — so the
# collider shrank to nothing exactly where a landed or crashed quad sits. The
# hull could get 5 cm INSIDE the drawn trunk at y = 0.02 m while contact at
# y = 2 m was correct, which is how a quad ends up visually wedged into a tree
# the simulator never told it it had hit. Trunks are cylinders now.
#
#   godot --headless --path client-godot --script res://tests/tree_collider_probe.gd
extends SceneTree

const Main = preload("res://scripts/main.gd")
const PwWorld = preload("res://scripts/world.gd")
const PwProtocol = preload("res://scripts/protocol.gd")

var _main: Node3D
var _frames := 0


func _initialize() -> void:
	OS.set_environment("PROPWASH_SCENE", "park")
	OS.set_environment("PROPWASH_CORE", "/nonexistent/core")
	_main = Node3D.new()
	_main.set_script(Main)
	root.add_child(_main)


func _process(_d: float) -> bool:
	_frames += 1
	if _frames < 6:            # static bodies need a few frames to register
		return false
	_check()
	return true


# Sweep the hull toward a trunk at a given height and report the first x at
# which the client reports a SURF_TREE contact.
func _first_contact_x(tree_x: float, tree_z: float, height: float) -> float:
	var x := tree_x + 3.0
	while x > tree_x - 1.0:
		_main._rot = Quaternion.IDENTITY
		_main._pos = Vector3(x, height, tree_z)
		_main._prev_pos = _main._pos
		_main._detect_contacts()
		for c in _main._pending_contacts:
			if c.surface == PwProtocol.SURF_TREE:
				return x - tree_x        # gap from the trunk axis
		x -= 0.005
	return -1.0


func _check() -> void:
	# NEAR_TREES entry: x, z, height, conifer
	var t: Array = PwWorld.NEAR_TREES[4]
	var tx: float = t[0]
	var tz: float = t[1]
	var h: float = t[2]
	var conifer: bool = t[3]
	var wide: float = h * (0.30 if conifer else 0.62)

	var visual_r: float = 0.075 * wide          # trunk mesh bottom radius
	var collider_r: float = clampf(0.09 * wide, 0.06, 0.5)

	print("[tree] tree at (%.1f, %.1f) h=%.1f wide=%.2f" % [tx, tz, h, wide])
	print("[tree] trunk DRAWN radius at the base   %.3f m" % visual_r)
	print("[tree] capsule collider radius          %.3f m" % collider_r)
	print("[tree] capsule spans y 0..%.2f, so its bottom hemisphere occupies"
			% (0.9 * h) + " y 0..%.3f" % collider_r)

	var fails := 0
	for y in [0.02, 0.05, 0.10, 0.20, 0.40, 1.00, 2.00]:
		var gap := _first_contact_x(tx, tz, y)
		# how far INSIDE the drawn trunk can the hull centre get before the
		# simulator notices anything at all?
		var overlap := visual_r - gap if gap >= 0.0 else INF
		var flag := ""
		if gap < 0.0:
			flag = "  <-- NO CONTACT ANYWHERE"
			fails += 1
		elif overlap > 0.02:
			flag = "  <-- %.0f cm INSIDE the drawn trunk" % (overlap * 100.0)
			fails += 1
		print("[tree] hull at y=%.2f m: first contact at %.3f m from the axis%s"
				% [y, gap, flag])

	print("[tree] %s" % ("PASS" if fails == 0 else "FAIL (%d heights)" % fails))
	quit(1 if fails > 0 else 0)
