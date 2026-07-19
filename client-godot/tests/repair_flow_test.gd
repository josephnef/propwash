# Regression test for the T-repair flow, headless with a REAL core.
#
# Pins the two ways "press T to repair" has failed:
#  - the armed gate silently ignoring the key (now it prints + the banner
#    says "disarm first");
#  - the one-deep-pipeline stale echo: the reply consumed right after a pose
#    override answers the PRE-override packet, and applying its pose forked
#    the trajectory into two lineages flickering against each other forever.
#
# Flow: boot -> pin RC disarmed via PW_RC_OVERRIDE (immune to any connected
# handset; the core itself is spawned --no-js via PROPWASH_NO_JS) -> teleport
# to 10 m -> free-fall -> CRASHED latch -> settle -> _repair_in_place() ->
# assert: damage cleared, latch cleared, upright, and NO pose flicker.
#
#   PROPWASH_CORE=... PROPWASH_NO_JS=1 \
#     godot --headless --path client-godot --script res://tests/repair_flow_test.gd
extends SceneTree

const Main = preload("res://scripts/main.gd")
const PwProtocol = preload("res://scripts/protocol.gd")

var _main: Node3D
var _udp := PacketPeerUDP.new()
var _phase := "boot"
var _t := 0.0
var _mark := 0.0
var _flips := 0
var _prev_up := 1.0
var _min_up_after := 1.0


func _initialize() -> void:
	_main = Node3D.new()
	_main.set_script(Main)
	root.add_child(_main)
	var port := 9100
	var port_env := OS.get_environment("PROPWASH_PORT")
	if port_env.is_valid_int():
		port = int(port_env)
	_udp.connect_to_host("127.0.0.1", port)


func _process(delta: float) -> bool:
	_t += delta
	if _t > 60.0:
		print("[repair_flow] FAIL: timeout in phase ", _phase)
		quit(1)
		return true

	var out: Dictionary = _main._last_out
	match _phase:
		"boot":
			if not out.is_empty():
				# pin RC: sticks safe, ARM off, ANGLE on — beats client rc
				_udp.put_packet(PwProtocol.pack_rc_override(
						[0.0, 0.0, -1.0, 0.0, -1.0, 1.0, -1.0, -1.0]))
				_phase = "lift"
		"lift":
			_main._pos = Vector3(0, 10.0, 0)
			_main._prev_pos = _main._pos
			_main._drone.reset_physics_interpolation()
			_phase = "fall"
		"fall":
			if not out.is_empty() and out.get("crash_flags", 0) & 1:
				print("[repair_flow] crashed at t=%.1f dmg=%s" % [
						_t, str(out.get("prop_damage", []))])
				_mark = _t
				_phase = "settle"
		"settle":
			if _t > _mark + 1.2:
				_main._repair_in_place()
				_mark = _t
				_prev_up = (Basis(_main._rot) * Vector3.UP).y
				_flips = 0
				_min_up_after = 1.0
				_phase = "verify"
		"verify":
			# any sign alternation of body-up = the stale-echo pose fork
			var up := (Basis(_main._rot) * Vector3.UP).y
			if up * _prev_up < 0.0:
				_flips += 1
			_prev_up = up
			if _t > _mark + 0.3:   # give the pop a moment to land
				_min_up_after = minf(_min_up_after, up)
			if _t > _mark + 1.5:
				var dmg: Array = out.get("prop_damage", [1.0])
				var flags: int = out.get("crash_flags", -1)
				var max_dmg := 0.0
				for d in dmg:
					max_dmg = maxf(max_dmg, d)
				var ok := flags == 0 and max_dmg == 0.0 \
						and _min_up_after > 0.9 and _flips == 0
				print("[repair_flow] flags=%d max_dmg=%.3f up_min=%.2f flips=%d" % [
						flags, max_dmg, _min_up_after, _flips])
				print("[repair_flow] %s" % ("PASS" if ok else "FAIL"))
				quit(0 if ok else 1)
				return true
	return false
