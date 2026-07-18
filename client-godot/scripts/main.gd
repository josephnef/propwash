# propwash Godot client (MIT): first flyable.
#
# Spawns propwash-core, drives it in lockstep (one PW_STATE_IN per physics
# frame), applies the returned pose to the drone, owns ground collision.
#
# The sim frame is y-up with +z forward and Unity-style handedness (SimITL
# physics lineage); Godot is y-up right-handed with -z forward. Conversion:
# mirror the z axis (positions/velocities negate z; quaternions negate x,y).
#
# Controls. RC source, in priority order:
#   1. an RC handset (RadioMaster / EdgeTX in USB Joystick mode). On Linux the
#      core reads it directly; on macOS (no kernel js API) the client reads it
#      here via Godot's cross-platform Input and sends it as rc. Either way it
#      wins — the server prefers a live handset over these client keys.
#   2. keyboard, when no handset is connected:
#        arrows    right stick (roll / pitch)
#        W/S       throttle up / down       A/D  yaw
#        E         toggle ARM (ch5)         Q    toggle ANGLE (ch6, on by default)
#        R         reset
extends Node3D

# explicit preload: the class_name global cache doesn't exist on a first
# headless run (no .godot import cache yet)
const PwProtocol = preload("res://scripts/protocol.gd")

const CORE_PORT := 9100
const REST_H := 0.12   # drone body height when resting on the ground (gear)

var _udp := PacketPeerUDP.new()
var _core_pid := -1
var _frame_id := 0

var _drone: Node3D
var _hud: Label
var _osd: Label       # 16x30 Betaflight OSD overlay, monospace, centered

# client-owned pose state (fed back to the core each frame)
var _pos := Vector3(0, REST_H, 0)   # start resting on the pad, not clipping ground
var _rot := Quaternion.IDENTITY   # sim frame
var _angvel := Vector3.ZERO       # sim frame
var _linvel := Vector3.ZERO       # sim frame

# keyboard rc state
var _rc := [0.0, 0.0, -1.0, 0.0, -1.0, 1.0, -1.0, -1.0]
var _throttle := -1.0
var _armed_sw := false
var _angle_sw := true

# RC handset (joystick) state. EdgeTX "Joystick (Channels)" mode presents
# CH1-8 as HID axes 0-7 = AETR + switches, which Godot normalises to -1..1 —
# the same convention the firmware's rcData wants, so axes pass straight to
# channels. Layouts vary, so the axis->channel map and per-channel inversion
# are overridable without recompiling:
#   PROPWASH_JS_MAP="0,1,2,3,4,5,6,7"  (rc channel i reads joy axis map[i])
#   PROPWASH_JS_INVERT="2"             (comma list of channels to negate)
var _js_dev := -1
var _js_logged := false
var _js_axis_map := [0, 1, 2, 3, 4, 5, 6, 7]
var _js_invert := [false, false, false, false, false, false, false, false]

var _last_out := {}
var _await_warned := false
var _got_first_reply := false
var _boot_elapsed := 0.0      # wall time since start, for the no-reply grace

# autotest (PROPWASH_AUTOTEST=1): no keyboard/radio — arm at t=5.2 s, run
# the reference hover controller, assert altitude/tilt, exit 0/1. This is
# how the GDScript client itself is verified headless in CI.
var _autotest := false
var _at_time := 0.0
var _at_alts: Array[float] = []
var _at_armed_seen := false
var _osd_glyphs := 0

# demo flight (PROPWASH_DEMO=acro): fly forward through the gates in ACRO
# mode (self-level OFF) using a cascaded PD controller — attitude error and
# body rate drive the acro rate sticks. Signs tuned empirically (Godot<->
# firmware conventions): +14deg setpoint flew -z (into the gates) once the
# pitch sign was flipped; ramping + rate damping keep it from tumbling.
var _demo := ""
const ACRO_KP := 1.4          # attitude(rad) -> rate stick
const ACRO_KD := 0.10         # body rate(rad/s) damping
const ACRO_PITCH_SIGN := 1.0  # loop stability (regulation was stable at +1)
const ACRO_ROLL_SIGN := 1.0
const GATE_ALT := 1.0         # gate centre height
var _des_pitch := 0.0         # ramped setpoint (rad)

# screenshot capture (PROPWASH_SHOTS=/dir): save frames at set times
var _shot_dir := ""
var _shots_taken := {}
var _shot_times := [3.0, 8.0, 12.0, 16.0]

# multi-monitor: with a second screen attached, fly fullscreen on it and leave
# the primary free for the Configurator / CLI / logs — the usual bench setup.
# Borderless (MODE_FULLSCREEN), not macOS's native fullscreen Space, which
# would shunt the sim onto its own Space and animate on every focus change.
#   PROPWASH_SCREEN=off   stay windowed on the primary (old behaviour)
#   PROPWASH_SCREEN=2     force a screen index (0-based)
const SCREEN_ENV := "PROPWASH_SCREEN"


func _ready() -> void:
	_autotest = OS.get_environment("PROPWASH_AUTOTEST") == "1"
	_demo = OS.get_environment("PROPWASH_DEMO")
	_shot_dir = OS.get_environment("PROPWASH_SHOTS")
	if _demo == "acro":
		# capture through the whole gate run
		_shot_times = [4.0, 7.0, 9.0, 11.0, 13.0, 15.0, 17.0, 19.0]
	_parse_js_env()
	_setup_display()
	_build_world()
	_spawn_core()
	_udp.connect_to_host("127.0.0.1", CORE_PORT)


func _exit_tree() -> void:
	if _core_pid > 0:
		OS.kill(_core_pid)


func _setup_display() -> void:
	# --headless has no real DisplayServer, and the autotests assert on stdout,
	# not pixels — never grab a screen out from under them.
	if DisplayServer.get_name() == "headless" or _autotest:
		return
	var want := OS.get_environment(SCREEN_ENV)
	if want == "off":
		return
	var n := DisplayServer.get_screen_count()
	var screen := -1
	if want.is_valid_int():
		screen = int(want)
		if screen < 0 or screen >= n:
			push_warning("%s=%s out of range (%d screen(s)) — staying windowed"
					% [SCREEN_ENV, want, n])
			return
	else:
		screen = _secondary_screen()
		if screen < 0:
			return   # single screen: windowed, as before
	var win := get_window()
	# order matters: move while still windowed, then go fullscreen. Setting the
	# mode first makes the move a no-op on some platforms — the window already
	# owns the old screen's fullscreen surface.
	win.current_screen = screen
	win.mode = Window.MODE_FULLSCREEN
	print("[pw][display] fullscreen on screen %d/%d %s" % [
			screen, n, DisplayServer.screen_get_size(screen)])


# First screen that isn't the primary, or -1 when only one is attached.
func _secondary_screen() -> int:
	var n := DisplayServer.get_screen_count()
	if n < 2:
		return -1
	var primary := DisplayServer.get_primary_screen()
	for i in n:
		if i != primary:
			return i
	return -1


func _spawn_core() -> void:
	var path := OS.get_environment("PROPWASH_CORE")
	if path.is_empty():
		path = ProjectSettings.globalize_path("res://") + "../build/propwash-core"
	if not FileAccess.file_exists(path):
		push_warning("propwash-core not found at %s — start it manually" % path)
		return
	var args := ["--port", str(CORE_PORT)]
	var eeprom := OS.get_environment("PROPWASH_EEPROM")
	if not eeprom.is_empty():
		args += ["--eeprom", eeprom]
	# autotest / demo drive RC from the script — a physically-connected Pocket
	# would otherwise override it (joystick has priority) and hijack the run
	if _autotest or _demo == "acro":
		args += ["--no-js"]
	_core_pid = OS.create_process(path, args)
	print("propwash-core pid ", _core_pid)


func _physics_process(delta: float) -> void:
	_boot_elapsed += delta
	if _demo == "acro":
		_update_acro_demo_rc(delta)
	elif _autotest:
		_update_autotest_rc(delta)
	else:
		_update_manual_rc(delta)

	# --- sim-frame pose (convert Godot -> sim: negate z / negate qx,qy)
	var sim_pos := Vector3(_pos.x, _pos.y, -_pos.z)
	var sim_rot := Quaternion(-_rot.x, -_rot.y, _rot.z, _rot.w)
	# angular velocity is a pseudovector: mirror(z) maps it to (-x, -y, +z)
	var sim_av := Vector3(-_angvel.x, -_angvel.y, _angvel.z)
	var sim_lv := Vector3(_linvel.x, _linvel.y, -_linvel.z)
	var contact := _pos.y <= REST_H + 0.01

	var pkt := PwProtocol.pack_state_in(_frame_id, delta, _rc,
			sim_pos, sim_rot, sim_av, sim_lv, 16.8, contact)
	_udp.put_packet(pkt)

	# --- lockstep wait (up to ~8 ms)
	var got := false
	for i in range(800):
		if _udp.get_available_packet_count() > 0:
			got = true
			break
		OS.delay_usec(10)
	if not got:
		# don't cry wolf during the core's ~1-2 s boot (joystick enumerate +
		# Betaflight init + UDP bind). Only warn once we've either seen a reply
		# before (a real mid-flight dropout) or waited out the startup grace.
		if not _await_warned and (_got_first_reply or _boot_elapsed > 4.0):
			push_warning("no reply from propwash-core (is it running on udp:%d?)" % CORE_PORT)
			_await_warned = true
		return
	_await_warned = false
	_got_first_reply = true

	var out := {}
	while _udp.get_available_packet_count() > 0:      # drain, keep newest
		var raw := _udp.get_packet()
		if raw.size() >= 8 and raw[5] == PwProtocol.PW_OSD:
			_update_osd(PwProtocol.unpack_osd(raw))
			continue
		var d := PwProtocol.unpack_state_out(raw)
		if not d.is_empty():
			out = d
	if out.is_empty():
		return
	_last_out = out
	_frame_id += 1

	# --- back to Godot frame
	var q: Quaternion = out.rotation
	_rot = Quaternion(-q.x, -q.y, q.z, q.w).normalized()
	var lv: Vector3 = out.linvel
	_linvel = Vector3(lv.x, lv.y, -lv.z)
	var av: Vector3 = out.angvel
	_angvel = Vector3(-av.x, -av.y, av.z)

	var armed: bool = out.get("armed", false)
	_pos += _linvel * delta

	# --- ground plane (client owns collision), body rests at REST_H
	if _pos.y <= REST_H:
		_pos.y = REST_H
		if armed:
			if _linvel.y < 0.0:
				_linvel.y = 0.0
			_angvel = Vector3.ZERO
		else:
			# disarmed on the pad: sit still and settle level, don't drift
			# with physics noise (a real quad just sits there)
			_linvel = Vector3.ZERO
			_angvel = Vector3.ZERO
			_rot = _rot.slerp(Quaternion.IDENTITY, 0.2)

	_drone.transform = Transform3D(Basis(_rot), _pos)
	_spin_props(delta)
	_update_hud()
	if _autotest:
		_autotest_check(delta)


# Manual flight: prefer a live RC handset, fall back to the keyboard. Checked
# every frame so hot-plugging the handset just works.
func _update_manual_rc(delta: float) -> void:
	_js_dev = _pick_joystick()
	if _js_dev >= 0:
		_update_joystick_rc()
	else:
		if _js_logged:                       # handset was unplugged
			_js_logged = false
		_update_keyboard_rc(delta)


# Pick an RC handset from the connected joysticks: prefer one that names itself
# EdgeTX/OpenTX/RadioMaster/Betaflight/Taranis, else the first joystick.
func _pick_joystick() -> int:
	var pads := Input.get_connected_joypads()
	if pads.is_empty():
		return -1
	for d in pads:
		var n := Input.get_joy_name(d).to_lower()
		if n.contains("edgetx") or n.contains("opentx") or n.contains("radiomaster") \
				or n.contains("betaflight") or n.contains("taranis") or n.contains("frsky"):
			return d
	return pads[0]


func _update_joystick_rc() -> void:
	if not _js_logged:
		var known := "" if Input.is_joy_known(_js_dev) else "  [generic HID]"
		print("[pw][client] RC handset: '%s' (dev %d)%s" % [
			Input.get_joy_name(_js_dev), _js_dev, known])
		print("[pw][client] axis->ch map %s  invert %s  (PROPWASH_JS_MAP / PROPWASH_JS_INVERT to change)" % [
			str(_js_axis_map), str(_js_invert)])
		_js_logged = true
	for ch in range(8):
		var ax: int = _js_axis_map[ch]
		var v := Input.get_joy_axis(_js_dev, ax)
		if _js_invert[ch]:
			v = -v
		_rc[ch] = clampf(v, -1.0, 1.0)


# Parse optional axis-map / inversion overrides for handsets whose USB layout
# differs from the default EdgeTX Classic (axes 0-7 = CH1-8).
func _parse_js_env() -> void:
	var m := OS.get_environment("PROPWASH_JS_MAP")
	if not m.is_empty():
		var parts := m.split(",", false)
		if parts.size() == 8:
			for i in range(8):
				_js_axis_map[i] = int(parts[i].strip_edges())
		else:
			push_warning("PROPWASH_JS_MAP needs 8 comma-separated axes; ignoring '%s'" % m)
	var inv := OS.get_environment("PROPWASH_JS_INVERT")
	if not inv.is_empty():
		for tok in inv.split(",", false):
			var ch := int(tok.strip_edges())
			if ch >= 0 and ch < 8:
				_js_invert[ch] = true


func _update_keyboard_rc(delta: float) -> void:
	_rc[0] = Input.get_axis("ui_left", "ui_right")           # roll
	_rc[1] = Input.get_axis("ui_down", "ui_up")              # pitch
	var thr_move := 0.0
	if Input.is_physical_key_pressed(KEY_W): thr_move += 1.0
	if Input.is_physical_key_pressed(KEY_S): thr_move -= 1.0
	_throttle = clampf(_throttle + thr_move * delta * 1.2, -1.0, 1.0)
	_rc[2] = _throttle
	var yaw := 0.0
	if Input.is_physical_key_pressed(KEY_D): yaw += 1.0
	if Input.is_physical_key_pressed(KEY_A): yaw -= 1.0
	_rc[3] = yaw
	_rc[4] = 1.0 if _armed_sw else -1.0
	_rc[5] = 1.0 if _angle_sw else -1.0


func _unhandled_key_input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed and not event.echo:
		match event.physical_keycode:
			KEY_E:
				_armed_sw = not _armed_sw
			KEY_Q:
				_angle_sw = not _angle_sw
			KEY_R:
				_udp.put_packet(PwProtocol.pack_command(PwProtocol.PW_CMD_RESET))
				_pos = Vector3(0, 0, 0)
				_rot = Quaternion.IDENTITY
				_linvel = Vector3.ZERO
				_angvel = Vector3.ZERO
				_throttle = -1.0
				_armed_sw = false


func _update_osd(lines: PackedStringArray) -> void:
	if lines.size() != 16:
		return
	if _osd:
		_osd.text = "\n".join(lines)
	if _autotest:
		for ln in lines:
			_osd_glyphs += ln.strip_edges().length()


func _update_hud() -> void:
	if _last_out.is_empty():
		return
	var o := _last_out
	var rc_line: String
	if _js_dev >= 0:
		# live channels help confirm the AETR+switch mapping / spot inversions
		rc_line = "RC '%s'  A%+.2f E%+.2f T%+.2f R%+.2f  5:%+.2f 6:%+.2f 7:%+.2f 8:%+.2f" % [
			Input.get_joy_name(_js_dev),
			_rc[0], _rc[1], _rc[2], _rc[3], _rc[4], _rc[5], _rc[6], _rc[7]]
	else:
		rc_line = "E arm | Q angle | R reset | WASD+arrows fly (keyboard)"
	_hud.text = "%s   alt %5.1f m   vbat %4.1f V   thr %4.1f\nrpm %5.0f %5.0f %5.0f %5.0f   dis 0x%x\n%s" % [
		"ARMED" if o.armed else "DISARMED", _pos.y, o.vbat, _throttle,
		o.motor_rpm[0], o.motor_rpm[1], o.motor_rpm[2], o.motor_rpm[3],
		o.arming_disable, rc_line]


# --------------------------------------------------------------- autotest
func _update_autotest_rc(delta: float) -> void:
	_rc[0] = 0.0
	_rc[1] = 0.0
	_rc[3] = 0.0
	_rc[5] = 1.0                                    # ANGLE on
	_rc[4] = 1.0 if _at_time >= 5.2 else -1.0       # ARM at 5.2 s
	if not _last_out.is_empty() and _last_out.armed:
		var u: float = -0.3 + 0.5 * (2.0 - _pos.y) - 0.4 * _linvel.y
		u = clampf(u, -1.0, 0.6)
		_throttle = clampf(u, _throttle - 2.0 * delta, _throttle + 2.0 * delta)
	else:
		_throttle = -1.0
	_rc[2] = _throttle


func _maybe_shoot() -> void:
	if _shot_dir.is_empty():
		return
	for at in _shot_times:
		var key := str(at)
		if _at_time >= at and not _shots_taken.has(key):
			_shots_taken[key] = true
			await RenderingServer.frame_post_draw
			var img := get_viewport().get_texture().get_image()
			var path := "%s/propwash_t%02d.png" % [_shot_dir, int(at)]
			img.save_png(path)
			print("[shot] ", path)


# Autonomous fly-through: ANGLE mode (self-level ON) so the firmware holds
# attitude; the demo just commands a forward lean (pitch stick = target
# angle) and an altitude-hold throttle to cruise the drone through the gates
# (forward = Godot -z). Reliable and reproducible; acro freestyle is meant to
# be hand-flown with the Pocket, not scripted open-loop.
func _update_acro_demo_rc(delta: float) -> void:
	_at_time += delta
	_maybe_shoot()

	var armed: bool = not _last_out.is_empty() and _last_out.armed
	if armed:
		_at_armed_seen = true

	# --- flight plan: forward-lean stick (angle mode), ramped so the climb
	# settles before cruising. Negative pitch stick leans forward (-z).
	var goal_lean := 0.0
	var target_alt := 1.6
	if _at_time < 5.2:
		pass                                  # on the pad, wait to arm
	elif _at_time < 8.5:
		goal_lean = 0.0                       # climb + settle, hover level
	elif _at_time < 18.0:
		goal_lean = 0.32                      # cruise forward (-z) through gates
	else:
		goal_lean = -0.15                     # flare/brake

	_des_pitch = move_toward(_des_pitch, goal_lean, 0.8 * delta)

	# --- altitude hold
	var alt := _pos.y
	var vy := _linvel.y
	var u := -0.2 + 0.55 * (target_alt - alt) - 0.35 * vy
	if armed:
		_throttle = clampf(u, -1.0, 0.7)
	else:
		_throttle = -1.0

	_rc[0] = 0.0                              # roll centred
	_rc[1] = _des_pitch                       # pitch = target lean (angle mode)
	_rc[2] = _throttle
	_rc[3] = 0.0                              # no yaw
	_rc[4] = 1.0 if _at_time >= 5.2 else -1.0 # ARM ch5
	_rc[5] = 1.0                              # ANGLE ch6 ON (self-level)

	# track that it stayed airborne through the cruise (gates are y~1)
	if _at_time >= 9.0 and _at_time <= 17.0:
		_at_alts.append(_pos.y)

	if _at_time >= 22.0:
		var min_alt := 1e9
		for a in _at_alts:
			min_alt = minf(min_alt, a)
		# forward = -z; gates at z = -6/-14/-22. Pass = flew past the last
		# gate while staying airborne (never near the ground) and near centre.
		var flew_through := _pos.z < -30.0
		var stayed_up := _at_alts.size() > 0 and min_alt > 0.7
		var on_line := absf(_pos.x) < 3.0
		var ok := _at_armed_seen and flew_through and stayed_up and on_line
		print("[demo] fly-through: end=%s min_alt=%.2f armed=%s" % [str(_pos), min_alt, str(_at_armed_seen)])
		print("[demo] %s" % ("PASS" if ok else "FAIL"))
		get_tree().quit(0 if ok else 1)


func _autotest_check(delta: float) -> void:
	_at_time += delta
	_maybe_shoot()
	if not _last_out.is_empty() and _last_out.armed:
		_at_armed_seen = true
	if _at_time >= 13.0:
		_at_alts.append(_pos.y)
	if fmod(_at_time, 1.0) < delta:
		print("[autotest] t=%.1f alt=%.2f armed=%s dis=0x%x" % [
			_at_time, _pos.y, str(not _last_out.is_empty() and _last_out.armed),
			_last_out.arming_disable if not _last_out.is_empty() else -1])
	if _at_time >= 20.0:
		var lo := 1e9
		var hi := -1e9
		for a in _at_alts:
			lo = minf(lo, a)
			hi = maxf(hi, a)
		var ok := _at_armed_seen and _at_alts.size() > 0 and lo > 1.5 and hi < 2.5 and _osd_glyphs > 0
		print("[autotest] hover band [%.2f, %.2f] armed_seen=%s osd_glyphs=%d" % [lo, hi, str(_at_armed_seen), _osd_glyphs])
		print("[autotest] %s" % ("PASS" if ok else "FAIL"))
		get_tree().quit(0 if ok else 1)


var _props: Array[Node3D] = []   # 4 prop meshes, spun by motor RPM
var _prop_angle := 0.0


# Spin the props from the firmware's motor RPM (PW_STATE_OUT.motor_rpm) and
# fade the blur discs in with RPM. At real RPM the blades strobe, which reads
# as a translucent disc — exactly the real O3 look.
func _spin_props(delta: float) -> void:
	if _props.is_empty():
		return
	var rpm := [0.0, 0.0, 0.0, 0.0]
	if not _last_out.is_empty() and _last_out.has("motor_rpm"):
		rpm = _last_out.motor_rpm
	# a single shared visual rate (scaled down; true RPM would just alias)
	var avg: float = (rpm[0] + rpm[1] + rpm[2] + rpm[3]) / 4.0
	_prop_angle += (avg / 60.0) * TAU * delta * 0.05   # 5% so blades stay visible
	for i in range(_props.size()):
		_props[i].rotation.y = _prop_angle * (1.0 if (_props[i].scale.x > 0) else -1.0)


# A procedural CineLog35-style 3.5" ducted cinewhoop: centre plate, 4 ducts
# with motors + props, a top battery, and the O3 camera pod up front. Forward
# is -z. Motor layout matches the physics profile (+/-54 mm), ~89 mm props.
func _build_drone_model(root: Node3D) -> void:
	var carbon := StandardMaterial3D.new()
	carbon.albedo_color = Color(0.09, 0.09, 0.11)
	carbon.metallic = 0.3
	carbon.roughness = 0.5
	var duct_mat := StandardMaterial3D.new()
	duct_mat.albedo_color = Color(0.06, 0.06, 0.07)
	duct_mat.roughness = 0.7

	# centre frame plate
	var plate := MeshInstance3D.new()
	var pm := BoxMesh.new()
	pm.size = Vector3(0.075, 0.008, 0.11)
	pm.material = carbon
	plate.mesh = pm
	root.add_child(plate)

	# battery on top
	var batt := MeshInstance3D.new()
	var btm := BoxMesh.new()
	btm.size = Vector3(0.035, 0.022, 0.075)
	var bmat := StandardMaterial3D.new()
	bmat.albedo_color = Color(0.55, 0.15, 0.15)
	btm.material = bmat
	batt.mesh = btm
	batt.position = Vector3(0, 0.02, 0.005)
	root.add_child(batt)

	# O3 camera pod up front (what the FPV cam looks out of)
	var pod := MeshInstance3D.new()
	var podm := BoxMesh.new()
	podm.size = Vector3(0.021, 0.028, 0.02)
	podm.material = carbon
	pod.mesh = podm
	pod.position = Vector3(0, 0.038, -0.03)
	pod.rotation_degrees = Vector3(25, 0, 0)
	root.add_child(pod)

	# 4 ducts + motors + props at the motor positions (Godot: forward = -z)
	var motors := [
		Vector3( 0.054, 0.0, -0.054),  # front-right
		Vector3(-0.054, 0.0, -0.054),  # front-left
		Vector3( 0.054, 0.0,  0.054),  # rear-right
		Vector3(-0.054, 0.0,  0.054),  # rear-left
	]
	var spin_dir := [1.0, -1.0, -1.0, 1.0]
	for i in range(4):
		var p: Vector3 = motors[i]

		# duct ring (torus lip) — the hallmark of a ducted whoop
		var duct := MeshInstance3D.new()
		var tm := TorusMesh.new()
		tm.inner_radius = 0.044
		tm.outer_radius = 0.05
		tm.material = duct_mat
		duct.mesh = tm
		duct.position = p + Vector3(0, 0.006, 0)
		root.add_child(duct)

		# motor bell
		var motor := MeshInstance3D.new()
		var mm := CylinderMesh.new()
		mm.top_radius = 0.012
		mm.bottom_radius = 0.013
		mm.height = 0.016
		mm.material = carbon
		motor.mesh = mm
		motor.position = p + Vector3(0, 0.004, 0)
		root.add_child(motor)

		# prop — a translucent disc (spinning blur) with 3 faint blades
		var prop := Node3D.new()
		prop.position = p + Vector3(0, 0.014, 0)
		prop.scale = Vector3(spin_dir[i], 1, 1)  # cheap CW/CCW hint
		var disc := MeshInstance3D.new()
		var dm := CylinderMesh.new()
		dm.top_radius = 0.043
		dm.bottom_radius = 0.043
		dm.height = 0.001
		var dmat := StandardMaterial3D.new()
		dmat.albedo_color = Color(0.7, 0.7, 0.75, 0.12)
		dmat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
		dm.material = dmat
		disc.mesh = dm
		prop.add_child(disc)
		for b in range(3):
			var blade := MeshInstance3D.new()
			var blm := BoxMesh.new()
			blm.size = Vector3(0.084, 0.004, 0.01)
			var blmat := StandardMaterial3D.new()
			blmat.albedo_color = Color(0.15, 0.15, 0.17)
			blm.material = blmat
			blade.mesh = blm
			blade.rotation_degrees = Vector3(0, b * 120.0, 0)
			prop.add_child(blade)
		root.add_child(prop)
		_props.append(prop)


# ------------------------------------------------------------------ world
func _add_box(pos: Vector3, size: Vector3, color: Color) -> void:
	var mi := MeshInstance3D.new()
	var mesh := BoxMesh.new()
	mesh.size = size
	var mat := StandardMaterial3D.new()
	mat.albedo_color = color
	mesh.material = mat
	mi.mesh = mesh
	mi.position = pos
	add_child(mi)


func _build_world() -> void:
	# light + sky
	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-55, -30, 0)
	sun.shadow_enabled = true
	add_child(sun)

	var env := WorldEnvironment.new()
	var e := Environment.new()
	var sky := Sky.new()
	sky.sky_material = ProceduralSkyMaterial.new()
	e.background_mode = Environment.BG_SKY
	e.sky = sky
	env.environment = e
	add_child(env)

	# ground: 60x60 checkered
	var ground := MeshInstance3D.new()
	var plane := PlaneMesh.new()
	plane.size = Vector2(60, 60)
	var gmat := StandardMaterial3D.new()
	gmat.albedo_color = Color(0.25, 0.45, 0.2)
	plane.material = gmat
	ground.mesh = plane
	add_child(ground)

	# grid lines for motion perception
	for i in range(-30, 31, 2):
		_add_box(Vector3(i, 0.01, 0), Vector3(0.03, 0.02, 60), Color(0.35, 0.55, 0.3))
		_add_box(Vector3(0, 0.01, i), Vector3(60, 0.02, 0.03), Color(0.35, 0.55, 0.3))

	# a few gates
	for i in range(3):
		var z := -6.0 - i * 8.0
		_add_box(Vector3(-1.2, 1.0, z), Vector3(0.15, 2.0, 0.15), Color(0.9, 0.4, 0.1))
		_add_box(Vector3(1.2, 1.0, z), Vector3(0.15, 2.0, 0.15), Color(0.9, 0.4, 0.1))
		_add_box(Vector3(0, 2.05, z), Vector3(2.55, 0.15, 0.15), Color(0.9, 0.4, 0.1))

	# drone + FPV camera
	_drone = Node3D.new()
	add_child(_drone)
	_build_drone_model(_drone)

	# FPV camera where the DJI O3 sits on a CineLog35: front-top of the frame,
	# looking forward (-z) with ~25 deg uptilt. The front ducts + props are
	# just below the view, exactly like real cinewhoop footage.
	var cam := Camera3D.new()
	cam.fov = 105
	cam.near = 0.005
	cam.position = Vector3(0, 0.045, -0.02)
	cam.rotation_degrees = Vector3(25, 0, 0)
	_drone.add_child(cam)
	cam.current = true

	# HUD
	var ui := CanvasLayer.new()
	add_child(ui)
	_hud = Label.new()
	_hud.position = Vector2(12, 8)
	_hud.add_theme_font_size_override("font_size", 15)
	ui.add_child(_hud)
	_hud.text = "connecting to propwash-core..."

	# OSD overlay — the real Betaflight 16x30 character grid, centered like
	# FPV goggles. Uses a monospace font so columns line up.
	_osd = Label.new()
	_osd.set_anchors_preset(Control.PRESET_CENTER)
	_osd.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	_osd.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	_osd.add_theme_font_override("font", ThemeDB.fallback_font)
	_osd.add_theme_font_size_override("font_size", 20)
	_osd.add_theme_color_override("font_color", Color(1, 1, 1))
	_osd.add_theme_color_override("font_outline_color", Color(0, 0, 0))
	_osd.add_theme_constant_override("outline_size", 4)
	ui.add_child(_osd)
