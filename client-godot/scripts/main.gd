# propwash Godot client (MIT): first flyable.
#
# Spawns propwash-core, drives it in lockstep (one PW_STATE_IN per physics
# frame), applies the returned pose to the drone, owns ground collision.
#
# The sim frame is y-up with +z forward and Unity-style handedness (SimITL
# physics lineage); Godot is y-up right-handed with -z forward. Conversion:
# mirror the z axis (positions/velocities negate z; quaternions negate x,y).
#
# Controls (only used when the core has NO joystick — with the RadioMaster
# Pocket plugged in, the core reads it directly and ignores our rc):
#   arrows      right stick (roll / pitch)
#   W/S         throttle up / down       A/D  yaw
#   E           toggle ARM (ch5)         Q    toggle ANGLE (ch6, on by default)
#   R           reset
extends Node3D

# explicit preload: the class_name global cache doesn't exist on a first
# headless run (no .godot import cache yet)
const PwProtocol = preload("res://scripts/protocol.gd")

const CORE_PORT := 9100

var _udp := PacketPeerUDP.new()
var _core_pid := -1
var _frame_id := 0

var _drone: Node3D
var _hud: Label
var _osd: Label       # 16x30 Betaflight OSD overlay, monospace, centered

# client-owned pose state (fed back to the core each frame)
var _pos := Vector3(0, 0.0, 0)
var _rot := Quaternion.IDENTITY   # sim frame
var _angvel := Vector3.ZERO       # sim frame
var _linvel := Vector3.ZERO       # sim frame

# keyboard rc state
var _rc := [0.0, 0.0, -1.0, 0.0, -1.0, 1.0, -1.0, -1.0]
var _throttle := -1.0
var _armed_sw := false
var _angle_sw := true

var _last_out := {}
var _await_warned := false

# autotest (PROPWASH_AUTOTEST=1): no keyboard/radio — arm at t=5.2 s, run
# the reference hover controller, assert altitude/tilt, exit 0/1. This is
# how the GDScript client itself is verified headless in CI.
var _autotest := false
var _at_time := 0.0
var _at_alts: Array[float] = []
var _at_armed_seen := false
var _osd_glyphs := 0

# screenshot capture (PROPWASH_SHOTS=/dir): save frames at set times
var _shot_dir := ""
var _shots_taken := {}


func _ready() -> void:
	_autotest = OS.get_environment("PROPWASH_AUTOTEST") == "1"
	_shot_dir = OS.get_environment("PROPWASH_SHOTS")
	_build_world()
	_spawn_core()
	_udp.connect_to_host("127.0.0.1", CORE_PORT)


func _exit_tree() -> void:
	if _core_pid > 0:
		OS.kill(_core_pid)


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
	_core_pid = OS.create_process(path, args)
	print("propwash-core pid ", _core_pid)


func _physics_process(delta: float) -> void:
	if _autotest:
		_update_autotest_rc(delta)
	else:
		_update_keyboard_rc(delta)

	# --- sim-frame pose (convert Godot -> sim: negate z / negate qx,qy)
	var sim_pos := Vector3(_pos.x, _pos.y, -_pos.z)
	var sim_rot := Quaternion(-_rot.x, -_rot.y, _rot.z, _rot.w)
	# angular velocity is a pseudovector: mirror(z) maps it to (-x, -y, +z)
	var sim_av := Vector3(-_angvel.x, -_angvel.y, _angvel.z)
	var sim_lv := Vector3(_linvel.x, _linvel.y, -_linvel.z)
	var contact := _pos.y <= 0.001

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
		if not _await_warned:
			push_warning("no reply from propwash-core (is it running on udp:%d?)" % CORE_PORT)
			_await_warned = true
		return
	_await_warned = false

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

	_pos += _linvel * delta

	# --- ground plane (client owns collision)
	if _pos.y <= 0.0:
		_pos.y = 0.0
		if _linvel.y < 0.0:
			_linvel.y = 0.0
		_angvel = Vector3.ZERO

	_drone.transform = Transform3D(Basis(_rot), _pos)
	_update_hud()
	if _autotest:
		_autotest_check(delta)


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
	_hud.text = "%s   alt %5.1f m   vbat %4.1f V   thr %4.1f\nrpm %5.0f %5.0f %5.0f %5.0f   dis 0x%x\nE arm | Q angle | R reset | WASD+arrows fly (keyboard mode)" % [
		"ARMED" if o.armed else "DISARMED", _pos.y, o.vbat, _throttle,
		o.motor_rpm[0], o.motor_rpm[1], o.motor_rpm[2], o.motor_rpm[3],
		o.arming_disable]


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
	# capture at a few representative moments (seconds)
	for at in [3.0, 8.0, 12.0, 16.0]:
		var key := str(at)
		if _at_time >= at and not _shots_taken.has(key):
			_shots_taken[key] = true
			await RenderingServer.frame_post_draw
			var img := get_viewport().get_texture().get_image()
			var path := "%s/propwash_t%02d.png" % [_shot_dir, int(at)]
			img.save_png(path)
			print("[shot] ", path)


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
	var body := MeshInstance3D.new()
	var bm := BoxMesh.new()
	bm.size = Vector3(0.15, 0.05, 0.15)
	var bmat := StandardMaterial3D.new()
	bmat.albedo_color = Color(0.15, 0.15, 0.18)
	bm.material = bmat
	body.mesh = bm
	_drone.add_child(body)

	var cam := Camera3D.new()
	cam.fov = 100
	cam.near = 0.05
	# sim +z (forward) maps to Godot -z, which is exactly where Camera3D
	# looks by default — only the FPV uptilt is needed
	cam.rotation_degrees = Vector3(20, 0, 0)
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
