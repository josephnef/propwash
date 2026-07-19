# propwash Godot client (MIT): first flyable.
#
# Spawns propwash-core, drives it in lockstep (one PW_STATE_IN per physics
# frame), applies the returned pose to the drone, and owns collision
# DETECTION: a 5-sphere hull is tested against the world each frame (analytic
# ground plane + engine shape queries for gates/trees) and the resulting
# contact manifold is sent to the core, which resolves it as forces inside
# the physics tick. The client never resolves collision response itself —
# velocities are core-authoritative as of protocol v2.
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

# Default core port. PROPWASH_PORT overrides it — the ctest harnesses each
# use their own port so tests never collide with (or hijack) a live flying
# session on 9100, whose handset would win RC priority and "fly" the test.
const CORE_PORT := 9100
var _core_port := CORE_PORT

# Collision hull: belly (battery stack) + 4 duct spheres, mirroring the
# PW_HULL_* constants in protocol/propwash_protocol.h — every sender derives
# its manifolds from the same five spheres. Godot frame (x symmetric, z
# symmetric, so the coordinates match the sim-frame values).
const HULL_SPHERES := [
	[Vector3(0.0, 0.030, 0.0), 0.045],       # belly
	[Vector3( 0.054, 0.010, -0.054), 0.030], # FR duct
	[Vector3(-0.054, 0.010, -0.054), 0.030], # FL duct
	[Vector3( 0.054, 0.010,  0.054), 0.030], # RR duct
	[Vector3(-0.054, 0.010,  0.054), 0.030], # RL duct
]
const HULL_REST_H := 0.020    # body-origin height resting on flat ground
const CONTACT_SLOP := 0.004   # depenetration residual left for the core spring
const CONTACT_MARGIN := 0.005 # speculative band: report near-contacts at depth 0
const MAX_CONTACTS := 6

var _udp := PacketPeerUDP.new()
var _core_pid := -1
var _frame_id := 0

var _drone: Node3D
var _hud: Label
var _osd: Label       # 16x30 Betaflight OSD overlay, monospace, centered
var _crash_banner: Label   # big center-screen CRASHED — not a squint-sized HUD row
var _crash_hint: Label
var _toast: Label          # transient on-screen action feedback (T/R keys)
var _toast_until := 0.0

# Betaflight 4.5.2 arming-disable bit for its native crash detection
# (ARMING_DISABLED_CRASH_DETECTED) — lights when the dump has
# `set crash_recovery = DISARM` and the firmware itself killed the motors.
const ARMING_CRASH_DETECTED := 1 << 6

# client-owned pose state (fed back to the core each frame), Godot frame
var _pos := Vector3(0, HULL_REST_H, 0)   # start resting on the pad
var _rot := Quaternion.IDENTITY
var _angvel := Vector3.ZERO
var _linvel := Vector3.ZERO
var _prev_pos := Vector3(0, HULL_REST_H, 0)  # for the anti-tunnel sweep

# contact manifold detected this frame, already converted to the sim frame
# (sent with the next PW_STATE_IN)
var _pending_contacts: Array = []
# A client-side pose override (R reset / T repair) is answered one frame
# late: the reply consumed right after the override was produced from the
# packet sent BEFORE it. Applying that stale pose forks the trajectory into
# two alternating lineages (the one-deep pipeline echoes both forever), so
# the override sets this and exactly one reply's pose is skipped.
var _skip_stale_pose := 0
var _sphere_cache := {}       # radius -> SphereShape3D, reused across frames
var _strict := false          # PROPWASH_STRICT=1: no T-repair, crashes are final
var _contact_log := false     # PROPWASH_CONTACT_LOG=1: print contact events
var _was_touching := false

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
# frames where the reply was not already queued — should stay at 0 in normal
# operation; a rising count means the core is not keeping up
var _slow_replies := 0
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
# gate-plane clearance tracking: with solid gates, "passed the gate" must
# also mean "did not clip it" — recorded while crossing each gate plane
var _gate_max_absx := 0.0
var _gate_min_y := 1e9
var _gate_max_y := -1e9
var _max_dmg := 0.0           # worst prop damage seen during the run

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

# Lockstep rate. FIXED, and deliberately not tied to the monitor.
#
# The client is the sole source of simulated time, so the rate it sends at is an
# input to the simulation. Varying it — per monitor, or mid-flight from a
# framerate watchdog — made runs unreproducible: three identical demo runs ended
# 20 cm apart. A constant rate is what makes "identical inputs, identical
# trajectory" true for the client path, not just the headless one.
#
# 250 Hz because 1/250 = 4000 us divides exactly into the core's 50 us tick
# quantum: 80 ticks per packet, no sub-tick residue carried between frames.
# 240 Hz would leave 16 us over and vary the tick count frame to frame.
#
# Display smoothness is handled by physics interpolation, not by matching the
# panel — so a high-refresh monitor loses nothing here.
const PW_SIM_HZ := 250
const PW_SIM_DT := 1.0 / float(PW_SIM_HZ)

# Quality tiers. Auto-selected from the monitor's frame budget so a high-refresh
# setup keeps its framerate and a 60 Hz one gets the pretty version.
#   PROPWASH_QUALITY=low|medium|high|auto   (default auto)
const PwQuality = preload("res://scripts/quality.gd")
const QUALITY_ENV := "PROPWASH_QUALITY"
# opt-in only; the default is always native resolution
const SCALE_ENV := "PROPWASH_SCALE"
var _tier := "medium"
var _q := {}
# A real RenderingDevice is the only reliable signal that Forward+ effects will
# actually run: ProjectSettings still reports "forward_plus" even after
# fallback_to_opengl3 has quietly dropped us onto the Compatibility renderer.
var _has_rd := true


# DJI O3 goggle-feed treatment (shaders/goggle.gdshader). Set PROPWASH_GOGGLE=off
# to see the raw render. Effects that are basically free run at every tier — the
# feed treatment IS the look, so it must not silently disappear on a fast panel.
const GOGGLE_ENV := "PROPWASH_GOGGLE"
var _goggle: TextureRect
var _goggle_mat: ShaderMaterial
var _block_env := 0.0        # codec stress envelope: fast attack, slow decay
var _feed_time := 0.0

# Auto-exposure, done analytically rather than with CameraAttributesPractical.
# That is Forward+-only (so it vanishes under the fallback) and, more to the
# point, WELL damped — whereas DJI's AE is characteristically under-damped: it
# overshoots and settles, and that hunt is the recognisable behaviour. Measuring
# real luminance would mean get_image() every frame, a GPU->CPU stall that would
# wreck the high-refresh path, so this is computed from sky fraction and sun
# alignment, both of which are known exactly.
# Range is deliberately narrow. DJI's AE is good: it keeps the image properly
# exposed and only hunts a little when the framing swings between ground and
# sky. A wide range reads as a broken camera, which is what a first attempt at
# 1.1*sky_frac produced -- it stopped down hard every time the horizon dropped
# and turned the whole scene navy.
const AE_OMEGA := 5.0
const AE_ZETA := 0.55        # under 1.0 -> slight overshoot, then settles
const AE_BASE := 1.35        # matches the static tonemap_exposure from Stage 1
const AE_SKY := 0.22         # how much a sky-filled frame stops down
const AE_SUN := 0.55         # extra stop-down looking near the sun
var _ae := AE_BASE
var _ae_vel := 0.0
var _env: Environment
var _sun: DirectionalLight3D
var _cam: Camera3D

# The screen we actually ended up on. Re-reading Window.current_screen after the
# fullscreen transition reports the primary again on macOS, so both the tick
# rate and the quality tier read this instead of asking twice.
var _active_screen := 0

# The 3D world renders into this SubViewport rather than straight to the screen,
# and the goggle pass samples its texture directly. The reason is measured: a
# canvas shader using hint_screen_texture forces a full-screen backbuffer copy
# every frame, and at 4.1 MP that copy cost ~41 fps of a 212 fps frame -- a
# one-tap passthrough shader measured identically to the full goggle shader,
# which is what proved the cost was the copy and not the maths.
var _subvp: SubViewport
var _world: Node3D          # everything 3D parents here, not to self


func _ready() -> void:
	_autotest = OS.get_environment("PROPWASH_AUTOTEST") == "1"
	_demo = OS.get_environment("PROPWASH_DEMO")
	_shot_dir = OS.get_environment("PROPWASH_SHOTS")
	_strict = OS.get_environment("PROPWASH_STRICT") == "1"
	_contact_log = OS.get_environment("PROPWASH_CONTACT_LOG") == "1"
	var port_env := OS.get_environment("PROPWASH_PORT")
	if port_env.is_valid_int():
		_core_port = int(port_env)
	if _demo == "acro":
		# capture through the whole gate run
		_shot_times = [4.0, 7.0, 9.0, 11.0, 13.0, 15.0, 17.0, 19.0]
	_parse_js_env()
	# before _setup_display: that returns early under headless/autotest, and the
	# sim rate has to be identical on every path or the tests do not exercise
	# what the client actually does
	_pin_sim_rate()
	_setup_display()
	_resolve_quality()   # before _build_world: tree count is baked at build time
	_build_world()
	_spawn_core()
	_udp.connect_to_host("127.0.0.1", _core_port)


func _exit_tree() -> void:
	if _core_pid > 0:
		OS.kill(_core_pid)


func _resolve_quality() -> void:
	var headless := DisplayServer.get_name() == "headless" or _autotest
	_has_rd = RenderingServer.get_rendering_device() != null
	var refresh := 0.0
	var pixels := 0.0
	if not headless:
		refresh = DisplayServer.screen_get_refresh_rate(_active_screen)
		var sz := DisplayServer.screen_get_size(_active_screen)
		pixels = float(sz.x) * float(sz.y)
	_tier = PwQuality.resolve(OS.get_environment(QUALITY_ENV), refresh, pixels, headless)
	# Compatibility renderer is the floor, not a separate tier: the Forward+-only
	# effects above medium would silently do nothing there.
	if not _has_rd and _tier == "high":
		_tier = "medium"
	_q = PwQuality.TIERS[_tier]
	if headless:
		return   # nothing to configure; the dummy driver renders nothing
	_apply_quality()
	print("[pw][gfx] tier=%s renderer=%s %.0fHz %.1fMP=%.0fMP/s scale3d=%.2f msaa=%d" % [
			_tier, "forward+" if _has_rd else "compatibility", refresh,
			pixels / 1_000_000.0, pixels * refresh / 1_000_000.0,
			get_viewport().scaling_3d_scale, _q.msaa])


func _apply_quality() -> void:
	var vp := get_viewport()
	# Live per-Viewport properties.
	vp.msaa_3d = _q.msaa
	vp.screen_space_aa = Viewport.SCREEN_SPACE_AA_DISABLED
	vp.use_debanding = true
	# Native resolution. Rendering the 3D below panel resolution and upscaling is
	# not an acceptable default here: the OSD and HUD stay native on their own
	# layer, so the world looks soft next to crisp text, and on a display someone
	# chose deliberately that is the wrong trade to make silently. Frames get
	# found by cutting scene cost, not by cutting pixels.
	# PROPWASH_SCALE is an explicit escape hatch for a GPU that cannot cope.
	var scale: float = _q.scale_3d
	var pin := OS.get_environment(SCALE_ENV)
	if pin.is_valid_float():
		scale = clampf(pin.to_float(), 0.5, 1.0)
	vp.scaling_3d_mode = Viewport.SCALING_3D_MODE_FSR   # only used if scale < 1
	vp.scaling_3d_scale = scale

	# These are read once at boot, so ProjectSettings.set_setting() at runtime is
	# a no-op -- they have to go through RenderingServer. This is the trap that
	# makes naive preset systems appear to work while doing nothing.
	RenderingServer.directional_shadow_atlas_set_size(_q.shadow_atlas, true)
	RenderingServer.directional_soft_shadow_filter_set_quality(_q.shadow_filter)


# Applied after the sun/environment exist, from _build_sky_and_sun.
func _apply_quality_to_env(e: Environment, sun: DirectionalLight3D) -> void:
	sun.light_angular_distance = _q.sun_angular
	sun.directional_shadow_mode = _q.shadow_splits
	sun.directional_shadow_blend_splits = _q.shadow_blend
	sun.directional_shadow_max_distance = _q.shadow_max_dist

	e.glow_enabled = _q.glow
	if _q.glow:
		e.glow_intensity = 0.5
		e.glow_strength = 1.0
		e.glow_bloom = 0.05
		e.glow_blend_mode = Environment.GLOW_BLEND_MODE_SOFTLIGHT
		e.glow_hdr_threshold = 1.0

	# Forward+ only -- gated on a real RenderingDevice, not on the project
	# setting, which still claims forward_plus after the OpenGL3 fallback.
	e.ssao_enabled = _q.ssao and _has_rd
	if e.ssao_enabled:
		e.ssao_radius = 0.6        # small: this geometry is cm-scale
		e.ssao_intensity = 1.5
		e.ssao_power = 1.5
	e.ssil_enabled = _q.ssil and _has_rd
	e.volumetric_fog_enabled = _q.volfog and _has_rd
	if e.volumetric_fog_enabled:
		e.volumetric_fog_density = 0.008
		e.volumetric_fog_albedo = Color(0.80, 0.85, 0.90)
		e.volumetric_fog_anisotropy = 0.4   # forward scatter -> sun shafts
		e.volumetric_fog_length = 128.0
		# default temporal reprojection is tuned for slow cameras; on a quad at
		# 30 m/s and 800 deg/s it smears trails behind the fog volume
		e.volumetric_fog_temporal_reprojection_enabled = false



func _setup_display() -> void:
	# --headless has no real DisplayServer, and the autotests assert on stdout,
	# not pixels — never grab a screen out from under them, and leave the tick
	# rate at the project default so CI stays comparable run to run.
	if DisplayServer.get_name() == "headless" or _autotest:
		return
	var win := get_window()
	var screen := _target_screen()
	if screen >= 0:
		# order matters: move while still windowed, then go fullscreen. Setting
		# the mode first makes the move a no-op on some platforms — the window
		# already owns the old screen's fullscreen surface.
		win.current_screen = screen
		win.mode = Window.MODE_FULLSCREEN
		print("[pw][display] fullscreen on screen %d/%d %s" % [
				screen, DisplayServer.get_screen_count(),
				DisplayServer.screen_get_size(screen)])
	# whichever screen we ended up on, windowed or not
	_active_screen = screen if screen >= 0 else win.current_screen


# Screen to go fullscreen on, or -1 to stay windowed where we are.
func _target_screen() -> int:
	var want := OS.get_environment(SCREEN_ENV)
	if want == "off":
		return -1
	var n := DisplayServer.get_screen_count()
	if want.is_valid_int():
		var forced := int(want)
		if forced < 0 or forced >= n:
			push_warning("%s=%s out of range (%d screen(s)) — staying windowed"
					% [SCREEN_ENV, want, n])
			return -1
		return forced
	if n < 2:
		return -1   # single screen: windowed, as before
	var primary := DisplayServer.get_primary_screen()
	for i in n:
		if i != primary:
			return i
	return -1


# The lockstep rate is fixed (see PW_SIM_HZ) and is NOT derived from the
# display. Godot's physics rate is pinned to it so one _physics_process call
# maps to exactly one simulation step of PW_SIM_DT.
func _pin_sim_rate() -> void:
	Engine.physics_ticks_per_second = PW_SIM_HZ


func _spawn_core() -> void:
	var path := OS.get_environment("PROPWASH_CORE")
	if path.is_empty():
		path = ProjectSettings.globalize_path("res://") + "../build/propwash-core"
	if not FileAccess.file_exists(path):
		push_warning("propwash-core not found at %s — start it manually" % path)
		return
	var args := ["--port", str(_core_port)]
	var eeprom := OS.get_environment("PROPWASH_EEPROM")
	if not eeprom.is_empty():
		args += ["--eeprom", eeprom]
	# wind, forwarded to the core (which owns all aerodynamics):
	#   PROPWASH_WIND="x,y,z"  steady m/s, sim frame (+z = toward the gates)
	#   PROPWASH_GUST="1.5"    gust amplitude m/s on top
	var wind := OS.get_environment("PROPWASH_WIND")
	if not wind.is_empty():
		args += ["--wind", wind]
	var gust := OS.get_environment("PROPWASH_GUST")
	if not gust.is_empty():
		args += ["--gust", gust]
	# autotest / demo drive RC from the script — a physically-connected Pocket
	# would otherwise override it (joystick has priority) and hijack the run.
	# PROPWASH_NO_JS=1 forces the same for scripted harnesses (tests).
	if _autotest or _demo == "acro" or OS.get_environment("PROPWASH_NO_JS") == "1":
		args += ["--no-js"]
	_core_pid = OS.create_process(path, args)
	print("propwash-core pid ", _core_pid)


func _process(delta: float) -> void:
	_update_goggle(delta)
	_update_toast()


func _toast_msg(text: String) -> void:
	print("[pw][ui] ", text)
	if _toast == null:
		return
	_toast.text = text
	_toast.visible = true
	_toast.modulate.a = 1.0
	_toast_until = _boot_elapsed + 2.5


func _update_toast() -> void:
	if _toast == null or not _toast.visible:
		return
	var left := _toast_until - _boot_elapsed
	if left <= 0.0:
		_toast.visible = false
	elif left < 0.5:
		_toast.modulate.a = left / 0.5   # quick fade instead of a hard pop


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
	var contact := _pending_contacts.size() > 0

	# per-motor ground effect from motor height over the flat field: thrust
	# gains up to +27% below ~half a prop diameter (core applies 1 + 0.7*ge^2)
	var ge := [0.0, 0.0, 0.0, 0.0]
	var xf := Transform3D(Basis(_rot), _pos)
	for g in range(4):
		var h: float = (xf * MOTOR_OFFSETS[g]).y
		ge[RPM_FOR_PROP[g]] = clampf(1.0 - h / GE_FADE_H, 0.0, 1.0)

	var pkt := PwProtocol.pack_state_in(_frame_id, PW_SIM_DT, _rc,
			sim_pos, sim_rot, sim_av, sim_lv, 16.8, contact,
			_pending_contacts, ge)
	_udp.put_packet(pkt)

	# --- consume exactly one STATE_OUT per frame
	#
	# The packet for this frame has just been sent. Rather than blocking until
	# ITS reply comes back, take the oldest reply still queued — normally the
	# previous frame's, which the core produced a whole frame ago and is already
	# waiting. That is a one-deep pipeline: the pose applied is one step behind
	# the packet just sent, consistently.
	#
	# Consistently is what matters. Exactly one reply is consumed per frame, so
	# the client and core stay in step and the lag is identical on every run —
	# a fixed lag costs nothing in a lockstep sim, where the property is
	# identical inputs producing identical trajectories, not zero latency.
	# The previous code drained however many packets happened to have arrived
	# and kept the newest, so the effective lag varied with timing.
	#
	# It also means the spin below effectively never runs: the reply is already
	# there. It stays only as a backstop, so a genuinely dead core is reported
	# rather than silently skipped.
	var out := {}
	var spun := false
	for _spin in range(6000):
		while _udp.get_available_packet_count() > 0:
			var raw := _udp.get_packet()
			if raw.size() >= 8 and raw[5] == PwProtocol.PW_OSD:
				_update_osd(PwProtocol.unpack_osd(raw))
				continue
			var d := PwProtocol.unpack_state_out(raw)
			if not d.is_empty():
				out = d
				break                      # exactly one, never "keep newest"
		if not out.is_empty():
			break
		spun = true
		OS.delay_usec(10)
	if out.is_empty():
		# don't cry wolf during the core's ~1-2 s boot (joystick enumerate +
		# Betaflight init + UDP bind)
		if not _await_warned and (_got_first_reply or _boot_elapsed > 4.0):
			push_warning("no reply from propwash-core (is it running on udp:%d?)" % _core_port)
			_await_warned = true
		return
	if spun and _got_first_reply:
		_slow_replies += 1
	_await_warned = false
	_got_first_reply = true

	_last_out = out
	_frame_id += 1

	# --- back to Godot frame
	if _skip_stale_pose > 0:
		# pose override in flight: consume the reply (keeps the pipeline in
		# step) but hold our overridden pose; velocities are core-owned and
		# continuous, so they apply as usual
		_skip_stale_pose -= 1
	else:
		var q: Quaternion = out.rotation
		_rot = Quaternion(-q.x, -q.y, q.z, q.w).normalized()
	var lv: Vector3 = out.linvel
	_linvel = Vector3(lv.x, lv.y, -lv.z)
	var av: Vector3 = out.angvel
	_angvel = Vector3(-av.x, -av.y, av.z)

	_prev_pos = _pos
	_pos += _linvel * delta

	# --- collision detection (the core resolves the response as forces):
	# analytic ground plane + engine shape queries against gates/trees,
	# depenetration with a slop residual, manifold queued for the next send.
	# No velocity zeroing, no forced leveling — physics owns rest behavior,
	# which is exactly what lets the quad tumble and lie inverted.
	_detect_contacts()

	# sanity floor: never expected to fire; a detection bug must be loud
	# rather than an infinite fall
	if _pos.y < -2.0:
		push_warning("fell through the world at %s — detection bug?" % str(_pos))
		_pos.y = HULL_REST_H + 0.03

	_drone.transform = Transform3D(Basis(_rot), _pos)
	_spin_props(delta)
	_update_hud()
	if _autotest:
		_autotest_check(delta)


# ------------------------------------------------------------- collision
# Detect hull/world contacts, depenetrate the client-owned position with a
# slop residual (the core's spring keeps holding the rest), and queue the
# manifold in sim-frame form for the next send. Spheres within CONTACT_MARGIN
# of a surface are reported at depth 0: the core arms them as one-sided
# dampers, so a corner about to re-impact between frames is damped instead of
# pumping a standing wobble (see Physics::advanceContactDepths).
func _detect_contacts() -> void:
	_pending_contacts = []
	var basis := Basis(_rot)
	var space: PhysicsDirectSpaceState3D = null
	if _drone.get_world_3d() != null:
		space = _drone.get_world_3d().direct_space_state

	# anti-tunnel: at 30 m/s the quad moves 12 cm per frame — a thin gate tube
	# fits between two poses. Sweep the belly sphere along the frame's motion
	# and clip to the first obstacle hit. (The ground can't tunnel: its
	# analytic depth below only grows with penetration.)
	var motion := _pos - _prev_pos
	if space != null and motion.length() > 0.05:
		var sweep := PhysicsShapeQueryParameters3D.new()
		sweep.shape = _sphere(HULL_SPHERES[0][1])
		sweep.transform = Transform3D(Basis.IDENTITY,
				_prev_pos + basis * HULL_SPHERES[0][0])
		sweep.motion = motion
		var frac := space.cast_motion(sweep)
		if frac[0] < 1.0:
			_pos = _prev_pos + motion * frac[0]

	# candidate contacts: exact ground plane + engine queries for obstacles
	var found: Array = []
	for s in HULL_SPHERES:
		var center: Vector3 = _pos + basis * s[0]
		var r: float = s[1]
		var gd: float = r - center.y
		if gd > -CONTACT_MARGIN:
			found.append({world_point = Vector3(center.x, center.y - r, center.z),
					normal = Vector3.UP, depth = gd,
					surface = PwProtocol.SURF_GROUND})
		if space == null:
			continue
		var q := PhysicsShapeQueryParameters3D.new()
		q.shape = _sphere(r + CONTACT_MARGIN)
		q.transform = Transform3D(Basis.IDENTITY, center)
		var rest := space.get_rest_info(q)
		if rest.is_empty():
			continue
		var n: Vector3 = rest.normal
		var depth: float = (r + CONTACT_MARGIN) - (center - rest.point).dot(n) \
				- CONTACT_MARGIN
		var surface: int = PwProtocol.SURF_OBJECT
		var collider := instance_from_id(rest.collider_id)
		if collider != null and collider.has_meta("pw_surface"):
			surface = collider.get_meta("pw_surface")
		found.append({world_point = rest.point, normal = n,
				depth = depth, surface = surface})

	# depenetrate deepest-first; later contacts see the shift already taken
	found.sort_custom(func(a, b): return a.depth > b.depth)
	var shift := Vector3.ZERO
	for c in found:
		var d: float = c.depth - shift.dot(c.normal)
		if d > CONTACT_SLOP:
			shift += c.normal * (d - CONTACT_SLOP)
	_pos += shift

	# queue in the sim frame (mirror z; normals are true vectors: negate z)
	for c in found:
		if _pending_contacts.size() >= MAX_CONTACTS:
			break
		var d: float = c.depth - shift.dot(c.normal)
		var p_body: Vector3 = basis.inverse() * (c.world_point - _pos)
		_pending_contacts.append({
			point_body = Vector3(p_body.x, p_body.y, -p_body.z),
			normal_world = Vector3(c.normal.x, c.normal.y, -c.normal.z),
			depth = maxf(d, 0.0),
			surface = c.surface,
		})

	if _contact_log:
		var touching := _pending_contacts.size() > 0
		if touching and not _was_touching:
			var deepest: Dictionary = _pending_contacts[0]
			print("[pw][contact] t=%.2f n=%d surface=%d depth=%.4f pos=%s" % [
					_boot_elapsed, _pending_contacts.size(), deepest.surface,
					deepest.depth, str(_pos)])
		_was_touching = touching


func _sphere(r: float) -> SphereShape3D:
	var key := snappedf(r, 0.0001)
	if not _sphere_cache.has(key):
		var s := SphereShape3D.new()
		s.radius = r
		_sphere_cache[key] = s
	return _sphere_cache[key]


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
				_pos = Vector3(0, HULL_REST_H, 0)
				_prev_pos = _pos
				_rot = Quaternion.IDENTITY
				_linvel = Vector3.ZERO
				_angvel = Vector3.ZERO
				_throttle = -1.0
				_armed_sw = false
				_skip_stale_pose = 1
				# a teleport, not motion — without this the interpolator
				# smears the drone from where it was back to the pad
				_drone.reset_physics_interpolation()
				_toast_msg("reset to pad")
			KEY_T:
				_repair_in_place()


# T: repair + flip upright where it lies — like walking over and fixing the
# quad by hand, so you don't lose the spot. Disarmed only (you don't grab
# spinning props), and disabled entirely in strict mode (PROPWASH_STRICT=1:
# a crash ends the flight; only R resets). Pose is client-owned, so leveling
# roll/pitch while keeping yaw is a legal pose correction; the core clears
# damage via PW_CMD_REPAIR.
func _repair_in_place() -> void:
	if _strict:
		_toast_msg("repair disabled (strict mode) — R to reset")
		return
	if not _last_out.is_empty() and _last_out.armed:
		# you don't grab spinning props — the banner says so too
		_toast_msg("disarm first (E / ARM switch), then T to repair")
		return
	_udp.put_packet(PwProtocol.pack_command(PwProtocol.PW_CMD_REPAIR))
	var fwd: Vector3 = Basis(_rot) * Vector3(0, 0, -1)
	fwd.y = 0.0
	if fwd.length() < 0.05:
		_rot = Quaternion.IDENTITY
	else:
		_rot = Basis.looking_at(fwd.normalized(), Vector3.UP).get_rotation_quaternion()
	_pos.y = maxf(_pos.y, HULL_REST_H + 0.05)   # small pop clear of the ground
	_prev_pos = _pos
	_skip_stale_pose = 1
	_drone.reset_physics_interpolation()
	_toast_msg("repaired — props replaced")


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
		rc_line = "E arm | Q angle | R reset | %sWASD+arrows fly (keyboard)" % \
				("" if _strict else "T repair | ")
	var dmg: Array = o.get("prop_damage", [0.0, 0.0, 0.0, 0.0])
	var flags: int = o.get("crash_flags", 0)
	var dmg_line := "dmg %3.0f%% %3.0f%% %3.0f%% %3.0f%%" % [
			dmg[0] * 100.0, dmg[1] * 100.0, dmg[2] * 100.0, dmg[3] * 100.0]
	if flags & 1:
		dmg_line += "   CRASHED — %s" % ("R to reset" if _strict else "T to repair")
	if flags & 2:
		dmg_line += "   CRASH-REC"
	_update_crash_banner(flags, o.arming_disable, o.armed)
	_hud.text = "%s   alt %5.1f m   vbat %4.1f V   thr %4.1f\nrpm %5.0f %5.0f %5.0f %5.0f   dis 0x%x\n%s\n%s" % [
		"ARMED" if o.armed else "DISARMED", _pos.y, o.vbat, _throttle,
		o.motor_rpm[0], o.motor_rpm[1], o.motor_rpm[2], o.motor_rpm[3],
		o.arming_disable, dmg_line, rc_line]


# Center-screen crash state. Sim structural crash (bit0) is primary; the
# firmware's own crash-detection disarm (dump-driven `crash_recovery =
# DISARM`) gets its own banner, because the quad is intact there and the fix
# is cycling the ARM switch, not repairing.
func _update_crash_banner(flags: int, arming_disable: int, armed: bool) -> void:
	if _crash_banner == null:
		return
	var text := ""
	var hint := ""
	if flags & 1:
		text = "CRASHED"
		if _strict:
			hint = "press R to reset"
		elif armed:
			# T is (rightly) refused while armed — say so, or T "doesn't work"
			hint = "disarm first (E / ARM switch), then T to repair"
		else:
			hint = "press T to repair — R to reset"
	elif arming_disable & ARMING_CRASH_DETECTED:
		text = "CRASH DETECTED"
		hint = "firmware disarmed — cycle the ARM switch"
	_crash_banner.visible = text != ""
	_crash_hint.visible = text != ""
	if text != "":
		_crash_banner.text = text
		_crash_hint.text = hint
		# slow pulse: reads as a live alert, not a stuck overlay
		_crash_banner.modulate.a = 0.72 + 0.28 * sin(_boot_elapsed * 5.0)


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

	# gates are solid now: record clearance while crossing each gate plane,
	# and the worst damage — a silent scrape must fail the run
	for gz in [-6.0, -14.0, -22.0]:
		if absf(_pos.z - gz) < 0.5:
			_gate_max_absx = maxf(_gate_max_absx, absf(_pos.x))
			_gate_min_y = minf(_gate_min_y, _pos.y)
			_gate_max_y = maxf(_gate_max_y, _pos.y)
	if not _last_out.is_empty():
		for d in _last_out.get("prop_damage", []):
			_max_dmg = maxf(_max_dmg, d)

	if _at_time >= 22.0:
		var min_alt := 1e9
		for a in _at_alts:
			min_alt = minf(min_alt, a)
		# forward = -z; gates at z = -6/-14/-22. Pass = flew past the last
		# gate while staying airborne (never near the ground) and near centre.
		var flew_through := _pos.z < -30.0
		var stayed_up := _at_alts.size() > 0 and min_alt > 0.7
		var on_line := absf(_pos.x) < 3.0
		# posts at x = +/-1.2, bar at 2.05: require real clearance margins
		var cleared := _gate_max_absx < 0.9 and _gate_min_y > 0.35 and _gate_max_y < 1.85
		var undamaged := _max_dmg < 0.05
		var ok := _at_armed_seen and flew_through and stayed_up and on_line \
				and cleared and undamaged
		print("[demo] fly-through: end=%s min_alt=%.2f armed=%s" % [str(_pos), min_alt, str(_at_armed_seen)])
		print("[demo] gate clearance: |x|max=%.2f y=[%.2f, %.2f] max_dmg=%.3f" % [
				_gate_max_absx, _gate_min_y, _gate_max_y, _max_dmg])
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
		# a clean arm-and-hover must end with zero damage and no crash latch —
		# resting on the pad and taking off never counts as an impact
		var dmg_ok := true
		var flags: int = 0
		if not _last_out.is_empty():
			flags = _last_out.get("crash_flags", 0)
			for d in _last_out.get("prop_damage", []):
				if d > 0.0:
					dmg_ok = false
		var ok := _at_armed_seen and _at_alts.size() > 0 and lo > 1.5 and hi < 2.5 \
				and _osd_glyphs > 0 and dmg_ok and flags == 0
		print("[autotest] hover band [%.2f, %.2f] armed_seen=%s osd_glyphs=%d dmg_ok=%s flags=%d" % [
				lo, hi, str(_at_armed_seen), _osd_glyphs, str(dmg_ok), flags])
		print("[autotest] %s" % ("PASS" if ok else "FAIL"))
		get_tree().quit(0 if ok else 1)


var _props: Array[Node3D] = []   # 4 prop meshes, spun by motor RPM
var _prop_angles := [0.0, 0.0, 0.0, 0.0]


# Spin the props from the firmware's motor RPM (PW_STATE_OUT.motor_rpm) and
# fade the blur discs in with RPM. At real RPM the blades strobe, which reads
# as a translucent disc — exactly the real O3 look.
# Godot prop index -> firmware/physics motor index.
#
# These orders genuinely differ and it is not obvious, because the two frames
# disagree about which way is forward. The sim frame is +z forward
# (core/sim/profile_cinelog35.h): motors are RR, FR, RL, FL. Godot is -z
# forward (main.gd builds them FR, FL, RR, RL). So prop i must read motor
# RPM_FOR_PROP[i], not motor i.
#
# Cross-check: physics.cpp:431 has motor_dir = {-1, 1, 1, -1} in sim order.
# Permuting that by this map gives [+1, -1, -1, +1] — exactly PROP_SPIN below,
# which is independent confirmation the mapping is right rather than plausible.
const RPM_FOR_PROP := [1, 3, 0, 2]
const PROP_SPIN := [1.0, -1.0, -1.0, 1.0]

# Motor positions in the Godot frame (FR, FL, RR, RL — forward is -z), shared
# by the drone model and the per-motor ground-effect estimate. Matches the
# physics profile's +/-54 mm.
const MOTOR_OFFSETS := [
	Vector3( 0.054, 0.0, -0.054),  # front-right
	Vector3(-0.054, 0.0, -0.054),  # front-left
	Vector3( 0.054, 0.0,  0.054),  # rear-right
	Vector3(-0.054, 0.0,  0.054),  # rear-left
]
# ground effect fades out ~1.5 prop diameters (3.5" props = 0.089 m) up
const GE_FADE_H := 1.5 * 0.089

# Visual layer for airframe parts the O3 physically cannot see. The FPV camera's
# cull mask excludes it; a chase camera could still render them.
#
# This has been lost once before: a rewrite of the drone model dropped both the
# layer assignment and the camera's mask, and nothing noticed. Procedural scenes
# have no schema, so tests/fpv_cull_test.gd guards it.
#
# Ducts, motors and props deliberately stay on the default layer: a real
# cinewhoop feed does show its own front ducts and blades, and that is a strong
# authenticity cue rather than an artifact.
const FPV_HIDDEN_LAYER := 20


func _hide_from_fpv(node: Node) -> void:
	if node is VisualInstance3D:
		node.set_layer_mask_value(1, false)
		node.set_layer_mask_value(FPV_HIDDEN_LAYER, true)
	for c in node.get_children():
		_hide_from_fpv(c)


func _spin_props(delta: float) -> void:
	if _props.is_empty():
		return
	var rpm := [0.0, 0.0, 0.0, 0.0]
	if not _last_out.is_empty() and _last_out.has("motor_rpm"):
		rpm = _last_out.motor_rpm
	for i in range(_props.size()):
		var r: float = rpm[RPM_FOR_PROP[i]]
		# 5% of true rate so the blades stay legible instead of strobing
		_prop_angles[i] += (r / 60.0) * TAU * delta * 0.05 * PROP_SPIN[i]
		_props[i].rotation.y = _prop_angles[i]


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
	_hide_from_fpv(plate)

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
	_hide_from_fpv(batt)

	# O3 camera pod up front (what the FPV cam looks out of)
	var pod := MeshInstance3D.new()
	var podm := BoxMesh.new()
	podm.size = Vector3(0.021, 0.028, 0.02)
	podm.material = carbon
	pod.mesh = podm
	pod.position = Vector3(0, 0.038, -0.03)
	pod.rotation_degrees = Vector3(25, 0, 0)
	root.add_child(pod)
	_hide_from_fpv(pod)

	# 4 ducts + motors + props at the motor positions (Godot: forward = -z)
	for i in range(4):
		var p: Vector3 = MOTOR_OFFSETS[i]

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
const GATE_COLOR := Color(0.85, 0.30, 0.06)
const GROUND_SIZE := 1000.0   # the flythrough asserts z < -30; the old 60x60
                              # plane ended exactly there, so the test passed by
                              # flying off the last polygon
const WORLD_SEED := 0x9E3779B9   # fixed: the world must be identical every run

# Materials are shared per colour. _add_box used to allocate a fresh BoxMesh AND
# StandardMaterial3D on every call -- 71 unique pairs for what is really two
# distinct looks.
var _mat_cache := {}


func _shared_material(color: Color, rough: float, metal: float) -> StandardMaterial3D:
	var key := "%s|%.2f|%.2f" % [color.to_html(), rough, metal]
	if _mat_cache.has(key):
		return _mat_cache[key]
	var mat := StandardMaterial3D.new()
	mat.albedo_color = color
	mat.roughness = rough
	mat.metallic = metal
	_mat_cache[key] = mat
	return mat


func _add_box(pos: Vector3, size: Vector3, color: Color) -> void:
	var mi := MeshInstance3D.new()
	var mesh := BoxMesh.new()
	mesh.size = size
	mesh.material = _shared_material(color, 0.55, 0.0)
	mi.mesh = mesh
	mi.position = pos
	_world.add_child(mi)


func _build_sky_and_sun() -> void:
	# Sun lower than the old -55 deg: longer shadows read as shape, and a low sun
	# is what an evening flying session actually looks like.
	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-42, -35, 0)
	sun.light_energy = 1.5
	sun.light_color = Color(1.0, 0.97, 0.92)
	sun.shadow_enabled = true
	# shadow_bias defaults to 0.1 -- comparable to the whole 0.11 m airframe, so
	# the quad had effectively no self-shadowing and peter-panned off the ground
	sun.shadow_bias = 0.035
	sun.shadow_normal_bias = 2.4
	# the quad flies low and close, so bias the cascades hard toward the near field
	sun.directional_shadow_split_1 = 0.05
	sun.directional_shadow_split_2 = 0.15
	sun.directional_shadow_split_3 = 0.40
	sun.directional_shadow_fade_start = 0.9
	_world.add_child(sun)

	var env := WorldEnvironment.new()
	var e := Environment.new()

	var sky := Sky.new()
	var psm := PhysicalSkyMaterial.new()   # real Rayleigh/Mie, sun disk matches the light
	psm.rayleigh_coefficient = 2.0
	psm.mie_coefficient = 0.005
	psm.mie_eccentricity = 0.8
	psm.turbidity = 10.0
	psm.sun_disk_scale = 1.0
	psm.ground_color = Color(0.22, 0.25, 0.18)
	# the sky is the brightest thing in a daylit outdoor scene; at 1.0 against a
	# sunlit field it rendered as dusk-navy with the field over-exposed
	psm.energy_multiplier = 2.2
	# The sun never moves in this scene, so the radiance map is static. AUTOMATIC
	# keeps reprocessing it; QUALITY bakes it once and caches. Measured at ~15 fps
	# of a 199 fps frame before this change.
	sky.process_mode = Sky.PROCESS_MODE_QUALITY
	sky.radiance_size = Sky.RADIANCE_SIZE_128
	sky.sky_material = psm
	e.background_mode = Environment.BG_SKY
	e.sky = sky

	# sky-sourced ambient AND reflection: nearly free, and it is what stops every
	# material reading as flat gouraud plastic
	e.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
	e.ambient_light_sky_contribution = 1.0
	e.reflected_light_source = Environment.REFLECTION_SOURCE_SKY

	# Tonemapping is the single biggest item here. The default is LINEAR at
	# exposure 1.0, which is why everything looked washed out. AgX rolls off
	# highlights and desaturates near white much closer to what a small-sensor
	# camera ISP does; ACES tends to crush and over-saturate greens, and this
	# scene is mostly green.
	e.tonemap_mode = Environment.TONE_MAPPER_AGX
	e.tonemap_exposure = 1.35
	e.tonemap_white = 6.0
	e.adjustment_enabled = true
	e.adjustment_contrast = 1.05
	e.adjustment_saturation = 1.08

	# aerial perspective: the biggest single "outdoor" cue, and it works on every
	# renderer including the Compatibility fallback
	e.fog_enabled = true
	e.fog_mode = Environment.FOG_MODE_DEPTH
	e.fog_light_color = Color(0.62, 0.70, 0.80)
	e.fog_light_energy = 1.0
	e.fog_sun_scatter = 0.2
	e.fog_depth_begin = 40.0
	e.fog_depth_end = 900.0
	e.fog_depth_curve = 1.1
	e.fog_aerial_perspective = 0.45   # tint by the sky cubemap
	e.fog_sky_affect = 0.0           # PhysicalSky already has its own haze

	_apply_quality_to_env(e, sun)   # tier-dependent: shadows, glow, ssao, ssil, volfog
	env.environment = e
	_world.add_child(env)
	_env = e      # the exposure hunt drives tonemap_exposure each frame
	_sun = sun


func _build_ground() -> void:
	var ground := MeshInstance3D.new()
	var plane := PlaneMesh.new()
	plane.size = Vector2(GROUND_SIZE, GROUND_SIZE)
	# NOTE: deliberately flat and un-displaced. _detect_contacts() tests the
	# hull analytically against the y=0 plane (exact, engine-independent, and
	# the same math as the C++/python harnesses), so any visual relief would
	# desync -- the drone would sink into hills and hover over valleys.
	var smat := ShaderMaterial.new()
	smat.shader = load("res://shaders/ground.gdshader")
	plane.material = smat
	ground.mesh = plane
	_world.add_child(ground)


# A ring of low-poly conifers at 80-300 m in ONE draw call. No assets, and it is
# the largest single cue that this is a place rather than a plane -- parallax
# against distant objects is most of what sells outdoor flight.
# A single cone and a single sphere read as exactly what they are. Real trees at
# distance are irregular clustered foliage masses sitting on a visible trunk, and
# the giveaway is silhouette variety, not polygon count. So: several distinct
# meshes, each assembled from overlapping jittered blobs, scattered by its own
# MultiMesh. Still only TREE_VARIANTS draw calls for the whole treeline.
const TREE_VARIANTS := 6
var _leaf_tex: ImageTexture
const TRUNK_COLOR := Color(0.085, 0.062, 0.045)


func _build_treeline() -> void:
	_leaf_tex = _make_leaf_texture()   # one mask shared by every variant
	var rng := RandomNumberGenerator.new()
	rng.seed = WORLD_SEED
	var per := int(_q.trees / float(TREE_VARIANTS))
	for v in TREE_VARIANTS:
		var conifer := v < 3
		var mesh := _make_tree_mesh(rng, conifer)
		var tint := Color(0.115, 0.165, 0.085) if conifer else Color(0.150, 0.195, 0.100)
		_scatter_trees(mesh, per, 0.30 if conifer else 0.62, tint)


# Procedural leaf mask, shared by every tree. Alpha is 0 wherever there is no
# leaf, so the quad that carries it disappears and only leaf shapes render.
# Generated rather than shipped: no binary asset, no licence question, no repo
# weight — the whole reason this approach was chosen over sourcing models.
const LEAF_TEX_SIZE := 192


func _make_leaf_texture() -> ImageTexture:
	var img := Image.create(LEAF_TEX_SIZE, LEAF_TEX_SIZE, true, Image.FORMAT_RGBA8)
	var clump := FastNoiseLite.new()
	clump.seed = WORLD_SEED
	clump.noise_type = FastNoiseLite.TYPE_SIMPLEX
	clump.frequency = 0.018
	clump.fractal_octaves = 3
	var detail := FastNoiseLite.new()
	detail.seed = WORLD_SEED + 17
	detail.noise_type = FastNoiseLite.TYPE_SIMPLEX
	detail.frequency = 0.075

	var half := LEAF_TEX_SIZE * 0.5
	for y in LEAF_TEX_SIZE:
		for x in LEAF_TEX_SIZE:
			var u := (x - half) / half
			var v := (y - half) / half
			var r: float = sqrt(u * u + v * v)
			var n := clump.get_noise_2d(x, y) * 0.5 + 0.5
			var d := detail.get_noise_2d(x, y) * 0.5 + 0.5
			# radial falloff keeps foliage off the card's rectangular edges, so
			# the quad boundary never becomes visible
			var mask := n * 0.72 + d * 0.28 - r * 0.62
			if mask > 0.20:
				var shade := 0.62 + d * 0.38          # per-leaf tonal break-up
				img.set_pixel(x, y, Color(shade, shade, shade, 1.0))
			else:
				img.set_pixel(x, y, Color(0, 0, 0, 0))
	img.generate_mipmaps()
	return ImageTexture.create_from_image(img)


# One tree: a tapered trunk plus a stack of alpha-cut foliage cards. Each card is
# a flat quad; the leaf mask discards everything that is not a leaf, so the
# silhouette comes out ragged and porous with sky visible through the gaps. That
# porous outline is what actually reads as foliage — a solid blob never will,
# regardless of how many blobs you overlap.
func _make_tree_mesh(rng: RandomNumberGenerator, conifer: bool) -> ArrayMesh:
	var foliage := SurfaceTool.new()
	foliage.begin(Mesh.PRIMITIVE_TRIANGLES)

	var card := PlaneMesh.new()
	card.orientation = PlaneMesh.FACE_Z   # vertical quad
	card.size = Vector2(1.0, 1.0)

	# Alpha-tested foliage is overdraw-heavy — each card shades every fragment it
	# covers whether or not the leaf mask keeps it, and the cards are two-sided.
	# Card count is therefore the main foliage cost knob, tiered.
	var base: int = _q.leaf_cards
	var cards := rng.randi_range(base, base + 4)
	for i in cards:
		var t := float(i) / float(maxi(cards - 1, 1))   # 0 at base, 1 at tip
		# conifers: cards shrink toward a point and droop. broadleaf: cards fill
		# a rough ellipsoid crown.
		var w: float
		var y: float
		var tilt: float
		if conifer:
			w = lerpf(1.15, 0.34, t) * rng.randf_range(0.9, 1.1)
			y = lerpf(0.32, 1.02, t) + rng.randf_range(-0.04, 0.04)
			tilt = rng.randf_range(0.06, 0.20)          # gentle branch droop
		else:
			w = rng.randf_range(0.72, 1.05)
			y = rng.randf_range(0.46, 1.00)
			tilt = rng.randf_range(-0.18, 0.18)
		# Golden-angle yaw rather than random. Random leaves whole directions
		# bare, and a card seen edge-on is a thin line -- several of those
		# aligning is what produced the diagonal streaks and the false "every
		# tree leans the same way" read.
		var yaw := float(i) * 2.39996 + rng.randf_range(-0.25, 0.25)
		# push cards off-axis so the crown has volume rather than all planes
		# crossing at the trunk
		var off := Vector3(cos(yaw + 1.2), 0.0, sin(yaw + 1.2)) \
				* rng.randf_range(0.0, 0.22) * w
		var basis := Basis(Vector3.UP, yaw) * Basis(Vector3.RIGHT, tilt)
		foliage.append_from(card, 0,
				Transform3D(basis.scaled(Vector3(w, w * 0.78, w)), Vector3(0, y, 0) + off))

	foliage.generate_normals()
	var mesh: ArrayMesh = foliage.commit()
	var fmat := StandardMaterial3D.new()
	fmat.albedo_texture = _leaf_tex
	fmat.albedo_color = Color(0.52, 0.56, 0.42)   # multiplies the instance tint
	fmat.roughness = 0.95
	# Alpha SCISSOR, not blend: blended foliage needs depth sorting, which at
	# 105 deg FOV across ~900 instances is both wrong and expensive. Scissor just
	# discards the fragment. Alpha-to-coverage keeps the cut edges from crawling
	# once MSAA is on.
	fmat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA_SCISSOR
	fmat.alpha_scissor_threshold = 0.5
	# alpha-to-coverage resolves per MSAA sample, so it only helps (and only
	# costs) when MSAA is actually on -- pointless on the low tier
	if _q.msaa != Viewport.MSAA_DISABLED:
		fmat.alpha_antialiasing_mode = BaseMaterial3D.ALPHA_ANTIALIASING_ALPHA_TO_COVERAGE
	fmat.cull_mode = BaseMaterial3D.CULL_DISABLED   # cards are visible both sides
	fmat.vertex_color_use_as_albedo = true   # per-instance tint varies the band
	mesh.surface_set_material(0, fmat)

	# trunk: visible below the canopy, which is most of what says "tree" in a
	# silhouette against the sky
	var trunk := SurfaceTool.new()
	trunk.begin(Mesh.PRIMITIVE_TRIANGLES)
	var tm := CylinderMesh.new()
	tm.top_radius = 0.035
	tm.bottom_radius = 0.075
	tm.height = 0.9
	tm.radial_segments = 6
	tm.rings = 1
	# trunk spans local y 0..0.9, so local y=0 is the base of the tree and the
	# instance can simply be planted at ground level
	trunk.append_from(tm, 0, Transform3D(Basis.IDENTITY, Vector3(0, 0.45, 0)))
	trunk.generate_normals()
	var tmesh: ArrayMesh = trunk.commit()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES,
			tmesh.surface_get_arrays(0))
	var tmat := StandardMaterial3D.new()
	tmat.albedo_color = TRUNK_COLOR
	tmat.roughness = 1.0
	mesh.surface_set_material(1, tmat)
	return mesh


func _scatter_trees(mesh: Mesh, count: int, width: float, tint: Color) -> void:
	var rng := RandomNumberGenerator.new()
	# fixed and per-variant: the world must be identical every run, but each
	# variant must land somewhere different
	rng.seed = WORLD_SEED + count * 7919 + int(width * 1000.0)

	var mm := MultiMesh.new()
	mm.transform_format = MultiMesh.TRANSFORM_3D
	mm.use_colors = true
	mm.mesh = mesh
	mm.instance_count = count
	for i in count:
		var ang := rng.randf() * TAU
		# denser near the inner edge so it reads as a receding mass, not a ring
		var rad: float = 95.0 + pow(rng.randf(), 1.7) * 380.0
		var h := rng.randf_range(5.0, 13.0)
		var w := h * width * rng.randf_range(0.85, 1.2)
		# mesh base is at local y=0, so plant directly on the ground; the old
		# h*0.5 offset was for a centred primitive and left these hovering
		var t := Transform3D(Basis.IDENTITY.scaled(Vector3(w, h, w)),
				Vector3(cos(ang) * rad, 0.0, sin(ang) * rad))
		mm.set_instance_transform(i, t)
		var v := rng.randf_range(0.72, 1.28)
		mm.set_instance_color(i, Color(tint.r * v, tint.g * v, tint.b * v * 0.95))

		# trunk collision: a vertical capsule per tree. Trunk only — the
		# crowns are visually porous alpha cards, and a canopy collider would
		# block what clearly looks flyable-through.
		var body := StaticBody3D.new()
		body.set_meta("pw_surface", PwProtocol.SURF_TREE)
		var shape := CollisionShape3D.new()
		var cap := CapsuleShape3D.new()
		cap.radius = clampf(0.09 * w, 0.05, 0.5)
		cap.height = 0.9 * h
		shape.shape = cap
		shape.position = Vector3(0, 0.45 * h, 0)
		body.add_child(shape)
		body.position = t.origin
		_world.add_child(body)

	var mmi := MultiMeshInstance3D.new()
	mmi.multimesh = mm
	# 95-475 m out: their shadows are invisible but would pollute every cascade
	mmi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	_world.add_child(mmi)


# ------------------------------------------------------------------ gates
# Geometry the flythrough depends on: uprights at x = +/-GATE_HALF, top bar at
# GATE_H, and the demo cruises through at GATE_ALT (1.0 m). The opening must stay
# clear -- decoration goes outside it, never across it.
const GATE_HALF := 1.2
const GATE_H := 2.05
const TUBE_R := 0.055
const GATE_STRIPE_PX := 64

var _gate_tube_mat: StandardMaterial3D
var _gate_foot_mat: StandardMaterial3D


# Hazard banding, generated rather than shipped: alternating safety-orange and
# off-white along the tube axis, the way real race-gate poles are taped.
func _make_stripe_texture() -> ImageTexture:
	var img := Image.create(8, GATE_STRIPE_PX, false, Image.FORMAT_RGBA8)
	for y in GATE_STRIPE_PX:
		# 4 bands over the tile; slight tonal noise so it is not perfectly flat
		var band := int(floor(y / float(GATE_STRIPE_PX) * 2.0)) % 2
		var c := Color(0.85, 0.30, 0.06) if band == 0 else Color(0.88, 0.87, 0.84)
		for x in 8:
			var j := 1.0 + (sin(float(y) * 2.3 + float(x)) * 0.02)
			img.set_pixel(x, y, Color(c.r * j, c.g * j, c.b * j, 1.0))
	img.generate_mipmaps()
	return ImageTexture.create_from_image(img)


func _gate_materials() -> void:
	if _gate_tube_mat != null:
		return
	_gate_tube_mat = StandardMaterial3D.new()
	_gate_tube_mat.albedo_texture = _make_stripe_texture()
	# powder-coated tube: not a mirror, but it catches the sky, which is most of
	# what separates "real object" from "flat orange box"
	_gate_tube_mat.roughness = 0.38
	_gate_tube_mat.metallic = 0.0
	_gate_tube_mat.uv1_scale = Vector3(1.0, 1.0, 1.0)

	_gate_foot_mat = StandardMaterial3D.new()
	_gate_foot_mat.albedo_color = Color(0.07, 0.07, 0.08)
	_gate_foot_mat.roughness = 0.75


func _add_tube(parent: Node3D, from: Vector3, to: Vector3, radius: float) -> void:
	var mesh := CylinderMesh.new()
	mesh.top_radius = radius
	mesh.bottom_radius = radius
	mesh.height = from.distance_to(to)
	mesh.radial_segments = 12      # round profile; a box silhouette reads as CG
	mesh.rings = 1
	# repeat the banding along the tube rather than stretching one tile over it
	# tile the banding at a fixed world size (~22 cm per band) instead of
	# stretching one tile over the whole tube, which read as half orange /
	# half white rather than striped
	var mat: StandardMaterial3D = _gate_tube_mat.duplicate()
	mat.uv1_scale = Vector3(1.0, maxf(1.0, mesh.height / 0.44), 1.0)
	mesh.material = mat
	var mi := MeshInstance3D.new()
	mi.mesh = mesh
	mi.position = (from + to) * 0.5
	# CylinderMesh runs along +Y; rotate that onto the segment direction
	var dir := (to - from).normalized()
	if absf(dir.dot(Vector3.UP)) < 0.999:
		var axis := Vector3.UP.cross(dir).normalized()
		mi.rotate(axis, Vector3.UP.angle_to(dir))
	parent.add_child(mi)

	# matching collision capsule (gates are solid now); the rounded caps
	# overreach each end by `radius`, along the tube axis only — outside the
	# opening, so the flyable gap is exactly the visual gap
	var body := StaticBody3D.new()
	body.set_meta("pw_surface", PwProtocol.SURF_GATE)
	var shape := CollisionShape3D.new()
	var cap := CapsuleShape3D.new()
	cap.radius = radius
	cap.height = mesh.height + 2.0 * radius
	shape.shape = cap
	body.add_child(shape)
	body.transform = mi.transform
	parent.add_child(body)


func _build_gate(z: float, idx: int) -> void:
	_gate_materials()
	var rng := RandomNumberGenerator.new()
	rng.seed = WORLD_SEED + idx * 131   # deterministic per-gate variation

	var gate := Node3D.new()
	gate.position = Vector3(0, 0, z)
	gate.rotate_y(rng.randf_range(-0.05, 0.05))   # nothing on a field is square
	_world.add_child(gate)

	# uprights and top bar -- the opening itself, unchanged from the box version
	_add_tube(gate, Vector3(-GATE_HALF, 0.0, 0), Vector3(-GATE_HALF, GATE_H, 0), TUBE_R)
	_add_tube(gate, Vector3(GATE_HALF, 0.0, 0), Vector3(GATE_HALF, GATE_H, 0), TUBE_R)
	_add_tube(gate, Vector3(-GATE_HALF - TUBE_R, GATE_H, 0),
			Vector3(GATE_HALF + TUBE_R, GATE_H, 0), TUBE_R)

	# corner braces: short diagonals just under the top bar. Outside the flight
	# line, and they stop the frame reading as three disconnected sticks.
	var brace := 0.34
	_add_tube(gate, Vector3(-GATE_HALF, GATE_H - brace, 0),
			Vector3(-GATE_HALF + brace, GATE_H, 0), TUBE_R * 0.6)
	_add_tube(gate, Vector3(GATE_HALF, GATE_H - brace, 0),
			Vector3(GATE_HALF - brace, GATE_H, 0), TUBE_R * 0.6)

	# feet: a gate standing on nothing is one of the strongest "floating CG"
	# cues, and these also ground it against the shadow
	for sx in [-1.0, 1.0]:
		var foot := MeshInstance3D.new()
		var fm := BoxMesh.new()
		fm.size = Vector3(0.34, 0.045, 0.30)
		fm.material = _gate_foot_mat
		foot.mesh = fm
		foot.position = Vector3(sx * GATE_HALF, 0.022, 0)
		gate.add_child(foot)

		var body := StaticBody3D.new()
		body.set_meta("pw_surface", PwProtocol.SURF_OBJECT)
		var shape := CollisionShape3D.new()
		var box := BoxShape3D.new()
		box.size = fm.size
		shape.shape = box
		body.add_child(shape)
		body.position = foot.position
		gate.add_child(body)


func _build_goggle_layer() -> void:
	if DisplayServer.get_name() == "headless" or _autotest:
		return   # nothing is rasterised; the tests read stdout
	if OS.get_environment(GOGGLE_ENV) == "off":
		return
	var mat := ShaderMaterial.new()
	mat.shader = load("res://shaders/goggle.gdshader")
	mat.set_shader_parameter("block_enable", 1.0 if _q.goggle_block else 0.0)
	mat.set_shader_parameter("rs_amt", _q.goggle_rs)
	_goggle_mat = mat

	var rect := TextureRect.new()
	rect.texture = _subvp.get_texture()
	rect.expand_mode = TextureRect.EXPAND_IGNORE_SIZE
	rect.stretch_mode = TextureRect.STRETCH_SCALE
	rect.set_anchors_preset(Control.PRESET_FULL_RECT)
	rect.mouse_filter = Control.MOUSE_FILTER_IGNORE
	rect.material = mat
	var layer := CanvasLayer.new()
	layer.layer = 0
	add_child(layer)
	layer.add_child(rect)
	_goggle = rect


# Feed the shader what only the sim knows, and run the exposure hunt.
func _update_goggle(delta: float) -> void:
	if _goggle_mat == null:
		return
	_feed_time += delta

	# Codec stress from angular rate: fast attack, slow decay, so a whip-pan
	# mushes the frame and it recovers over roughly ten frames — which is what
	# a real H.264 link does when the bitrate budget blows out.
	var rate := _angvel.length()
	var target := clampf((rate - 1.2) / 6.0, 0.0, 1.0)
	if target > _block_env:
		_block_env = lerpf(_block_env, target, 0.5)
	else:
		_block_env = lerpf(_block_env, target, 0.06)

	_goggle_mat.set_shader_parameter("angvel", _angvel)
	_goggle_mat.set_shader_parameter("block_strength", _block_env)
	_goggle_mat.set_shader_parameter("time_seed", _feed_time * 60.0)

	_update_auto_exposure(delta)


func _update_auto_exposure(delta: float) -> void:
	if _env == null or _cam == null or _sun == null:
		return
	var fwd := -_cam.global_transform.basis.z
	var sky_frac := clampf(0.5 + fwd.y * 1.2, 0.0, 1.0)
	var sun_align := maxf(0.0, fwd.dot(-_sun.global_transform.basis.z))
	var target := AE_BASE / (1.0 + AE_SKY * sky_frac + AE_SUN * pow(sun_align, 8.0))
	# under-damped second-order follower -> overshoot then settle, like DJI's AE
	_ae_vel += (target - _ae) * AE_OMEGA * AE_OMEGA * delta \
			- 2.0 * AE_ZETA * AE_OMEGA * _ae_vel * delta
	_ae += _ae_vel * delta
	_env.tonemap_exposure = clampf(_ae, 0.85, 1.6)


func _make_world_root() -> void:
	# Headless, autotest and goggle-off all render straight to the root viewport:
	# no goggle pass means no backbuffer copy to avoid, and keeping the tested
	# headless path structurally identical is worth more than the symmetry.
	if DisplayServer.get_name() == "headless" or _autotest \
			or OS.get_environment(GOGGLE_ENV) == "off":
		_world = self
		return
	_subvp = SubViewport.new()
	_subvp.size = DisplayServer.window_get_size()
	_subvp.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	_subvp.handle_input_locally = false
	# the quality settings belong on whichever viewport actually draws the 3D
	_subvp.msaa_3d = _q.msaa
	_subvp.use_debanding = true
	_subvp.scaling_3d_mode = get_viewport().scaling_3d_mode
	_subvp.scaling_3d_scale = get_viewport().scaling_3d_scale
	add_child(_subvp)
	_world = Node3D.new()
	_subvp.add_child(_world)
	get_window().size_changed.connect(_on_window_resized)


func _on_window_resized() -> void:
	if _subvp:
		_subvp.size = DisplayServer.window_get_size()


func _build_world() -> void:
	_make_world_root()
	_build_sky_and_sun()
	_build_ground()
	_build_treeline()

	for i in range(3):
		_build_gate(-6.0 - i * 8.0, i)

	# drone + FPV camera
	_drone = Node3D.new()
	_world.add_child(_drone)
	_build_drone_model(_drone)

	# FPV camera where the DJI O3 sits on a CineLog35: front-top of the frame,
	# looking forward (-z) with ~25 deg uptilt. The front ducts + props are
	# just below the view, exactly like real cinewhoop footage.
	var cam := Camera3D.new()
	cam.fov = 105
	# near 0.005 against the default far 4000 was an 800,000:1 depth ratio --
	# harmless in a 60 m world, severe z-fighting once there is a treeline at
	# 300 m. Nearest airframe geometry (duct lip / blade tips) is ~0.084 m, so
	# 0.02 is still 4x clear of it.
	cam.near = 0.02
	cam.far = 1500.0
	cam.position = Vector3(0, 0.045, -0.02)
	cam.rotation_degrees = Vector3(25, 0, 0)
	cam.set_cull_mask_value(FPV_HIDDEN_LAYER, false)   # see _hide_from_fpv
	_drone.add_child(cam)
	cam.current = true
	_cam = cam

	# Layer 0: the goggle-feed pass over the 3D render.
	_build_goggle_layer()

	# Layer 1: OSD and HUD, ABOVE the feed pass and untouched by it. On a real
	# DJI system the Betaflight OSD is drawn by the goggles from MSP-DisplayPort
	# data, not encoded into the video, so it is never distorted, blocked,
	# sharpened or noised — it is crisp at panel resolution.
	var ui := CanvasLayer.new()
	ui.layer = 1
	add_child(ui)
	_hud = Label.new()
	_hud.position = Vector2(12, 8)
	_hud.add_theme_font_size_override("font_size", 15)
	ui.add_child(_hud)
	_hud.text = "connecting to propwash-core..."

	# OSD overlay — the real Betaflight 16x30 character grid, centered like
	# FPV goggles.
	_osd = Label.new()
	_osd.set_anchors_preset(Control.PRESET_CENTER)
	_osd.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	_osd.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	# ThemeDB.fallback_font is PROPORTIONAL despite the old comment here claiming
	# monospace, so the fixed 16x30 grid never actually lined up into columns.
	var mono := SystemFont.new()
	mono.font_names = ["Menlo", "Consolas", "DejaVu Sans Mono", "Courier New", "monospace"]
	_osd.add_theme_font_override("font", mono)
	_osd.add_theme_font_size_override("font_size", 20)
	_osd.add_theme_color_override("font_color", Color(1, 1, 1))
	_osd.add_theme_color_override("font_outline_color", Color(0, 0, 0))
	_osd.add_theme_constant_override("outline_size", 2)   # DJI's is a tight shadow
	ui.add_child(_osd)

	# Crash banner: a crash must be unmissable. Center-screen, above the OSD
	# grid, big pulsing red — the HUD damage row stays as the detail readout.
	_crash_banner = Label.new()
	_crash_banner.set_anchors_preset(Control.PRESET_CENTER)
	_crash_banner.grow_horizontal = Control.GROW_DIRECTION_BOTH
	_crash_banner.grow_vertical = Control.GROW_DIRECTION_BOTH
	_crash_banner.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	_crash_banner.add_theme_font_size_override("font_size", 46)
	_crash_banner.add_theme_color_override("font_color", Color(1.0, 0.22, 0.15))
	_crash_banner.add_theme_color_override("font_outline_color", Color(0, 0, 0))
	_crash_banner.add_theme_constant_override("outline_size", 8)
	_crash_banner.offset_top -= 170
	_crash_banner.offset_bottom -= 170
	_crash_banner.visible = false
	ui.add_child(_crash_banner)

	_crash_hint = Label.new()
	_crash_hint.set_anchors_preset(Control.PRESET_CENTER)
	_crash_hint.grow_horizontal = Control.GROW_DIRECTION_BOTH
	_crash_hint.grow_vertical = Control.GROW_DIRECTION_BOTH
	_crash_hint.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	_crash_hint.add_theme_font_size_override("font_size", 20)
	_crash_hint.add_theme_color_override("font_color", Color(1, 1, 1))
	_crash_hint.add_theme_color_override("font_outline_color", Color(0, 0, 0))
	_crash_hint.add_theme_constant_override("outline_size", 4)
	_crash_hint.offset_top -= 122
	_crash_hint.offset_bottom -= 122
	_crash_hint.visible = false
	ui.add_child(_crash_hint)

	# Action feedback toast: every key that declines or acts must say so ON
	# SCREEN — a console print does not exist for a pilot in goggles. Lower
	# center, clear of the OSD grid and the crash banner.
	_toast = Label.new()
	_toast.set_anchors_preset(Control.PRESET_CENTER)
	_toast.grow_horizontal = Control.GROW_DIRECTION_BOTH
	_toast.grow_vertical = Control.GROW_DIRECTION_BOTH
	_toast.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	_toast.add_theme_font_size_override("font_size", 22)
	_toast.add_theme_color_override("font_color", Color(1.0, 0.78, 0.25))
	_toast.add_theme_color_override("font_outline_color", Color(0, 0, 0))
	_toast.add_theme_constant_override("outline_size", 5)
	_toast.offset_top += 210
	_toast.offset_bottom += 210
	_toast.visible = false
	ui.add_child(_toast)
