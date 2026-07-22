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

# Sky, ground, treeline and gates live in scripts/world.gd — see the note there
# on why the ground has to stay an analytic flat plane. PROPWASH_SCENE picks a
# scene; the default is the original flying field, which three ctests assert
# against, so a new scene must be opt-in rather than replace it.
const PwWorld = preload("res://scripts/world.gd")
const SCENE_ENV := "PROPWASH_SCENE"
var _world_builder: PwWorld

# The demo pilot (scripts/demo_pilot.gd) flies every PROPWASH_DEMO chapter
# except the original `acro`/`flythrough` gate run, which stays exactly as it
# was because the `flythrough` ctest asserts against it.
const PwDemoPilot = preload("res://scripts/demo_pilot.gd")
var _pilot: PwDemoPilot

# Camera rig + captions (scripts/demo_director.gd). Built on every path, not
# just in demo mode: a chase view is genuinely useful for hand flying too, and
# `C` cycles it. Only the demo drives the cuts automatically.
const PwDemoDirector = preload("res://scripts/demo_director.gd")
var _director: PwDemoDirector

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
var _turtle_sw := false   # ch7 = FLIP OVER AFTER CRASH box (arm to flip)

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
	_jl = OS.get_environment("PROPWASH_JITTER_LOG") == "1"
	if _jl:
		Engine.max_fps = 60   # comparable A/B runs
	var port_env := OS.get_environment("PROPWASH_PORT")
	if port_env.is_valid_int():
		_core_port = int(port_env)
	if _demo == "acro":
		# capture through the whole gate run
		_shot_times = [4.0, 7.0, 9.0, 11.0, 13.0, 15.0, 17.0, 19.0]
	elif not _demo.is_empty():
		# the pilot's chapters run considerably longer; spread the stills over
		# the whole reel rather than bunching them in the first 20 s
		_shot_times = []
		for i in range(24):
			_shot_times.append(6.0 + i * 2.5)
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
	if _autotest or not _demo.is_empty() or OS.get_environment("PROPWASH_NO_JS") == "1":
		args += ["--no-js"]
	_core_pid = OS.create_process(path, args)
	print("propwash-core pid ", _core_pid)


# PROPWASH_JITTER_LOG=1 — measure the ACTUAL symptom: where things are DRAWN
# on screen, frame to frame.
#
# Everything else here (tick spacing, steps per frame) measures a proxy. This
# measures what the eye sees. get_global_transform_interpolated() returns the
# transform Godot actually renders — the physics-interpolated one — rather than
# the value last written from script, and unproject_position uses the camera's
# own drawn transform. So this is the rendered pixel position of the drone.
#
# Two points are tracked through the SAME camera:
#   - the drone
#   - a fixed point in the world (gate 1's top bar)
# because the reported symptom is "the world is smooth, only the drone
# trembles". Measuring both turns that into two numbers instead of an opinion.
#
# The metric is the mean absolute SECOND difference of screen position, in
# pixels. Under smooth motion — still, constant velocity, or a steady turn —
# successive screen steps change slowly and this is near zero. Frame-to-frame
# tremble alternates the step size and makes it large. It is scale-free: it
# does not care how fast anything is moving, only whether the motion is smooth.
const JITTER_REF := Vector3(0.0, 2.05, -6.0)   # gate 1 top bar: static world
var _jl := false
var _jl_n := 0
var _jl_prev := [Vector2.ZERO, Vector2.ZERO, Vector2.ZERO]
var _jl_d1 := [Vector2.ZERO, Vector2.ZERO, Vector2.ZERO]
var _jl_jerk := [0.0, 0.0, 0.0]
var _jl_ang := 0.0
var _jl_rate: Array[float] = []
var _jl_aim: Array[float] = []
var _jl_samples := 0


func _jitter_log(_delta: float) -> void:
	if _drone == null:
		return
	var vp := _drone.get_viewport()
	if vp == null:
		return
	var cam := vp.get_camera_3d()
	if cam == null:
		return
	# DRAWN transform (physics-interpolated), not the value last written
	var x: Transform3D = _drone.get_global_transform_interpolated()

	# Blade probe: the rendered ANGULAR RATE of a prop, in the drone's own
	# frame so the airframe's motion cancels out.
	#
	# NOT a point on the rim. A rim point orbits, so its screen
	# second-difference is dominated by centripetal acceleration and a
	# perfectly smooth spin scores enormous — the first version of this probe
	# measured exactly that and said nothing useful. What matters is whether
	# the angle advances at a constant rate per unit of DISPLAYED time.
	if not _props.is_empty() and _delta > 0.0:
		var pr := _props[0] as Node3D
		var rel := x.affine_inverse() * pr.get_global_transform_interpolated()
		var ang := atan2(rel.basis.x.z, rel.basis.x.x)
		var step := wrapf(ang - _jl_ang, -PI, PI)
		_jl_ang = ang
		if _jl_n > 2:
			_jl_rate.append(absf(step) / _delta)
	# THE camera-aim error: the director aims the chase/LOS cameras at _pos,
	# the raw physics position, but the drone is DRAWN at x.origin, the
	# interpolated one. If that gap varies frame to frame, the drone wobbles
	# against frame centre while the static world — which the gap does not
	# touch — stays put.
	_jl_aim.append((x.origin - _pos).length())
	# Three screen-space probes through the SAME camera:
	#   origin - the body; translation only
	#   duct   - offset from centre, so it also picks up ROTATION. An airframe
	#            that shimmers in place moves this and NOT the origin, which is
	#            why a centre-only probe found nothing.
	#   world  - static geometry: the "is everything else smooth" control that
	#            turns "only the drone trembles" into two comparable numbers.
	var pts := [x.origin, x * Vector3(0.0718, 0.0, -0.0718), JITTER_REF]
	var out := []
	for q in pts:
		if cam.is_position_behind(q):
			return
		out.append(cam.unproject_position(q))
	_jl_n += 1
	if _jl_n > 2:
		for k in range(3):
			var d1: Vector2 = out[k] - _jl_prev[k]
			_jl_jerk[k] += (d1 - _jl_d1[k]).length()
			_jl_d1[k] = d1
		_jl_samples += 1
	_jl_prev = out.duplicate()
	if _jl_samples < 120:
		return
	var n := float(_jl_samples)
	var rm := 0.0
	for v in _jl_rate:
		rm += v
	rm /= maxf(float(_jl_rate.size()), 1.0)
	var rv := 0.0
	for v in _jl_rate:
		rv += (v - rm) * (v - rm)
	var rsd := sqrt(rv / maxf(float(_jl_rate.size()), 1.0))
	# median absolute deviation as well as SD: a single hitched frame blows up
	# an SD and looks identical to a systematic wobble, and the two need
	# telling apart.
	var sorted := _jl_rate.duplicate()
	sorted.sort()
	var med: float = sorted[sorted.size() / 2] if sorted.size() > 0 else 0.0
	var devs: Array[float] = []
	for v in _jl_rate:
		devs.append(absf(v - med))
	devs.sort()
	var mad: float = devs[devs.size() / 2] if devs.size() > 0 else 0.0
	var worst: float = sorted[sorted.size() - 1] if sorted.size() > 0 else 0.0
	print("[jitter] %d fr @ %.0f fps, cam %s | body %.2f | duct %.2f | world %.2f px/fr^2"
			% [_jl_samples, Engine.get_frames_per_second(), cam.name,
					_jl_jerk[0] / n, _jl_jerk[1] / n, _jl_jerk[2] / n]
			+ " || BLADE med %.0f MAD %.0f (%.1f%%) sd %.0f worst %.0f deg/s"
			% [rad_to_deg(med), rad_to_deg(mad), 100.0 * mad / maxf(med, 1e-6),
					rad_to_deg(rsd), rad_to_deg(worst)])
	var am := 0.0
	for v in _jl_aim:
		am += v
	am /= maxf(float(_jl_aim.size()), 1.0)
	var av := 0.0
	var amax := 0.0
	for v in _jl_aim:
		av += (v - am) * (v - am)
		amax = maxf(amax, v)
	print("[aim]    drawn-vs-physics gap %.4f +/- %.4f m (max %.4f) -- the "
			% [am, sqrt(av / maxf(float(_jl_aim.size()), 1.0)), amax]
			+ "camera aims at physics, the drone is drawn interpolated")
	_jl_aim.clear()
	_jl_rate.clear()
	_jl_jerk = [0.0, 0.0, 0.0]
	_jl_samples = 0


func _process(delta: float) -> void:
	_spin_props(delta)
	if _jl:
		_jitter_log(delta)
	_update_goggle(delta)
	_update_toast()
	# Camera work runs on the RENDER frame, not the physics frame: the chase
	# follow and the LOS zoom are presentation, and smoothing them at 250 Hz
	# while drawing at 60 would just be 190 wasted updates. Nothing here feeds
	# the simulation, so it cannot affect reproducibility.
	if _director != null:
		var cam := ""
		var cap := ""
		var sub := ""
		if _pilot != null:
			cam = _pilot.cam_hint
			cap = _pilot.caption
			sub = _pilot.subcaption
		_director.update(delta, cam, cap, sub)


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
	if _demo == "acro" or _demo == "flythrough":
		_update_acro_demo_rc(delta)
	elif not _demo.is_empty():
		_update_pilot_rc(delta)
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
	# fits between two poses. Sweep the hull along the frame's motion and clip
	# to the first obstacle hit. (The ground can't tunnel: its analytic depth
	# below only grows with penetration.)
	#
	# ALL FIVE spheres, not just the belly. The belly alone leaves the four duct
	# spheres — which stick out 5.4 cm to each side and are the parts that
	# actually clip a gate upright — free to pass straight through thin
	# geometry between frames. Rotation is held at the frame's end pose for the
	# sweep, the same approximation the single-sphere version made.
	var motion := _pos - _prev_pos
	if space != null and motion.length() > 0.05:
		var sweep := PhysicsShapeQueryParameters3D.new()
		sweep.motion = motion
		var first := 1.0
		for s in HULL_SPHERES:
			sweep.shape = _sphere(s[1])
			sweep.transform = Transform3D(Basis.IDENTITY, _prev_pos + basis * s[0])
			first = minf(first, space.cast_motion(sweep)[0])
		if first < 1.0:
			_pos = _prev_pos + motion * first

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
	_rc[6] = 1.0 if _turtle_sw else -1.0


func _unhandled_key_input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed and not event.echo:
		match event.physical_keycode:
			KEY_E:
				_armed_sw = not _armed_sw
			KEY_Q:
				_angle_sw = not _angle_sw
			KEY_F:
				# turtle: flip the switch while disarmed and upside down,
				# then arm — the props reverse and the sticks flip you over
				_turtle_sw = not _turtle_sw
				_toast_msg("turtle switch ON — arm to flip" if _turtle_sw
						else "turtle switch OFF")
			KEY_R:
				reset_sim()
				_toast_msg("reset to pad")
			KEY_C:
				# FPV -> chase -> LOS. Handy when hand flying: a chase view is
				# the fastest way to see what the quad is actually doing.
				if _director != null:
					_toast_msg("camera: %s" % _director.cycle())
			KEY_T:
				_repair_in_place()


# Reset core AND client to the pad. Both halves matter: PW_CMD_RESET rewinds
# the firmware's writable statics to the post-boot snapshot (so reset is
# equivalent to a fresh process — see docs/ARCHITECTURE.md), and the client
# owns world position, so it has to rewind its own integration to match. Miss
# either half and a "reset" run starts somewhere the other side doesn't agree
# with, which is exactly what the ghost chapter would surface as divergence.
func reset_sim() -> void:
	_udp.put_packet(PwProtocol.pack_command(PwProtocol.PW_CMD_RESET))
	_pos = Vector3(0, HULL_REST_H, 0)
	_prev_pos = _pos
	_rot = Quaternion.IDENTITY
	_linvel = Vector3.ZERO
	_angvel = Vector3.ZERO
	_throttle = -1.0
	_armed_sw = false
	_pending_contacts = []
	_skip_stale_pose = 1
	# a teleport, not motion — without this the interpolator smears the drone
	# from where it was back to the pad
	if _drone != null:
		_drone.reset_physics_interpolation()


# A translucent stand-in for a previously recorded run, used by the ghost
# chapter. Same GLB as the real airframe so the two are directly comparable —
# a simplified proxy would leave "are they really in the same place" open.
func spawn_ghost() -> Node3D:
	var g := _load_drone_glb()
	if g == null:
		return null
	var holder := Node3D.new()
	holder.name = "Ghost"
	holder.add_child(g)
	_world.add_child(holder)
	_ghostify(g)
	# never in the FPV feed: the pilot's camera is ON the live quad, and a
	# ghost hanging in front of the lens would be nonsense
	_hide_from_fpv(g)
	for c in g.get_children():
		_hide_from_fpv(c)
	return holder


func _ghostify(n: Node) -> void:
	if n is MeshInstance3D:
		var m := StandardMaterial3D.new()
		m.albedo_color = Color(0.35, 0.85, 1.0, 0.30)
		m.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
		m.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
		m.cull_mode = BaseMaterial3D.CULL_DISABLED
		(n as MeshInstance3D).material_override = m
		(n as MeshInstance3D).cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	for c in n.get_children():
		_ghostify(c)


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
	if not _demo.is_empty():
		# During a demo the keyboard help is noise — nobody is at the sticks.
		# The flight mode is what matters, and it comes from the firmware's own
		# flightModeFlags rather than from what we asked for on ch6: this line
		# is the difference between "the demo claims ACRO" and "Betaflight
		# reports ACRO".
		rc_line = "mode %s   %s" % [_mode_name(o.get("mode_flags", 0)),
				_demo.to_upper()]
	elif _js_dev >= 0:
		# live channels help confirm the AETR+switch mapping / spot inversions
		rc_line = "RC '%s'  A%+.2f E%+.2f T%+.2f R%+.2f  5:%+.2f 6:%+.2f 7:%+.2f 8:%+.2f" % [
			Input.get_joy_name(_js_dev),
			_rc[0], _rc[1], _rc[2], _rc[3], _rc[4], _rc[5], _rc[6], _rc[7]]
	else:
		rc_line = "E arm | Q angle | F turtle | R reset | %sWASD+arrows fly (keyboard)" % \
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


# Betaflight flightModeFlags, low bits (src/main/fc/runtime_config.h):
# bit0 ANGLE_MODE, bit1 HORIZON_MODE. Neither set = acro, which is the state
# the demo cares about and the one that has no flag of its own.
const FLIGHT_MODE_ANGLE := 1 << 0
const FLIGHT_MODE_HORIZON := 1 << 1


func _mode_name(flags: int) -> String:
	if flags & FLIGHT_MODE_ANGLE:
		return "ANGLE"
	if flags & FLIGHT_MODE_HORIZON:
		return "HORIZON"
	return "ACRO"


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


# --------------------------------------------------------- demo chapters
# Everything except the original `acro` gate run is flown by demo_pilot.gd. The
# pilot owns all eight channels (including ARM), prints its own report and says
# when it is done; this is only the plumbing.
# Booth mode: the demo runs on repeat, and anyone can pick up the sticks.
#
# The takeover is deliberately implicit — no button to find, no mode to
# explain. Moving a stick past the deadband is how you take control, which is
# the only instruction a stranger at a stand needs. Control goes back to the
# demo after the sticks have been still for a while.
const TAKEOVER_DEADBAND := 0.18
const TAKEOVER_HOLD := 6.0        # seconds of stick silence before the demo resumes
var _takeover := false
var _takeover_idle := 0.0


func _update_pilot_rc(delta: float) -> void:
	if _pilot == null:
		_pilot = PwDemoPilot.new()
		_pilot.begin(self, "reel" if _demo == "loop" else _demo)
	_at_time += delta
	_maybe_shoot()          # PROPWASH_SHOTS: stills through the run

	if _demo == "loop" and _check_takeover(delta):
		_update_manual_rc(delta)
		return

	_rc = _pilot.update(delta)
	_throttle = _rc[2]           # keeps the HUD's throttle readout honest
	if _pilot.finished():
		var fails: Array[String] = _pilot.failures()
		for f in fails:
			print("[demo] FAIL: %s" % f)
		print("[demo] %s" % ("PASS" if fails.is_empty() else "FAIL"))
		if _demo == "loop":
			# never exits: start the reel again from the pad
			print("[demo] loop: restarting")
			reset_sim()
			_pilot = null
			return
		get_tree().quit(0 if fails.is_empty() else 1)


# True while a human is flying. Only ever consults a real handset — keyboard
# input would make the demo hand control over to a stray arrow key.
func _check_takeover(delta: float) -> bool:
	var dev := _pick_joystick()
	if dev < 0:
		return false
	var moved := false
	for ch in range(4):
		var ax: int = _js_axis_map[ch]
		var v := Input.get_joy_axis(dev, ax)
		# throttle rests at -1, the other three at centre
		var rest := -1.0 if ch == 2 else 0.0
		if absf(v - rest) > TAKEOVER_DEADBAND:
			moved = true
			break
	if moved:
		if not _takeover:
			_takeover = true
			_toast_msg("you have control")
		_takeover_idle = 0.0
		return true
	if _takeover:
		_takeover_idle += delta
		if _takeover_idle >= TAKEOVER_HOLD:
			_takeover = false
			_toast_msg("demo resuming")
			return false
		return true
	return false


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
	for gz in PwWorld.GATE_Z:
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
var _prop_discs: Array[StandardMaterial3D] = []   # blur discs, alpha driven by RPM


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
	Vector3( 0.050205, 0.0, -0.050205),  # front-right
	Vector3(-0.050205, 0.0, -0.050205),  # front-left
	Vector3( 0.050205, 0.0,  0.050205),  # rear-right
	Vector3(-0.050205, 0.0,  0.050205),  # rear-left
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


# RPM at which the blur disc reaches full opacity. Hover is ~14.5k (see the
# HUD line in docs/demo-hover-real-tune.png), so the disc is already most of
# the way in whenever the quad is actually flying.
const PROP_BLUR_RPM := 12000.0
const PROP_BLUR_ALPHA := 0.12


# Driven on the RENDER frame, with the render delta — not on the physics tick.
#
# This is the fix for a measured, visible defect. At 250 Hz physics into 60 Hz
# render, each rendered frame consumes 4 or 5 physics steps in a fixed 5:1
# ratio, so anything advanced per-physics-tick moves 25% further every sixth
# frame. On the airframe that is invisible (it is physics-interpolated, and its
# screen jerk measures ~1 px/frame^2). On a blade turning at ~4200 deg/s it is
# 56 deg versus 70 deg per frame on a THREE-blade prop, and it measured 14-46
# px/frame^2 — up to 18x the body through the same camera. Zoomed in from a
# chase view that reads as the whole drone vibrating, while the airframe and
# the world around it are perfectly smooth. Which is exactly how it was
# reported, and exactly why looking at the flight controller found nothing.
#
# Angle is a pure visual: nothing reads _prop_angles, so advancing it by render
# delta is both correct and the only way to make the step size match the frame
# actually being displayed.
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
		if i < _prop_discs.size():
			var a: float = clampf(r / PROP_BLUR_RPM, 0.0, 1.0) * PROP_BLUR_ALPHA
			_prop_discs[i].albedo_color.a = a


# The airframe, generated from model/cinelog35_v3.scad. See model/README.md —
# the .scad is the source of truth and the GLB is a committed build product,
# regenerated by model/build_asset.py and pinned by the model_regen ctest.
const MODEL_PATH := "res://assets/cinelog35_v3.glb"

# Loaded at RUNTIME through GLTFDocument rather than as an imported resource.
# The client has no imported assets at all: nothing under .godot/imported, no
# .import sidecars, and ctest runs `godot --headless --path client-godot` with
# no import pass. Going through the importer would mean a CMake import fixture
# AND a second generated artifact to keep in step with every .scad change.
# assets/.gdignore keeps the editor from importing it behind our back.
#
# (One consequence worth knowing: .gdignore'd files are excluded from an
# exported .pck. propwash always runs from source, so this costs nothing today.)

# Parts the O3 physically cannot see: they sit under or behind the lens. The
# TPU group includes the camera cradle the FPV camera is mounted inside, so it
# would fill the frame. Everything NOT listed here — the guards, motor
# bells, copper windings and all four props — stays on the default layer,
# because a real cinewhoop feed does show its own front ducts and blades.
const MODEL_FPV_HIDDEN := [
	"CarbonFrame", "Hardware", "PCB", "Components", "BatteryRails", "TPU",
]

# GLB prop node names in propwash's prop order (FR, FL, RR, RL — see
# MOTOR_OFFSETS). Bound BY NAME, never by child order: glTF node order is an
# exporter detail, and getting it wrong would silently swap which motor spins
# which blade. fpv_cull_test asserts all four resolve.
const MODEL_PROP_NODES := ["Prop_FR", "Prop_FL", "Prop_RR", "Prop_RL"]

# FPV camera pose, where the DJI O3 sits on a CineLog35: front-top of the frame
# looking forward (-z) with ~25 deg uptilt. Shared with the (FPV-hidden) camera
# pod below so the two cannot drift apart.
#
# y sits ~9 mm above the airframe's top (+0.0182), i.e. an O3 resting on its TPU
# cradle rather than floating above the quad. Tighter than it looks: the airframe
# is only 22 mm tall and the view is 105 deg, so a few mm of camera height is the
# difference between the front guards holding the bottom ~6% of the frame and the
# quad vanishing off the bottom edge entirely.
#
# z is deliberately NOT pushed forward to where the real O3 nacelle sits: past
# about -0.03 the camera clears the front duct and the guards leave the frustum
# sideways, taking the airframe out of frame completely.
#
# Both are measured, not dialled in by eye — fpv_cull_test projects the hull into
# this camera and fails if the airframe leaves the lower band.
const CAM_POS := Vector3(0, 0.027, -0.02)
const CAM_TILT_DEG := 25.0


func _build_drone_model(root: Node3D) -> void:
	var model := _load_drone_glb()
	if model == null:
		return
	root.add_child(model)

	# Visual/physics alignment check, expressed as a scale so it can never be
	# silently wrong. The model's prop shafts and MOTOR_OFFSETS are now the same
	# 142 mm wheelbase — the .scad's `wheelbase = 142` and the physics profile's
	# +/-50.205 mm are the same number — so this resolves to 1.0.
	#
	# It stays derived rather than assumed because the two live in different
	# files and different languages. If they ever diverge the model is scaled to
	# match the collision hull (visually right, quietly rescaled) AND warns,
	# rather than rendering ducts that sit somewhere the hull isn't.
	#
	# NB the fix for a future divergence is NOT to widen `wheelbase` in the
	# .scad: adjacent duct rings overlap by 2.69 mm and that overlap is what
	# fuses them into the two molded halves — spreading the motors apart breaks
	# the cage into four floating hoops (model/README.md).
	var prop_fr := model.find_child(MODEL_PROP_NODES[0], true, false) as Node3D
	if prop_fr == null:
		push_error("[pw] %s has no %s node" % [MODEL_PATH, MODEL_PROP_NODES[0]])
		return
	var scale_to_hull: float = MOTOR_OFFSETS[0].x / prop_fr.position.x
	if absf(scale_to_hull - 1.0) > 0.01:
		push_warning("[pw] airframe model wheelbase disagrees with the physics "
				+ "profile by %.1f%% — rescaling the model to match the hull. "
						% (100.0 * (scale_to_hull - 1.0))
				+ "Reconcile model/cinelog35_v3.scad with MOTOR_OFFSETS / "
				+ "core/sim/profile_cinelog35.h.")
	model.scale = Vector3.ONE * scale_to_hull

	# Seat it on the pad: HULL_REST_H is where the body origin sits when the hull
	# spheres rest on flat ground, so the model's lowest point belongs exactly
	# there. Derived from the mesh rather than dialled in by eye — get this wrong
	# and the quad either floats above the pad or sinks into it.
	var box := _model_aabb(model)
	model.position.y = -HULL_REST_H - box.position.y

	for name in MODEL_FPV_HIDDEN:
		var part := model.find_child(name, true, false)
		if part == null:
			push_error("[pw] %s has no %s node" % [MODEL_PATH, name])
			continue
		_hide_from_fpv(part)

	for i in range(MODEL_PROP_NODES.size()):
		var prop := model.find_child(MODEL_PROP_NODES[i], true, false) as Node3D
		if prop == null:
			push_error("[pw] %s has no %s node" % [MODEL_PATH, MODEL_PROP_NODES[i]])
			continue
		# written every RENDER frame by _spin_props, so Godot must not also
		# interpolate them between physics snapshots — that is interpolating a
		# value that is already render-rate, and it reintroduces the wobble
		prop.physics_interpolation_mode = Node.PHYSICS_INTERPOLATION_MODE_OFF
		_props.append(prop)
		_prop_discs.append(_add_blur_disc(prop))

	_add_battery_and_pod(root, model)


func _load_drone_glb() -> Node3D:
	var doc := GLTFDocument.new()
	var state := GLTFState.new()
	var err := doc.append_from_file(MODEL_PATH, state)
	if err != OK:
		push_error("[pw] cannot read %s (error %d). Regenerate it with "
				% [MODEL_PATH, err] + "`python3 model/build_asset.py`.")
		return null
	# generate_scene() wraps the glTF scene in a root of its own, so every
	# lookup below is a recursive find_child rather than a direct child index.
	return doc.generate_scene(state) as Node3D


# Merged AABB of every mesh under `node`, in `node`'s parent space (so the
# node's own scale and position are included).
func _model_aabb(node: Node3D) -> AABB:
	var boxes: Array[AABB] = []
	_collect_mesh_aabbs(node, Transform3D.IDENTITY, boxes)
	if boxes.is_empty():
		return AABB()
	var merged := boxes[0]
	for i in range(1, boxes.size()):
		merged = merged.merge(boxes[i])
	return merged


func _collect_mesh_aabbs(node: Node, xform: Transform3D, out: Array[AABB]) -> void:
	var here := xform
	if node is Node3D:
		here = xform * (node as Node3D).transform
	if node is MeshInstance3D and (node as MeshInstance3D).mesh != null:
		out.append(here * (node as MeshInstance3D).mesh.get_aabb())
	for c in node.get_children():
		_collect_mesh_aabbs(c, here, out)


# A faint translucent disc over the blades. At real RPM a prop strobes into a
# smear rather than four legible blades, and _spin_props already renders at 5%
# of true rate to keep them readable — the disc is what sells the difference
# between "parked" and "spinning". Sized from the prop's own mesh so it tracks
# the model.
func _add_blur_disc(prop: Node3D) -> StandardMaterial3D:
	var radius := 0.045
	if prop is MeshInstance3D and (prop as MeshInstance3D).mesh != null:
		var box := (prop as MeshInstance3D).mesh.get_aabb()
		radius = maxf(maxf(absf(box.position.x), absf(box.end.x)),
				maxf(absf(box.position.z), absf(box.end.z)))
	var mat := StandardMaterial3D.new()
	mat.albedo_color = Color(0.7, 0.7, 0.75, 0.0)   # alpha driven by _spin_props
	mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	var mesh := CylinderMesh.new()
	mesh.top_radius = radius
	mesh.bottom_radius = radius
	mesh.height = 0.001
	mesh.material = mat
	var disc := MeshInstance3D.new()
	disc.mesh = mesh
	prop.add_child(disc)
	return mat


# The GLB is the published dry airframe — no battery, camera, VTX or antenna
# (model/README.md). Both of these are hidden from the FPV camera, so they only
# matter to a chase view, but the quad reads as unflyable without them.
func _add_battery_and_pod(root: Node3D, model: Node3D) -> void:
	var batt := MeshInstance3D.new()
	var btm := BoxMesh.new()
	btm.size = Vector3(0.035, 0.022, 0.075)
	var bmat := StandardMaterial3D.new()
	bmat.albedo_color = Color(0.55, 0.15, 0.15)
	btm.material = bmat
	batt.mesh = btm
	# Sit it on the battery rails the model actually has, rather than at a
	# hard-coded height that the scale and seat offset would invalidate.
	var rails := model.find_child("BatteryRails", true, false) as MeshInstance3D
	var rail_top := 0.012
	if rails != null and rails.mesh != null:
		rail_top = (model.transform * rails.transform * rails.mesh.get_aabb()).end.y \
				+ model.position.y
	batt.position = Vector3(0, rail_top + btm.size.y * 0.5, 0.005)
	root.add_child(batt)
	_hide_from_fpv(batt)

	# O3 camera pod — the thing the FPV camera looks out of, placed at the
	# camera's own pose so a chase view shows it where the view comes from.
	var pod := MeshInstance3D.new()
	var podm := BoxMesh.new()
	podm.size = Vector3(0.021, 0.028, 0.02)
	var pmat := StandardMaterial3D.new()
	pmat.albedo_color = Color(0.09, 0.09, 0.11)
	pmat.metallic = 0.3
	pmat.roughness = 0.5
	podm.material = pmat
	pod.mesh = podm
	pod.position = CAM_POS
	pod.rotation_degrees = Vector3(CAM_TILT_DEG, 0, 0)
	root.add_child(pod)
	_hide_from_fpv(pod)


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

	# Scenery (sky, ground, treeline, gates). The tier is baked in at build
	# time — tree count and leaf-card count are geometry, not a live setting —
	# which is why _resolve_quality runs before this.
	var scene := OS.get_environment(SCENE_ENV)
	if scene.is_empty():
		scene = PwWorld.DEFAULT_SCENE
	_world_builder = PwWorld.new()
	_world_builder.build(scene, _world, _q)
	_env = _world_builder.env    # the exposure hunt drives tonemap_exposure
	_sun = _world_builder.sun    # ...and needs the sun for its alignment term

	# drone + FPV camera
	_drone = Node3D.new()
	_world.add_child(_drone)
	_build_drone_model(_drone)

	# FPV camera at the O3's mounting pose (CAM_POS / CAM_TILT_DEG, shared with
	# the camera pod in _add_battery_and_pod). The front ducts + props sit just
	# below the view, exactly like real cinewhoop footage.
	var cam := Camera3D.new()
	cam.fov = 105
	# near 0.005 against the default far 4000 was an 800,000:1 depth ratio --
	# harmless in a 60 m world, severe z-fighting once there is a treeline at
	# 300 m. Nearest airframe geometry (duct lip / blade tips) is ~0.084 m, so
	# 0.02 is still 4x clear of it.
	cam.near = 0.02
	cam.far = 1500.0
	cam.position = CAM_POS
	cam.rotation_degrees = Vector3(CAM_TILT_DEG, 0, 0)
	cam.set_cull_mask_value(FPV_HIDDEN_LAYER, false)   # see _hide_from_fpv
	# Named, because the director adds two more cameras to the scene and
	# fpv_cull_test has to assert against THIS one specifically — "the first
	# Camera3D found by a depth-first walk" stopped being an identity the
	# moment there was more than one.
	cam.name = "FpvCamera"
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

	# Camera rig + captions. Last, because it needs the FPV camera, the OSD
	# label and the goggle material to already exist.
	_director = PwDemoDirector.new()
	_director.setup(self, _world, FPV_HIDDEN_LAYER)
	_director.build_captions(ui)
