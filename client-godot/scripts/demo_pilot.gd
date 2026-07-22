# The demo pilot: a cascaded controller that flies the quad in ACRO.
#
# WHY ACRO. The old PROPWASH_DEMO=acro flew in ANGLE mode despite its name: it
# held a fixed pitch-stick lean and let the firmware's self-levelling do the
# rest. That demonstrates almost nothing — every sim can hold a lean. In ACRO
# the stick commands an angular RATE, so the pilot below has to close every loop
# itself and the firmware's own rate curve and PID loop are genuinely in the
# path. That is the thing worth showing.
#
# THE CASCADE, outermost first:
#
#   position error  -> desired velocity      (clamped to V_MAX)
#   velocity error  -> desired acceleration  (clamped to A_MAX)
#   desired accel + gravity -> desired thrust DIRECTION and magnitude
#   thrust direction vs measured attitude -> desired angular velocity
#   angular-velocity error -> stick deflection   (PI)
#
# The last stage is the interesting one. Stick maps to rate through Betaflight's
# rate curve (rc_rate / expo / srate / rates_type from the pilot's own dump),
# which this file deliberately does NOT model: reading it would mean talking MSP
# mid-run, and MSP traffic arrives on a wall-clock thread and forfeits
# determinism (README "Determinism"). A PI on rate ERROR converges against any
# monotonic curve, so the pilot stays tune-agnostic and the run stays
# reproducible. The I term is what absorbs the unknown curve slope.
#
# ATTITUDE IS TRACKED AS A THRUST VECTOR, not as Euler angles. Roll/pitch
# angles are singular at 90 deg and meaningless past it, and Phase 2 wants
# flips. Rotating the measured body-up onto the desired body-up is well behaved
# all the way to 180 deg, and ballistic rotations simply bypass this stage and
# command a rate directly.
#
# FRAME-INDEXED, NEVER WALL-CLOCK. Every schedule below counts physics frames,
# because at PW_SIM_HZ one frame is exactly one simulation step. Keying off
# accumulated delta would make the demo's own stick stream depend on render
# timing, which is precisely what the ghost/replay act must not do.
extends RefCounted

const PwWorld = preload("res://scripts/world.gd")
const PwProtocol = preload("res://scripts/protocol.gd")

# Chapters this pilot can fly (PROPWASH_DEMO=<name>). `acro`/`flythrough` are
# NOT here — they stay on main.gd's original ANGLE-mode routine, because the
# `flythrough` ctest asserts against that exact trajectory.
#
# `move:<kind>` is the tuning harness: arm, climb to a safe altitude, fly ONE
# maneuver, report. Every maneuver in the library was tuned through it, and it
# is how you bisect which element of a freestyle run went wrong.
const CHAPTERS := ["reel", "freestyle", "turtle", "ghost", "retune", "gateline",
		"probe", "move:<kind>"]

# ------------------------------------------------------------------- signs
# Which body-axis rotation each stick commands, and in which direction.
#
# MEASURED, not derived. The chain from stick to a rate this file can see runs
# through Betaflight's AETR convention, the sim's +z-forward frame, and the
# client's Godot -z-forward mirror — three sign conventions, any of which is
# easy to get backwards and all of which produce a plausible-looking quad that
# flies into the ground. `PROPWASH_DEMO=probe` pulses one axis at a time and
# prints the response; these are its output. Re-run it if the wire frame or the
# AETR mapping ever changes.
#
# Body axes are Godot's: +x right, +y up, +z BACKWARD (forward is -z).
#
# Probe output, +0.30 stick held on one axis at a time:
#   roll  rc[0] -> body w = (-0.001, -0.002, -1.362) rad/s
#   pitch rc[1] -> body w = (-1.363, -0.004, -0.000) rad/s
#   yaw   rc[3] -> body w = (-0.001, -1.323, +0.001) rad/s
# Each stick drives exactly one body axis (cross-coupling is under 0.3%), and
# all three are NEGATIVE-going, so commanding a desired rate means negating it.
const SIGN_ROLL := -1.0    # rc[0] -> rotation about body +z
const SIGN_PITCH := -1.0   # rc[1] -> rotation about body +x
const SIGN_YAW := -1.0     # rc[3] -> rotation about body +y

# Measured stick->rate slope near centre, |rad/s| per unit stick. Only used to
# scale KP_RATE sensibly — the loop does NOT depend on it being right, which is
# the whole point of closing on rate error. Note this is the EXPO-COMPRESSED
# slope: at full stick the curve is far steeper, so a gain tuned here stays
# conservative where it matters.
const RATE_SLOPE := 4.5

# ---------------------------------------------------------------- tunables
const G := 9.81

const KP_POS := 0.9           # position error (m) -> desired velocity (m/s)
const KP_POS_Y := 1.6         # vertical is tighter: altitude is what kills you
const V_MAX := 7.0            # horizontal cruise cap, m/s
const V_MAX_Y := 3.0

const KP_VEL := 2.6           # velocity error -> desired acceleration
const A_MAX := 7.0            # lateral accel cap; 7 m/s^2 is ~36 deg of tilt
# Usable upward reserve for the descent envelope. Nothing like the (TWR-1)*g
# = 20 m/s^2 the spec sheet implies: in a dive the quad is pitched 35 deg over
# so a third of the thrust is pointing sideways, the motors have to spool from
# the unloaded descent, and the sim models prop wash getting WORSE the faster
# you descend (propWashFactor 1.2 — ducts wash early). 8.0 still planted it at
# 0.26 damage; 4.0 flares cleanly and is still a real dive at altitude.
const A_BRAKE := 4.0
const GROUND_MARGIN := 0.5    # flare to a stop this far above the ground

const KP_ATT := 7.0           # attitude error (rad) -> desired body rate (rad/s)
const KP_YAW := 2.5
const W_MAX := 8.0            # desired-rate cap, rad/s

# FEEDFORWARD, and it is not optional. Without it the rate loop had to generate
# the whole stick deflection from error, so at a loop gain of KP_RATE *
# RATE_SLOPE = 0.5 it only ever delivered half the commanded rate promptly and
# left the integrator (0.37 s) to find the rest. That ~0.4 s of attitude lag ate
# the damping out of the position loop above it, and the quad settled into a
# 0.4 Hz pitch limit cycle at 18 deg/s — small enough to hold station, big
# enough to read as the airframe trembling on video.
#
# Feeding w_des through the measured slope makes the stick approximately right
# on the first frame, so P and I only handle what the estimate got wrong. It
# buys response without buying loop gain, which is the whole point: stability
# margin is unchanged.
const KFF_RATE := 1.0 / RATE_SLOPE
const KP_RATE := 0.11         # rate error (rad/s) -> stick
const KI_RATE := 0.30         # ...and its integral, which trims the FF estimate
const I_RATE_CLAMP := 0.8

# Throttle is carried as a FRACTION (0..1) internally and converted to the
# -1..1 stick on the way out: the tilt compensation below is multiplicative on
# thrust, and multiplying a stick value that is negative at hover would push the
# throttle the wrong way.
const THR_HOVER_F0 := 0.36    # measured by the probe; _thr_i trims from here
const KP_THR := 0.075         # climb-rate error (m/s) -> throttle fraction
const KI_THR := 0.030
const THR_I_CLAMP := 0.35
const TILT_COMP_MIN := 0.35   # stop compensating past ~70 deg of bank

# Arming. The firmware needs its gyro calibration to finish and the throttle to
# be low; 5.2 s is what the existing autotest and flythrough both use.
const ARM_FRAME := 1300       # 5.2 s at 250 Hz

# rc channel indices, AETR + switches (see main.gd _update_keyboard_rc)
const CH_ROLL := 0
const CH_PITCH := 1
const CH_THR := 2
const CH_YAW := 3
const CH_ARM := 4
const CH_ANGLE := 5
const CH_TURTLE := 6


var _main: Node3D
var _chapter := ""
var _frame := 0
var _rc := [0.0, 0.0, -1.0, 0.0, -1.0, -1.0, -1.0, -1.0]

var _i_rate := Vector3.ZERO    # rate-loop integrator, body frame
var _thr_i := 0.0              # learns the hover throttle fraction
var _armed_seen := false
var _finished := false
var _fail_reasons: Array[String] = []

# telemetry the chapters assert on
var _min_alt := 1e9
var _max_dmg := 0.0
var _gate_max_absx := 0.0
var _gate_min_y := 1e9
var _gate_max_y := -1e9


# `reel` is the capture chapter: the working acts chained into ONE continuous
# take, no cuts to black and no restarts between them. `retune` is deliberately
# NOT in it — it is blocked (see its note), and even working it would forfeit
# determinism for the ghost act that follows.
const REEL := ["freestyle", "turtle", "ghost"]
var _reel: Array = []
var _reel_i := 0


func begin(main: Node3D, chapter: String) -> void:
	_main = main
	if chapter == "reel":
		_reel = REEL.duplicate()
		_reel_i = 0
		chapter = _reel[0]
		print("[demo] reel: %s" % " -> ".join(REEL))
	_chapter = chapter


# Chain to the next act of a reel. Each act owns its own state, so advancing
# means resetting the shared controller bits and letting the next one start
# from wherever the last one left the quad — which is the point of a single
# take. Returns true if the whole reel is done.
func _end_chapter() -> void:
	if _advance_reel():
		_finished = true


func _advance_reel() -> bool:
	if _reel.is_empty() or _reel_i + 1 >= _reel.size():
		return true
	_reel_i += 1
	_chapter = _reel[_reel_i]
	print("[demo] --- reel act %d/%d: %s ---"
			% [_reel_i + 1, _reel.size(), _chapter])
	_i_rate = Vector3.ZERO
	_thr_i = 0.0
	_rot_acc = 0.0
	caption = ""
	subcaption = ""
	return false


func finished() -> bool:
	return _finished


# --------------------------------------------------------------- per frame
func update(dt: float) -> Array:
	_frame += 1
	var armed: bool = not _main._last_out.is_empty() and _main._last_out.armed
	if armed:
		_armed_seen = true

	# The turtle chapter owns every channel including ARM — it deliberately
	# disarms mid-run, which the shared arm schedule and the disarmed-idle
	# early-out below would both fight.
	if _chapter == "turtle":
		_run_turtle(dt)
		_track_telemetry()
		return _rc
	if _chapter == "ghost":
		_run_ghost(dt)
		_track_telemetry()
		return _rc
	if _chapter == "retune":
		_run_retune(dt)
		_track_telemetry()
		return _rc

	# ARM on ch5 once the gyro calibration has had time to finish; ANGLE off
	# (ch6 low) for the whole run — this is the point of the exercise.
	_rc[CH_ARM] = 1.0 if _frame >= ARM_FRAME else -1.0
	_rc[CH_ANGLE] = -1.0
	_rc[CH_TURTLE] = -1.0

	if not armed:
		# Disarmed: sticks centred, throttle down. The firmware refuses to arm
		# on anything else, and letting the integrators run against a quad that
		# cannot move would just wind them up.
		_rc[CH_ROLL] = 0.0
		_rc[CH_PITCH] = 0.0
		_rc[CH_YAW] = 0.0
		_rc[CH_THR] = -1.0
		_i_rate = Vector3.ZERO
		_thr_i = 0.0
		return _rc

	match _chapter:
		"probe":
			_run_probe(dt)
		"gateline":
			_run_flythrough(dt)
		"freestyle":
			_run_steps(dt)
		_ when _chapter.begins_with("move:"):
			_run_steps(dt)
		_:
			# loud, not a silent fallback: a typo in PROPWASH_DEMO used to fly
			# *something*, which reads as the demo working
			_fail_reasons.append("unknown chapter '%s' (have: %s)"
					% [_chapter, ", ".join(CHAPTERS)])
			_finished = true

	_track_telemetry()
	return _rc


func _track_telemetry() -> void:
	if not _main._last_out.is_empty():
		for d in _main._last_out.get("prop_damage", []):
			_max_dmg = maxf(_max_dmg, d)
	var p: Vector3 = _main._pos
	# clearance while crossing each gate plane: with solid gates, "passed the
	# gate" has to also mean "did not clip it"
	for gz in PwWorld.GATE_Z:
		if absf(p.z - gz) < 0.5:
			_gate_max_absx = maxf(_gate_max_absx, absf(p.x))
			_gate_min_y = minf(_gate_min_y, p.y)
			_gate_max_y = maxf(_gate_max_y, p.y)


# ------------------------------------------------------------- the cascade
# Fly toward `target`, holding heading `yaw_des` (radians, 0 = -z). Emits into
# _rc; returns the remaining distance so a chapter can sequence on arrival.
func _fly_to(dt: float, target: Vector3, yaw_des: float, v_max: float,
		v_max_y: float = V_MAX_Y) -> float:
	var pos: Vector3 = _main._pos
	var vel: Vector3 = _main._linvel
	var basis := Basis(_main._rot)

	# --- position -> velocity (horizontal and vertical have their own gains:
	# altitude error is the one that ends a flight)
	var err := target - pos
	var v_des := Vector3(err.x, 0.0, err.z) * KP_POS
	if v_des.length() > v_max:
		v_des = v_des.normalized() * v_max
	v_des.y = clampf(err.y * KP_POS_Y, -v_max_y, v_max_y)

	# Never descend faster than you can still pull out of. A quad's upward
	# reserve is (TWR - 1) * g, about 20 m/s^2 at 3:1, and half of that is the
	# honest working figure once the pack has sagged; v = sqrt(2*a*h) is the
	# speed that reserve can arrest in the remaining height.
	#
	# Without this the first dive commanded 10 m/s and simply ARRIVED at the
	# ground with 10 m/s still on it — 0.21 prop damage, a metre below the
	# altitude it was asked to hold. Clamping the descent rate globally instead
	# would have made the dive slow everywhere; this way it falls fast where
	# there is room and flares where there is not.
	var head := maxf(pos.y - GROUND_MARGIN, 0.0)
	v_des.y = maxf(v_des.y, -sqrt(2.0 * A_BRAKE * head))

	# --- velocity -> desired acceleration (horizontal only; vertical is the
	# throttle loop's job)
	var a_des := Vector3(v_des.x - vel.x, 0.0, v_des.z - vel.z) * KP_VEL
	if a_des.length() > A_MAX:
		a_des = a_des.normalized() * A_MAX

	_attitude_and_rate(dt, a_des, yaw_des, basis)
	_throttle(dt, v_des.y - vel.y, basis)
	return err.length()


# Point the thrust vector along (a_des + g), turn the attitude error into a
# desired angular velocity, and run the rate PI that produces the sticks.
func _attitude_and_rate(dt: float, a_des: Vector3, yaw_des: float,
		basis: Basis) -> void:
	var z_des := (a_des + Vector3(0.0, G, 0.0)).normalized()
	var z_b := basis.y                       # measured body up, world frame

	# Rotation taking measured body-up onto desired body-up. Axis-angle rather
	# than Euler: no singularity at 90 deg, and it degrades gracefully when the
	# quad is inverted instead of flipping sign discontinuously.
	var axis := z_b.cross(z_des)
	var s := axis.length()
	var w_des := Vector3.ZERO
	if s > 1e-6:
		w_des = axis / s * (atan2(s, z_b.dot(z_des)) * KP_ATT)

	# Heading, about world up. Godot forward is -z, so a heading of 0 is -z and
	# positive is to the left. _yaw_rate overrides the hold outright — that is
	# how yaw_spin keeps position control while spinning the nose.
	if _yaw_rate != 0.0:
		w_des += Vector3(0.0, _yaw_rate, 0.0)
	else:
		var fwd := basis * Vector3(0, 0, -1)
		var yaw_now := atan2(-fwd.x, -fwd.z)
		w_des += Vector3(0.0, KP_YAW * wrapf(yaw_des - yaw_now, -PI, PI), 0.0)

	if w_des.length() > W_MAX:
		w_des = w_des.normalized() * W_MAX

	_rate_loop_body(dt, basis.inverse() * w_des, basis)


# The one stage that touches sticks, and the only place the unknown rate curve
# matters. The core's angular_velocity is WORLD-frame (physics.cpp derives the
# body-frame gyro from it by inverse transform, and main.gd only mirrors z on
# the way in), so the measurement is rotated into the body here and the desired
# rate arrives already in body axes — which is what lets the ballistic
# maneuvers command a body rate directly.
func _rate_loop_body(dt: float, w_des_body: Vector3, basis: Basis) -> void:
	var w_meas: Vector3 = _main._angvel
	var err := w_des_body - basis.inverse() * w_meas

	_i_rate += err * (KI_RATE * dt)
	_i_rate.x = clampf(_i_rate.x, -I_RATE_CLAMP, I_RATE_CLAMP)
	_i_rate.y = clampf(_i_rate.y, -I_RATE_CLAMP, I_RATE_CLAMP)
	_i_rate.z = clampf(_i_rate.z, -I_RATE_CLAMP, I_RATE_CLAMP)

	var u := w_des_body * KFF_RATE + err * KP_RATE + _i_rate
	_rc[CH_ROLL] = clampf(SIGN_ROLL * u.z, -1.0, 1.0)
	_rc[CH_PITCH] = clampf(SIGN_PITCH * u.x, -1.0, 1.0)
	_rc[CH_YAW] = clampf(SIGN_YAW * u.y, -1.0, 1.0)


# PI on climb-rate error. The integrator IS the hover-throttle estimate: it
# absorbs pack sag, prop damage and the throttle-to-thrust curve without any of
# them being modelled here.
func _throttle(dt: float, vy_err: float, basis: Basis) -> void:
	_thr_i = clampf(_thr_i + KI_THR * vy_err * dt, -THR_I_CLAMP, THR_I_CLAMP)
	# banked flight needs T/cos(tilt) to hold the same vertical component
	var tilt_comp := 1.0 / maxf(basis.y.y, TILT_COMP_MIN)
	var f := (THR_HOVER_F0 + _thr_i + KP_THR * vy_err) * tilt_comp
	_rc[CH_THR] = clampf(clampf(f, 0.0, 1.0) * 2.0 - 1.0, -1.0, 1.0)


# ------------------------------------------------------------------ probe
# PROPWASH_DEMO=probe — measure what each stick actually does.
#
# Climb on the throttle loop alone (which needs no sign knowledge), then pulse
# one axis at a time and report the body-frame rate response. The opposite pulse
# cancels most of the rotation so the next axis starts from something like
# level. The quad is not expected to fly well during this; the numbers are the
# product.
const PROBE_CLIMB_FRAMES := 1000     # 4 s of climbing to ~6 m
const PROBE_PULSE := 0.30
const PROBE_ON := 75                 # 0.3 s
const PROBE_SETTLE := 100            # 0.4 s
var _probe_axis := 0
var _probe_t := 0
var _probe_acc := Vector3.ZERO
var _probe_n := 0


func _run_probe(dt: float) -> void:
	var basis := Basis(_main._rot)
	var pos: Vector3 = _main._pos
	var vel: Vector3 = _main._linvel
	# hold ~6 m the whole time; no attitude control at all
	_throttle(dt, clampf(6.0 - pos.y, -2.0, 2.5) - vel.y, basis)
	_rc[CH_ROLL] = 0.0
	_rc[CH_PITCH] = 0.0
	_rc[CH_YAW] = 0.0
	if _frame < ARM_FRAME + PROBE_CLIMB_FRAMES:
		return
	if _probe_axis >= 3:
		print("[probe] hover throttle fraction ~%.3f (stick %.3f)" % [
				THR_HOVER_F0 + _thr_i, _rc[CH_THR]])
		print("[probe] done")
		_finished = true
		return

	var ch: int = [CH_ROLL, CH_PITCH, CH_YAW][_probe_axis]
	var label: String = ["roll rc[0]", "pitch rc[1]", "yaw rc[3]"][_probe_axis]
	_probe_t += 1
	if _probe_t <= PROBE_ON:
		_rc[ch] = PROBE_PULSE
		# average over the back half, once the rate has had time to build
		if _probe_t > PROBE_ON / 2:
			_probe_acc += basis.inverse() * _main._angvel
			_probe_n += 1
	elif _probe_t <= PROBE_ON * 2:
		_rc[ch] = -PROBE_PULSE          # cancel the rotation
	elif _probe_t <= PROBE_ON * 2 + PROBE_SETTLE:
		_rc[ch] = 0.0
	else:
		var w: Vector3 = _probe_acc / maxf(float(_probe_n), 1.0)
		# report which body axis moved most, and in which direction
		var mag := [absf(w.x), absf(w.y), absf(w.z)]
		var dom: int = mag.find(mag.max())
		var axis_name: String = ["+x (pitch)", "+y (yaw)", "+z (roll)"][dom]
		var dom_val: float = w[dom]
		print("[probe] %-12s stick %+.2f -> body w = (%+.3f, %+.3f, %+.3f) rad/s"
				% [label, PROBE_PULSE, w.x, w.y, w.z]
				+ "  dominant %s  sign %s" % [
						axis_name, "+" if dom_val > 0.0 else "-"])
		_probe_axis += 1
		_probe_t = 0
		_probe_acc = Vector3.ZERO
		_probe_n = 0


# ------------------------------------------------------- flythrough (acro)
# The same gate run the ANGLE-mode demo flies, in ACRO. This is the acceptance
# test for the cascade above: same clearance margins, same zero-damage
# requirement, but now every loop is closed by this file instead of by the
# firmware's self-levelling.
const FT_ALT := 1.15
const FT_END_Z := -34.0
const FT_HOLD_FRAMES := 1250          # 5 s to climb and settle before cruising
const FT_TIMEOUT := 250 * 34


func _run_flythrough(dt: float) -> void:
	var t := _frame - ARM_FRAME
	var target: Vector3
	if t < FT_HOLD_FRAMES:
		target = Vector3(0.0, FT_ALT, 0.0)          # climb and settle on the pad
	else:
		target = Vector3(0.0, FT_ALT, FT_END_Z)     # cruise down the gate line
	_fly_to(dt, target, 0.0, V_MAX)

	if t >= FT_HOLD_FRAMES:
		_min_alt = minf(_min_alt, _main._pos.y)

	var arrived: bool = _main._pos.z < FT_END_Z + 1.0
	if arrived or _frame > FT_TIMEOUT:
		_report_flythrough(arrived)
		_finished = true


func _report_flythrough(arrived: bool) -> void:
	var p: Vector3 = _main._pos
	if not _armed_seen:
		_fail_reasons.append("never armed")
	if not arrived:
		_fail_reasons.append("did not reach z=%.1f (ended %.1f)" % [FT_END_Z, p.z])
	if _min_alt < 0.7:
		_fail_reasons.append("dropped to %.2f m during the cruise" % _min_alt)
	if absf(p.x) > 3.0:
		_fail_reasons.append("ended %.2f m off the line" % p.x)
	# posts at x = +/-1.2, bar at 2.05: the same real clearance margins the
	# ANGLE-mode flythrough asserts
	if _gate_max_absx > 0.9 or _gate_min_y < 0.35 or _gate_max_y > 1.85:
		_fail_reasons.append("gate clearance |x|=%.2f y=[%.2f, %.2f]"
				% [_gate_max_absx, _gate_min_y, _gate_max_y])
	if _max_dmg > 0.05:
		_fail_reasons.append("prop damage %.3f — it clipped something" % _max_dmg)

	print("[demo] acro fly-through: end=%s min_alt=%.2f armed=%s"
			% [str(p), _min_alt, str(_armed_seen)])
	print("[demo] gate clearance: |x|max=%.2f y=[%.2f, %.2f] max_dmg=%.3f"
			% [_gate_max_absx, _gate_min_y, _gate_max_y, _max_dmg])


# Non-empty means the chapter failed; each entry is a human-readable reason.
func failures() -> Array[String]:
	return _fail_reasons


# ======================================================== maneuver library
#
# A chapter is a list of step dictionaries; each step is one maneuver. Steps run
# in order and each reports when it is done, so the sequence is data and the
# control law is code.
#
# Two families, and the split matters:
#
#   TRACKING steps (goto/hold/dive/orbit/yaw_spin) go through the full cascade
#   in _fly_to. They track a position and are what gets the quad from one trick
#   to the next.
#
#   BALLISTIC steps (flip/roll/split_s) do NOT. A 360 deg rotation has no
#   meaningful "desired attitude" halfway round, and thrust only points one way,
#   so a position controller fights the maneuver instead of flying it. Real
#   pilots throw full stick and manage the throttle; that is exactly what these
#   do. The rate loop is left running on the other two axes to keep the
#   rotation clean, and the driven axis's integrator is held at zero so it
#   cannot wind up against a stick it is not driving.
var _steps: Array = []
var _step := 0
var _step_frame := 0
var _step_init := false
var _rot_acc := 0.0        # accumulated rotation for the current ballistic step
var _phase := 0            # sub-phase within a multi-part maneuver
var _orbit_ang := 0.0
var _yaw_rate := 0.0       # non-zero overrides heading hold (yaw_spin)
var _steps_done := 0

# What the director should be showing, and what the caption should say. Read by
# main.gd every render frame. Held here rather than pushed, so a step only has
# to declare its shot once in the chapter script.
var cam_hint := "chase"
var caption := ""
var subcaption := ""

const DEFAULT_BUDGET := 250 * 14      # frames; every step must finish or fail


func _hover_f() -> float:
	return THR_HOVER_F0 + _thr_i


func _set_thr(f: float) -> void:
	_rc[CH_THR] = clampf(clampf(f, 0.0, 1.0) * 2.0 - 1.0, -1.0, 1.0)


func _run_steps(dt: float) -> void:
	if _steps.is_empty():
		_steps = _build_chapter()
		if _steps.is_empty():
			_finished = true
			return
	if _step >= _steps.size():
		_report_steps()
		_end_chapter()
		return

	var s: Dictionary = _steps[_step]
	var basis := Basis(_main._rot)
	if not _step_init:
		_begin_step(s, basis)
		_step_init = true
	_step_frame += 1

	var done := _step_update(dt, s, basis)

	# Hard per-step frame budget. A maneuver that never satisfies its exit
	# condition has to fail the run loudly — the alternative is a demo that
	# hangs forever on a step nobody notices is stuck.
	var budget: int = s.get("budget", DEFAULT_BUDGET)
	if not done and _step_frame > budget:
		_fail_reasons.append("step %d (%s) did not finish in %.1f s"
				% [_step, s.get("k", "?"), budget / 250.0])
		done = true
	if done:
		_steps_done += 1
		_step += 1
		_step_frame = 0
		_step_init = false


func _begin_step(s: Dictionary, basis: Basis) -> void:
	_rot_acc = 0.0
	_phase = 0
	_yaw_rate = 0.0
	var p: Vector3 = _main._pos
	if s.k == "orbit":
		# start the sweep from wherever the quad already is, so the target does
		# not jump across the circle on the first frame and yank it sideways
		var c: Vector3 = s.center
		_orbit_ang = atan2(p.z - c.z, p.x - c.x)
	# a step without a shot of its own keeps the previous one — cutting on
	# every waypoint would be unwatchable
	cam_hint = s.get("cam", "")
	if s.has("cap"):
		caption = s.cap
		subcaption = s.get("sub", "")
	print("[demo] step %d/%d %s  at %s" % [
			_step + 1, _steps.size(), _step_label(s), _fmt(p)])


func _step_label(s: Dictionary) -> String:
	match s.k:
		"flip":
			return "flip %s %s" % [s.axis, "forward" if s.dir > 0.0 else "back"]
		"split_s":
			return "split-S"
		"orbit":
			return "orbit r=%.0f x%.1f" % [s.radius, s.revs]
		"yaw_spin":
			return "yaw spin x%.1f" % s.revs
		_:
			return str(s.k)


func _step_update(dt: float, s: Dictionary, basis: Basis) -> bool:
	match s.k:
		"goto":
			return _goto_step(dt, s, basis)
		"hold":
			_fly_to(dt, s.to, s.get("yaw", 0.0), V_MAX)
			if s.get("diag", false):
				_diag(dt)
			return _step_frame >= int(s.frames)
		"dive":
			return _goto_step(dt, s, basis)
		"flip", "roll":
			return _ballistic_step(dt, s, basis)
		"split_s":
			return _split_s_step(dt, basis)
		"orbit":
			return _orbit_step(dt, s, basis)
		"yaw_spin":
			return _yaw_spin_step(dt, s, basis)
	_fail_reasons.append("unknown maneuver '%s'" % s.get("k", "?"))
	return true


# --- tracking maneuvers ---------------------------------------------------
func _goto_step(dt: float, s: Dictionary, _basis: Basis) -> bool:
	var to: Vector3 = s.to
	var left := _fly_to(dt, to, s.get("yaw", 0.0), s.get("v", V_MAX),
			s.get("vy", V_MAX_Y))
	var vel: Vector3 = _main._linvel
	# "arrived" means arrived AND settled: a waypoint hit at 7 m/s hands the
	# next maneuver an entry state it was not designed for
	return left < s.get("tol", 1.2) and vel.length() < s.get("v_tol", 2.5)


func _orbit_step(dt: float, s: Dictionary, _basis: Basis) -> bool:
	var c: Vector3 = s.center
	var r: float = s.radius
	var rate: float = s.get("rate", 0.75)      # rad/s; ground speed = r * rate
	# _rot_acc is the swept angle (reset per step); _orbit_ang was seeded from
	# the entry bearing so the target never jumps across the circle
	_rot_acc += rate * dt
	_orbit_ang += rate * dt
	var tgt := c + Vector3(cos(_orbit_ang) * r, 0.0, sin(_orbit_ang) * r)
	tgt.y = s.alt
	# nose pointed at the centre the whole way round — this is what makes an
	# orbit read as an orbit rather than as a wide turn
	var p: Vector3 = _main._pos
	var to_c := c - p
	_fly_to(dt, tgt, atan2(-to_c.x, -to_c.z), V_MAX)
	return _rot_acc >= TAU * s.revs


func _yaw_spin_step(dt: float, s: Dictionary, basis: Basis) -> bool:
	var rate: float = s.get("rate", 4.0)       # rad/s
	_yaw_rate = rate
	_fly_to(dt, s.to, 0.0, 2.0)
	var w: Vector3 = _main._angvel
	_rot_acc += absf((basis.inverse() * w).y) * dt
	return _rot_acc >= TAU * s.revs


# --- ballistic maneuvers --------------------------------------------------
# Throttle schedule through a rotation, as a multiple of the learned hover
# throttle. Punch in to buy the height the rotation costs, unload through the
# inverted arc (thrust points at the sky there — holding hover throttle
# upside-down is how you plant it), then catch it on the way out.
const BAL_PUNCH := 1.80
const BAL_PUNCH_UNTIL := 0.13     # fraction of the rotation
const BAL_UNLOAD := 0.20
const BAL_CATCH_FROM := 0.84
const BAL_CATCH := 1.55


func _ballistic_throttle(frac: float) -> void:
	var hov := _hover_f()
	if frac < BAL_PUNCH_UNTIL:
		_set_thr(hov * BAL_PUNCH)
	elif frac < BAL_CATCH_FROM:
		_set_thr(hov * BAL_UNLOAD)
	else:
		_set_thr(hov * BAL_CATCH)


# Drive one axis with full stick and hold the other two quiet. `dir` is the
# stick sign; because every SIGN_* is negative (see the probe output), the body
# rotates in the -dir direction, which is what the accumulator corrects for.
func _drive_axis(dt: float, ch: int, ax: int, dir: float, basis: Basis) -> void:
	_rate_loop_body(dt, Vector3.ZERO, basis)
	_rc[ch] = dir
	# the driven axis's integrator must not wind up against a stick the rate
	# loop is not actually setting
	if ax == 0:
		_i_rate.x = 0.0
	else:
		_i_rate.z = 0.0
	var w: Vector3 = _main._angvel
	var w_body := basis.inverse() * w
	var wa: float = w_body[ax]
	_rot_acc += wa * -dir * dt


func _ballistic_step(dt: float, s: Dictionary, basis: Basis) -> bool:
	var is_roll: bool = s.k == "roll" or s.get("axis", "pitch") == "roll"
	var ch: int = CH_ROLL if is_roll else CH_PITCH
	var ax: int = 2 if is_roll else 0
	var dir: float = s.get("dir", -1.0)
	var total: float = TAU * s.get("turns", 1.0)

	_drive_axis(dt, ch, ax, dir, basis)
	_ballistic_throttle(_rot_acc / total)
	return _rot_acc >= total


# Split-S: half roll onto your back, then pull through the second half of a
# loop and come out level, heading reversed and lower. Two phases, because the
# exit condition of the first is an angle and of the second is being upright
# again — a single accumulator cannot express that.
func _split_s_step(dt: float, basis: Basis) -> bool:
	if _phase == 0:
		_drive_axis(dt, CH_ROLL, 2, 1.0, basis)
		_set_thr(_hover_f() * 0.25)
		if _rot_acc >= PI * 0.92:
			_phase = 1
			_rot_acc = 0.0
		return false
	# pull: nose up in the body frame, which from inverted swings the nose down
	# through the vertical and back to level
	_drive_axis(dt, CH_PITCH, 0, -1.0, basis)
	_set_thr(_hover_f() * (0.25 if _rot_acc < PI * 0.7 else 1.45))
	return _rot_acc >= PI * 0.85 and basis.y.y > 0.55


# --------------------------------------------------------- chapter scripts
func _build_chapter() -> Array:
	if _chapter.begins_with("move:"):
		return _build_move_chapter(_chapter.substr(5))
	if _chapter == "freestyle":
		return _freestyle_steps()
	return []


# One maneuver, in isolation, from a safe altitude — the tuning harness.
func _build_move_chapter(kind: String) -> Array:
	var setup: Array = [
		{k = "goto", to = Vector3(0, 1.5, -2.0), tol = 0.8, budget = 250 * 10},
		{k = "goto", to = Vector3(0, 6.0, -6.0), tol = 1.0, budget = 250 * 10},
	]
	var move: Array
	match kind:
		"flip":
			move = [{k = "flip", axis = "pitch", dir = -1.0, budget = 250 * 5}]
		"frontflip":
			move = [{k = "flip", axis = "pitch", dir = 1.0, budget = 250 * 5}]
		"roll":
			move = [{k = "roll", dir = 1.0, budget = 250 * 5}]
		"split_s":
			move = [{k = "split_s", budget = 250 * 6}]
		"dive":
			move = [
				{k = "goto", to = Vector3(0, 16.0, -6.0), tol = 1.2,
						budget = 250 * 14},
				{k = "dive", to = Vector3(0, 2.6, -14.0), vy = 12.0, v = 9.0,
						tol = 1.8, v_tol = 3.0, budget = 250 * 12},
			]
		"orbit":
			# around the tower, exactly as the freestyle chapter does it: an
			# orbit over open ground proves nothing and an orbit through the
			# near trees proves the wrong thing
			move = [{k = "orbit", center = TOWER_XZ, radius = 6.0,
					alt = 4.0, revs = 1.0, budget = 250 * 22}]
		"yaw_spin":
			move = [{k = "yaw_spin", to = Vector3(0, 6.0, -6.0), revs = 2.0,
					budget = 250 * 12}]
		"hover":
			# not a maneuver — the hover-quality bench. Prints body-rate RMS and
			# oscillation frequency so "it trembles" becomes a number.
			move = [{k = "hold", to = Vector3(0, 6.0, -6.0), frames = 250 * 10,
					diag = true, budget = 250 * 12}]
		_:
			_fail_reasons.append("unknown move '%s'" % kind)
			return []
	# every maneuver has to be RECOVERED from, not just entered: a flip that
	# ends in a spiral is not a flip
	return setup + move + [
		{k = "goto", to = Vector3(0, 6.0, -6.0), tol = 2.0, budget = 250 * 10},
		{k = "hold", to = Vector3(0, 6.0, -6.0), frames = 250},
	]


# The freestyle run: the reel's flying half. Ordered so the quad is always set
# up for the next element — energy and heading carry through rather than being
# reset between tricks.
func _freestyle_steps() -> Array:
	return [
		# off the pad and straight down the race gates
		{k = "goto", to = Vector3(0, 1.5, -2.0), tol = 0.8, budget = 250 * 10,
				cam = "los", cap = "Betaflight 4.5.2, in the loop",
				sub = "the real firmware — real PIDs, real rates, real arming logic"},
		{k = "goto", to = Vector3(0, 1.15, -24.0), tol = 1.5, v_tol = 4.0,
				budget = 250 * 14, cam = "fpv", cap = "ACRO — self-levelling OFF",
				sub = "sticks command rates; the pilot's own rate curve flies it"},
		# climb out and backflip in front of the loop gate
		{k = "goto", to = Vector3(0, 5.0, -26.0), tol = 1.2, budget = 250 * 10,
				cam = "chase", cap = ""},
		{k = "flip", axis = "pitch", dir = -1.0, budget = 250 * 5,
				cam = "los", cap = "Backflip",
				sub = "full stick and a throttle profile — no animation, no keyframes"},
		{k = "goto", to = Vector3(0, 5.0, -26.0), tol = 2.0, budget = 250 * 10,
				cap = ""},
		# through the tall loop gate, then weave the slalom
		{k = "goto", to = Vector3(0, 2.0, -33.0), tol = 1.5, v_tol = 4.0,
				budget = 250 * 10, cam = "fpv", cap = "Gates are solid",
				sub = "clipping one costs props and momentum, not a lap penalty"},
		{k = "goto", to = Vector3(-1.9, 1.6, -37.5), tol = 1.2, v_tol = 4.0},
		{k = "goto", to = Vector3(1.9, 1.6, -40.5), tol = 1.2, v_tol = 4.0},
		{k = "goto", to = Vector3(-1.9, 1.6, -43.5), tol = 1.2, v_tol = 4.0},
		{k = "goto", to = Vector3(1.9, 1.6, -46.5), tol = 1.2, v_tol = 4.0,
				cap = ""},
		# roll on the way back up the field
		{k = "goto", to = Vector3(0, 6.0, -40.0), tol = 1.5, budget = 250 * 12,
				cam = "chase"},
		{k = "roll", dir = 1.0, budget = 250 * 5, cam = "los", cap = "Roll",
				sub = "thrust only points one way — the throttle has to come off"},
		# Up the scaffold tower and dive off it. Routed UP THE SIDE at x = -18
		# and over the top from behind: the direct line from the slalom to the
		# platform passes through the tower at about 13 m, and a 14 m tower is
		# exactly the kind of obstacle a straight-line waypoint controller
		# happily flies into.
		{k = "goto", to = Vector3(-18.0, 12.0, -30.0), tol = 2.0, budget = 250 * 16,
				cam = "chase", cap = ""},
		{k = "goto", to = Vector3(-12.0, 16.5, -23.0), tol = 1.5, budget = 250 * 12},
		# ...and the dive goes AWAY from the tower in z, never back across it
		{k = "dive", to = Vector3(-12.0, 2.8, -29.0), vy = 12.0, v = 9.0,
				tol = 2.0, v_tol = 3.5, budget = 250 * 12,
				cam = "fpv", cap = "15 m dive",
				sub = "the wobble on the pull-out is modelled prop wash, not noise"},
		# orbit it at low level, where the height reads against the structure
		{k = "orbit", center = TOWER_XZ, radius = 6.0, alt = 4.0, revs = 1.0,
				budget = 250 * 22, cam = "los", cap = "Orbit",
				sub = "ground effect, battery sag and drag are all in the loop"},
		# climb out to the left and split-S back down
		{k = "goto", to = Vector3(-20.0, 10.0, -14.0), tol = 1.8, budget = 250 * 14,
				cam = "chase", cap = ""},
		{k = "split_s", budget = 250 * 6, cam = "los", cap = "Split-S",
				sub = "half roll inverted, then pull through"},
		{k = "goto", to = Vector3(-16.0, 4.0, -10.0), tol = 2.0, budget = 250 * 10,
				cap = ""},
		# yaw spin on the spot, then home
		{k = "yaw_spin", to = Vector3(-16.0, 4.0, -10.0), revs = 2.0,
				budget = 250 * 12, cam = "fpv", cap = "Yaw spin",
				sub = "the feed mushes on the whip — the codec is losing its budget"},
		{k = "goto", to = Vector3(0, 2.0, -3.0), tol = 1.5, budget = 250 * 16,
				cam = "chase", cap = ""},
		{k = "hold", to = Vector3(0, 2.0, -3.0), frames = 250, cam = "los",
				cap = "propwash", sub = "real Betaflight, deterministic, open source"},
	]

const TOWER_XZ := Vector3(-12.0, 0.0, -16.0)   # matches world.gd TOWER_POS


func _report_steps() -> void:
	var p: Vector3 = _main._pos
	var flags: int = 0
	if not _main._last_out.is_empty():
		flags = _main._last_out.get("crash_flags", 0)
	if not _armed_seen:
		_fail_reasons.append("never armed")
	if _steps_done < _steps.size():
		_fail_reasons.append("only %d of %d maneuvers completed"
				% [_steps_done, _steps.size()])
	if flags & 1:
		_fail_reasons.append("ended with the structural-crash latch set")
	if _max_dmg > 0.20:
		_fail_reasons.append("prop damage %.3f — it hit something hard" % _max_dmg)
	if p.y < 0.5:
		_fail_reasons.append("ended on the ground at %.2f m" % p.y)
	# Hover quality is a gated number, not an impression. A marginally stable
	# cascade still holds station — it just buzzes, which no position or damage
	# assertion can see, and which reads on video as the airframe trembling.
	if _chapter == "move:hover":
		if _dg_rms.x > HOVER_RMS_LIMIT or _dg_rms.z > HOVER_RMS_LIMIT:
			_fail_reasons.append("hover is not settled: body-rate rms "
					+ "pitch %.3f roll %.3f rad/s (limit %.2f)"
					% [_dg_rms.x, _dg_rms.z, HOVER_RMS_LIMIT])
		else:
			print("[demo] settled hover: body-rate rms pitch %.3f roll %.3f rad/s"
					% [_dg_rms.x, _dg_rms.z])
	print("[demo] freestyle: %d/%d maneuvers, end=%s max_dmg=%.3f flags=%d"
			% [_steps_done, _steps.size(), _fmt(p), _max_dmg, flags])


func _fmt(v: Vector3) -> String:
	return "(%.1f, %.1f, %.1f)" % [v.x, v.y, v.z]


# =========================================================== retune chapter
#
# Change the tune over the real Betaflight CLI between two IDENTICAL stick
# inputs, and watch the same input produce a different quad. Same text protocol
# the Configurator drives on TCP 5761; nothing here knows anything about
# propwash. Measured result: 683 -> 1031 deg/s peak roll rate, x1.51.
#
# The sequence is dictated by the firmware, and every step of it was learned by
# having the obvious version fail:
#
#   1. PIN the before-value (`set roll_srate = 60`, `save`). Without this the
#      chapter measures whatever the eeprom already held — and since step 5
#      SAVES, a second run starts already retuned and reads a silent x1.00.
#   2. FLY the roll and measure peak body roll rate.
#   3. LAND AND DISARM. Stock Betaflight ignores the `#` CLI escape while
#      armed — fc/tasks.c:147 passes MSP_SKIP_NON_MSP_DATA when
#      ARMING_FLAG(ARMED) — so the banner never comes. Raw MSP keeps working
#      throughout, which is what makes this so confusing from outside: the port
#      answers, the connection is accepted, `#` just does nothing.
#   4. RETUNE with `set` + `save`. NOT `exit`: cliExit prints "unsaved changes
#      lost" and calls cliReboot(), so a RAM-only `set` is discarded on the way
#      out. `save` writes the eeprom and reboots.
#   5. PW_CMD_RESET. ARMING_DISABLED_CLI survives the in-process reboot
#      (BF::init() does not clear firmware statics), so without this the quad
#      can never arm again. The reset rewinds the statics to the pre-CLI
#      snapshot and BF::init() re-reads the eeprom — so the saved retune
#      survives while the arming block clears.
#   6. RE-ARM only once arming_disable reads 0. Raising the switch during the
#      post-reset gyro calibration makes Betaflight latch "flip ARM switch OFF
#      then ON" and refuse forever.
#   7. FLY THE SAME INPUT and compare peak rate.
#
# THIS CHAPTER FORFEITS DETERMINISM — CLI/MSP traffic arrives on dyad's
# wall-clock thread — so it is excluded from `reel`, which contains the ghost
# act. It also issues a real `save`, so run it against a throwaway eeprom
# (the demo_retune ctest goes through tools/tester/demo_chapter_check.py for
# exactly that reason).
const RT_ALT := Vector3(0.0, 8.0, -5.0)
const RT_LAND_TO := Vector3(0.0, 0.05, -5.0)
const RT_SETTLE := 250          # consecutive settled frames before rolling
const RT_ROLL_FRAMES := 150      # 0.6 s of full stick
const RT_RECOVER := 250 * 4
const RT_SRATE_FROM := 60        # what the retune sets roll_srate to...
const RT_SRATE_TO := 100         # ...before and after
# The second roll must be clearly faster. Betaflight's super-rate curve is
# non-linear, so this is a "meaningfully different" threshold, not an exact
# ratio prediction.
const RT_MIN_RATIO := 1.25

enum {RT_PRESET, RT_RESET0, RT_CLIMB, RT_ROLL_A, RT_RECOVER_A, RT_LAND, RT_CLI,
		RT_RESET, RT_REARM, RT_ROLL_B, RT_RECOVER_B, RT_DONE}

var _rt_phase := RT_PRESET
var _rt_t := 0
var _rt_peak_a := 0.0
var _rt_peak_b := 0.0
var _rt_cli_ok := false
var _rt_cli_note := ""
var _rt_settled := 0


func _rt_next(phase: int) -> void:
	_rt_phase = phase
	_rt_t = 0
	_rot_acc = 0.0
	var p: Vector3 = _main._pos
	print("[demo] retune phase %d at %s" % [phase, _fmt(p)])


# Whole-chapter watchdog. The retune runs two CLI sessions, two resets and two
# gyro calibrations, and several of its phases wait on the FIRMWARE rather than
# on a frame count — a CLI that never answers, or an arming block that never
# clears, would otherwise sit there until the ctest timeout kills it 400 s
# later with no clue which step was stuck. Fail fast and name the phase.
const RT_WATCHDOG := 250 * 200        # 200 s of sim time; a good run takes ~40


func _run_retune(dt: float) -> void:
	if _frame > RT_WATCHDOG and _rt_phase != RT_DONE:
		_fail_reasons.append("retune stalled in phase %d after %.0f s of sim time"
				% [_rt_phase, _frame / 250.0])
		_report_retune()
		_finished = true
		return
	var basis := Basis(_main._rot)
	var armed: bool = not _main._last_out.is_empty() and _main._last_out.armed
	var w: Vector3 = _main._angvel
	var roll_rate: float = absf((basis.inverse() * w).z)
	_rt_t += 1
	_rc[CH_ANGLE] = -1.0
	_rc[CH_TURTLE] = -1.0
	_rc[CH_ARM] = 1.0 if _frame >= ARM_FRAME else -1.0

	match _rt_phase:
		RT_PRESET:
			# Pin the BEFORE value, so the chapter measures its own change
			# rather than whatever the eeprom happened to hold. `save` persists,
			# so without this a second run starts already retuned and the
			# comparison silently reads x1.00.
			cam_hint = "chase"
			caption = "Betaflight's own CLI, over TCP 5761"
			subcaption = "the same protocol the Configurator speaks"
			_idle_sticks()
			_set_thr(0.0)
			_rc[CH_ARM] = -1.0
			if _cli_tick(["set roll_srate = %d" % RT_SRATE_FROM, "save"]):
				_cli_new_session()
				_rt_next(RT_RESET0)

		RT_RESET0:
			_idle_sticks()
			_set_thr(0.0)
			_rc[CH_ARM] = -1.0
			if _rt_t == 1:
				_main.reset_sim()
			if _rt_t > 60:
				_rt_next(RT_CLIMB)

		RT_CLIMB:
			cam_hint = "chase"
			caption = "The Configurator's own CLI, live"
			subcaption = "same text protocol, on TCP 5761, mid-flight"
			if _wait_for_arm(armed):
				return
			var left := _fly_to(dt, RT_ALT, 0.0, V_MAX)
			# Exit on ARRIVING, not on a frame count. RT_SETTLE was 1250 frames
			# from chapter start, which expires before ARM_FRAME (1300) — so the
			# first roll was flown on the launch pad, and both "rolls" measured
			# the same nothing.
			_rt_settled = _rt_settled + 1 if left < 1.0 \
					and _main._linvel.length() < 1.0 else 0
			if _rt_settled >= RT_SETTLE:
				_rt_next(RT_ROLL_A)

		RT_ROLL_A:
			cam_hint = "los"
			caption = "roll_srate = %d" % RT_SRATE_FROM
			subcaption = "full roll stick — peak %.1f deg/s" % rad_to_deg(_rt_peak_a)
			_drive_axis(dt, CH_ROLL, 2, 1.0, basis)
			_ballistic_throttle(float(_rt_t) / float(RT_ROLL_FRAMES))
			_rt_peak_a = maxf(_rt_peak_a, roll_rate)
			if _rt_t >= RT_ROLL_FRAMES:
				_rt_next(RT_RECOVER_A)

		RT_RECOVER_A:
			_fly_to(dt, RT_ALT, 0.0, V_MAX)
			if _rt_t >= RT_RECOVER:
				_rt_next(RT_LAND)

		RT_LAND:
			cam_hint = "chase"
			caption = "Land and disarm to retune"
			subcaption = "Betaflight refuses to open the CLI while armed — same as the real quad"
			_fly_to(dt, RT_LAND_TO, 0.0, 3.0)
			var p: Vector3 = _main._pos
			if p.y < 0.35 and _main._linvel.length() < 0.6:
				_rc[CH_ARM] = -1.0        # disarm; the CLI needs it
				if _rt_t > 250 and not armed:
					_rt_next(RT_CLI)
			if _rt_t > 250 * 14:
				_fail_reasons.append("never landed to retune (y=%.2f)" % p.y)
				_rt_next(RT_DONE)

		RT_CLI:
			# on the ground, disarmed, sticks safe — and still stepping the sim
			# every frame, because a blocking CLI call would make the core think
			# the client had left (see _cli_tick)
			_idle_sticks()
			_set_thr(0.0)
			_rc[CH_ARM] = -1.0
			if _cli_tick(["set roll_srate = %d" % RT_SRATE_TO, "save"]):
				_rt_cli_ok = _cli_ok
				_rt_next(RT_RESET)

		RT_RESET:
			# Clear ARMING_DISABLED_CLI.
			#
			# On real hardware the reboot after `save` clears RAM and with it the
			# CLI arming block. propwash reboots IN-PROCESS, and BF::init() does
			# not clear the firmware's statics (docs/ARCHITECTURE.md), so the
			# flag survives and the quad can never arm again — which is exactly
			# why CLAUDE.md says to load the tune in one instance and fly a
			# fresh one.
			#
			# PW_CMD_RESET is the in-process equivalent of that fresh start: it
			# rewinds the statics to the post-first-boot snapshot, taken before
			# any CLI existed. The retune is NOT lost with it, because the
			# following BF::init() re-reads the eeprom — and we just saved to it.
			_idle_sticks()
			_set_thr(0.0)
			_rc[CH_ARM] = -1.0
			if _rt_t == 1:
				_main.reset_sim()
			if _rt_t > 60:
				_rt_next(RT_REARM)

		RT_REARM:
			cam_hint = "chase"
			caption = "Same tune file, one value changed"
			subcaption = "saved and rebooted, exactly like hitting Save in the Configurator"
			if not armed:
				# Hold the ARM switch OFF until the firmware reports it is ready.
				#
				# Raising it earlier does not work: the post-reset gyro
				# calibration takes ~5 s, and if the switch is already ON when
				# calibration finishes Betaflight latches the "was on at boot"
				# guard and answers `flip ARM switch OFF then ON` forever. The
				# arming-disable mask reaching 0 is the firmware itself saying
				# the switch may now be raised.
				var dis: int = _main._last_out.get("arming_disable", -1) \
						if not _main._last_out.is_empty() else -1
				_rc[CH_ARM] = 1.0 if dis == 0 else -1.0
				_idle_sticks()
				_set_thr(0.0)
				_thr_i = 0.0
				return
			_rc[CH_ARM] = 1.0
			var left2 := _fly_to(dt, RT_ALT, 0.0, V_MAX)
			_rt_settled = _rt_settled + 1 if left2 < 1.0 \
					and _main._linvel.length() < 1.0 else 0
			if _rt_settled >= RT_SETTLE:
				_rt_settled = 0
				_rt_next(RT_ROLL_B)
			if _rt_t > 250 * 40:
				_fail_reasons.append("did not get back to altitude after the retune "
						+ "— check for block:CLI, meaning the `exit` never landed")
				_rt_next(RT_DONE)

		RT_ROLL_B:
			cam_hint = "los"
			caption = "roll_srate = %d" % RT_SRATE_TO
			subcaption = "identical stick input — peak %.1f deg/s" % rad_to_deg(_rt_peak_b)
			_drive_axis(dt, CH_ROLL, 2, 1.0, basis)
			_ballistic_throttle(float(_rt_t) / float(RT_ROLL_FRAMES))
			_rt_peak_b = maxf(_rt_peak_b, roll_rate)
			if _rt_t >= RT_ROLL_FRAMES:
				_rt_next(RT_RECOVER_B)

		RT_RECOVER_B:
			cam_hint = "chase"
			caption = "Your tune, live"
			subcaption = "nothing was recompiled and nothing was restarted"
			_fly_to(dt, RT_ALT, 0.0, V_MAX)
			if _rt_t >= RT_RECOVER:
				_rt_next(RT_DONE)

		RT_DONE:
			_idle_sticks()
			_set_thr(0.0)
			_rc[CH_ARM] = -1.0
			_report_retune()
			_finished = true


# Open the CLI, apply some `set` lines, leave. Blocking, and deliberately so —
# see the chapter note on why this chapter cannot be deterministic anyway.
#
# NO `save`. A save reboots the FC in-process, which re-runs init and would
# drop the quad out of the sky mid-demo; `set` writes the parameter group in
# RAM and rate changes take effect on the next PID loop, which is exactly the
# live-tuning behaviour being demonstrated.
#
# The `exit` is NOT optional: opening the CLI sets ARMING_DISABLED_CLI and it
# stays set until the firmware sees a clean exit, so skipping it means the quad
# silently refuses to arm ever again (CLAUDE.md).
const CLI_HOST := "127.0.0.1"
const CLI_PORT := 5761


# NON-BLOCKING, one step per physics frame, and that is not a style choice.
#
# The first version did the whole CLI conversation synchronously inside one
# _physics_process call. That stops the client sending PW_STATE_IN for as long
# as it takes — and the core treats a client that has gone quiet for longer than
# CLIENT_IDLE_MS as departed and resumes idle-ticking (server.cpp: `clientDriving`).
# So a 1.5 s blocking CLI call handed the sim 1.5 s of simulated time nobody
# asked for, the quad fell out of the air unattended, and the second roll
# measured 80 deg/s against the first one's 846. The lockstep must keep running
# THROUGH the conversation.
var _cli: StreamPeerTCP
var _cli_step := 0
var _cli_wait := 0
var _cli_idx := 0
var _cli_buf := ""
var _cli_age := 0
var _cli_ok := false

# Wait for the PROMPT, not for a fixed number of frames.
#
# A fixed wait does not work here. Betaflight's reply path crosses two clocks:
# dyad is pumped on a dedicated wall-clock worker thread (pw_sitl.c), while the
# firmware task that actually produces the reply only runs when the sim ticks —
# and in headless the client ticks as fast as the core answers, so a "120 frame"
# wait can be a tenth of a second of wall time. That is what produced an empty
# banner and a silent no-op retune. Polling for the "# " prompt is exactly what
# tools/bfcli/pw_cli.py does, and for the same reason.
const CLI_PROMPT := "# "
const CLI_BUDGET := 4000         # frames before giving up on a reply
const CLI_REPOKE := 1200         # re-send '#' if the banner has not arrived


# Reset the stepper so a second CLI session can run in the same flight.
func _cli_new_session() -> void:
	_cli = null
	_cli_step = 0
	_cli_idx = 0
	_cli_age = 0
	_cli_buf = ""
	_cli_ok = false


func _cli_tick(lines: Array) -> bool:
	if _cli_step > 0 and _cli != null:
		_cli.poll()
		var n := _cli.get_available_bytes()
		if n > 0:
			var r: Array = _cli.get_partial_data(n)
			if r[0] == OK:
				_cli_buf += (r[1] as PackedByteArray).get_string_from_utf8()
	if _cli_wait > 0:
		_cli_wait -= 1
		return false

	match _cli_step:
		0:
			_cli = StreamPeerTCP.new()
			if _cli.connect_to_host(CLI_HOST, CLI_PORT) != OK:
				_rt_cli_note = "could not connect to %s:%d" % [CLI_HOST, CLI_PORT]
				_cli_step = 99
				return true
			_cli_step = 1
		1:
			var st := _cli.get_status()
			if st == StreamPeerTCP.STATUS_CONNECTED:
				# a bare '#' drops Betaflight into the CLI
				_cli.put_data("#\r\n".to_utf8_buffer())
				_cli_buf = ""
				_cli_age = 0
				_cli_step = 2
			elif st == StreamPeerTCP.STATUS_ERROR:
				_rt_cli_note = "CLI connection failed"
				_cli_step = 99
				return true
		2:
			_cli_age += 1
			if _cli_buf.contains(CLI_PROMPT):
				print("[demo][cli] entered (%d byte banner)" % _cli_buf.length())
				_cli_buf = ""
				_cli_step = 3
			elif _cli_age % CLI_REPOKE == 0:
				_cli.put_data("#\r\n".to_utf8_buffer())   # rare: '#' was dropped
			elif _cli_age > CLI_BUDGET:
				_rt_cli_note = "no CLI banner — the session never opened"
				_cli_step = 99
				return true
		3:
			if _cli_idx >= lines.size():
				# A trailing `save` already rebooted the FC, which clears
				# ARMING_DISABLED_CLI on its own — sending `exit` into a dead
				# socket afterwards would just error.
				if lines.size() > 0 and String(lines[-1]).begins_with("save"):
					_cli_ok = true
					_cli.disconnect_from_host()
					return true
				# Otherwise the exit is NOT optional: opening the CLI sets
				# ARMING_DISABLED_CLI and it stays set until the firmware sees a
				# clean exit. Clear the buffer FIRST — it still holds the previous
				# command's trailing "# ", and step 4's prompt test matched that
				# instantly, closing the socket about one frame after sending
				# `exit` and leaving `block:CLI` set forever after.
				_cli_buf = ""
				_cli.put_data("exit\r\n".to_utf8_buffer())
				_cli_age = 0
				_cli_step = 4
				return false
			_cli.put_data((lines[_cli_idx] + "\r\n").to_utf8_buffer())
			_cli_buf = ""
			_cli_age = 0
			_cli_step = 5
		5:
			_cli_age += 1
			if String(lines[_cli_idx]).begins_with("save"):
				# `save` writes the eeprom and REBOOTS: no prompt ever comes
				# back and the firmware drops the connection (tcpReconfigure
				# ends it on re-init, by design). Wait a fixed spell instead.
				if _cli_age < 250:
					return false
				print("[demo][cli] save -> eeprom written, FC rebooting")
				_cli_idx += 1
				_cli_step = 3
				return false
			if not _cli_buf.contains(CLI_PROMPT):
				if _cli_age > CLI_BUDGET:
					_rt_cli_note = "no reply to: %s" % lines[_cli_idx]
					_cli_step = 98
				return false
			var reply := _cli_buf.strip_edges()
			print("[demo][cli] %s -> %s" % [lines[_cli_idx], reply.replace("\n", " | ")])
			var low := reply.to_lower()
			if low.contains("invalid") or low.contains("error"):
				_rt_cli_note = "firmware rejected: %s" % lines[_cli_idx]
				_cli.put_data("exit\r\n".to_utf8_buffer())
				_cli_age = 0
				_cli_step = 98
				return false
			_cli_idx += 1
			_cli_step = 3
		4:
			_cli_age += 1
			# Fixed wait, not a prompt test: leaving the CLI produces no prompt
			# to wait for, and the firmware only actions the `exit` on its next
			# serial task. 1 s of sim time is generous and costs nothing —
			# getting this wrong leaves the quad permanently unable to arm.
			if _cli_age < 250:
				return false
			_cli_ok = true
			_cli.disconnect_from_host()
			return true
		98:
			_cli.disconnect_from_host()
			return true
		99:
			return true
	return false


func _report_retune() -> void:
	var ratio := _rt_peak_b / maxf(_rt_peak_a, 1e-3)
	if not _armed_seen:
		_fail_reasons.append("never armed")
	if not _rt_cli_ok:
		_fail_reasons.append("the CLI retune did not apply: %s. " % _rt_cli_note
				+ "KNOWN: entering the CLI is unreliable while a client drives "
				+ "the sim (raw MSP still works) — see the note above.")
	if _rt_peak_a < 0.5:
		_fail_reasons.append("first roll barely moved (%.2f rad/s) — "
				% _rt_peak_a + "the measurement is not measuring anything")
	if ratio < RT_MIN_RATIO:
		_fail_reasons.append("same stick gave %.1f vs %.1f deg/s (ratio %.2f) — "
				% [rad_to_deg(_rt_peak_a), rad_to_deg(_rt_peak_b), ratio]
				+ "the live retune did not change the handling")
	print("[demo] retune: roll_srate %d -> %d, peak roll rate %.1f -> %.1f deg/s "
			% [RT_SRATE_FROM, RT_SRATE_TO,
					rad_to_deg(_rt_peak_a), rad_to_deg(_rt_peak_b)]
			+ "(x%.2f)" % ratio)


# ============================================================= ghost chapter
#
# The headline claim, made visible. README "Determinism" says identical inputs
# produce byte-identical trajectories across runs, processes and resets, and
# two ctests gate it — but a viewer can only see that as an exit code. This
# chapter shows it.
#
#   1. RECORD  fly a short maneuver closed-loop, keeping every stick sample.
#   2. RESET   PW_CMD_RESET, which rewinds the firmware's writable statics to
#              the post-boot snapshot, so reset is equivalent to a fresh core.
#   3. REPLAY  play the recorded sticks back OPEN LOOP, with a translucent
#              ghost flying the recorded trajectory. They stay welded, and the
#              live separation readout sits at 0.000 m.
#   4. PERTURB reset again, replay the same tape with ONE stick sample altered
#              by 1e-4, and watch the two come apart.
#
# Step 4 is the honest half. Determinism is not stability: the trajectory is
# reproducible to the bit AND exquisitely sensitive to its inputs, and showing
# only the first half would imply the second is false.
#
# THE TAPE STARTS AT THE RESET, not at the maneuver. Recording only the
# interesting part would mean replaying from a state the reset does not restore
# — so arming and the climb are on the tape too, and the replay starts from the
# same pad the recording did.
#
# AIRBORNE MANEUVER, deliberately. The replay's reproducibility depends on the
# client's contact detection being reproducible too (it runs engine shape
# queries), and while it should be, a maneuver flown in clear air simply does
# not put that in the loop.
const GH_ARM_PAD := 60           # frames of slack after the reset before ARM
const GH_CLIMB_TO := Vector3(0.0, 5.0, -5.0)
const GH_CLIMB_FRAMES := 1000    # 4 s
const GH_ROLL_FRAMES := 500      # 2 s: roll and catch
const GH_SETTLE_FRAMES := 375    # 1.5 s
const GH_RESET_WAIT := 30        # frames to let the reset land before replaying
# The perturbation: ONE MICROSECOND of stick, on one frame out of thousands.
#
# Not an arbitrary small float. core/sim/bf.cpp does
# `rcDataCache[i] = uint16_t(1500 + data[i] * 500)`, so the wire quantizes
# sticks to whole microseconds and 0.002 normalised units is exactly 1 us —
# the finest change the RC link can physically express. The first attempt used
# 1e-4, which is 0.05 us: it was truncated away before it reached the firmware
# and both replays came back byte-identical, which looked like the perturbation
# proving nothing when in fact it never happened.
const GH_NUDGE := 0.002
# WHEN to nudge, expressed against the arm event rather than as a fraction of
# the tape. A fraction is the wrong unit twice over: 0.55 grew to 10 cm running
# alone but 7 mm as the third act of a reel, and 0.30 landed while the quad was
# still DISARMED, where the sticks are ignored and the nudge did precisely
# nothing. One second after arming is unambiguous — the quad is flying, and
# there is most of the tape left for the difference to grow.
const GH_NUDGE_AFTER_ARM := 250  # frames past the arm point
# What counts as "identical". The intent is exact equality; this is the
# assertion tolerance so the test reports a real number rather than a bool.
const GH_EXACT := 1.0e-6
# ...and what counts as diverged. The clean replay is EXACTLY 0.0, so any
# non-zero separation already proves input sensitivity; this is set well above
# the float noise floor rather than at a "looks big on video" figure, because
# the visible size of the divergence depends on where in the flight it lands.
const GH_DIVERGED := 0.002

# EVERY sequence must start from the same place, and "the same place" means the
# same sim time since the reset, not just the same pose.
#
# The first attempt recorded straight from process start and replayed after a
# PW_CMD_RESET plus a settling wait. Both replays then came out 0.4929 m from
# the recording — the clean one and the perturbed one by the SAME amount, which
# is the tell: chaos would have separated them, so this was a fixed offset, not
# a determinism failure. The cause was those extra settling frames. They are
# PW_STATE_IN packets, the client is the clock, so they advance firmware time
# and battery sag before the tape starts, and the replay armed into a slightly
# different quad.
#
# So the recording is now preceded by exactly the same reset-and-wait as the
# replays. All three sequences are reset -> GH_RESET_WAIT idle frames -> tape.
enum {GH_INIT, GH_RECORD, GH_RESET1, GH_REPLAY, GH_RESET2, GH_PERTURB, GH_DONE}

var _gh_phase := GH_INIT
var _gh_t := 0                   # frames in the current phase
var _gh_reset_sent := false
var _gh_rc: Array = []           # the tape: one 8-float stick sample per frame
var _gh_pos: Array = []          # recorded trajectory, for the ghost
var _gh_rot: Array = []
var _gh_i := 0                   # replay cursor
var _gh_node: Node3D
var _gh_max_clean := 0.0
var _gh_max_perturb := 0.0
var _gh_div := 0.0


func _gh_next(phase: int) -> void:
	_gh_phase = phase
	_gh_t = 0
	_gh_i = 0


func _run_ghost(dt: float) -> void:
	var basis := Basis(_main._rot)
	var armed: bool = not _main._last_out.is_empty() and _main._last_out.armed
	_gh_t += 1
	_rc[CH_ANGLE] = -1.0            # ACRO throughout
	_rc[CH_TURTLE] = -1.0

	match _gh_phase:
		GH_INIT:
			# Same preamble the replays get. The ghost is spawned here too, so
			# the scene graph is identical during the recording and during both
			# replays rather than gaining a node halfway through.
			_idle_sticks()
			_set_thr(0.0)
			_rc[CH_ARM] = -1.0
			if not _gh_reset_sent:
				# a couple of frames of contact first, so the core has finished
				# its own first-contact reset before we ask for ours
				if _gh_t < 5:
					return
				_main.reset_sim()
				_gh_reset_sent = true
				_gh_node = _main.spawn_ghost()
				_gh_t = 0
				return
			if _gh_t >= GH_RESET_WAIT:
				_gh_next(GH_RECORD)

		GH_RECORD:
			cam_hint = "chase"
			caption = "Recording"
			subcaption = "every stick sample, from the reset onward"
			_gh_fly(dt, basis, armed)
			# the tape is written AFTER the sticks for this frame are decided,
			# so sample i is exactly what frame i was flown with
			_gh_rc.append(_rc.duplicate())
			_gh_pos.append(_main._pos)
			_gh_rot.append(_main._rot)
			if _gh_t >= GH_ARM_PAD + ARM_FRAME + GH_CLIMB_FRAMES + GH_ROLL_FRAMES \
					+ GH_SETTLE_FRAMES:
				print("[demo] recorded %d frames (%.1f s)"
						% [_gh_rc.size(), _gh_rc.size() / 250.0])
				_main.reset_sim()
				_gh_next(GH_RESET1)

		GH_RESET1, GH_RESET2:
			_idle_sticks()
			_set_thr(0.0)
			_rc[CH_ARM] = -1.0
			if _gh_t >= GH_RESET_WAIT:
				_gh_next(GH_REPLAY if _gh_phase == GH_RESET1 else GH_PERTURB)

		GH_REPLAY, GH_PERTURB:
			var perturbed := _gh_phase == GH_PERTURB
			cam_hint = "chase"
			if _gh_replay(perturbed):
				if perturbed:
					_gh_next(GH_DONE)
				else:
					print("[demo] clean replay: max separation %.9f m"
							% _gh_max_clean)
					_main.reset_sim()
					_gh_next(GH_RESET2)

		GH_DONE:
			_idle_sticks()
			_set_thr(0.0)
			_rc[CH_ARM] = -1.0
			_report_ghost()
			_end_chapter()


# The recorded maneuver: arm, climb, one roll, settle. Closed-loop — this is
# the only phase that uses the controller at all.
func _gh_fly(dt: float, basis: Basis, armed: bool) -> void:
	var t := _gh_t - GH_ARM_PAD
	_rc[CH_ARM] = 1.0 if t >= ARM_FRAME else -1.0
	if not armed:
		_idle_sticks()
		_set_thr(0.0)
		_thr_i = 0.0
		return
	var since := t - ARM_FRAME
	if since < GH_CLIMB_FRAMES:
		_fly_to(dt, GH_CLIMB_TO, 0.0, V_MAX)
	elif since < GH_CLIMB_FRAMES + GH_ROLL_FRAMES:
		if _rot_acc < TAU:
			_drive_axis(dt, CH_ROLL, 2, 1.0, basis)
			_ballistic_throttle(_rot_acc / TAU)
		else:
			_fly_to(dt, GH_CLIMB_TO, 0.0, V_MAX)
	else:
		_fly_to(dt, GH_CLIMB_TO, 0.0, V_MAX)


# Play the tape back with no feedback whatsoever, and measure how far the live
# quad is from where the recording was at the same frame.
func _gh_replay(perturbed: bool) -> bool:
	if _gh_i >= _gh_rc.size():
		return true
	var sample: Array = _gh_rc[_gh_i]
	for i in range(8):
		_rc[i] = sample[i]
	var nudge_frame := GH_ARM_PAD + ARM_FRAME + GH_NUDGE_AFTER_ARM
	if perturbed and _gh_i == nudge_frame:
		_rc[CH_PITCH] = clampf(_rc[CH_PITCH] + GH_NUDGE, -1.0, 1.0)

	# the ghost flies the recorded trajectory; the live quad flies the tape
	var rec_pos: Vector3 = _gh_pos[_gh_i]
	if _gh_node != null:
		_gh_node.transform = Transform3D(Basis(_gh_rot[_gh_i]), rec_pos)
	_gh_div = _main._pos.distance_to(rec_pos)
	if perturbed:
		_gh_max_perturb = maxf(_gh_max_perturb, _gh_div)
		caption = "One stick sample, one microsecond different"
		subcaption = "same core, same tape — separation %.3f m" % _gh_div
	else:
		_gh_max_clean = maxf(_gh_max_clean, _gh_div)
		caption = "Replaying the same sticks"
		subcaption = "ghost is the previous run — separation %.3f m" % _gh_div
	_gh_i += 1
	return false


func _report_ghost() -> void:
	if not _armed_seen:
		_fail_reasons.append("never armed")
	if _gh_rc.is_empty():
		_fail_reasons.append("recorded no tape")
	# The claim is byte-identical, so the clean replay must reproduce the
	# recording to the float. Anything else means some client-side state
	# survives PW_CMD_RESET, which is a determinism bug, not a tolerance to
	# widen.
	if _gh_max_clean > GH_EXACT:
		_fail_reasons.append("clean replay diverged by %.9f m — something "
				% _gh_max_clean + "survives the reset")
	# ...and the perturbed one must NOT reproduce it, or the "identical inputs"
	# claim is vacuous: a run that ignores its inputs is trivially repeatable.
	if _gh_max_perturb < GH_DIVERGED:
		# GDScript's % has no %g. The first version of this line used one, so
		# the format silently failed and the failure printed its own template.
		_fail_reasons.append("a 1 us stick nudge changed nothing (max %.6f m)"
				% _gh_max_perturb + " — is the tape actually being replayed?")
	print("[demo] ghost: clean max separation %.9f m, perturbed %.4f m "
			% [_gh_max_clean, _gh_max_perturb]
			+ "(%d frames)" % _gh_rc.size())


# ============================================================ turtle chapter
#
# Crash inverted, flip back over on reversed props, fly away. The act no other
# simulator can run, because the flip is not an animation: arming with the
# turtle box active makes real Betaflight send DSHOT SPIN_DIRECTION commands
# through its stock command queue, the virtual ESC applies them to the physics,
# and the quad levers itself over its own duct edge against the contact solver.
#
# GETTING INVERTED HONESTLY. The quad is rolled to inverted in the air and the
# throttle is chopped — it then falls and settles on its back under the contact
# solver, which is the behaviour the contact_drop ctest already gates ("the
# inverted one STAYS inverted"). Teleporting it upside-down would have been
# three lines and would have been exactly the kind of fakery this act exists to
# disprove.
#
# BURST-AND-COAST. A held roll stick does NOT work: two reversed motors make
# about 1.2x weight on a ducted whoop, and as the quad comes up the thrust
# vector rotates toward the ground, so it stalls at the balance point. Short
# bursts build pivot momentum between them instead. Same technique, same
# constants as the headless turtle_flip test (tools/tester/main.cpp).
#
# small_angle = 180 is a PREREQUISITE — Betaflight refuses to arm past 25 deg
# of tilt otherwise, and the whole act starts from upside-down. It comes from
# the pilot's own dump (config/cinelog35v3.diff), which is why this chapter
# needs a baked eeprom and the others do not.
# The drop zone, and it has to be genuinely clear for METRES, not just at the
# point of impact. Turtling is not a pirouette: the quad claws itself along the
# ground on two reversed props and travelled 1.6 m in testing. At the old
# (0, 3, -4) that was enough to reach gate 1's FOOT — the feet sit at
# x = +/-1.2, z = -6.0 and are SURF_OBJECT — which jams the flip against
# scenery and looks like the turtle mode failing. Sitting it back toward the
# pad puts 4.5 m between the drop and the nearest gate.
const TU_CLIMB_TO := Vector3(0.0, 3.0, -1.5)
const TU_BURST_ON := 125       # 0.5 s at 250 Hz
const TU_BURST_OFF := 88       # 0.35 s
# Body-up y that counts as back on its feet. NOT 0.5: at 0.5 the quad is still
# 60 deg off level and only just past the balance point, and releasing the
# stick there let it flop straight back onto its back (measured: righted at
# +0.51, on its back again at -0.82 two phases later). 0.85 is ~32 deg from
# Same 0.5 the headless turtleTest uses: past the balance point, let it coast
# over. Driving all the way to 0.75 makes the quad claw at the ground for far
# longer, and it arrives upright having skidded 5 m with 100% prop damage —
# which the physics then correctly refuses to fly.
# Past the balance point, and NOT higher. Driving further has been measured
# twice now to make things worse, not better: the extra clawing costs prop
# damage faster than the extra margin buys reliability (0.65 took a clean 0.10
# run to 1.00 and wrecked the quad). Reliability has to come from settling and
# retrying, not from pushing harder.
const TU_UP_RIGHTED := 0.5
const TU_STILL_SPEED := 0.4
# Bound each ATTEMPT, not just the act. A flip that is not working should hand
# over to the settle-and-retry loop quickly rather than claw for 12 s.
const TU_FLIP_BUDGET := 250 * 5
# 5, not 3. The flip is a contact-dynamics problem and its success depends on
# the exact resting attitude — observed needing 1 retry on a fresh config and
# all 3 on another, which is too close to the ceiling for something that fails
# the whole reel when it runs out.
const TU_RETRIES := 5
# A half roll at full stick takes ~0.2-0.4 s on any sane tune. 1.2 s is a
# generous ceiling; past it, stop driving and drop from wherever we are rather
# than fly the quad out of the demo.
const TU_ROLL_BUDGET := 300
# Prop damage ceiling, and deliberately loose. The act crashes the quad ON
# PURPOSE and every retry of the flip legitimately claws more, so this cannot
# be a tight "did the drop go well" gate without failing honest runs: observed
# 0.10 with no retry, 0.53 with two. The assertions that actually mean
# something are the scenery check, the flip retry, and the fly-away distance —
# this only catches the quad being destroyed outright, which is worth naming
# separately because at 1.00 the physics correctly refuses to fly it and every
# downstream failure becomes a confusing consequence of that.
const TU_DMG_LIMIT := 0.8
# Where "and fly away" actually goes. Above gate 1's 2.05 m bar, and far enough
# down the line that it reads as leaving rather than hovering.
const TU_FLYAWAY_ALT := 4.0
const TU_FLYAWAY_Z := -16.0
const TU_FLYAWAY_MIN_DIST := 8.0

enum {TU_CLIMB, TU_ROLL_OVER, TU_SETTLE, TU_DISARM, TU_BOX, TU_FLIP,
		TU_RESTORE, TU_FLYAWAY, TU_DONE}

var _tu_phase := TU_CLIMB
var _tu_t := 0                 # frames in the current phase
var _tu_turtle_seen := false
var _tu_max_up := -1.0
var _tu_rearmed := false
var _tu_righted := false
var _tu_tries := 0
# The turtle act assumes clear ground: it drops the quad on its back and levers
# it over its own duct edge. Landing against scenery makes that impossible, and
# before this the run just timed out reporting "never came fully upright" —
# technically true, and useless for working out why.
var _tu_obstacle := ""
var _tu_obstacle_at := Vector3.ZERO


const TU_PHASE_NAMES := ["CLIMB", "ROLL_OVER", "SETTLE", "DISARM", "BOX",
		"FLIP", "RESTORE", "FLYAWAY", "DONE"]


func _tu_next(phase: int) -> void:
	_tu_phase = phase
	_tu_t = 0
	_rot_acc = 0.0
	var p: Vector3 = _main._pos
	var v: Vector3 = _main._linvel
	print("[demo] turtle -> %s at %s  vel %.1f m/s  up.y %+.2f"
			% [TU_PHASE_NAMES[phase], _fmt(p), v.length(),
					Basis(_main._rot).y.y])


# Whole-act watchdog. Several phases wait on the FIRMWARE — an arming block
# clearing, an attitude settling — and a wait with no ceiling is how the act
# sat there forever instead of failing with a reason.
const TU_WATCHDOG := 250 * 120        # 120 s of sim time; a good run takes ~45
var _tu_frames := 0
# fc/runtime_config.h armingDisableFlags_e
const ARMING_DISABLED_ANGLE := 1 << 8
var _tu_angle_blocked := false


func _run_turtle(dt: float) -> void:
	_tu_frames += 1
	if _tu_frames > TU_WATCHDOG and _tu_phase != TU_DONE:
		_fail_reasons.append("turtle act stalled in %s after %.0f s"
				% [TU_PHASE_NAMES[_tu_phase], _tu_frames / 250.0])
		_tu_next(TU_DONE)
	var basis := Basis(_main._rot)
	var up_y := basis.y.y
	var out: Dictionary = _main._last_out
	var armed: bool = not out.is_empty() and out.armed
	var flags: int = out.get("crash_flags", 0) if not out.is_empty() else 0
	if flags & 4:
		_tu_turtle_seen = true
	# Arming refused because the quad is tilted past small_angle. After the
	# flip that is the signature of an eeprom that cannot arm inverted, which
	# is the whole prerequisite for this act.
	if _tu_phase >= TU_BOX and not _main._last_out.is_empty():
		var dis: int = _main._last_out.get("arming_disable", 0)
		if dis & ARMING_DISABLED_ANGLE:
			_tu_angle_blocked = true
	# Only from the FLIP onward. Tracking this from frame 1 meant it read 1.00
	# off the CLIMB, while the quad was still happily upright and flying — so
	# the "never came fully upright" assertion was satisfied before the act had
	# even started, and a completely failed flip still passed it.
	if _tu_phase >= TU_FLIP:
		_tu_max_up = maxf(_tu_max_up, up_y)
	_tu_t += 1
	# anything that is not the ground has no business being in this act
	if _tu_obstacle.is_empty():
		for c in _main._pending_contacts:
			if c.surface != PwProtocol.SURF_GROUND:
				_tu_obstacle = ["ground", "a gate", "a tree", "an object"][
						clampi(c.surface, 0, 3)]
				_tu_obstacle_at = _main._pos
				print("[demo] turtle: hit %s at %s — the drop zone is not clear"
						% [_tu_obstacle, _fmt(_tu_obstacle_at)])

	# defaults for the frame; each phase overrides what it needs
	# ACRO, like every other chapter. Two reasons, both learned the hard way:
	#
	# 1. You cannot roll inverted in ANGLE mode. Full stick there commands a
	#    bank ANGLE capped by angle_limit, not a continuous rotation, so the
	#    half roll stalls at the limit — measured, it reached 48 deg in 1.2 s
	#    against the 600-800 deg/s full stick gives in acro. The quad then flew
	#    on under power instead of rolling, 17 m into a tree.
	# 2. _fly_to is an ACRO controller end to end: its inner loop closes on a
	#    RATE error, and in angle mode the stick means an angle, so the whole
	#    cascade is driven through the wrong plant.
	#
	# ...but ONLY for the phases that FLY. The turtle phases themselves keep
	# ANGLE on, which is what tools/tester/main.cpp's turtleTest does and what
	# the flip was tuned against; switching those to acro as well made the quad
	# claw itself 8 m across the field without ever staying upright.
	#
	# The chapter used to set ANGLE on throughout because "turtle is a
	# recovery, not freestyle", which sounds reasonable and is wrong for every
	# phase that uses _fly_to.
	_rc[CH_ANGLE] = 1.0 if _tu_phase in [TU_DISARM, TU_BOX, TU_FLIP, TU_RESTORE] \
			else -1.0
	_rc[CH_TURTLE] = -1.0
	_rc[CH_ARM] = 1.0

	match _tu_phase:
		TU_CLIMB:
			cam_hint = "chase"
			caption = "Every crash is real"
			subcaption = "contacts are forces inside the physics tick — the firmware feels them"
			if _frame < ARM_FRAME:
				_rc[CH_ARM] = -1.0
				_idle_sticks()
				_set_thr(0.0)
				return
			if _arm_when_ready(armed):
				return
			_fly_to(dt, TU_CLIMB_TO, 0.0, V_MAX)
			# SETTLED, not merely arrived. Rolling inverted while still
			# translating throws the quad sideways and lands it hard on a duct
			# edge; entering at 2.0 m/s instead of 0.9 was the difference
			# between 0.10 and 0.75 prop damage on the same code.
			var v: Vector3 = _main._linvel
			if _main._pos.distance_to(TU_CLIMB_TO) < 0.8 and v.length() < 0.6 \
					and _tu_t > 250:
				_tu_next(TU_ROLL_OVER)
			if _tu_t > 250 * 20:
				_fail_reasons.append("never settled over the drop zone")
				_tu_next(TU_DONE)

		TU_ROLL_OVER:
			cam_hint = "los"
			caption = "Rolled inverted, throttle chopped"
			subcaption = "it falls and settles on its back — nothing is teleported"
			# Ballistic half roll, then hands off and let it drop.
			#
			# BUDGETED, and that is not paranoia. This phase holds FULL roll
			# stick until the accumulated rotation reaches pi. How long that
			# takes depends entirely on the pilot's rate curve, because full
			# stick means the tune decides the rate — and on a tune where the
			# accumulator does not converge, the quad simply barrel-rolls away
			# across the park under power. Measured on a real dump: it left the
			# drop zone at (0, 2.7, -3.4) and next touched down 17 m away at
			# (17.3, 0.1, -7.5), wrapped around a tree, 100% prop damage, and
			# the whole act was unrecoverable from there. Every step in the
			# maneuver library carries a `budget` for exactly this reason; this
			# hand-written phase machine never got one.
			if _rot_acc < PI * 0.95 and _tu_t < TU_ROLL_BUDGET:
				_drive_axis(dt, CH_ROLL, 2, 1.0, basis)
				_set_thr(_hover_f() * 0.25)
			else:
				if _rot_acc < PI * 0.95:
					print("[demo] turtle: half roll only reached %.0f deg in "
							% rad_to_deg(_rot_acc)
							+ "%.1f s — dropping from here" % (_tu_t / 250.0))
				_rate_loop_body(dt, Vector3.ZERO, basis)
				_set_thr(0.0)
				_tu_next(TU_SETTLE)

		TU_SETTLE:
			# no throttle, no attitude control: physics owns the fall and the
			# rest, exactly as it does for a real crash
			_idle_sticks()
			_set_thr(0.0)
			# Properly on its back, not merely past vertical. -0.4 is only 66
			# deg over, and starting the flip from that attitude makes the quad
			# claw sideways instead of levering over its duct edge — it arrived
			# upright having skidded 3 m with 75% prop damage, which the
			# physics then correctly refuses to fly.
			var still: bool = _main._linvel.length() < 0.25
			if _tu_t > 250 and still and up_y < -0.85:
				_tu_next(TU_DISARM)
			elif _tu_t > 250 * 8:
				_fail_reasons.append("never settled inverted (up.y %.2f)" % up_y)
				_tu_next(TU_DONE)

		TU_DISARM:
			cam_hint = "los"
			caption = "Upside down on the ground"
			subcaption = "disarm, flip the turtle switch, arm again"
			_idle_sticks()
			_set_thr(0.0)
			_rc[CH_ARM] = -1.0
			if _tu_t > 250:
				_tu_next(TU_BOX)

		TU_BOX:
			# turtle box goes active BEFORE arming: the mode is latched at the
			# arm transition, not polled
			_idle_sticks()
			_set_thr(0.0)
			_rc[CH_ARM] = -1.0
			_rc[CH_TURTLE] = 1.0
			# same: do not start a flip attempt from a quad that is still moving
			var box_still: bool = _main._linvel.length() < TU_STILL_SPEED
			if _tu_t > 125 and (box_still or _tu_t > 250 * 4):
				_tu_next(TU_FLIP)

		TU_FLIP:
			cam_hint = "los"
			caption = "Crashflip"
			subcaption = "props reversed over real DSHOT spin-direction commands"
			_idle_sticks()
			_set_thr(-1.0)          # throttle stays down; roll does the work
			_rc[CH_TURTLE] = 1.0
			# Drive until it is past the balance point, then STOP and disarm.
			#
			# The flop-back that made "...and fly away" grind into the grass is
			# not a threshold problem — it is the reversed props still pushing
			# while the quad teeters. Chasing it with a higher threshold and a
			# hold timer just made the quad claw further: it reached 0.76, could
			# not hold, kept bursting for 12 s and travelled 10 m with 100% prop
			# damage. Cutting power at the balance point lets it fall onto its
			# feet instead of being pushed past them, and TU_RESTORE then
			# settles it with the motors off and retries if it did not take.
			if up_y > TU_UP_RIGHTED:
				_rc[CH_ROLL] = 0.0
				_tu_next(TU_RESTORE)
			else:
				# burst-and-coast, not a held stick (see the note above)
				var cycle := _tu_t % (TU_BURST_ON + TU_BURST_OFF)
				_rc[CH_ROLL] = 1.0 if cycle < TU_BURST_ON else 0.0
			if _tu_t > TU_FLIP_BUDGET:
				_rc[CH_ROLL] = 0.0
				_tu_next(TU_RESTORE)   # let the retry loop judge it, settled

		TU_RESTORE:
			# Disarm, box off, and let it settle with the motors STOPPED — this
			# is the only moment the quad is free of reversed-prop thrust, so
			# it is the only honest place to ask "is it actually on its feet?".
			# If it is not, go round again rather than handing an inverted quad
			# to the fly-away.
			_idle_sticks()
			_set_thr(-1.0)
			_rc[CH_ARM] = -1.0
			# STILL, not just "250 frames have passed". Judging attitude while
			# the quad is sliding at 2.3 m/s reads whatever it happened to be
			# rolling through — which is how CI saw it "settle back" four times
			# at up.y +0.12, -0.06, +0.25, +0.28 without ever having stopped.
			var settled: bool = _main._linvel.length() < TU_STILL_SPEED
			if _tu_t > 250 and (settled or _tu_t > 250 * 5):
				if up_y > TU_UP_RIGHTED:
					_tu_righted = true
					_tu_next(TU_FLYAWAY)
				elif _tu_tries < TU_RETRIES:
					_tu_tries += 1
					print("[demo] turtle: settled back at up.y %+.2f — retry %d"
							% [up_y, _tu_tries])
					_tu_next(TU_BOX)
				else:
					_fail_reasons.append("the flip would not hold after %d "
							% TU_RETRIES + "attempts (up.y %+.2f)" % up_y)
					_tu_next(TU_DONE)

		TU_FLYAWAY:
			cam_hint = "chase"
			caption = "...and fly away"
			subcaption = "normal re-arm restores the motor directions"
			if not out.is_empty() and armed and not (flags & 4):
				_tu_rearmed = true
			if _arm_when_ready(armed):
				return
			# Never fly an inverted quad. _fly_to happily commands full throttle
			# regardless of attitude, so entering this phase upside-down means
			# grinding the props into the ground at thr 1.0 — visible in the
			# HUD as `alt 0.1 m  thr 1.0` and going nowhere. If the flip did not
			# actually leave it on its feet, say so instead of destroying it.
			if Basis(_main._rot).y.y < 0.5:
				_idle_sticks()
				_set_thr(0.0)
				if _tu_t > 250 * 2:
					_fail_reasons.append("still inverted at the fly-away "
							+ "(up.y %.2f) — the flip did not hold"
							% Basis(_main._rot).y.y)
					_tu_next(TU_DONE)
				return
			# Actually leave.
			#
			# Two bugs lived here in turn. First the target was (0, 2.5, -6.0),
			# which is gate 1 — top bar at 2.05 m — so the recovery flew into
			# it on every run, unnoticed. Moving it back to (0, 2.5, -2.0) fixed
			# that and created the second: once the drop zone also moved to
			# z = -1.5, the target was 0.7 m away, so "...and fly away" hopped
			# up and hovered on the spot.
			#
			# Climb out over the drop zone first, THEN run down the line. The
			# climb matters: threading gate 1's opening from a standing start on
			# damaged props is not a shot worth risking, whereas 4 m clears its
			# bar by 2 m.
			var tgt := Vector3(0.0, TU_FLYAWAY_ALT, TU_CLIMB_TO.z)
			if _tu_t > 250 * 3:
				tgt = Vector3(0.0, TU_FLYAWAY_ALT, TU_FLYAWAY_Z)
			_fly_to(dt, tgt, 0.0, V_MAX)
			if _tu_t > 250 * 14:
				_tu_next(TU_DONE)

		TU_DONE:
			_idle_sticks()
			_set_thr(-1.0)
			_rc[CH_ARM] = -1.0
			_report_turtle()
			_end_chapter()


# Hold everything safe until the firmware has actually armed, and say so.
#
# Betaflight refuses to arm with the throttle up (ARMING_DISABLED_THROTTLE), so
# a phase that raises the ARM switch and asks the position controller for a
# climb in the same frame deadlocks: the climb demand holds the throttle high,
# which blocks the arm, which means it never climbs. The shared update() path
# has this guard built in; the turtle chapter bypasses update() to own ARM, so
# it needs its own. Returns true if the caller should do nothing else.
# Raise the ARM switch only once the firmware reports it is ready, and hold it
# LOW until then. Holding it high through an arming block makes Betaflight latch
# ARMING_DISABLED_ARM_SWITCH ("flip ARM switch OFF then ON") and the quad can
# never arm again — which is how the turtle act's fly-away hung forever on a
# tune where the quad was still tilted past small_angle after the flip.
func _arm_when_ready(armed: bool) -> bool:
	if armed:
		_rc[CH_ARM] = 1.0
		return false
	var dis: int = _main._last_out.get("arming_disable", -1) \
			if not _main._last_out.is_empty() else -1
	_rc[CH_ARM] = 1.0 if dis == 0 else -1.0
	_idle_sticks()
	_set_thr(0.0)
	_thr_i = 0.0
	return true


func _wait_for_arm(armed: bool) -> bool:
	if armed:
		return false
	_idle_sticks()
	_set_thr(0.0)
	_thr_i = 0.0
	return true


func _idle_sticks() -> void:
	_rc[CH_ROLL] = 0.0
	_rc[CH_PITCH] = 0.0
	_rc[CH_YAW] = 0.0
	_i_rate = Vector3.ZERO


func _report_turtle() -> void:
	var p: Vector3 = _main._pos
	if not _armed_seen:
		_fail_reasons.append("never armed")
	if not _tu_turtle_seen:
		_fail_reasons.append("turtle mode never activated (crash_flags bit2) — "
				+ "is small_angle=180 in the eeprom, and is the protocol DSHOT?")
	if _tu_angle_blocked:
		# name the actual cause rather than leaving five downstream symptoms
		_fail_reasons.append("the firmware refused to arm because the quad was "
				+ "tilted (ARMING_DISABLED_ANGLE): this eeprom has a small "
				+ "small_angle. Turtle needs `set small_angle = 180` — bake the "
				+ "real dump with tools/bfcli/load_config.sh")
	if not _tu_obstacle.is_empty():
		# reported whether or not the flip then succeeded: turtling against
		# scenery is luck, not a demonstration
		_fail_reasons.append("the drop zone was not clear — hit %s at %s"
				% [_tu_obstacle, _fmt(_tu_obstacle_at)])
	if _tu_max_up < 0.9:
		_fail_reasons.append("never came fully upright (max up.y %.2f)" % _tu_max_up)
	if not _tu_rearmed:
		_fail_reasons.append("never re-armed normally with turtle inactive")
	if p.y < 1.0:
		_fail_reasons.append("did not fly away (ended at %.2f m)" % p.y)
	# "flew away" has to mean it went somewhere. Altitude alone passed while
	# the quad hovered 0.7 m from where it crashed, which is not the shot.
	var travelled := Vector2(p.x - TU_CLIMB_TO.x, p.z - TU_CLIMB_TO.z).length()
	if travelled < TU_FLYAWAY_MIN_DIST:
		_fail_reasons.append("only got %.1f m from the drop zone — the "
				% travelled + "recovery is supposed to fly OFF, not hover")
	# The act crashes the quad on purpose, so some damage is expected — but a
	# CLEAN run reads about 0.10. It read 0.46 for as long as the act flew in
	# angle mode and tumbled instead of rolling, which is exactly why "damage
	# looks high" was dismissed as normal for months. With the baseline honest,
	# this is worth gating.
	if _max_dmg > TU_DMG_LIMIT:
		_fail_reasons.append("prop damage %.2f — the quad is destroyed, "
				% _max_dmg + "so nothing after this means anything")
	print("[demo] turtle: activated=%s max_up=%.2f rearmed=%s max_dmg=%.3f end=%s"
			% [str(_tu_turtle_seen), _tu_max_up, str(_tu_rearmed), _max_dmg,
					_fmt(p)])


# --------------------------------------------------------------- diagnostic
# Hover quality, measured rather than eyeballed. A cascaded controller that is
# marginally stable does not fall out of the sky — it buzzes, which on video
# reads as the airframe trembling. RMS says how bad, and the zero-crossing
# count says at what frequency, which is what identifies WHICH loop is ringing.
var _dg_n := 0
var _dg_w2 := Vector3.ZERO      # sum of squared body rates
var _dg_s2 := Vector3.ZERO      # sum of squared stick deflections
var _dg_cross := Vector3.ZERO   # sign changes per axis
var _dg_prev := Vector3.ZERO
var _dg_t := 0.0
var _dg_rms := Vector3.ZERO     # last completed window, asserted by move:hover

# Settled hover must be quieter than this on pitch and roll, rad/s RMS. The
# 0.4 Hz limit cycle that prompted the feedforward term sat at 0.31; with FF a
# settled window reads about 0.03. 0.10 is comfortably between the two, so this
# catches a return of the tremble without failing on noise.
const HOVER_RMS_LIMIT := 0.10


func _diag(dt: float) -> void:
	var basis := Basis(_main._rot)
	var w: Vector3 = _main._angvel
	var wb := basis.inverse() * w
	_dg_t += dt
	_dg_n += 1
	_dg_w2 += Vector3(wb.x * wb.x, wb.y * wb.y, wb.z * wb.z)
	var st := Vector3(_rc[CH_PITCH], _rc[CH_YAW], _rc[CH_ROLL])
	_dg_s2 += Vector3(st.x * st.x, st.y * st.y, st.z * st.z)
	if _dg_n > 1:
		if signf(wb.x) != signf(_dg_prev.x):
			_dg_cross.x += 1.0
		if signf(wb.y) != signf(_dg_prev.y):
			_dg_cross.y += 1.0
		if signf(wb.z) != signf(_dg_prev.z):
			_dg_cross.z += 1.0
	_dg_prev = wb
	if _dg_n % 500 != 0:
		return
	# WINDOWED, not cumulative: a cumulative average keeps falling for as long
	# as you watch it, which makes a settled hover and a slowly-converging one
	# look the same. Each print is an independent 2 s window, so the last one
	# is the settled figure and is what gets asserted.
	var n := float(_dg_n)
	var rms := Vector3(sqrt(_dg_w2.x / n), sqrt(_dg_w2.y / n), sqrt(_dg_w2.z / n))
	var srms := Vector3(sqrt(_dg_s2.x / n), sqrt(_dg_s2.y / n), sqrt(_dg_s2.z / n))
	# a full oscillation is two sign changes
	var hz := _dg_cross / maxf(_dg_t, 1e-3) * 0.5
	print("[diag] body-rate rms (p,y,r) = %.3f %.3f %.3f rad/s" % [rms.x, rms.y, rms.z]
			+ "  stick rms %.3f %.3f %.3f" % [srms.x, srms.y, srms.z]
			+ "  osc %.1f %.1f %.1f Hz" % [hz.x, hz.y, hz.z])
	_dg_rms = rms
	_dg_n = 0
	_dg_t = 0.0
	_dg_w2 = Vector3.ZERO
	_dg_s2 = Vector3.ZERO
	_dg_cross = Vector3.ZERO
