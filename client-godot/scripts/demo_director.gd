# The demo director: which camera the audience is watching, and what the
# caption says it is looking at.
#
# WHY THIS EXISTS. A flip in FPV is close to unreadable if you do not fly — the
# horizon rotates and nothing else changes, so it looks like the video glitched.
# The trick only becomes legible from outside the airframe. But FPV is the
# authentic view and the one the OSD belongs to, so the answer is to cut
# between them per maneuver rather than pick one.
#
# THREE CAMERAS:
#
#   FPV    the O3's pose on the airframe (main.gd CAM_POS). Owned by main.gd,
#          the director only selects it. Goggle treatment and OSD ON.
#   CHASE  behind and above, NOT parented to the drone. Parenting it would make
#          it inherit roll, and a chase view that barrel-rolls with the quad is
#          unwatchable. It also follows the VELOCITY heading rather than the
#          nose, so a yaw spin spins the quad in frame instead of whipping the
#          camera around it.
#   LOS    a spectator on the ground. Placed at a cut and then held perfectly
#          still — that stillness is what makes it read as a person filming
#          rather than a third drone. This is also a working prototype of the
#          LOS spectator mode in issue #32.
#
# The goggle feed and the OSD are FPV-only. On real hardware the OSD is drawn
# by the goggles from MSP-DisplayPort data, so it belongs to the pilot's view
# and to nothing else.
extends RefCounted

enum {CAM_FPV, CAM_CHASE, CAM_LOS}

const CAM_NAMES := {"fpv": CAM_FPV, "chase": CAM_CHASE, "los": CAM_LOS}

# Chase geometry. Far enough back that the whole airframe plus some world fits
# in frame, high enough to see the ground go past underneath.
const CHASE_BACK := 3.2
const CHASE_UP := 1.1
const CHASE_LAG := 3.5         # 1/s; higher snaps harder to the ideal pose
const CHASE_MIN_SPEED := 1.5   # below this the velocity heading is just noise

# LOS tripod. Eye height, a sensible distance, and offset to the side so the
# quad crosses the frame rather than flying straight at the lens.
#
# LOS_FRAME_M is the width of world the zoom tries to hold in frame, and it is
# the number that decides whether this camera is worth having. The first value
# was 11 m, which is a perfectly sensible-sounding "see the quad and its
# surroundings" — and rendered a 142 mm airframe as a 25-pixel speck at 1280
# wide. 2.5 m puts the quad at ~8% of frame width, which is what makes a flip
# legible. The long lens that implies (6-20 deg) would be unusable handheld;
# held perfectly still it reads as a tripod, which is exactly the intent.
const LOS_EYE := 1.65
const LOS_DIST := 10.0
const LOS_FRAME_M := 2.5
const LOS_FOV_MIN := 6.0
const LOS_FOV_MAX := 55.0

const CUT_FADE := 0.18         # seconds to crossfade the goggle treatment

# PROPWASH_CAM_ZOOM=<factor> tightens the EXTERNAL cameras so the airframe
# fills more of the frame — for looking at the quad itself (does it tremble? do
# the props spin the right way? does the model look right?) rather than for
# filming the flight. 2.0 is a good inspection setting.
#
# Chase pulls in by the factor; LOS narrows the world width it holds, which is
# a zoom rather than a move — the tripod stays put, as it should.
#
# The FPV camera is deliberately NOT affected. It is a 105 deg lens bolted to
# the airframe; "zooming" it would be inventing hardware, and all it can see of
# the quad is the front ducts anyway.
const ZOOM_ENV := "PROPWASH_CAM_ZOOM"
const ZOOM_MIN := 0.5
const ZOOM_MAX := 6.0
var _zoom := 1.0

var _main: Node3D
var _chase: Camera3D
var _los: Camera3D
var _active := CAM_FPV
var _chase_pos := Vector3.ZERO
var _chase_ready := false
var _los_anchor := Vector3.ZERO
var _feed := 1.0               # current goggle mix, crossfaded on a cut

var _caption: Label
var _subcaption: Label
var _caption_text := ""


func setup(main: Node3D, world: Node3D, fpv_hidden_layer: int) -> void:
	_main = main
	var z := OS.get_environment(ZOOM_ENV)
	if z.is_valid_float():
		_zoom = clampf(z.to_float(), ZOOM_MIN, ZOOM_MAX)
		print("[pw][cam] external cameras zoomed x%.2f" % _zoom)

	_chase = Camera3D.new()
	_chase.name = "ChaseCamera"
	_chase.fov = 62.0            # a normal lens; the 105 deg FPV lens out here
	_chase.near = 0.05           # would make everything look tiny and distant
	_chase.far = 1500.0
	world.add_child(_chase)

	_los = Camera3D.new()
	_los.name = "LosCamera"
	_los.near = 0.05
	_los.far = 1500.0
	world.add_child(_los)

	# Both external cameras see the whole airframe, including the parts hidden
	# from the FPV feed (battery, camera pod, PCB) — from outside, a quad with
	# no battery reads as a render of a part rather than a machine. The layer
	# number is passed in rather than restated here: main.gd owns it, and
	# fpv_cull_test exists because that assignment has been silently dropped
	# once already.
	_chase.set_cull_mask_value(fpv_hidden_layer, true)
	_los.set_cull_mask_value(fpv_hidden_layer, true)

	# Physics interpolation OFF for both, and this is the fix for a real,
	# visible bug rather than a micro-optimisation.
	#
	# These two are driven from _process, at render rate. Godot's physics
	# interpolation assumes a node's transform is written on the PHYSICS frame
	# and renders it between the last two physics snapshots — so a node written
	# every render frame gets interpolated between values that are already
	# render-rate, and its drawn transform wobbles against its set transform.
	#
	# On a camera that wobble is worse than it sounds. A camera TRANSLATION
	# error moves near geometry across the frame far more than distant geometry
	# (plain parallax), so the drone 3.2 m away visibly trembles while the
	# treeline at 100-400 m does not move at all. That is exactly the "only the
	# drone is jittery, the world is smooth" report this came from.
	#
	# The FPV camera is deliberately NOT touched: it is a child of the drone,
	# is written on the physics frame with it, and must stay interpolated so it
	# inherits the same smoothing.
	_chase.physics_interpolation_mode = Node.PHYSICS_INTERPOLATION_MODE_OFF
	_los.physics_interpolation_mode = Node.PHYSICS_INTERPOLATION_MODE_OFF


func build_captions(ui: CanvasLayer) -> void:
	# Anchored to the BOTTOM edge, not to centre-plus-an-offset. The OSD is a
	# centred 16x30 character grid whose height depends on the font, and a
	# caption placed by centre offset landed straight on top of its bottom rows
	# at 720p while looking fine at other sizes. Bottom-anchored, the lower
	# third stays the lower third at any resolution.
	_caption = _make_caption(ui, 30, Color(1, 1, 1), 6, -86)
	# the second line carries the claim; the first only names the maneuver
	_subcaption = _make_caption(ui, 19, Color(1.0, 0.82, 0.35), 5, -52)


func _make_caption(ui: CanvasLayer, size: int, color: Color, outline: int,
		from_bottom: float) -> Label:
	var l := Label.new()
	l.set_anchors_preset(Control.PRESET_BOTTOM_WIDE)
	l.grow_horizontal = Control.GROW_DIRECTION_BOTH
	l.grow_vertical = Control.GROW_DIRECTION_BEGIN
	l.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	l.add_theme_font_size_override("font_size", size)
	l.add_theme_color_override("font_color", color)
	l.add_theme_color_override("font_outline_color", Color(0, 0, 0))
	l.add_theme_constant_override("outline_size", outline)
	l.offset_top = from_bottom
	l.offset_bottom = from_bottom + size + 8
	l.visible = false
	ui.add_child(l)
	return l


# Called every frame with the pilot's current hint. `cam` is one of the
# CAM_NAMES keys (empty keeps the current shot).
func update(dt: float, cam: String, caption: String, subcaption: String) -> void:
	if not cam.is_empty() and CAM_NAMES.has(cam):
		select(CAM_NAMES[cam])
	_set_caption(caption, subcaption)
	_track(dt)
	_blend_feed(dt)


func select(which: int) -> void:
	if which == _active:
		return
	_active = which
	if which == CAM_LOS:
		_place_los()
	_apply()


func active() -> int:
	return _active


# Cycle FPV -> CHASE -> LOS. Bound to `C` for hand flying, where a chase view
# is genuinely useful for learning what the quad is doing.
func cycle() -> String:
	select((_active + 1) % 3)
	return ["FPV", "chase", "LOS"][_active]


func _apply() -> void:
	var fpv: Camera3D = _main._cam
	if fpv != null:
		fpv.current = _active == CAM_FPV
	_chase.current = _active == CAM_CHASE
	_los.current = _active == CAM_LOS
	# the OSD is drawn by the goggles, so it exists only in the pilot's view
	if _main._osd != null:
		_main._osd.visible = _active == CAM_FPV


func _track(dt: float) -> void:
	# Aim at where the drone is DRAWN, not where physics says it is.
	#
	# The drone is physics-interpolated, so its rendered origin lags the raw
	# _pos by a fraction of a physics step that changes every frame. Framing the
	# shot on _pos therefore puts the quad a few mm off centre by a varying
	# amount — measured at 0.8 +/- 1.3 mm, peaking at 4.5 mm under acceleration
	# and exactly zero at constant velocity. Small, but it lands on the drone
	# and on nothing else, because no other object has an interpolation gap.
	var p: Vector3 = _main._drone.get_global_transform_interpolated().origin \
			if _main._drone != null else _main._pos
	var vel: Vector3 = _main._linvel
	var basis := Basis(_main._rot)

	# --- chase: critically-damped follow of an ideal pose behind the quad.
	# exp(-k*dt) rather than a fixed lerp factor so the smoothing is the same
	# at any frame rate — the client's physics rate is pinned but the render
	# rate is not, and this runs on the render frame.
	var heading := Vector3(vel.x, 0.0, vel.z)
	if heading.length() < CHASE_MIN_SPEED:
		# too slow for velocity to mean anything: fall back to where the nose
		# points, flattened so a pitched-over quad does not drag the camera
		# into the ground
		var nose := basis * Vector3(0, 0, -1)
		heading = Vector3(nose.x, 0.0, nose.z)
	if heading.length() < 0.01:
		heading = Vector3(0, 0, -1)
	heading = heading.normalized()

	var want := p - heading * (CHASE_BACK / _zoom) + Vector3.UP * (CHASE_UP / _zoom)
	# never clip through the ground plane; the floor has to come down with the
	# zoom or a tight chase just sits on the 0.45 m clamp and stops tracking
	want.y = maxf(want.y, 0.45 / _zoom)
	if not _chase_ready:
		_chase_pos = want
		_chase_ready = true
	else:
		# Lag scales WITH the zoom. The follow time constant is what actually
		# sets the on-screen distance while moving: at 7 m/s the default 3.5/s
		# trails about 2 m, which swamps a 1.3 m zoomed stand-off and leaves the
		# quad just as small as before. Tightening the shot has to tighten the
		# tracking too.
		_chase_pos = _chase_pos.lerp(want, 1.0 - exp(-CHASE_LAG * _zoom * dt))
	_chase.position = _chase_pos
	if _chase_pos.distance_to(p) > 0.05:
		_chase.look_at(p, Vector3.UP)

	# --- LOS: the tripod does not move. Only the pan, tilt and zoom do, which
	# is the whole difference between "someone is filming this" and "a third
	# drone is following it".
	if _los_anchor != Vector3.ZERO:
		_los.position = _los_anchor
		var d := _los_anchor.distance_to(p)
		if d > 0.2:
			_los.look_at(p, Vector3.UP)
			_los.fov = clampf(
					rad_to_deg(2.0 * atan((LOS_FRAME_M / _zoom) / (2.0 * d))),
					LOS_FOV_MIN, LOS_FOV_MAX)


# Stand the operator off to one side of the quad's current heading, so the
# action crosses frame instead of coming at the lens.
func _place_los() -> void:
	var p: Vector3 = _main._pos
	var vel: Vector3 = _main._linvel
	var heading := Vector3(vel.x, 0.0, vel.z)
	if heading.length() < CHASE_MIN_SPEED:
		var nose := Basis(_main._rot) * Vector3(0, 0, -1)
		heading = Vector3(nose.x, 0.0, nose.z)
	if heading.length() < 0.01:
		heading = Vector3(0, 0, -1)
	heading = heading.normalized()
	var side := heading.cross(Vector3.UP).normalized()
	_los_anchor = Vector3(p.x, 0.0, p.z) + side * LOS_DIST + Vector3.UP * LOS_EYE
	# keep the operator away from the origin pad so the shot never looks like
	# it is standing inside the take-off point
	_los_anchor.y = LOS_EYE


func _blend_feed(dt: float) -> void:
	var target := 1.0 if _active == CAM_FPV else 0.0
	var k := clampf(dt / CUT_FADE, 0.0, 1.0)
	_feed = lerpf(_feed, target, k)
	if _main._goggle_mat != null:
		_main._goggle_mat.set_shader_parameter("feed_mix", _feed)


func _set_caption(text: String, sub: String) -> void:
	if _caption == null:
		return
	if text != _caption_text:
		_caption_text = text
		_caption.text = text
		_subcaption.text = sub
	_caption.visible = not text.is_empty()
	_subcaption.visible = not sub.is_empty()
