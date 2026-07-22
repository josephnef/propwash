# The world: sky, ground, treeline, gates — and the scene registry.
#
# Split out of main.gd, which had grown to the point where the lockstep loop,
# collision detection, the demo and 450 lines of procedural scenery all lived in
# one file. The seam is clean because world-building only ever wrote nodes under
# one parent and read the quality tier; nothing else in main.gd depended on its
# internals.
#
# Everything parents under the root Node3D handed to build(), which is either
# main itself (headless / goggle-off) or the SubViewport's world node — see
# main.gd _make_world_root.
#
# NOTE the ground is deliberately a flat analytic y=0 plane and must stay one:
# _detect_contacts tests the hull against it in closed form, so any visual
# relief would desync (the drone would sink into hills and hover over valleys).
# Scenes add relief as separate StaticBody3D colliders instead, which go through
# the engine-query path that gates and tree trunks already use.
extends RefCounted

const PwProtocol = preload("res://scripts/protocol.gd")

# ---------------------------------------------------------- scene registry
# `field` is the original flying field and is the DEFAULT on every path: the
# flythrough, client_collision and fpv_cull ctests all assert against its
# layout, so a new scene must be opt-in (PROPWASH_SCENE) rather than replace it.
# `park` is field plus freestyle furniture — it never moves or removes anything
# field has, so a run in the park is still a superset of the tested layout.
const SCENES := ["field", "park"]
const DEFAULT_SCENE := "field"

# Geometry the demo and the tests depend on: uprights at x = +/-GATE_HALF, top
# bar at GATE_H, gates on the -z centreline. The opening must stay clear —
# decoration goes outside it, never across it.
const GATE_HALF := 1.2
const GATE_H := 2.05
const TUBE_R := 0.055
const GATE_Z := [-6.0, -14.0, -22.0]

const GROUND_SIZE := 1000.0   # the flythrough asserts z < -30; the old 60x60
                              # plane ended exactly there, so the test passed by
                              # flying off the last polygon
const WORLD_SEED := 0x9E3779B9   # fixed: the world must be identical every run

# Outputs. main.gd's auto-exposure hunt drives env.tonemap_exposure every frame
# and needs the sun to compute sun alignment, so both are handed back rather
# than kept private.
var env: Environment
var sun: DirectionalLight3D

var _root: Node3D
var _q: Dictionary


# Build `scene` under `root`. `q` is a PwQuality.TIERS entry — the tier is baked
# into the scene at build time (tree count, leaf cards), which is why the client
# resolves quality before building rather than after.
func build(scene: String, root: Node3D, q: Dictionary) -> void:
	_root = root
	_q = q
	if not SCENES.has(scene):
		push_warning("[pw] unknown scene '%s' — falling back to '%s'"
				% [scene, DEFAULT_SCENE])
		scene = DEFAULT_SCENE
	_build_sky_and_sun()
	_build_ground()
	_build_treeline()
	for i in range(GATE_Z.size()):
		_build_gate(GATE_Z[i], i)
	if scene == "park":
		_build_park()


# ------------------------------------------------------------- primitives
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
	_root.add_child(mi)


# A box that is also solid. Everything the park is built from goes through
# here, so a decorative box and a flyable-into box can never drift apart —
# which is exactly how you end up with scenery the quad passes through.
func _add_solid_box(parent: Node3D, pos: Vector3, size: Vector3, color: Color,
		rough: float = 0.85, surface: int = PwProtocol.SURF_OBJECT) -> void:
	var mi := MeshInstance3D.new()
	var mesh := BoxMesh.new()
	mesh.size = size
	mesh.material = _shared_material(color, rough, 0.0)
	mi.mesh = mesh
	mi.position = pos
	parent.add_child(mi)

	var body := StaticBody3D.new()
	body.set_meta("pw_surface", surface)
	var shape := CollisionShape3D.new()
	var box := BoxShape3D.new()
	box.size = size
	shape.shape = box
	body.add_child(shape)
	body.position = pos
	parent.add_child(body)


# --------------------------------------------------------------- sky / sun
func _build_sky_and_sun() -> void:
	# Sun lower than the old -55 deg: longer shadows read as shape, and a low sun
	# is what an evening flying session actually looks like.
	var lamp := DirectionalLight3D.new()
	lamp.rotation_degrees = Vector3(-42, -35, 0)
	lamp.light_energy = 1.5
	lamp.light_color = Color(1.0, 0.97, 0.92)
	lamp.shadow_enabled = true
	# shadow_bias defaults to 0.1 -- comparable to the whole 0.11 m airframe, so
	# the quad had effectively no self-shadowing and peter-panned off the ground
	lamp.shadow_bias = 0.035
	lamp.shadow_normal_bias = 2.4
	# the quad flies low and close, so bias the cascades hard toward the near field
	lamp.directional_shadow_split_1 = 0.05
	lamp.directional_shadow_split_2 = 0.15
	lamp.directional_shadow_split_3 = 0.40
	lamp.directional_shadow_fade_start = 0.9
	_root.add_child(lamp)

	var node := WorldEnvironment.new()
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

	_apply_quality_to_env(e, lamp)   # tier-dependent: shadows, glow, ssao, ssil, volfog
	node.environment = e
	_root.add_child(node)
	env = e      # the exposure hunt drives tonemap_exposure each frame
	sun = lamp


# Tier settings for the things this file builds. The viewport-level and goggle
# tier settings stay in main.gd — the split is by what owns the object, not by
# concept, so neither file reaches into the other's nodes.
func _apply_quality_to_env(e: Environment, lamp: DirectionalLight3D) -> void:
	lamp.light_angular_distance = _q.sun_angular
	lamp.directional_shadow_mode = _q.shadow_splits
	lamp.directional_shadow_blend_splits = _q.shadow_blend
	lamp.directional_shadow_max_distance = _q.shadow_max_dist

	e.glow_enabled = _q.glow
	if _q.glow:
		e.glow_intensity = 0.5
		e.glow_strength = 1.0
		e.glow_bloom = 0.05
		e.glow_blend_mode = Environment.GLOW_BLEND_MODE_SOFTLIGHT
		e.glow_hdr_threshold = 1.0

	# Forward+ only -- gated on a real RenderingDevice, not on the project
	# setting, which still claims forward_plus after the OpenGL3 fallback.
	var has_rd := RenderingServer.get_rendering_device() != null
	e.ssao_enabled = _q.ssao and has_rd
	if e.ssao_enabled:
		e.ssao_radius = 0.6        # small: this geometry is cm-scale
		e.ssao_intensity = 1.5
		e.ssao_power = 1.5
	e.ssil_enabled = _q.ssil and has_rd
	e.volumetric_fog_enabled = _q.volfog and has_rd
	if e.volumetric_fog_enabled:
		e.volumetric_fog_density = 0.008
		e.volumetric_fog_albedo = Color(0.80, 0.85, 0.90)
		e.volumetric_fog_anisotropy = 0.4   # forward scatter -> sun shafts
		e.volumetric_fog_length = 128.0
		# default temporal reprojection is tuned for slow cameras; on a quad at
		# 30 m/s and 800 deg/s it smears trails behind the fog volume
		e.volumetric_fog_temporal_reprojection_enabled = false


# ----------------------------------------------------------------- ground
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
	_root.add_child(ground)


# --------------------------------------------------------------- treeline
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

		# trunk collision: a vertical CYLINDER per tree. Trunk only — the
		# crowns are visually porous alpha cards, and a canopy collider would
		# block what clearly looks flyable-through.
		#
		# A cylinder and not a capsule, which is what this used to be. A
		# capsule's bottom hemisphere tapers to a point at ground level, so the
		# collider shrank to nothing exactly where a landed or crashed quad
		# sits: measured, the hull could get 5 cm INSIDE the drawn trunk at
		# y = 0.02 m while contact at y = 2 m was correct. A quad wedged into a
		# tree it was never told it had hit is what sent us looking for this.
		# A trunk is a cylinder; model it as one.
		var body := StaticBody3D.new()
		body.set_meta("pw_surface", PwProtocol.SURF_TREE)
		var shape := CollisionShape3D.new()
		var cyl := CylinderShape3D.new()
		cyl.radius = clampf(0.09 * w, 0.05, 0.5)
		cyl.height = 0.9 * h
		shape.shape = cyl
		shape.position = Vector3(0, 0.45 * h, 0)
		body.add_child(shape)
		body.position = t.origin
		_root.add_child(body)

	var mmi := MultiMeshInstance3D.new()
	mmi.multimesh = mm
	# 95-475 m out: their shadows are invisible but would pollute every cascade
	mmi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	_root.add_child(mmi)


# ------------------------------------------------------------------ gates
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


# `plain` swaps the hazard banding for a flat material. Course furniture (gates,
# slalom poles) is taped; a scaffold tower is not, and striping it made the park
# look like the whole world was made of race gates.
func _add_tube(parent: Node3D, from: Vector3, to: Vector3, radius: float,
		plain: StandardMaterial3D = null) -> void:
	var mesh := CylinderMesh.new()
	mesh.top_radius = radius
	mesh.bottom_radius = radius
	mesh.height = from.distance_to(to)
	mesh.radial_segments = 12      # round profile; a box silhouette reads as CG
	mesh.rings = 1
	if plain != null:
		mesh.material = plain
	else:
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
	_root.add_child(gate)

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


# ------------------------------------------------------------------- park
# `field` plus freestyle furniture. Everything here is ADDITIVE — the pad, the
# three gates and the treeline stay exactly where `field` puts them — so the
# park is a superset of the layout the ctests assert against and the acro gate
# line flies identically in both.
#
# Two things drove the layout. First, SPEED IS INVISIBLE WITHOUT NEAR
# REFERENCES: `field`'s treeline starts at 95 m, which is why 7 m/s there looks
# like hovering. The near trees and the wall exist to give the eye something
# that moves. Second, the elements sit either side of the -z gate line rather
# than strung along it, so a single camera can hold the quad and the obstacle
# in one frame instead of panning 60 m.
#
# The ground stays the analytic y=0 plane (see the header) — every piece of
# relief below is a box or tube collider.
const LOOP_GATE_Z := -30.0     # power loops need a taller opening than 2.05 m
const LOOP_GATE_H := 3.6
const LOOP_GATE_HALF := 2.2

const TOWER_POS := Vector3(-12.0, 0.0, -16.0)   # dive platform, left of the line
const TOWER_H := 14.0
const TOWER_HALF := 1.25

const BANDO_POS := Vector3(14.0, 0.0, -20.0)    # gap threads, right of the line
const BANDO_HALF_X := 6.0
const BANDO_HALF_Z := 4.0
const BANDO_H := 5.0

const SLALOM_Z0 := -36.0
const SLALOM_DZ := -3.0
const SLALOM_N := 6
const SLALOM_X := 1.8

const CONCRETE := Color(0.62, 0.60, 0.57)
const CONCRETE_DARK := Color(0.44, 0.43, 0.41)
const STEEL := Color(0.38, 0.40, 0.43)


func _build_park() -> void:
	_build_loop_gate()
	_build_tower()
	_build_bando()
	_build_slalom()
	_build_wall()
	_build_near_trees()


# A second gate, tall and wide enough to power-loop: the quad has to fit through
# the opening travelling forward AND come back over the top bar, which the 2.05 m
# race gates cannot give it.
func _build_loop_gate() -> void:
	_gate_materials()
	var gate := Node3D.new()
	gate.position = Vector3(0, 0, LOOP_GATE_Z)
	_root.add_child(gate)
	_add_tube(gate, Vector3(-LOOP_GATE_HALF, 0.0, 0),
			Vector3(-LOOP_GATE_HALF, LOOP_GATE_H, 0), TUBE_R * 1.4)
	_add_tube(gate, Vector3(LOOP_GATE_HALF, 0.0, 0),
			Vector3(LOOP_GATE_HALF, LOOP_GATE_H, 0), TUBE_R * 1.4)
	_add_tube(gate, Vector3(-LOOP_GATE_HALF - TUBE_R, LOOP_GATE_H, 0),
			Vector3(LOOP_GATE_HALF + TUBE_R, LOOP_GATE_H, 0), TUBE_R * 1.4)


# Scaffold tower: the dive platform. A dive is the single most legible freestyle
# move from an external camera, and it needs somewhere to dive FROM — height the
# eye can measure against.
func _build_tower() -> void:
	_gate_materials()
	var tower := Node3D.new()
	tower.position = TOWER_POS
	_root.add_child(tower)
	var h := TOWER_H
	var d := TOWER_HALF
	var steel := _shared_material(STEEL, 0.55, 0.7)
	for sx in [-1.0, 1.0]:
		for sz in [-1.0, 1.0]:
			var foot := Vector3(sx * d, 0.0, sz * d)
			_add_tube(tower, foot, foot + Vector3(0, h, 0), 0.06, steel)
	# horizontal ties every 3.5 m: without them four verticals read as four
	# poles that happen to stand near each other, not as one structure
	var lvl := 3.5
	while lvl <= h:
		for sx in [-1.0, 1.0]:
			_add_tube(tower, Vector3(sx * d, lvl, -d), Vector3(sx * d, lvl, d),
					0.04, steel)
		for sz in [-1.0, 1.0]:
			_add_tube(tower, Vector3(-d, lvl, sz * d), Vector3(d, lvl, sz * d),
					0.04, steel)
		lvl += 3.5
	# the platform itself, solid: the demo dives off its edge
	_add_solid_box(tower, Vector3(0, h + 0.06, 0),
			Vector3(d * 2.4, 0.12, d * 2.4), CONCRETE_DARK)


# An open concrete shell — a bando, which is where 3.5" cinewhoops actually get
# flown. Four columns, a roof with a hole in it, one wall with two window gaps
# and one open doorway: every opening is a real gap the quad can be threaded
# through, and the columns give proximity at speed.
func _build_bando() -> void:
	var b := Node3D.new()
	b.position = BANDO_POS
	_root.add_child(b)
	var hx := BANDO_HALF_X
	var hz := BANDO_HALF_Z
	var h := BANDO_H
	var col := 0.4

	for sx in [-1.0, 1.0]:
		for sz in [-1.0, 1.0]:
			_add_solid_box(b, Vector3(sx * (hx - col * 0.5), h * 0.5, sz * (hz - col * 0.5)),
					Vector3(col, h, col), CONCRETE)

	# Roof: four slabs around a central 3 x 3 m opening, so the quad can drop
	# in from above and exit through a wall.
	var oh := 1.5                                  # half the roof opening
	var rt := 0.25
	var ry := h + rt * 0.5
	_add_solid_box(b, Vector3(0, ry, (oh + hz) * 0.5),
			Vector3(hx * 2.0, rt, hz - oh), CONCRETE_DARK)
	_add_solid_box(b, Vector3(0, ry, -(oh + hz) * 0.5),
			Vector3(hx * 2.0, rt, hz - oh), CONCRETE_DARK)
	_add_solid_box(b, Vector3((oh + hx) * 0.5, ry, 0),
			Vector3(hx - oh, rt, oh * 2.0), CONCRETE_DARK)
	_add_solid_box(b, Vector3(-(oh + hx) * 0.5, ry, 0),
			Vector3(hx - oh, rt, oh * 2.0), CONCRETE_DARK)

	# Back wall with two window gaps (1.6 m square, sills at 1.2 m). Built as
	# piers and lintels rather than as a wall with holes cut in it: box
	# colliders cannot have holes, and a CSG'd wall would be visually open and
	# physically solid, which is the worst possible failure here.
	var wt := 0.3
	var wz := -hz + wt * 0.5
	var win := 1.6
	var sill := 1.2
	var piers := [-hx, -win, win, hx]            # x edges of the three piers
	for i in range(0, piers.size(), 2):
		var x0: float = piers[i]
		var x1: float = piers[i + 1]
		_add_solid_box(b, Vector3((x0 + x1) * 0.5, h * 0.5, wz),
				Vector3(absf(x1 - x0), h, wt), CONCRETE)
	# centre pier between the two windows
	_add_solid_box(b, Vector3(0, h * 0.5, wz), Vector3(0.6, h, wt), CONCRETE)
	# sill and lintel spanning both windows
	for span in [[-win, -0.3], [0.3, win]]:
		var x0: float = span[0]
		var x1: float = span[1]
		_add_solid_box(b, Vector3((x0 + x1) * 0.5, sill * 0.5, wz),
				Vector3(x1 - x0, sill, wt), CONCRETE)
		var top := sill + win
		_add_solid_box(b, Vector3((x0 + x1) * 0.5, (top + h) * 0.5, wz),
				Vector3(x1 - x0, h - top, wt), CONCRETE)

	# Front wall: solid either side of a 2.4 m doorway, open above it.
	var door := 1.2
	var lint := 2.4
	var fz := hz - wt * 0.5
	for sx in [-1.0, 1.0]:
		_add_solid_box(b, Vector3(sx * (hx + door) * 0.5, h * 0.5, fz),
				Vector3(hx - door, h, wt), CONCRETE)
	_add_solid_box(b, Vector3(0, (lint + h) * 0.5, fz),
			Vector3(door * 2.0, h - lint, wt), CONCRETE)

	# rubble against the base — a clean shell reads as a CAD model
	var rng := RandomNumberGenerator.new()
	rng.seed = WORLD_SEED + 909
	for i in 14:
		var s := rng.randf_range(0.25, 0.7)
		_add_solid_box(b, Vector3(rng.randf_range(-hx, hx), s * 0.4,
						rng.randf_range(-hz, hz)),
				Vector3(s, s * 0.8, s * rng.randf_range(0.6, 1.4)), CONCRETE_DARK)


# Slalom poles, alternating either side of the centreline. The cheapest possible
# demonstration that the quad has real roll authority, and it reads instantly.
func _build_slalom() -> void:
	_gate_materials()
	var s := Node3D.new()
	_root.add_child(s)
	for i in SLALOM_N:
		var x := SLALOM_X * (1.0 if i % 2 == 0 else -1.0)
		var z := SLALOM_Z0 + SLALOM_DZ * i
		_add_tube(s, Vector3(x, 0.0, z), Vector3(x, 2.6, z), 0.05)


# A low wall down the left of the flight line: a proximity reference to rip
# along. Broken into panels with gaps so it reads as derelict.
func _build_wall() -> void:
	var w := Node3D.new()
	_root.add_child(w)
	var z := -8.0
	var i := 0
	while z > -28.0:
		var seg := 3.4
		_add_solid_box(w, Vector3(-5.0, 0.6, z - seg * 0.5),
				Vector3(0.35, 1.2 + (0.25 if i % 2 == 0 else 0.0), seg), CONCRETE)
		z -= seg + 0.6
		i += 1


# A handful of trees close enough to matter. The MultiMesh treeline sits at
# 95-475 m, which gives the horizon depth but no sense of speed at all; these
# are the ones that whip past.
#
# PLACED BY HAND, not scattered. The first version seeded them randomly in a
# ring and they landed in the middle of the flight lines: the orbit around the
# tower flew straight through one and came back with 0.31 prop damage. Scenery
# a scripted pilot has to survive is level design, not decoration — these
# positions frame the corridors and stay out of them. Keep-clear zones are the
# gate line (|x| < 4), the tower orbit (12 m around TOWER_POS) and the bando
# footprint; the third column is height, which also sets canopy width.
const NEAR_TREES := [
	# x,      z,      height, conifer
	[  7.0,  -6.0,  6.5, true],
	[  9.0, -11.0,  8.0, false],
	[  7.0, -31.0,  5.5, true],
	[  9.0, -40.0,  7.5, false],
	[ -7.0,  -4.0,  7.0, false],
	[ -8.0, -29.0,  6.0, true],
	[ -7.0, -37.0,  8.5, false],
	[ -9.0, -45.0,  5.0, true],
	[-22.0,  -7.0,  9.0, false],
	[-23.0, -26.0,  7.5, true],
	[ 18.0,  -7.0,  8.0, false],
	[ 20.0, -33.0,  6.5, true],
]


func _build_near_trees() -> void:
	var rng := RandomNumberGenerator.new()
	rng.seed = WORLD_SEED + 4242
	if _leaf_tex == null:
		_leaf_tex = _make_leaf_texture()
	for t in NEAR_TREES:
		var conifer: bool = t[3]
		var h: float = t[2]
		var mesh := _make_tree_mesh(rng, conifer)
		var wide := h * (0.30 if conifer else 0.62)
		var pos := Vector3(t[0], 0.0, t[1])

		var mi := MeshInstance3D.new()
		mi.mesh = mesh
		mi.position = pos
		mi.scale = Vector3(wide, h, wide)
		_root.add_child(mi)

		# Trunk only, like the treeline: the canopy is porous alpha cards and a
		# canopy collider would block what plainly looks flyable-through. It
		# also means a proximity pass can brush the leaves without damage,
		# which is the shot you actually want.
		var body := StaticBody3D.new()
		body.set_meta("pw_surface", PwProtocol.SURF_TREE)
		var shape := CollisionShape3D.new()
		# cylinder, not a capsule — see the note in _scatter_trees: a capsule's
		# bottom cap tapers away exactly where a crashed quad rests
		var cyl := CylinderShape3D.new()
		cyl.radius = clampf(0.09 * wide, 0.06, 0.5)
		cyl.height = 0.9 * h
		shape.shape = cyl
		shape.position = Vector3(0, 0.45 * h, 0)
		body.add_child(shape)
		body.position = pos
		_root.add_child(body)
