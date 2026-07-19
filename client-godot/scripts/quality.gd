# Quality tiers: pure data plus the resolution rule. No node access, no side
# effects -- render_rig applies these. Kept as a GDScript Dictionary rather than
# a .tres Resource deliberately: a Resource would add an import step and a
# binary-ish artifact to review, and nobody is going to tweak these in an
# inspector on a project whose scene file is six lines long. A Dictionary diffs
# cleanly in a pull request, which is what actually matters here.
extends RefCounted

# Forward+-only effects are marked; under the gl_compatibility fallback they
# silently no-op, so render_rig gates them on an actual RenderingDevice probe
# rather than pretending they applied.
const TIERS := {
	"low": {
		"msaa": Viewport.MSAA_2X,
		"scale_3d": 1.0,           # native. never render below the panel.
		"shadow_atlas": 2048,
		"shadow_filter": RenderingServer.SHADOW_QUALITY_HARD,
		"shadow_splits": DirectionalLight3D.SHADOW_PARALLEL_2_SPLITS,
		"shadow_max_dist": 40.0,
		"shadow_blend": false,
		"sun_angular": 0.0,          # hard edge; no penumbra cost
		"glow": false,
		"ssao": false,               # Forward+ only
		"ssil": false,               # Forward+ only
		"volfog": false,             # Forward+ only
		"trees": 190,
		# goggle feed: the cheap effects are on at every tier, because the feed
		# treatment IS the look and must never silently vanish
		"goggle_block": false,
		"goggle_rs": 0.0,
		"leaf_cards": 7,
	},
	"medium": {
		"msaa": Viewport.MSAA_2X,
		"scale_3d": 1.0,
		"shadow_atlas": 4096,
		"shadow_filter": RenderingServer.SHADOW_QUALITY_SOFT_LOW,
		"shadow_splits": DirectionalLight3D.SHADOW_PARALLEL_4_SPLITS,
		"shadow_max_dist": 60.0,
		"shadow_blend": true,
		"sun_angular": 0.4,
		"glow": true,
		"ssao": true,
		"ssil": false,
		"volfog": false,
		"trees": 900,
		"goggle_block": true,
		"goggle_rs": 0.0,
		"leaf_cards": 14,
	},
	"high": {
		"msaa": Viewport.MSAA_4X,
		"scale_3d": 1.0,
		"shadow_atlas": 4096,
		"shadow_filter": RenderingServer.SHADOW_QUALITY_SOFT_MEDIUM,
		"shadow_splits": DirectionalLight3D.SHADOW_PARALLEL_4_SPLITS,
		"shadow_max_dist": 80.0,
		"shadow_blend": true,
		"sun_angular": 0.5,
		"glow": true,
		"ssao": true,
		"ssil": true,
		"volfog": true,
		"trees": 1400,
		"goggle_block": true,
		"goggle_rs": 1.0,
		"leaf_cards": 20,
	},
}

# SDFGI is deliberately absent from every tier. An open field has almost no
# occluders, so sky ambient plus SSIL already does the work it would do; it costs
# 2-4 ms; and its cascades pop visibly when the camera translates fast, which is
# this sim's entire use case.

# Auto-selection is a PIXEL-RATE budget rule, not "fast monitor means fast GPU".
#
# Keying on refresh alone was wrong and measurement caught it: a 60 Hz 2940x1846
# Retina panel got "high" and then ran at 51 fps, below its own refresh, because
# it pushes 2.6x the pixels of 1080p. What the GPU actually has to sustain is
# width * height * refresh, so that is what the rule uses.
#
# Reference points (megapixels/sec):
#   1920x1080 @60  = 124   -> high
#   2940x1846 @60  = 326   -> medium   (measured 117 fps, comfortably over 60)
#   1920x1080 @144 = 299   -> medium
#   2560x1600 @240 = 983   -> low      (measured 326 fps, comfortably over 240)
const MPS_HIGH := 200.0
const MPS_MEDIUM := 600.0

static func auto_tier(refresh: float, pixels: float) -> String:
	if refresh <= 0.0 or pixels <= 0.0:
		return "medium"   # some drivers report -1; don't guess
	var mps := pixels * refresh / 1_000_000.0
	if mps < MPS_HIGH:
		return "high"
	if mps < MPS_MEDIUM:
		return "medium"
	return "low"


# Precedence: headless floor, then explicit env var, then measured budget.
static func resolve(want: String, refresh: float, pixels: float, headless: bool) -> String:
	if headless:
		return "low"   # CI asserts on stdout, not pixels; keep runs comparable
	if TIERS.has(want):
		return want
	return auto_tier(refresh, pixels)
