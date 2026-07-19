// Ground: grass with an analytically-filtered reference grid.
//
// This replaces 62 thin BoxMesh grid lines. Those could not be anti-aliased --
// a 0.03 m box seen at 30 m is sub-pixel, so it crawls and moires no matter how
// much MSAA you throw at it, and its bottom face was coplanar with the ground
// plane so it z-fought as well.
//
// The fix is to compute line coverage analytically: fwidth() gives the pixel
// footprint in grid space, so a line narrower than a pixel resolves to a partial
// coverage value instead of a stair-stepped sample. Two scales are drawn -- a 1 m
// grid that fades out at middle distance and a 10 m grid that carries to the fog
// line -- so the pilot always has a motion reference at whatever distance they
// are reading, and neither ever aliases.
shader_type spatial;

uniform vec3 grass_dark : source_color = vec3(0.145, 0.205, 0.080);
uniform vec3 grass_light : source_color = vec3(0.235, 0.300, 0.120);
uniform float mow_width = 4.0;      // metres per mown band
uniform float mow_strength = 0.16;  // subtle: this is lay-of-the-grass, not paint

varying vec3 world_pos;

// Antialiased square wave in [0,1], period 2*w. Returns 0.5 (the average) once a
// band is near pixel-sized, so the far field converges to flat grass instead of
// moireing -- which is exactly what the old thin box grid lines could not do.
float band(float x, float w) {
	float t = x / (2.0 * w);
	float d = fwidth(t);
	float tri = abs(fract(t) - 0.5) * 2.0;              // 0..1 triangle
	float s = smoothstep(0.5 - d * 2.0, 0.5 + d * 2.0, tri);
	return mix(0.5, s, 1.0 - smoothstep(0.15, 0.5, d));  // fade to average
}

// sin() based hashing is needlessly expensive for a full-screen surface; this
// is the standard cheap integer-ish variant and is visually indistinguishable
// at the scales this shader uses it.
float hash(vec2 p) {
	vec3 q = fract(vec3(p.xyx) * vec3(0.1031, 0.1030, 0.0973));
	q += dot(q, q.yzx + 33.33);
	return fract((q.x + q.y) * q.z);
}

float vnoise(vec2 p) {
	vec2 i = floor(p);
	vec2 f = fract(p);
	f = f * f * (3.0 - 2.0 * f);
	return mix(mix(hash(i), hash(i + vec2(1.0, 0.0)), f.x),
	           mix(hash(i + vec2(0.0, 1.0)), hash(i + vec2(1.0, 1.0)), f.x), f.y);
}

void vertex() {
	world_pos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
}

void fragment() {
	vec2 p = world_pos.xz;

	// three octaves of value noise so the field has large-scale patchiness and
	// fine tonal break-up instead of reading as one flat green
	// two octaves, not three: the third was ~9 m detail that the mow banding and
	// the goggle grain already break up, so it cost a full noise evaluation per
	// pixel for something invisible in motion
	float n = vnoise(p * 0.35) * 0.65
	        + vnoise(p * 2.30) * 0.35;

	vec3 base = mix(grass_dark, grass_light, n);

	// Cross-mown pattern: two perpendicular band sets, as on a real sports
	// field. Gives the pilot a distance and motion reference along both axes
	// without the arcade look of glowing grid lines.
	float m = (band(p.x, mow_width) + band(p.y, mow_width * 1.5)) * 0.5;
	base *= 1.0 + (m - 0.5) * 2.0 * mow_strength;

	ALBEDO = base;
	ROUGHNESS = mix(0.96, 0.84, n);
	SPECULAR = 0.10;
}
