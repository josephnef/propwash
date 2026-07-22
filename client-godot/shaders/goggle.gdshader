// DJI O3 goggle-feed treatment.
//
// Design note, learned the hard way: real O3 footage is CLEAN. It is a ~50 Mbps
// 1080p digital link through good glass with digital lens correction. The
// recognisable signature is NOT rainbow fringing, heavy grain and mush -- that
// reads as a failing analogue link, which is the opposite era. What actually
// says "this is an O3 feed" is:
//
//   1. aggressive ISP sharpening, leaving bright halos on high-contrast edges
//   2. slightly punchy, faintly cool contrast with gently crushed blacks
//   3. mild corner falloff
//   4. artifacts ONLY under stress -- a whip-pan blowing the bitrate budget
//
// Everything here is therefore deliberately restrained. If you can point at an
// individual effect while flying, it is turned up too far.
//
// A plain canvas_item shader, NOT a CompositorEffect: CompositorEffect needs a
// RenderingDevice and would die silently under the fallback_to_opengl3 path,
// taking the signature look with it. The only thing it buys is depth/normal
// access, which none of this needs.
//
// The OSD is deliberately NOT processed here. On a real DJI system the
// Betaflight OSD is drawn by the goggles from MSP-DisplayPort data, not encoded
// into the video, so it stays crisp -- it lives on a CanvasLayer above this.
shader_type canvas_item;

// Reads TEXTURE -- the TextureRect's own texture, which IS the SubViewport the
// 3D renders into. Deliberately NOT hint_screen_texture: that forces a
// full-screen backbuffer copy every frame, measured at ~41 fps of a 212 fps
// frame at 4.1 MP (a one-tap passthrough cost the same as this whole shader,
// which is what identified the copy rather than the maths as the expense).

// Fed from the sim each frame; angvel is known exactly, so rolling shutter and
// codec stress need no motion-vector buffer and no frame history.
uniform vec3 angvel = vec3(0.0);        // rad/s, Godot frame
uniform float block_strength = 0.0;     // 0..1 codec stress envelope
uniform float time_seed = 0.0;

uniform float sharpen_amt = 0.42;       // the primary tell -- keep it the loudest
uniform float ca_amt = 0.22;            // barely perceptible, corners only
uniform float vignette_amt = 0.13;
uniform float noise_amt = 0.006;        // daylight sensor floor, not film grain
uniform float contrast_amt = 0.055;     // gentle S-curve; digital "punch"
uniform float rs_amt = 0.0;             // rolling shutter (0 disables)
uniform float block_enable = 0.0;       // tier gate

// How much of the goggle feed to apply. 1 = the O3 feed, 0 = the raw render.
//
// This exists for the demo director's external cameras. Everything above
// models a camera bolted to the airframe sending video down a digital link;
// applied to a chase or ground camera it would be claiming those views come
// through the same link, which they obviously do not. The director crossfades
// this on a cut rather than switching hard, so the transition does not pop.
uniform float feed_mix = 1.0;

float hash12(vec2 p) {
	return fract(sin(dot(p, vec2(12.9898, 78.233))) * 43758.5453);
}

// The feed treatment itself, as a plain function.
//
// It lives here rather than inline in fragment() for one hard reason: Godot
// forbids `return` inside a processor function (vertex/fragment/light), and an
// early-out is exactly what the feed_mix == 0 path wants. Writing it inline
// with a `return` compiles fine as GLSL and is rejected by Godot's parser with
// "Using 'return' in the 'fragment' processor function is incorrect" — which
// fails the WHOLE shader, silently drops the goggle look, and then floods the
// log with `version_get_shader: Parameter "version" is null` once per frame
// forever. A normal function has no such restriction.
vec3 goggle_feed(sampler2D tex, vec2 uv, vec2 px) {
	// --- rolling shutter. Rows are scanned top to bottom, so a row read later is
	// displaced by whatever the airframe did in between. For yaw this shear IS
	// the correct artifact. Small: an O3 reads out fast, it is not a GoPro of a
	// decade ago. Only visible on a hard whip.
	if (rs_amt > 0.0) {
		float row = 1.0 - uv.y;
		uv.x += rs_amt * angvel.y * row * 0.006;
		uv.y += rs_amt * angvel.x * row * 0.0025;
	}

	// --- chromatic aberration. Real O3 applies digital lens correction, so this
	// is a whisper at the extreme corners and nothing at all across the middle
	// two-thirds. The r^4 falloff keeps it out of the centre entirely.
	// CA only where it is actually visible. r^4 already confines it to the
	// corners, so across the middle of frame the extra two fetches were buying a
	// difference of zero -- skip them there entirely.
	vec2 c = uv - 0.5;
	float r2 = dot(c, c);
	vec3 col;
	if (r2 > 0.10) {
		vec2 off = c * r2 * r2 * ca_amt * 0.02;
		col = vec3(
			texture(tex, uv + off).r,
			texture(tex, uv).g,
			texture(tex, uv - off).b);
	} else {
		col = texture(tex, uv).rgb;
	}

	// --- H.264 stress. Not steady-state macroblocking: the convincing part is
	// the transient. A whip-pan blows the bitrate budget, the frame softens, and
	// it recovers over ~10 frames. Softening toward the block average rather than
	// hard-snapping to it -- modern encoders lose detail before they show edges.
	if (block_enable > 0.5 && block_strength > 0.004) {
		vec2 bs = px * 16.0;
		vec3 bcol = texture(tex, (floor(uv / bs) + 0.5) * bs).rgb;
		col = mix(col, bcol, clamp(block_strength, 0.0, 0.45));
	}

	// --- unsharp mask: the single most recognisable O3 trait. Halos on every
	// high-contrast edge, from an ISP that sharpens harder than it should.
	// 2 diagonal taps rather than a 4-tap cross: at 4.1 MP those two extra
	// fetches cost more than the halo quality they buy, and the halo is what
	// reads as "DJI ISP" either way
	vec3 blur = texture(tex, uv + px).rgb
	          + texture(tex, uv - px).rgb;
	col += (col - blur * 0.5) * sharpen_amt;

	// --- digital contrast: a gentle S-curve with slightly crushed blacks and a
	// faintly cool cast. This is most of what separates "camera" from "render".
	col = mix(col, col * col * (3.0 - 2.0 * col), contrast_amt * 4.0);
	col.b *= 1.012;
	col.r *= 0.995;

	// --- sensor floor. Animated off time or it reads as dirt on the lens.
	// Shadows only: a daylit O3 image is essentially noise-free in the highlights.
	float n = hash12(uv * 997.0 + time_seed);
	float lum = dot(col, vec3(0.299, 0.587, 0.114));
	col += (n - 0.5) * noise_amt * smoothstep(0.55, 0.0, lum);

	// --- corner falloff
	col *= 1.0 - vignette_amt * pow(length(c) * 1.414, 3.0);

	return clamp(col, vec3(0.0), vec3(1.0));
}


void fragment() {
	vec2 uv = UV;
	vec2 px = TEXTURE_PIXEL_SIZE;
	vec3 raw = texture(TEXTURE, uv).rgb;

	// Fully off: one tap and out. An external camera should not be paying for
	// the sharpen, CA and noise taps only to have them mixed back to zero.
	if (feed_mix < 0.004) {
		COLOR = vec4(raw, 1.0);
	} else {
		COLOR = vec4(mix(raw, goggle_feed(TEXTURE, uv, px), feed_mix), 1.0);
	}
}
