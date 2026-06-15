// Procedural grass for GWGrassField (original work, MIT).
//
// Designed for a field that FOLLOWS a target: the MultiMesh is a fixed local grid
// of blades and the GWGrassField node slides over the world (snapped to the grid
// cell). Per-blade variety (yaw, jitter, height, colour) is hashed from the WORLD
// cell in the vertex stage, so it stays put as the window slides — no swimming.
//
// Opaque (no discard => early-Z). All "fades" shrink the blade toward a degenerate
// point in the vertex stage, costing zero fragments far away:
//   * render distance (camera)   * soft patch edge (radial)
//   * altitude (fade out when the camera is high above the ground)
//   * runway/world-rect exclusion
shader_type spatial;
render_mode cull_disabled, diffuse_lambert;

uniform vec3 base_color : source_color = vec3(0.13, 0.33, 0.09);
uniform vec3 tip_color : source_color = vec3(0.42, 0.68, 0.22);
uniform float color_variation = 0.18;

uniform float wind_strength = 0.12;
uniform float wind_speed = 1.6;
uniform float wind_scale = 0.05;

uniform float cell_size = 1.0;
uniform float jitter = 0.45;          // in-cell xz scatter (fraction of a cell)
uniform float height_variation = 0.4;

uniform float render_distance = 130.0;
uniform float fade_margin = 30.0;
uniform vec3 field_center = vec3(0.0);
uniform float field_radius = 0.0;     // 0 = no edge fade
uniform float edge_band = 25.0;

uniform float ground_y = 0.0;         // height of the grass plane (field-centre ground)
uniform float altitude_fade_start = 80.0;
uniform float altitude_fade_end = 180.0;

uniform vec3 runway_center = vec3(0.0);
uniform vec2 runway_half = vec2(0.0); // world-space exclusion rect; (0,0) = none

uniform sampler2D heightmap : hint_default_black, filter_linear;
uniform float hm_extent = 0.0;        // world size the heightmap covers (0 = flat ground)
uniform vec3 hm_center = vec3(0.0);

varying float v_height;
varying float v_tint;

float hash12(vec2 p) {
	return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453);
}

void vertex() {
	// Cell centre in world (MultiMesh grid offset + the node's follow position).
	vec3 cellpos = (MODEL_MATRIX * vec4(0.0, 0.0, 0.0, 1.0)).xyz;
	vec2 cell = floor(cellpos.xz / cell_size + 0.5);
	float h1 = hash12(cell);
	float h2 = hash12(cell + 17.3);
	float h3 = hash12(cell + 41.7);

	float yaw = h1 * 6.2831853;
	vec2 off = (vec2(h2, h3) - 0.5) * 2.0 * jitter * cell_size;
	float hscale = 1.0 + (h3 - 0.5) * 2.0 * height_variation;

	// Rotate the blade quad around Y and scale its height (per world cell).
	float ca = cos(yaw);
	float sa = sin(yaw);
	vec3 v = VERTEX;
	v.y *= hscale;
	vec3 rv = vec3(v.x * ca - v.z * sa, v.y, v.x * sa + v.z * ca);

	vec3 wbase = cellpos + vec3(off.x, 0.0, off.y);  // blade base, world
	float phase = (wbase.x + wbase.z) * wind_scale;
	float sway = sin(TIME * wind_speed + phase) + 0.3 * sin(TIME * wind_speed * 2.3 + phase * 1.7);
	rv.x += sway * wind_strength * UV.y;
	rv.z += sway * wind_strength * 0.6 * UV.y;

	v_height = UV.y;
	v_tint = h2;

	// --- fades (each 0..1; take the smallest, then shrink the blade by it) ---
	float fade = 1.0;
	fade = min(fade, clamp((render_distance - distance(wbase, CAMERA_POSITION_WORLD)) / max(fade_margin, 0.001), 0.0, 1.0));
	if (field_radius > 0.0) {
		fade = min(fade, clamp((field_radius - distance(wbase.xz, field_center.xz)) / max(edge_band, 0.001), 0.0, 1.0));
	}
	fade = min(fade, 1.0 - smoothstep(altitude_fade_start, altitude_fade_end, CAMERA_POSITION_WORLD.y - ground_y));
	if (runway_half.x > 0.0) {
		vec2 rd = abs(wbase.xz - runway_center.xz) - runway_half;
		if (rd.x < 0.0 && rd.y < 0.0) {
			fade = 0.0;
		}
	}

	rv += vec3(off.x, 0.0, off.y);   // place within the cell (local == world here)
	VERTEX = rv * fade;
	// Per-blade terrain height: lift the blade base to the sampled ground so the
	// field drapes over hills instead of sitting on a flat plane.
	if (hm_extent > 0.0) {
		vec2 huv = (wbase.xz - hm_center.xz) / hm_extent + 0.5;
		VERTEX.y += texture(heightmap, huv).r - ground_y;
	}
	NORMAL = vec3(0.0, 1.0, 0.0);
}

void fragment() {
	vec3 col = mix(base_color, tip_color, v_height);
	col *= 1.0 + (v_tint - 0.5) * 2.0 * color_variation;
	ALBEDO = col;
	ROUGHNESS = 0.9;
	if (!FRONT_FACING) {
		NORMAL = -NORMAL;
	}
}
