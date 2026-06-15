@tool
@icon("res://addons/godotwings/sensors/camera_icon.svg")
class_name GWGrassField
extends MultiMeshInstance3D

## A grass field that FOLLOWS a target (the active camera by default), so grass is
## always around the aircraft without scattering blades over the whole world.
##
## The MultiMesh is a fixed local grid that snaps to the target's grid cell each
## frame; the shader hashes per-blade variety from the world cell (so it doesn't
## swim), softens the patch edge, fades with altitude, and excludes a runway rect.

@export var enabled := true:
	set(v): enabled = v; _refresh_preview()
## Square side (m) of the patch that follows the target (its radius ~ half this).
@export var area_size := 400.0:
	set(v): area_size = v; _refresh_preview()
## Target blade count; the grid is sized from it (instances = grid x grid).
@export var blade_count := 90000:
	set(v): blade_count = v; _refresh_preview()
@export var render_distance := 110.0:
	set(v): render_distance = v; _refresh_preview()
@export var blade_width := 0.08:
	set(v): blade_width = v; _refresh_preview()
@export var blade_height := 0.6:
	set(v): blade_height = v; _refresh_preview()
@export_range(0.0, 1.0) var height_variation := 0.4:
	set(v): height_variation = v; _refresh_preview()
@export var base_color := Color(0.13, 0.33, 0.09):
	set(v): base_color = v; _refresh_preview()
@export var tip_color := Color(0.42, 0.68, 0.22):
	set(v): tip_color = v; _refresh_preview()
@export_range(0.0, 1.0) var color_variation := 0.18:
	set(v): color_variation = v; _refresh_preview()
## Soft-edge width (m) at the patch boundary (the field's density falloff).
@export var edge_fade := 25.0:
	set(v): edge_fade = v; _refresh_preview()

@export_group("Follow")
## Node to follow. Empty = the active Camera3D (grass appears wherever you look).
@export var follow_target: NodePath
## Physics layers the ground ray tests (to sit the field on the ground each frame).
@export_flags_3d_physics var ground_mask := 1
## Ground height used if the follow ray misses.
@export var ground_level := 0.0

@export_group("Altitude fade")
## Grass starts fading when the camera is this high above the ground (AGL)...
@export var altitude_fade_start := 150.0
## ...and is gone by this height (fly high => no grass, no cost).
@export var altitude_fade_end := 400.0

@export_group("Terrain following")
## World size to bake a height field over so blades drape over hills (0 = flat
## ground, no heightmap). Should comfortably cover where you fly low.
@export var terrain_extent := 0.0
@export var terrain_resolution := 128
@export var terrain_center := Vector3.ZERO

@export_group("Runway exclusion")
## World-space rect (half-extents, m) kept clear of grass; (0,0) = none.
@export var exclude_half_extents := Vector2.ZERO:
	set(v): exclude_half_extents = v; _refresh_preview()
@export var runway_center := Vector3.ZERO:
	set(v): runway_center = v; _refresh_preview()

const GRASS_SHADER := "res://addons/godotwings/world/grass.gdshader"
var _cell := 1.0
var _ground_y := 0.0


func _ready() -> void:
	if not enabled:
		return
	_build()
	if not Engine.is_editor_hint():
		set_process(true)
		await get_tree().physics_frame
		await get_tree().physics_frame
		_bake_heightmap()


func _process(_dt: float) -> void:
	if Engine.is_editor_hint() or not (material_override is ShaderMaterial):
		return
	var t := _target()
	if t == null:
		return
	var tp := t.global_position
	# One ground ray per frame at the field centre (not per blade).
	var gy := ground_level
	var space := get_world_3d().direct_space_state
	if space != null:
		var hit := space.intersect_ray(PhysicsRayQueryParameters3D.create(
				tp + Vector3(0, 500, 0), tp - Vector3(0, 500, 0), ground_mask))
		if not hit.is_empty():
			gy = hit.position.y
	_ground_y = gy
	# Snap to the grid cell so the blade pattern doesn't swim as we move.
	global_position = Vector3(round(tp.x / _cell) * _cell, gy, round(tp.z / _cell) * _cell)
	var m: ShaderMaterial = material_override
	m.set_shader_parameter("field_center", global_position)
	m.set_shader_parameter("ground_y", gy)
	m.set_shader_parameter("altitude_fade_start", altitude_fade_start)
	m.set_shader_parameter("altitude_fade_end", altitude_fade_end)


## Raycast a grid over `terrain_extent` once and feed it to the shader so each
## blade can sit on the real ground (flat ground -> all zeros, harmless).
func _bake_heightmap() -> void:
	if terrain_extent <= 0.0 or not is_inside_tree() or not (material_override is ShaderMaterial):
		return
	var space := get_world_3d().direct_space_state
	if space == null:
		return
	var res := maxi(2, terrain_resolution)
	var img := Image.create(res, res, false, Image.FORMAT_RF)
	var half := terrain_extent * 0.5
	for z in res:
		for x in res:
			var wx := terrain_center.x - half + (float(x) / float(res - 1)) * terrain_extent
			var wz := terrain_center.z - half + (float(z) / float(res - 1)) * terrain_extent
			var h := ground_level
			var hit := space.intersect_ray(PhysicsRayQueryParameters3D.create(
					Vector3(wx, terrain_center.y + 2000.0, wz),
					Vector3(wx, terrain_center.y - 2000.0, wz), ground_mask))
			if not hit.is_empty():
				h = hit.position.y
			img.set_pixel(x, z, Color(h, 0.0, 0.0))
	var m: ShaderMaterial = material_override
	m.set_shader_parameter("heightmap", ImageTexture.create_from_image(img))
	m.set_shader_parameter("hm_extent", terrain_extent)
	m.set_shader_parameter("hm_center", terrain_center)


func _target() -> Node3D:
	if not follow_target.is_empty():
		return get_node_or_null(follow_target) as Node3D
	return get_viewport().get_camera_3d()


func _refresh_preview() -> void:
	if Engine.is_editor_hint() and is_inside_tree() and enabled:
		_build()


func _build() -> void:
	if not is_inside_tree():
		return
	var grid := maxi(1, int(round(sqrt(float(blade_count)))))
	_cell = area_size / float(grid)
	var half := area_size * 0.5

	var mm := MultiMesh.new()
	mm.transform_format = MultiMesh.TRANSFORM_3D
	mm.mesh = _blade_mesh()
	mm.instance_count = grid * grid
	var i := 0
	for gz in grid:
		for gx in grid:
			var x := -half + (gx + 0.5) * _cell
			var z := -half + (gz + 0.5) * _cell
			mm.set_instance_transform(i, Transform3D(Basis(), Vector3(x, 0.0, z)))
			i += 1
	multimesh = mm
	material_override = _make_material()
	cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	gi_mode = GeometryInstance3D.GI_MODE_DISABLED
	var vh := blade_height * (1.0 + height_variation) + 2.0
	custom_aabb = AABB(Vector3(-half - 1.0, -1.0, -half - 1.0), Vector3(area_size + 2.0, vh, area_size + 2.0))


func _make_material() -> ShaderMaterial:
	var mat := ShaderMaterial.new()
	mat.shader = load(GRASS_SHADER)
	mat.set_shader_parameter("base_color", base_color)
	mat.set_shader_parameter("tip_color", tip_color)
	mat.set_shader_parameter("color_variation", color_variation)
	mat.set_shader_parameter("render_distance", render_distance)
	mat.set_shader_parameter("cell_size", _cell)
	mat.set_shader_parameter("height_variation", height_variation)
	mat.set_shader_parameter("field_radius", area_size * 0.5)
	mat.set_shader_parameter("edge_band", edge_fade)
	mat.set_shader_parameter("field_center", global_position)
	mat.set_shader_parameter("ground_y", _ground_y)
	mat.set_shader_parameter("runway_center", runway_center)
	mat.set_shader_parameter("runway_half", exclude_half_extents)
	# Disabled altitude fade for the editor preview; _process sets real values at runtime.
	mat.set_shader_parameter("altitude_fade_start", 1.0e9)
	mat.set_shader_parameter("altitude_fade_end", 1.0e9)
	return mat


## One tapered blade quad: base at the origin, growing up +Y. UV.y = height.
func _blade_mesh() -> ArrayMesh:
	var w := blade_width
	var tw := blade_width * 0.2
	var h := blade_height
	var verts := PackedVector3Array([
		Vector3(-w * 0.5, 0.0, 0.0), Vector3(w * 0.5, 0.0, 0.0),
		Vector3(tw * 0.5, h, 0.0), Vector3(-tw * 0.5, h, 0.0)])
	var uvs := PackedVector2Array([Vector2(0, 0), Vector2(1, 0), Vector2(1, 1), Vector2(0, 1)])
	var normals := PackedVector3Array([Vector3.UP, Vector3.UP, Vector3.UP, Vector3.UP])
	var indices := PackedInt32Array([0, 1, 2, 0, 2, 3])
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = verts
	arrays[Mesh.ARRAY_TEX_UV] = uvs
	arrays[Mesh.ARRAY_NORMAL] = normals
	arrays[Mesh.ARRAY_INDEX] = indices
	var mesh := ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	return mesh
