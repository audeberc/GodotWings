## Procedural rolling terrain for the demo, generated at runtime (no external
## plugins). Flat around the origin so the runway sits level, gentle hills
## further out for visual motion/scale reference.
##
## A trimesh collider is generated alongside the mesh (collision layer 1), so the
## FDM's terrain-following ground (GWFlightBody.terrain_following) can raycast it.
## Enable terrain_following on the aircraft to have the gear/roll-out/crash logic
## follow these hills instead of the flat plane.
@tool
extends MeshInstance3D

@export var world_size: float = 6000.0   ## terrain extent (m)
@export var cells: int = 96              ## grid resolution per side
@export var amplitude: float = 100.0      ## hill height (m)
@export var flat_radius: float = 10.0   ## flat zone radius around origin (m)
@export var regenerate: bool = false:    ## tick in the editor to rebuild
	set(v):
		if v:
			_generate()


func _ready() -> void:
	_generate()


func _height(x: float, z: float, noise: FastNoiseLite) -> float:
	var d := Vector2(x, z).length()
	var blend := clampf((d - flat_radius) / flat_radius, 0.0, 1.0)
	return noise.get_noise_2d(x, z) * amplitude * blend - 2.0 # sit just below sea level


func _generate() -> void:
	var noise := FastNoiseLite.new()
	noise.noise_type = FastNoiseLite.TYPE_SIMPLEX
	noise.frequency = 0.0006

	var st := SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)
	var step := world_size / float(cells)
	var half := world_size * 0.5
	for iz in cells:
		for ix in cells:
			var x0 := ix * step - half
			var z0 := iz * step - half
			var x1 := x0 + step
			var z1 := z0 + step
			var p00 := Vector3(x0, _height(x0, z0, noise), z0)
			var p10 := Vector3(x1, _height(x1, z0, noise), z0)
			var p01 := Vector3(x0, _height(x0, z1, noise), z1)
			var p11 := Vector3(x1, _height(x1, z1, noise), z1)
			# two triangles per cell, wound so the surface normals face UP
			st.add_vertex(p00); st.add_vertex(p10); st.add_vertex(p01)
			st.add_vertex(p10); st.add_vertex(p11); st.add_vertex(p01)
	st.generate_normals()
	mesh = st.commit()

	# Solid collider for terrain-following ground (and any shapecast obstacles).
	for c in get_children():
		if c is StaticBody3D:
			c.free()
	create_trimesh_collision()  # adds a StaticBody3D + ConcavePolygonShape on layer 1
