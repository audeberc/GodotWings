extends SceneTree

# Headless: tiles that carry their offset from the Earth's centre in their own
# glTF root node (as Google Photorealistic 3D Tiles do) land where float64
# says they should. Placed through float32 transforms alone, each lands up to
# ~0.75 m off at random, and neighbouring tiles open hairline gaps that show
# the sky; GWTiles3DContent.place_tile's `point_to_godot` removes that.
#
# Each case exports a box whose root node sits at a real ECEF position near
# the anchor (in Google's Y-up axes), places it both ways under an identity
# tileset transform, and compares the box's Godot position with float64.

const LAT := 46.52
const LON := 7.48
const ALT := 540.0

var _ok := true


func _check(cond: bool, msg: String) -> void:
	if cond:
		print("  PASS  ", msg)
	else:
		push_error("FAIL: " + msg)
		_ok = false


func _initialize() -> void:
	_run.call_deferred()


func _run() -> void:
	var parent := Node3D.new()
	root.add_child(parent)
	var rng := RandomNumberGenerator.new()
	rng.seed = 7
	var worst_before := 0.0
	var worst_after := 0.0
	for i in 40:
		var e := GWGeodeticConvert.geodetic_to_ecef_xyz(LAT + rng.randf_range(-0.03, 0.03),
				LON + rng.randf_range(-0.04, 0.04), ALT + rng.randf_range(-20.0, 80.0))
		# Google's axes: (X, Y, Z) = standard ECEF (X, Z, -Y). Rounded to
		# float32 here because that is what the exporter writes; the truth
		# below starts from the same rounded numbers.
		var t := Vector3(e[0], e[2], -e[1])
		var box := MeshInstance3D.new()
		box.mesh = BoxMesh.new()
		box.position = t
		var doc := GLTFDocument.new()
		var state := GLTFState.new()
		doc.append_from_scene(box, state)
		var glb := doc.generate_buffer(state)
		box.free()

		var truth := GWGeodeticConvert.ecef_xyz_to_godot_position(t.x, -t.z, t.y, LAT, LON, ALT)
		var xform := GWTiles3DTraversal.local_transform_to_godot(Transform3D.IDENTITY, LAT, LON, ALT)
		var to_godot := func(x: float, y: float, z: float) -> Vector3:
			return GWTiles3DTraversal.local_point_to_godot(Transform3D.IDENTITY, x, y, z, LAT, LON, ALT)

		var before := GWTiles3DContent.place_tile(parent, glb, xform, "google_yup_ecef", "before_%d" % i)
		var after := GWTiles3DContent.place_tile(parent, glb, xform, "google_yup_ecef", "after_%d" % i, to_godot)
		worst_before = maxf(worst_before, _mesh_position(before).distance_to(truth))
		worst_after = maxf(worst_after, _mesh_position(after).distance_to(truth))
		before.free()
		after.free()

	print("  worst tile offset: float32 path %.3f m, float64 path %.4f m" % [worst_before, worst_after])
	_check(worst_after < 0.01, "tiles placed through point_to_godot land within 1 cm of float64")
	_check(worst_after < worst_before, "and closer than through float32 transforms alone")
	_google_shape(parent)
	print("ALL PASS" if _ok else "SOME CHECKS FAILED")
	quit(0 if _ok else 1)


## Google's files come back from generate_scene() wrapped: an identity root
## with the tile node (Y-up to Z-up rotation, offset from the Earth's centre
## in its matrix) one level down. Godot's own exports don't reproduce that, so
## the generated scene and its glTF JSON are built here as Google's arrive.
func _google_shape(parent: Node3D) -> void:
	var e := GWGeodeticConvert.geodetic_to_ecef_xyz(LAT + 0.01, LON + 0.02, ALT + 30.0)
	var t := [e[0], e[2], -e[1]]
	var state := GLTFState.new()
	state.json = {"nodes": [{"mesh": 0, "matrix": [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0,
			t[0], t[1], t[2], 1.0]}]}
	state.root_nodes = PackedInt32Array([0])
	var scene := Node3D.new()
	var tile := MeshInstance3D.new()
	tile.transform = Transform3D(Basis(Vector3(1, 0, 0), Vector3(0, 0, 1), Vector3(0, -1, 0)), Vector3(t[0], t[1], t[2]))
	scene.add_child(tile)
	var xform := GWTiles3DTraversal.local_transform_to_godot(Transform3D.IDENTITY, LAT, LON, ALT)
	xform.basis = xform.basis * GWTiles3DContent.GOOGLE_YUP_ECEF_CORRECTION
	var to_godot := func(x: float, y: float, z: float) -> Vector3:
		return GWTiles3DTraversal.local_point_to_godot(Transform3D.IDENTITY, x, y, z, LAT, LON, ALT)
	var wrapper := Node3D.new()
	parent.add_child(wrapper)
	wrapper.transform = Transform3D(xform.basis,
			GWTiles3DContent.rebase_root_nodes(state, scene, xform.basis, GWTiles3DContent.GOOGLE_YUP_ECEF_CORRECTION,
					to_godot, xform.origin))
	wrapper.add_child(scene)
	var truth := GWGeodeticConvert.ecef_xyz_to_godot_position(e[0], e[1], e[2], LAT, LON, ALT)
	var off := tile.global_position.distance_to(truth)
	_check(off < 0.01, "a tile wrapped the way Google's arrive lands within 1 cm (%.4f m)" % off)
	_check(tile.position.length() < 0.001, "and its node keeps no offset of its own")
	wrapper.free()


func _mesh_position(wrapper: Node3D) -> Vector3:
	var meshes := wrapper.find_children("*", "MeshInstance3D", true, false)
	return (meshes[0] as Node3D).global_position if not meshes.is_empty() else Vector3.INF
