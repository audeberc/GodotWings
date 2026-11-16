## Shared tile-placement logic for GWTiles3DLoader (offline/backup path) and
## GWTiles3DStreamer (the real, live path) -- pulled out because both bugs
## fixed here were real, found live against actual Cesium ion data, and easy
## to reintroduce independently in either caller:
##
## 1. GLTFDocument.generate_scene() already bakes the content's OWN internal
##    positioning into the returned scene's transform. Google Photorealistic
##    3D Tiles bakes a real ECEF offset into the glTF's own root node matrix
##    (the tileset.json transform chain stays identity throughout); other
##    providers (e.g. Cesium's b3dm samples) put it in the tileset transform
##    chain instead and leave content in trivial local space. Overwriting
##    scene.transform instead of wrapping it silently discards whichever
##    half of the position lives in the content -- verified live: every
##    downloaded Google tile landed at the identical wrong position
##    (elevation off by Earth's radius) until this was a wrapper, not an
##    overwrite.
## 2. Google's baked offset uses a DIFFERENT ECEF axis convention than the
##    standard one this project's own ECEF math (GWGeodeticConvert) uses --
##    verified live by brute-force axis-permutation search against a real
##    downloaded tile's known location.
class_name GWTiles3DContent
extends RefCounted

## Google's (X,Y,Z) = standard ECEF's (X, Z, -Y), i.e. Y points at the north
## pole instead of Z. Godot loads a glTF node matrix raw -- it has no idea
## it's "ECEF" at all -- so this fixed rotation must be applied to the
## WRAPPER's basis (not its position, which always comes from the tileset
## transform chain in the standard convention) whenever the content came
## from Google's proxy.
## Basis(a,b,c) sets .x=a, .y=b, .z=c DIRECTLY (columns, exactly matching
## property assignment and the official docs) -- this constant is built from
## the matrix's COLUMNS, i.e. the transpose of how the row-form correction
## reads in prose ("(X,Y,Z) -> (X,-Z,Y)").
const GOOGLE_YUP_ECEF_CORRECTION := Basis(Vector3(1, 0, 0), Vector3(0, 0, 1), Vector3(0, -1, 0))


## Parses glb_bytes via GLTFDocument, WRAPS it (never overwrites the parsed
## scene's own transform -- see the class doc comment above), applies
## content_axis_correction if set, adds the wrapper under `parent` named
## `tile_name`, and returns it. Returns null if parsing/generation failed --
## the caller should push_warning with whatever path/URL context it has,
## since this helper doesn't know it.
##
## `point_to_godot` (optional) maps a point in the tile's own ECEF frame
## (standard axes, three float64 coordinates) to its Godot position, in
## float64 until the small result. Given it, each glTF root node's
## translation is taken from the glTF JSON and placed through it: see
## rebase_root_nodes() for why the float32 transforms can't do this.
static func place_tile(parent: Node3D, glb_bytes: PackedByteArray, wrapper_transform: Transform3D,
		content_axis_correction: String, tile_name: String,
		point_to_godot: Callable = Callable()) -> Node3D:
	var doc := GLTFDocument.new()
	var state := GLTFState.new()
	if doc.append_from_buffer(glb_bytes, "", state) != OK:
		return null
	var scene := doc.generate_scene(state)
	if scene == null:
		return null

	var wrapper := Node3D.new()
	wrapper.name = tile_name
	var xform := wrapper_transform
	var axes := Basis.IDENTITY
	if content_axis_correction == "google_yup_ecef":
		axes = GOOGLE_YUP_ECEF_CORRECTION
	xform.basis = xform.basis * axes
	if point_to_godot.is_valid():
		xform.origin = rebase_root_nodes(state, scene, xform.basis, axes, point_to_godot, xform.origin)
	wrapper.transform = xform
	parent.add_child(wrapper)
	if Engine.is_editor_hint():
		wrapper.owner = parent.get_tree().edited_scene_root
	wrapper.add_child(scene)
	if Engine.is_editor_hint():
		scene.owner = parent.get_tree().edited_scene_root
	return wrapper


## Google bakes each tile's offset from the Earth's centre (about 6,400 km)
## into the glTF root node, and its tileset chain is identity, so the wrapper
## sits at the Godot position of the Earth's centre. Both are float32 in
## Godot's transforms, a step of half a metre at that size; they cancel to a
## local position but leave each tile up to ~0.75 m off, at random, which
## opens hairline gaps between neighbours that show the sky. This takes each
## such root node's translation from the glTF JSON (float64), maps it through
## `point_to_godot` in float64, and puts the result on the wrapper; the nodes
## keep only their (small) offset from the first one. Returns the wrapper
## origin to use, or `fallback` when no root node is that far out.
##
## `scene` is what generate_scene() returned. A root node may be that node
## itself or one of its children (Godot wraps some single-root files, Google's
## among them), and the state's own node references go stale when
## generate_scene() swaps its importer nodes, so each root node is found by
## its translation, which the generated node still carries in float32.
static func rebase_root_nodes(state: GLTFState, scene: Node, wrapper_basis: Basis, axes: Basis,
		point_to_godot: Callable, fallback: Vector3) -> Vector3:
	var nodes: Array = state.json.get("nodes", [])
	var placed: Array = []   # [Node3D, its Godot position]
	for index in state.root_nodes:
		if index < 0 or index >= nodes.size():
			continue
		var t := _json_translation(nodes[index])
		if t.is_empty() or Vector3(t[0], t[1], t[2]).length() < FAR_ROOT_M:
			continue
		var node := _generated_node_at(scene, Vector3(t[0], t[1], t[2]))
		if node == null:
			continue
		# Content axes to the tile's ECEF axes. `axes` is a signed
		# permutation, so this is exact in float64.
		var x: float = axes.x.x * t[0] + axes.y.x * t[1] + axes.z.x * t[2]
		var y: float = axes.x.y * t[0] + axes.y.y * t[1] + axes.z.y * t[2]
		var z: float = axes.x.z * t[0] + axes.y.z * t[1] + axes.z.z * t[2]
		placed.append([node, point_to_godot.call(x, y, z)])
	if placed.is_empty():
		return fallback
	var origin: Vector3 = placed[0][1]
	var to_local := wrapper_basis.inverse()
	for p in placed:
		(p[0] as Node3D).position = to_local * (p[1] - origin)
	return origin


## Root-node translations at least this far out are placed in float64; below
## it float32 is already finer than a millimetre.
const FAR_ROOT_M := 10000.0


## The generated node (the scene root or one of its children) whose position
## is `translation`, as float32 holds it.
static func _generated_node_at(scene: Node, translation: Vector3) -> Node3D:
	var candidates: Array = [scene]
	candidates.append_array(scene.get_children())
	for c in candidates:
		if c is Node3D and (c as Node3D).position.distance_to(translation) < 1.0:
			return c
	return null


## A glTF node's translation as three float64s, from "matrix" (column-major,
## translation in 12..14) or "translation"; empty when it has neither.
static func _json_translation(node: Dictionary) -> Array:
	if node.has("matrix"):
		var m: Array = node["matrix"]
		return [float(m[12]), float(m[13]), float(m[14])] if m.size() == 16 else []
	if node.has("translation"):
		var t: Array = node["translation"]
		return [float(t[0]), float(t[1]), float(t[2])] if t.size() == 3 else []
	return []


## Manifest position is [east, up, -south] meters (already Godot EUS). Basis
## is stored ROW-major (json's nested-list-of-rows, from numpy's .tolist())
## whose COLUMNS are where the tile's local X/Y/Z axes land in Godot space --
## so Basis.x/.y/.z (Godot's column vectors) are built by reading down each
## column, not each row. Columns may carry non-uniform scale from the tile's
## own transform (verified against a real sample: orthogonal columns,
## uniform 100x scale) -- Godot's Basis holds that fine.
static func transform_from_manifest(tile: Dictionary) -> Transform3D:
	var pos: Array = tile.get("position", [0.0, 0.0, 0.0])
	var m: Array = tile.get("basis", [[1, 0, 0], [0, 1, 0], [0, 0, 1]])
	var basis := Basis()
	basis.x = Vector3(m[0][0], m[1][0], m[2][0])
	basis.y = Vector3(m[0][1], m[1][1], m[2][1])
	basis.z = Vector3(m[0][2], m[1][2], m[2][2])
	return Transform3D(basis, Vector3(pos[0], pos[1], pos[2]))
