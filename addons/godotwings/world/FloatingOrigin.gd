## Floating origin (origin rebasing) for large worlds.
##
## Single-precision rendering loses accuracy far from (0,0,0) — jitter sets in
## past a few km. This node keeps the rendered scene near the origin: when the
## tracked aircraft drifts past `threshold`, it shifts the aircraft's render
## offset and translates the static world (and camera) back by the same amount.
##
## The aircraft's TRUE state is untouched — GWVehicleBody keeps integrating in
## absolute NED and reports absolute NED to ArduPilot. Only the *rendering* is
## rebased, so it never affects the SITL link. Drop it in with no config: it
## auto-anchors the first GWVehicleBody and shifts the sibling Node3Ds with it.
class_name GWFloatingOrigin
extends Node

## The GWVehicleBody kept near origin. Empty = auto-find the first one in the scene.
@export var aircraft_path: NodePath
## Nodes translated by -delta on each rebase (e.g. world root, camera). Empty =
## auto-collect this node's sibling Node3Ds, excluding any aircraft (GWVehicleBody).
@export var shift_node_paths: Array[NodePath] = []
## Rebase when the aircraft's rendered distance from origin exceeds this (m).
@export var threshold: float = 2000.0

var _aircraft: GWVehicleBody
var _shift_nodes: Array[Node3D] = []


func _ready() -> void:
	_aircraft = _resolve_aircraft()
	if shift_node_paths.is_empty():
		_auto_collect_shift_nodes()
	else:
		for p in shift_node_paths:
			var n := get_node_or_null(p) as Node3D
			if n != null:
				_shift_nodes.append(n)
	if _aircraft == null:
		push_warning("GWFloatingOrigin: no GWVehicleBody found to anchor the origin.")


func _resolve_aircraft() -> GWVehicleBody:
	if not aircraft_path.is_empty():
		return get_node_or_null(aircraft_path) as GWVehicleBody
	return _find_flight_body(_scene_root())


## Topmost node of this scene (child of the SceneTree root), so auto-find works
## whether or not we're the active `current_scene`.
func _scene_root() -> Node:
	var n: Node = self
	var top := get_tree().root if get_tree() else null
	while n.get_parent() != null and n.get_parent() != top:
		n = n.get_parent()
	return n


func _find_flight_body(n: Node) -> GWVehicleBody:
	if n is GWVehicleBody:
		return n
	for c in n.get_children():
		var f := _find_flight_body(c)
		if f != null:
			return f
	return null


## Collect sibling Node3Ds (world, terrain, camera…) to shift on rebase, skipping
## any aircraft — those rebase themselves via their own render_origin.
func _auto_collect_shift_nodes() -> void:
	var parent := get_parent()
	if parent == null:
		return
	for c in parent.get_children():
		if c is Node3D and not (c is GWVehicleBody):
			_shift_nodes.append(c)


func _physics_process(_delta: float) -> void:
	if _aircraft == null:
		return
	var pos := _aircraft.global_position
	if pos.length() < threshold:
		return
	# Rebase by the aircraft's current rendered offset so it returns near origin.
	rebase(pos)


## Shift the rendered world by `delta`: the aircraft's render offset grows by
## delta (and re-renders immediately near origin), and the static world/camera
## nodes translate by -delta. Net relative geometry is preserved; the aircraft's
## true NED state is untouched.
func rebase(delta: Vector3) -> void:
	_aircraft.render_origin += delta
	_aircraft._sync_node() # re-render now at the new origin (no one-frame lag)
	for n in _shift_nodes:
		n.global_position -= delta
