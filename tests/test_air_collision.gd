extends SceneTree

# Air-to-air collision: each aircraft presents a passive hull collider on a shared
# aircraft_layer, and its per-frame obstacle shapecast (which now includes that
# layer, excluding its own hull) detects another aircraft overlapping it. Both
# aircraft detect the overlap independently, so both crash.

var _ok := true

func _check(cond: bool, msg: String) -> void:
	if cond: print("  PASS  ", msg)
	else: push_error("FAIL: " + msg); _ok = false


func _make(layer: int) -> GWFlightBody:
	var fb := GWFlightBody.new()
	fb.config = load("res://addons/godotwings/aircraft/Skywalker.tres")
	fb.aircraft_layer = layer
	return fb


func _initialize() -> void:
	var a := _make(2)
	var b := _make(2)
	get_root().add_child(a)
	get_root().add_child(b)
	await physics_frame
	await physics_frame

	_check(a.get_node_or_null("Hull") != null, "aircraft A built a hull collider on its layer")
	_check(b.get_node_or_null("Hull") != null, "aircraft B built a hull collider on its layer")

	# Both spawn at the origin -> hulls overlap. Each must detect the OTHER (and not
	# trip on its own hull).
	await physics_frame
	_check(a._check_obstacle(), "A detects B when overlapping")
	_check(b._check_obstacle(), "B detects A when overlapping")

	# Pull B far away -> no overlap; A alone must read clear (proves self-exclusion).
	b.global_position = Vector3(1000, 0, 0)
	await physics_frame
	await physics_frame
	_check(not a._check_obstacle(), "A clear once B is far (and ignores its own hull)")
	_check(not b._check_obstacle(), "B clear once far")

	# A different layer = no air-to-air (B back on top of A but on layer 4).
	b.global_position = a.global_position
	b._hull_body.collision_layer = 4
	await physics_frame
	await physics_frame
	_check(not a._check_obstacle(), "A ignores B on a different layer")

	a.free(); b.free()
	await physics_frame
	print("test_air_collision: ", "PASS" if _ok else "FAIL")
	quit(0 if _ok else 1)
