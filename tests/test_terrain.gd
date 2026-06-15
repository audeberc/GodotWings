extends SceneTree

# Terrain-following ground: the FDM raycasts world colliders instead of assuming
# a flat sea-level plane. Verifies the aircraft rests at gear height on an
# elevated collider, reports AGL relative to it, and that the flat-plane path is
# unchanged when terrain_following is off.

var _ok := true

func _check(cond: bool, msg: String) -> void:
	if cond: print("  PASS  ", msg)
	else: push_error("FAIL: " + msg); _ok = false


func _make_body(terrain: bool) -> GWFlightBody:
	var b := GWFlightBody.new()
	b.config = load("res://addons/godotwings/aircraft/Skywalker.tres")
	b.spawn_altitude = 0.5
	b.terrain_following = terrain
	return b


func _initialize() -> void:
	# A flat collider whose top face sits at render y = 11 (size.y=2, centre y=10).
	var sb := StaticBody3D.new()
	var col := CollisionShape3D.new()
	var box := BoxShape3D.new()
	box.size = Vector3(400, 2, 400)
	col.shape = box
	sb.add_child(col)
	sb.position = Vector3(0, 10, 0)
	get_root().add_child(sb)
	# Let the physics server register the new body before we query the space.
	await physics_frame
	await physics_frame

	# terrain_following ON: should rest on the collider top (11) + gear (0.5).
	var bt := _make_body(true)
	get_root().add_child(bt)         # _ready -> _reset_state samples the terrain
	await physics_frame
	var alt_t := -bt._pos_ned.z
	_check(absf(alt_t - 11.5) < 0.2, "rests on elevated terrain (alt=%.2f, want ~11.5)" % alt_t)
	_check(absf(bt._agl() - 0.5) < 0.05, "AGL = gear height on the ground (%.3f)" % bt._agl())

	# Fly it up 20 m and confirm AGL is terrain-relative, not MSL.
	bt._pos_ned.z = -31.0   # 31 m MSL over terrain at 11 m => 20 m AGL
	bt._update_ground_sample()
	_check(absf(bt._agl() - 20.0) < 0.2, "AGL is terrain-relative aloft (%.2f, want ~20)" % bt._agl())

	# terrain_following OFF: legacy flat plane at ground_level 0 -> rests at gear.
	var bf := _make_body(false)
	get_root().add_child(bf)
	await physics_frame
	_check(absf(-bf._pos_ned.z - 0.5) < 0.01, "flat-ground path unchanged (alt=%.2f)" % -bf._pos_ned.z)

	bt.queue_free(); bf.queue_free(); sb.queue_free()
	await physics_frame
	print("test_terrain: ", "PASS" if _ok else "FAIL")
	quit(0 if _ok else 1)
