extends SceneTree

# Hybrid crash: normal FDM flight, but a confirmed obstacle/hard-ground impact
# hands the wreck to a RigidBody3D (Jolt/GodotPhysics) for a real tumble, reads
# its pose back into the NED state, and recovers the FDM once it settles.

var _ok := true

func _check(cond: bool, msg: String) -> void:
	if cond: print("  PASS  ", msg)
	else: push_error("FAIL: " + msg); _ok = false


func _initialize() -> void:
	# Obstacle cube on layer 2, centred 20 m up at the render origin.
	var cube := StaticBody3D.new()
	cube.collision_layer = 2
	cube.collision_mask = 0
	var ccs := CollisionShape3D.new()
	var cbox := BoxShape3D.new()
	cbox.size = Vector3(10, 10, 10)
	ccs.shape = cbox
	cube.add_child(ccs)
	cube.position = Vector3(0, 20, 0)
	get_root().add_child(cube)
	await physics_frame
	await physics_frame

	var b := GWFlightBody.new()
	b.config = load("res://addons/godotwings/aircraft/Skywalker.tres")
	b.obstacle_mask = 2
	b.crash_mode = GWFlightBody.CrashMode.RAGDOLL
	get_root().add_child(b)
	await physics_frame

	# Place it airborne inside the cube, flying north at 20 m/s with some roll rate.
	b._pos_ned = Vector3(0, 0, -20)   # render (0, 20, 0) -> overlaps the cube
	b._dcm = GWCoordConvert.attitude_to_dcm(0.0, 0.0, 0.0)
	b._vel_ned = Vector3(20, 0, 0)
	b._omega = Vector3(0.5, 0, 0)
	b._sync_node()

	_check(b._check_obstacle(), "hull overlaps the obstacle cube")

	b._enter_crash()
	_check(b._ragdolling, "entered ragdoll on obstacle hit")
	var rb := b._ragdoll
	_check(rb != null and rb is RigidBody3D, "wreck RigidBody3D created")
	if rb:
		var want_v := GWCoordConvert.ned_to_world(Vector3(20, 0, 0))  # = (0,0,-20)
		_check(rb.linear_velocity.distance_to(want_v) < 0.05, "wreck inherits flight velocity")

	# Let physics advance a step, then read the wreck back into NED state.
	await physics_frame
	b._step_ragdoll(1.0 / 50.0)
	_check(is_finite(b._pos_ned.x) and is_finite(b._dcm.x.x), "ragdoll pose reads back finite")

	# Hold the wreck still and run past the max-time fallback; the FDM must recover
	# (wreck removed, level on-ground) without needing a restart.
	if rb:
		rb.gravity_scale = 0.0
		rb.linear_velocity = Vector3.ZERO
		rb.angular_velocity = Vector3.ZERO
	for i in 24:
		await physics_frame
		b._step_ragdoll(0.5)  # 24 * 0.5 s > RAGDOLL_MAX_TIME
	_check(not b._ragdolling and b._ragdoll == null, "recovered: wreck proxy removed")
	_check(b._on_ground, "recovered to an on-ground state")
	_check(absf(-b._pos_ned.z - 0.5) < 0.01, "recovered level at gear height (alt=%.2f)" % -b._pos_ned.z)
	# respawn_at_home (default): back at the spawn point, not where the wreck settled.
	_check(absf(b._pos_ned.x - b.spawn_north) < 0.01 and absf(b._pos_ned.y - b.spawn_east) < 0.01,
			"respawned at home (N=%.2f E=%.2f)" % [b._pos_ned.x, b._pos_ned.y])

	b.queue_free(); cube.queue_free()
	await physics_frame
	print("test_ragdoll: ", "PASS" if _ok else "FAIL")
	quit(0 if _ok else 1)
