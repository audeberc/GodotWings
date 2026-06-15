extends SceneTree

# Multiple aircraft, added manually (no fleet helper): each GWAircraft with its own
# sitl_instance must self-assemble a bridge on a distinct port (9002 + 10*i) and,
# with cameras on, distinct video/metadata/raw ports — so N drones in SITL each
# talk to their own Godot aircraft without collisions.

var _ok := true

func _check(cond: bool, msg: String) -> void:
	if cond: print("  PASS  ", msg)
	else: push_error("FAIL: " + msg); _ok = false


func _initialize() -> void:
	var n := 4
	var planes: Array = []
	for i in n:
		var ac := GWAircraft.new()
		ac.name = "Aircraft%d" % i
		ac.sitl_instance = i
		ac.spawn_east = i * 10.0   # manual spread so they don't stack at the origin
		ac.enable_camera = true
		get_root().add_child(ac)
		planes.append(ac)
	await physics_frame
	await physics_frame

	var bridge_ports: Array = []
	var video_ports: Array = []
	for i in n:
		var ac: GWAircraft = planes[i]
		var bridge := ac.get_node_or_null("GWSITLBridge") as GWSITLBridge
		var cam := ac.get_node_or_null("GWCamera")
		_check(bridge != null and bridge.listen_port == 9002 + 10 * i,
			"aircraft %d bridge on %d" % [i, bridge.listen_port if bridge else -1])
		if bridge: bridge_ports.append(bridge.listen_port)
		if cam: video_ports.append(cam.video_port)

	_check(_all_unique(bridge_ports), "bridge ports distinct %s" % str(bridge_ports))
	_check(video_ports.size() == n and _all_unique(video_ports), "camera video ports distinct %s" % str(video_ports))

	for ac in planes:
		ac.free()
	await physics_frame
	print("test_multi_aircraft: ", "PASS" if _ok else "FAIL")
	quit(0 if _ok else 1)


func _all_unique(a: Array) -> bool:
	for i in a.size():
		for j in range(i + 1, a.size()):
			if a[i] == a[j]:
				return false
	return true
