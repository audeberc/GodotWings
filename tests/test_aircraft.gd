extends SceneTree

# Tests the plug-and-play layer: GWAircraftSpec coefficient derivation and the
# GWAircraft self-assembling facade (bridge child + port offset + config + camera).

var _ok := true

func _check(cond: bool, msg: String) -> void:
	if cond:
		print("  PASS  ", msg)
	else:
		push_error("FAIL: " + msg); _ok = false

func _finite(c: GWAircraftConfig) -> bool:
	for v in [c.c_lift_0, c.c_lift_a, c.c_drag_p, c.c_l_p, c.c_l_deltaa,
			c.c_m_a, c.c_m_q, c.c_m_deltae, c.c_n_b, c.c_n_r, c.c_n_deltar,
			c.hover_throttle, c.s, c.b, c.c, c.mass]:
		if not is_finite(v):
			return false
	return true


func _initialize() -> void:
	print("== GWAircraftSpec ==")

	# Default spec at Skywalker geometry must reproduce the validated MOMENT coeffs
	# (inertia scale ~1, feel knobs at 0.5 -> 1.0).
	var base := GWAircraftSpec.new().build()
	_check(_finite(base), "default spec builds finite config")
	_check(absf(base.c_m_a - (-0.7)) < 0.05, "c_m_a reproduces baseline (%.3f)" % base.c_m_a)
	_check(absf(base.c_m_q - (-20.0)) < 1.0, "c_m_q reproduces baseline (%.2f)" % base.c_m_q)
	_check(absf(base.c_l_p - (-1.0)) < 0.05, "c_l_p reproduces baseline (%.3f)" % base.c_l_p)
	_check(absf(base.c_l_deltaa - 0.25) < 0.02, "c_l_deltaa reproduces baseline (%.3f)" % base.c_l_deltaa)
	_check(absf(base.c_lift_a - 6.9) < 0.1, "c_lift_a reproduces baseline (%.2f)" % base.c_lift_a)

	# Signs that keep it flyable, across presets.
	for spec in [GWAircraftSpec.trainer(), GWAircraftSpec.foamie(), GWAircraftSpec.fpv_wing()]:
		var c: GWAircraftConfig = spec.build()
		_check(_finite(c), "%dkg preset builds finite config" % int(spec.mass))
		_check(c.c_m_a < 0.0 and c.c_m_q < 0.0, "pitch stable+damped (a=%.2f q=%.2f)" % [c.c_m_a, c.c_m_q])
		_check(c.c_l_p < 0.0 and c.c_n_r < 0.0, "roll+yaw damped")
		_check(c.c_lift_a >= 3.0 and c.c_lift_a <= 7.5, "lift slope bounded (%.2f)" % c.c_lift_a)
		_check(c.hover_throttle > 0.0, "thrust positive")

	# Smaller/lighter plane => more angular-accel authority than the baseline.
	var foam := GWAircraftSpec.foamie().build()
	_check(absf(foam.c_l_deltaa) > absf(base.c_l_deltaa),
			"lighter plane has more roll authority (%.2f > %.2f)" % [foam.c_l_deltaa, base.c_l_deltaa])

	print("== GWAircraft facade ==")
	# Instance 2, camera on: bridge + camera children with offset ports.
	var ac := GWAircraft.new()
	ac.flight_model = GWAircraft.FlightModel.CUSTOM
	ac.spec = GWAircraftSpec.trainer()
	ac.sitl_instance = 2
	ac.enable_camera = true
	get_root().add_child(ac)
	await process_frame
	await process_frame

	_check(ac.config != null and absf(ac.config.mass - 2.0) < 0.01, "spec built into config")
	_check(ac._wing != null and ac._moment != null, "FDM subsystems initialised (super._ready ran)")

	var bridge := ac.get_node_or_null("GWSITLBridge") as GWSITLBridge
	_check(bridge != null, "bridge child auto-created")
	if bridge:
		_check(bridge.listen_port == 9002 + 20, "bridge port offset by instance (%d)" % bridge.listen_port)

	var cam := ac.get_node_or_null("GWCamera") as GWCamera
	_check(cam != null, "camera child auto-created from enable_camera")
	if cam:
		_check(cam.video_port == 5600 + 4, "camera video port offset by instance (%d)" % cam.video_port)

	ac.queue_free()
	await process_frame

	print("test_aircraft: ", "PASS" if _ok else "FAIL")
	quit(0 if _ok else 1)
