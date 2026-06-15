## Headless test suite for the GodotWings addon.
##   /Applications/Godot.app/Contents/MacOS/Godot --headless --path . --script res://tests/test_fdm.gd
## Exits 0 on success, 1 on any failure.
extends SceneTree

var _fail := 0


func check(name: String, ok: bool, detail: String = "") -> void:
	print(("  PASS  " if ok else "  FAIL  ") + name + ("" if ok else "  <-- " + detail))
	if not ok:
		_fail += 1


func _initialize() -> void:
	print("== GodotWings FDM tests ==")
	_test_frames()
	_test_proper_rotations()
	_test_stall()
	_test_flight_regression()
	_test_second_aircraft()
	_test_rate_independence()
	_test_floating_origin()
	print("== %s ==" % ("ALL PASSED" if _fail == 0 else "%d FAILED" % _fail))
	quit(1 if _fail > 0 else 0)


func _test_frames() -> void:
	print("[frames]")
	var max_err := 0.0
	for roll in [-1.0, -0.3, 0.0, 0.4, 1.0]:
		for pitch in [-1.0, 0.0, 0.5, 1.0]:
			for yaw in [-3.0, -1.0, 0.0, 2.0, 3.0]:
				var b := GWCoordConvert.attitude_to_dcm(roll, pitch, yaw)
				var a := GWCoordConvert.dcm_to_ned_attitude(b)
				max_err = maxf(max_err, absf(a[0] - roll))
				max_err = maxf(max_err, absf(a[1] - pitch))
				max_err = maxf(max_err, absf(wrapf(a[2] - yaw, -PI, PI)))
	check("attitude <-> dcm round-trip (err=%s)" % str(max_err), max_err < 1e-5, str(max_err))

	var v := Vector3(12.3, -4.5, 67.8) # arbitrary NED
	var rt: Vector3 = GWCoordConvert.world_to_ned(GWCoordConvert.ned_to_world(v))
	check("ned <-> world round-trip", rt.distance_to(v) < 1e-5, str(rt))


func _test_proper_rotations() -> void:
	print("[proper rotations]")
	for ang in [Vector3(0, 0, 0), Vector3(0.5, -0.3, 1.2), Vector3(-0.8, 0.2, -2.0)]:
		var b := GWCoordConvert.attitude_to_dcm(ang.x, ang.y, ang.z)
		check("attitude_to_dcm det=+1 @%s" % ang, absf(b.determinant() - 1.0) < 1e-4)
		var r := GWCoordConvert.dcm_to_render_basis(b)
		check("render basis det=+1 @%s" % ang, absf(r.determinant() - 1.0) < 1e-4)


func _test_stall() -> void:
	print("[stall]")
	var cfg: GWAircraftConfig = load("res://addons/godotwings/aircraft/Skywalker.tres")
	var wing := GWWingModel.new(cfg)
	check("CL(0) == c_lift_0", absf(wing.lift_coeff(0.0) - cfg.c_lift_0) < 1e-6)
	var cl_20 := wing.lift_coeff(deg_to_rad(20.0))
	var cl_30 := wing.lift_coeff(deg_to_rad(30.0))
	check("CL drops past stall (CL20=%.2f > CL30=%.2f)" % [cl_20, cl_30], cl_20 > cl_30)
	check("CL is bounded (no blow-up)", cl_20 < 5.0 and is_finite(cl_20))


func _make_body(config_path := "res://addons/godotwings/aircraft/Skywalker.tres") -> GWFlightBody:
	var fb := GWFlightBody.new()
	fb.config = load(config_path)
	fb._ready() # init modules + reset directly (no tree / bridge needed)
	return fb


## Drive the FDM directly (no UDP) and return final altitude (m AGL).
func _fly(fb: GWFlightBody, seconds: float, dt: float,
		throttle: float, elevator: float) -> float:
	fb._cmd = {"aileron": 0.0, "elevator": elevator, "throttle": throttle, "rudder": 0.0}
	var steps := int(seconds / dt)
	for _i in steps:
		fb._step(dt)
	return -fb._pos_ned.z


func _test_flight_regression() -> void:
	print("[flight regression]")
	var fb := _make_body()
	var alt := _fly(fb, 8.0, 0.02, 0.6, 0.0)
	var att := GWCoordConvert.dcm_to_ned_attitude(fb._dcm)
	check("takes off / climbs (alt=%.1f m)" % alt, alt > 2.0, "%.2f" % alt)
	check("attitude bounded (roll=%.2f pitch=%.2f)" % [att[0], att[1]],
			absf(att[0]) < 1.5 and absf(att[1]) < 1.5)
	check("state finite", is_finite(fb._pos_ned.x) and is_finite(fb._vel_ned.z) and is_finite(att[1]))
	fb.free()


func _test_second_aircraft() -> void:
	print("[2nd aircraft: Trainer]")
	var fb := _make_body("res://addons/godotwings/aircraft/Trainer.tres")
	var alt := _fly(fb, 8.0, 0.02, 0.7, 0.0)
	var att := GWCoordConvert.dcm_to_ned_attitude(fb._dcm)
	check("Trainer config flies (alt=%.1f m)" % alt,
			alt > 2.0 and is_finite(att[1]) and absf(att[0]) < 1.5, "%.2f" % alt)
	fb.free()


func _test_floating_origin() -> void:
	print("[floating origin]")
	# (Shift-node translation of camera/world uses global_position, which only
	# works for in-tree nodes — verified in a real scene, not this headless unit
	# test. Here we verify the core guarantee: rebasing is cosmetic and the NED
	# state / reported position are untouched.)
	var fb := _make_body()
	_fly(fb, 25.0, 0.02, 1.0, 0.05) # travel a long way from origin
	var render_far := fb.transform.origin
	var ned := fb._pos_ned
	var reported: Vector3 = fb._build_state()["position"]

	var fo := GWFloatingOrigin.new()
	fo._aircraft = fb
	fo.rebase(render_far) # rebase by the current render offset -> back to origin

	check("render was far before rebase (%.0f m)" % render_far.length(), render_far.length() > 100.0, "%.1f" % render_far.length())
	check("render_origin grew by delta", fb.render_origin.is_equal_approx(render_far))
	check("aircraft re-renders near origin after rebase", fb.transform.origin.length() < 1.0)
	check("NED state untouched by rebase", fb._pos_ned == ned)
	check("reported position untouched by rebase", fb._build_state()["position"] == reported)
	fb.free()
	fo.free()


func _test_rate_independence() -> void:
	print("[rate independence]")
	var fb50 := _make_body()
	var fb200 := _make_body()
	var alt_50 := _fly(fb50, 6.0, 0.02, 0.6, 0.0)   # 50 Hz host
	var alt_200 := _fly(fb200, 6.0, 0.005, 0.6, 0.0) # 200 Hz host
	var rel := absf(alt_50 - alt_200) / maxf(alt_50, 1.0)
	fb50.free()
	fb200.free()
	check("altitude rate-independent (50Hz=%.1f 200Hz=%.1f, %.1f%%)" % [alt_50, alt_200, rel * 100.0],
			rel < 0.15, "%.1f%%" % (rel * 100.0))
