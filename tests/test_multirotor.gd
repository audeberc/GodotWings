extends SceneTree

# Quadcopter FDM (GWMultirotorBody): motor mixing produces the right body moments,
# full thrust climbs and stays upright, below-hover stays grounded, and the fixed
# sub-stepping keeps it rate-independent. (Absolute yaw-reaction sign vs ArduCopter
# is validated in SITL; here we check relative correctness + dynamics.)

var _ok := true

func _check(cond: bool, msg: String) -> void:
	if cond: print("  PASS  ", msg)
	else: push_error("FAIL: " + msg); _ok = false


func _make() -> GWMultirotorBody:
	var q := GWMultirotorBody.new()
	q.config = load("res://addons/godotwings/aircraft/QuadX.tres")
	q._ready()  # builds frame + reset (no tree/bridge needed)
	return q


func _set4(q: GWMultirotorBody, a: float, b: float, c: float, d: float) -> void:
	var pwm := PackedInt32Array()
	pwm.resize(16)
	pwm.fill(1000)
	pwm[0] = 1000 + int(a * 1000); pwm[1] = 1000 + int(b * 1000)
	pwm[2] = 1000 + int(c * 1000); pwm[3] = 1000 + int(d * 1000)
	q._update_controls(pwm)


func _fly_alt(q: GWMultirotorBody, seconds: float, dt: float) -> float:
	var steps := int(seconds / dt)
	for _i in steps:
		q._step(dt)
	return -q._pos_ned.z


func _initialize() -> void:
	var q := _make()

	# --- motor mixing -> body moment (pure, deterministic) ---
	_check(q._frame_moment(PackedFloat32Array([1, 1, 1, 1])).length() < 1e-5,
		"equal thrust -> no moment")
	# motors 1 (front-right) + 3 (front-left) = front
	_check(q._frame_moment(PackedFloat32Array([1, 0, 1, 0])).y > 0.0,
		"front motors -> +pitch (nose up)")
	# motors 1 (front-right) + 4 (back-right) = right; right-heavy rolls left (m.x<0)
	_check(q._frame_moment(PackedFloat32Array([1, 0, 0, 1])).x < 0.0,
		"right motors -> roll left (-m.x)")
	# motors 1,2 = CCW (+yaw reaction); 3,4 = CW (-yaw reaction)
	_check(q._frame_moment(PackedFloat32Array([1, 1, 0, 0])).z > 0.0, "CCW props -> +yaw reaction")
	_check(q._frame_moment(PackedFloat32Array([0, 0, 1, 1])).z < 0.0, "CW props -> -yaw reaction")

	# --- full throttle climbs and stays upright ---
	var climb := _make()
	_set4(climb, 0.8, 0.8, 0.8, 0.8)
	var alt := _fly_alt(climb, 3.0, 0.01)
	var att := GWCoordConvert.dcm_to_ned_attitude(climb._dcm)
	_check(alt > 5.0, "full throttle climbs (alt=%.1f m)" % alt)
	_check(absf(att[0]) < 0.05 and absf(att[1]) < 0.05, "stays level while climbing")
	_check(is_finite(climb._pos_ned.z) and is_finite(att[2]), "state finite")

	# --- below hover stays on the ground ---
	var grounded := _make()
	_set4(grounded, 0.15, 0.15, 0.15, 0.15)  # 0.15 < ~0.2 hover (5:1 TWR) -> ground hold
	var galt := _fly_alt(grounded, 2.0, 0.01)
	_check(absf(galt - grounded.spawn_altitude) < 0.05, "below-hover stays grounded (alt=%.2f)" % galt)

	# --- pitch authority: front motors > back -> pitches nose-up. Sampled early:
	# the snappy default has very low inertia, so a held differential tumbles fast.
	var pitcher := _make()
	_set4(pitcher, 0.6, 0.4, 0.6, 0.4)  # 1,3 front high; 2,4 back low
	_fly_alt(pitcher, 0.15, 0.005)
	var pitch: float = GWCoordConvert.dcm_to_ned_attitude(pitcher._dcm)[1]
	_check(pitch > 0.05 and pitch < 1.5, "front-heavy pitches nose up (pitch=%.2f)" % pitch)

	# --- rate independence: climb similar at 50 Hz vs 200 Hz ---
	var a50 := _make(); _set4(a50, 0.8, 0.8, 0.8, 0.8)
	var a200 := _make(); _set4(a200, 0.8, 0.8, 0.8, 0.8)
	var alt50 := _fly_alt(a50, 2.0, 0.02)
	var alt200 := _fly_alt(a200, 2.0, 0.005)
	var rel := absf(alt50 - alt200) / maxf(alt50, 1.0)
	_check(rel < 0.15, "altitude rate-independent (50Hz=%.1f 200Hz=%.1f, %.1f%%)" % [alt50, alt200, rel * 100.0])

	print("test_multirotor: ", "PASS" if _ok else "FAIL")
	quit(0 if _ok else 1)
