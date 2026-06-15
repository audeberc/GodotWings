extends SceneTree

# GWWind: mean wind (from-bearing -> NED air velocity), altitude shear, zero-mean
# turbulence, windvane report, and FlightBody auto-finding the wind in the scene.

var _ok := true

func _check(cond: bool, msg: String) -> void:
	if cond: print("  PASS  ", msg)
	else: push_error("FAIL: " + msg); _ok = false


func _initialize() -> void:
	var w := GWWind.new()

	# Mean wind: FROM North (0deg) -> air moves South -> NED (-speed, 0, 0).
	w.wind_speed = 10.0
	w.wind_from_deg = 0.0
	_check(w.mean_wind_ned().distance_to(Vector3(-10, 0, 0)) < 1e-4, "wind FROM N -> NED (-10,0,0) (%s)" % w.mean_wind_ned())
	# FROM East (90deg) -> air moves West -> NED (0, -speed, 0).
	w.wind_from_deg = 90.0
	_check(w.mean_wind_ned().distance_to(Vector3(0, -10, 0)) < 1e-4, "wind FROM E -> NED (0,-10,0) (%s)" % w.mean_wind_ned())

	# Windvane reports FROM-bearing (rad) + speed.
	var vv := w.windvane()
	_check(absf(vv["direction"] - deg_to_rad(90.0)) < 1e-4 and absf(vv["speed"] - 10.0) < 1e-4, "windvane = {dir:90deg, spd:10}")

	# Turbulence off -> sample == mean, regardless of time.
	w.wind_from_deg = 0.0
	w.turbulence = 0.0
	w._build_modes()
	_check(w.sample(Vector3.ZERO, 3.3).distance_to(Vector3(-10, 0, 0)) < 1e-4, "no turbulence -> sample == mean")

	# Turbulence on: time-varying, but averages back to the mean (zero-mean gust).
	w.turbulence = 3.0
	w._build_modes()
	_check(w.sample(Vector3.ZERO, 0.0).distance_to(w.sample(Vector3.ZERO, 1.7)) > 0.01, "turbulence varies with time")
	var acc := Vector3.ZERO
	var n := 4000
	for i in n:
		acc += w.sample(Vector3.ZERO, i * 0.05)
	_check((acc / n).distance_to(Vector3(-10, 0, 0)) < 0.5, "gust averages to the mean (%s)" % (acc / n))

	# Altitude shear: higher = faster (sqrt law, ref 10 m).
	w.turbulence = 0.0
	w.shear_exponent = 0.5
	w.reference_height = 10.0
	var low := w.sample(Vector3(0, 0, -2.5), 0.0).length()   # h=2.5 -> 0.5x
	var high := w.sample(Vector3(0, 0, -40.0), 0.0).length() # h=40  -> 2x
	_check(absf(low - 5.0) < 0.1 and absf(high - 20.0) < 0.1, "shear: 5 m/s @2.5m, 20 m/s @40m (%.1f, %.1f)" % [low, high])

	# Integration: a FlightBody auto-finds the GWWind under a shared scene root.
	var world := Node.new()
	get_root().add_child(world)
	var wind2 := GWWind.new()
	wind2.wind_speed = 8.0
	wind2.wind_from_deg = 0.0
	world.add_child(wind2)
	var fb := GWFlightBody.new()
	fb.config = load("res://addons/godotwings/aircraft/Skywalker.tres")
	world.add_child(fb)  # _ready (deferred) resolves the wind
	await physics_frame
	_check(fb._wind == wind2, "FlightBody auto-found the GWWind")
	_check(fb._wind_ned().distance_to(Vector3(-8, 0, 0)) < 1e-4, "FlightBody samples wind (%s)" % fb._wind_ned())

	world.free()
	await physics_frame
	print("test_wind: ", "PASS" if _ok else "FAIL")
	quit(0 if _ok else 1)
