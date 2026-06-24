extends SceneTree

# GWManualInput: direct keyboard/joypad/RC command source.
# Covers the pure channel-build, action-driven sampling (throttle ramp + reset
# edge), and a no-SITL integration tick driving a GWFlightBody's ground roll.

var _ok := true

func _check(cond: bool, msg: String) -> void:
	if cond: print("  PASS  ", msg)
	else: push_error("FAIL: " + msg); _ok = false


func _initialize() -> void:
	_test_build_command()
	await _test_sampling()
	await _test_integration()
	print("test_manual_input: ", "PASS" if _ok else "FAIL")
	quit(0 if _ok else 1)


# --- pure channel construction (AETR, no Input access) -----------------------
func _test_build_command() -> void:
	var mi := GWManualInput.new()

	var centred := mi._build_command(0.0, 0.0, 0.0, 0.0)
	var pwm: PackedInt32Array = centred["pwm"]
	_check(pwm.size() == 16, "command has 16 channels")
	_check(pwm[0] == 1500 and pwm[1] == 1500 and pwm[3] == 1500, "centred surfaces = 1500us (ail/ele/rud)")
	_check(pwm[2] == 1000, "throttle 0 -> ch3 = 1000us")
	_check(pwm[4] == 1500, "aux ch5 rests neutral at 1500us")
	_check(centred["reset"] == false, "reset defaults false")

	var full := mi._build_command(1.0, -1.0, 1.0, 0.5)
	var fpwm: PackedInt32Array = full["pwm"]
	_check(fpwm[0] == 2000, "full roll -> ch1 = 2000us")
	_check(fpwm[1] == 1000, "full nose-down -> ch2 = 1000us")
	_check(fpwm[2] == 2000, "full throttle -> ch3 = 2000us")
	_check(fpwm[3] == 1750, "half yaw -> ch4 = 1750us")
	_check(absf(full["aileron"] - 1.0) < 0.001, "aileron field = 1.0")
	_check(absf(full["throttle"] - 1.0) < 0.001, "throttle field = 1.0")
	_check(absf(full["rudder"] - 0.5) < 0.001, "rudder field = 0.5")

	mi.free()


# --- action-driven sampling: sticky throttle ramp + reset rising edge --------
func _test_sampling() -> void:
	var mi := GWManualInput.new()
	get_root().add_child(mi)       # _ready() registers the default gw_* actions
	await process_frame
	_check(InputMap.has_action("gw_throttle_up"), "default actions registered")

	Input.action_press("gw_throttle_up")
	mi._sample(1.0)                # ramp 0.5/s for 1 s -> 0.5
	_check(absf(mi._throttle - 0.5) < 0.01, "throttle ramps up and holds (%.2f)" % mi._throttle)
	mi._sample(1.0)
	_check(absf(mi._throttle - 1.0) < 0.01, "throttle ramps to full")
	Input.action_release("gw_throttle_up")
	mi._sample(1.0)
	_check(absf(mi._throttle - 1.0) < 0.01, "throttle holds when stick released (sticky)")

	# Reset is a rising edge consumed exactly once.
	Input.action_press("gw_reset")
	mi._sample(0.1)
	var c1 := mi.take_command()
	_check(c1["reset"] == true, "reset latched on rising edge")
	var c2 := mi.take_command()
	_check(c2["reset"] == false, "reset consumed (not sticky)")
	mi._sample(0.1)                # still held -> no new edge
	_check(mi.take_command()["reset"] == false, "held reset does not re-fire")
	Input.action_release("gw_reset")

	mi.queue_free()
	await process_frame


# --- integration: MANUAL FlightBody starts its ground roll with no SITL ------
func _test_integration() -> void:
	var b := GWFlightBody.new()
	b.config = load("res://addons/godotwings/aircraft/Skywalker.tres")
	b.control_source = GWVehicleBody.ControlSource.MANUAL
	get_root().add_child(b)        # _ready() auto-creates a GWManualInput child
	await process_frame

	var mi := b.get_node_or_null("GWManualInput") as GWManualInput
	_check(mi != null, "MANUAL body auto-created a GWManualInput")
	_check(b._source == mi, "FlightBody resolved the manual source (no bridge)")

	Input.action_press("gw_throttle_up")
	mi._sample(3.0)                # peg throttle to full
	for _i in 120:                 # ~2.4 s of physics at 50 Hz
		mi._sample(0.02)
		b._physics_process(0.02)
	Input.action_release("gw_throttle_up")
	_check(b._vel_ned.x > 0.5, "full throttle drives the ground roll forward (vN=%.2f)" % b._vel_ned.x)

	b.queue_free()
	await process_frame
