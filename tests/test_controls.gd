extends SceneTree

# SITL control-channel exposure + GWChannelSwitch (GCS aux switch -> Godot event)
# + lifecycle signals.

var _ok := true
var _activated := 0
var _deactivated := 0
var _last_value := -1.0

func _check(cond: bool, msg: String) -> void:
	if cond: print("  PASS  ", msg)
	else: push_error("FAIL: " + msg); _ok = false

func _on_activated() -> void: _activated += 1
func _on_deactivated() -> void: _deactivated += 1
func _on_value(v: float) -> void: _last_value = v


func _pwm(ch7: int) -> PackedInt32Array:
	var a := PackedInt32Array(); a.resize(16)
	for i in 16: a[i] = 1500
	a[2] = 1000  # throttle low
	a[6] = ch7   # channel 7 (index 6)
	return a


func _initialize() -> void:
	var b := GWFlightBody.new()
	b.config = load("res://addons/godotwings/aircraft/Skywalker.tres")
	get_root().add_child(b)

	var sw := GWChannelSwitch.new()
	sw.channel = 7
	sw.high_threshold = 0.6
	sw.low_threshold = 0.4
	b.add_child(sw)            # auto-finds the parent GWFlightBody
	await process_frame        # let sw._ready() resolve the body + connect
	sw.activated.connect(_on_activated)
	sw.deactivated.connect(_on_deactivated)
	sw.value_changed.connect(_on_value)

	var got_signal := [false]
	b.controls_received.connect(func(_c): got_signal[0] = true)

	_check(sw._body == b, "switch resolved its GWFlightBody")

	# Channel 7 high -> normalised ~1.0, switch activates.
	b._update_controls(_pwm(2000))
	_check(got_signal[0], "controls_received emitted")
	_check(absf(b.control_norm(7) - 1.0) < 0.001, "control_norm(7) = 1.0 at 2000us (%.2f)" % b.control_norm(7))
	_check(b.control_pwm(7) == 2000, "control_pwm(7) = 2000")
	_check(absf(b.control_norm(3) - 0.0) < 0.001, "throttle (ch3) low = 0.0")
	_check(_activated == 1 and sw.is_on(), "switch activated on high channel")

	# Mid value within hysteresis band -> no toggle.
	b._update_controls(_pwm(1500))
	_check(_activated == 1 and _deactivated == 0 and sw.is_on(), "stays ON within hysteresis")

	# Low -> deactivate.
	b._update_controls(_pwm(1000))
	_check(_deactivated == 1 and not sw.is_on(), "switch deactivated on low channel")
	_check(absf(_last_value - 0.0) < 0.001, "value_changed delivered the latest value")

	b.queue_free()
	await process_frame
	print("test_controls: ", "PASS" if _ok else "FAIL")
	quit(0 if _ok else 1)
