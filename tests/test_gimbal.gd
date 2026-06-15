extends SceneTree

# GWCamera as an ArduPilot servo gimbal: mount servo PWM (read off the GWFlightBody
# control channels) decodes to pitch/yaw/roll angles and rotates the camera. The
# autopilot resolves the mount mode; here we just verify the PWM->angle->pointing math.

var _ok := true

func _check(cond: bool, msg: String) -> void:
	if cond: print("  PASS  ", msg)
	else: push_error("FAIL: " + msg); _ok = false


func _approx(a: float, b: float, eps := 1e-4) -> bool:
	return absf(a - b) < eps


func _initialize() -> void:
	var body := GWFlightBody.new()
	var pwm := PackedInt32Array()
	pwm.resize(16)
	for i in 16:
		pwm[i] = 1500
	body._controls_pwm = pwm

	var cam := GWCamera.new()
	cam._body = body
	cam.gimbal_enabled = true
	cam.gimbal_pitch_channel = 9    # SERVO9 = mount pitch
	cam.gimbal_yaw_channel = 10     # SERVO10 = mount yaw
	cam.gimbal_pitch_range_deg = Vector2(-90.0, 0.0)
	cam.gimbal_yaw_range_deg = Vector2(-180.0, 180.0)
	cam.gimbal_pwm_min = 1000
	cam.gimbal_pwm_max = 2000

	# --- PWM -> angle mapping (endpoints + midpoint) ---
	body._controls_pwm[8] = 1000
	_check(_approx(cam._gimbal_axis_deg(9, cam.gimbal_pitch_range_deg), -90.0), "pitch min PWM -> -90 deg")
	body._controls_pwm[8] = 2000
	_check(_approx(cam._gimbal_axis_deg(9, cam.gimbal_pitch_range_deg), 0.0), "pitch max PWM -> 0 deg")
	_check(_approx(cam._gimbal_axis_deg(10, cam.gimbal_yaw_range_deg), 0.0), "yaw mid PWM (1500) -> 0 deg")
	body._controls_pwm[9] = 2000
	_check(_approx(cam._gimbal_axis_deg(10, cam.gimbal_yaw_range_deg), 180.0), "yaw max PWM -> 180 deg")

	# An unassigned axis or no-data channel reads 0.
	_check(_approx(cam._gimbal_axis_deg(0, cam.gimbal_roll_range_deg), 0.0), "channel 0 (unassigned) -> 0 deg")

	# --- pointing: pitch -90 looks straight down (Godot -Y) ---
	body._controls_pwm[8] = 1000  # pitch -> -90
	body._controls_pwm[9] = 1500  # yaw -> 0
	var fwd_down := cam._gimbal_basis() * Vector3.FORWARD
	_check(fwd_down.distance_to(Vector3(0, -1, 0)) < 1e-4, "pitch -90 points camera down (%s)" % fwd_down)

	# --- pointing: pitch 0 + yaw 180 looks backward (Godot +Z) ---
	body._controls_pwm[8] = 2000  # pitch -> 0
	body._controls_pwm[9] = 2000  # yaw -> 180
	var fwd_back := cam._gimbal_basis() * Vector3.FORWARD
	_check(fwd_back.distance_to(Vector3(0, 0, 1)) < 1e-4, "yaw 180 points camera backward (%s)" % fwd_back)

	# --- yaw sense: +90 pans RIGHT (+X), matching ArduPilot's positive pan ---
	body._controls_pwm[8] = 2000  # pitch -> 0
	body._controls_pwm[9] = 1750  # yaw -> +90  (lerp(-180, 180, 0.75))
	var fwd_right := cam._gimbal_basis() * Vector3.FORWARD
	_check(fwd_right.distance_to(Vector3(1, 0, 0)) < 1e-4, "yaw +90 pans right / +X (%s)" % fwd_right)

	# --- roll sense: +90 rolls right-side-down, tipping the up vector to +X ---
	cam.gimbal_roll_channel = 11
	cam.gimbal_roll_range_deg = Vector2(-90.0, 90.0)
	body._controls_pwm[8] = 2000   # pitch -> 0
	body._controls_pwm[9] = 1500   # yaw -> 0
	body._controls_pwm[10] = 2000  # roll -> +90  (channel 11)
	var up_rolled := cam._gimbal_basis() * Vector3.UP
	_check(up_rolled.distance_to(Vector3(1, 0, 0)) < 1e-4, "roll +90 tips up to +X (%s)" % up_rolled)

	# --- disabled gimbal is identity (camera looks out its mount, -Z) ---
	cam.gimbal_enabled = false
	var fwd_fixed := cam._gimbal_basis() * Vector3.FORWARD
	_check(fwd_fixed.distance_to(Vector3(0, 0, -1)) < 1e-6, "gimbal off -> identity (forward -Z)")

	body.free()
	cam.free()
	print("test_gimbal: ", "PASS" if _ok else "FAIL")
	quit(0 if _ok else 1)
