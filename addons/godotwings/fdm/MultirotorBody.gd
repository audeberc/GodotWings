## Quadcopter 6-DOF dynamics — the copter analog of GWFlightBody, on the shared
## GWVehicleBody (SITL lockstep, ground/terrain, crash/ragdoll, wind, rendering).
##
## Follows ArduPilot's SITL multicopter frame model: each motor produces thrust
## along body-up plus a reaction (yaw) torque; these sum into a body force and
## moment, integrated as a rigid body with a real inertia tensor and the
## gyroscopic term (unlike the fixed-wing SIM_Plane port, which bakes 1/inertia
## into its coefficients). Quad X layout; channels 1-4 = motors.
class_name GWMultirotorBody
extends GWVehicleBody

## Default config shipped with the addon; used only if `config` is left unset.
const DEFAULT_CONFIG_PATH := "res://addons/godotwings/aircraft/QuadX.tres"

@export var config: GWMultirotorConfig
## SITL control-loop rate (Hz). A multirotor's rate controller needs a fast loop,
## and the ArduPilot exchange runs once per Godot physics tick — so this raises the
## engine's physics tick rate to match. The 60 Hz default makes the copter
## oscillate/vibrate (ArduCopter expects ~400 Hz). 0 = leave the project setting.
@export var control_rate_hz: int = 400

var _motor := PackedFloat32Array()    # lagged throttle per motor, 0..1
var _pos: Array[Vector3] = []         # motor positions (body FRD, m)
var _yawdir := PackedFloat32Array()   # +1 = CCW prop, -1 = CW prop
var _inertia := Vector3.ONE
var _per_motor_max := 0.0


func _setup_vehicle() -> bool:
	if config == null and ResourceLoader.exists(DEFAULT_CONFIG_PATH):
		config = load(DEFAULT_CONFIG_PATH)
	if config == null:
		push_error("GWMultirotorBody: no MultirotorConfig assigned (set the `config` property).")
		return false
	_build_frame()
	_ensure_control_rate()
	return true


## Raise Godot's physics tick rate so the SITL exchange (one per tick) runs the
## ArduCopter rate loop fast enough — a 60 Hz loop makes a copter oscillate.
func _ensure_control_rate() -> void:
	if control_rate_hz <= 0 or Engine.is_editor_hint():
		return
	if Engine.physics_ticks_per_second < control_rate_hz:
		Engine.physics_ticks_per_second = control_rate_hz
		# Don't let a slow render frame starve the physics/SITL exchange.
		Engine.max_physics_steps_per_frame = maxi(
				Engine.max_physics_steps_per_frame, ceili(control_rate_hz / 30.0))
		print("GWMultirotorBody: raised physics tick rate to %d Hz (copter rate loop)." % control_rate_hz)


## Quad X: motor 1 front-right (CCW), 2 back-left (CCW), 3 front-left (CW),
## 4 back-right (CW) — ArduCopter's standard layout. Position from a body-frame
## angle (deg, from +x forward toward +y right).
func _build_frame() -> void:
	_inertia = config.inertia()
	_per_motor_max = config.max_thrust_total() / maxf(config.motor_count, 1)
	var defs := [[45.0, 1.0], [-135.0, 1.0], [-45.0, -1.0], [135.0, -1.0]]
	_pos.clear()
	_yawdir.resize(defs.size())
	for i in defs.size():
		var a := deg_to_rad(defs[i][0])
		_pos.append(Vector3(cos(a), sin(a), 0.0) * config.arm_length)
		_yawdir[i] = defs[i][1]


func _reset_dynamics() -> void:
	_motor.resize(_pos.size())
	_motor.fill(0.0)


func _vehicle_mass() -> float:
	return config.mass


func _default_hull_size() -> Vector3:
	var d := config.arm_length * 2.0
	return Vector3(d, 0.2, d)


## One sub-step: spool the motors, then integrate attitude + position. Attitude is
## ALWAYS driven by the real motor moment — on the ground and in the air — so the
## autopilot's rate controller stays closed-loop (no force-leveling that would
## desync the gyro from its commands and tip the copter on takeoff).
func _integrate(h: float) -> void:
	var thrusts := _update_motors(h)
	var total := 0.0
	for t in thrusts:
		total += t
	_integrate_omega(_frame_moment(thrusts), h)
	# Linear: thrust (body up = -z) + lumped translational drag + gravity.
	var v_air_ned := _vel_ned - _wind_ned()
	var v_air_body := _dcm.transposed() * v_air_ned
	_airspeed = v_air_ned.length()
	var accel_body := Vector3(0.0, 0.0, -total / config.mass) \
			- (config.drag_coeff / config.mass) * v_air_body
	var accel_ned := _dcm * accel_body + Vector3(0.0, 0.0, G)
	_vel_ned += accel_ned * h
	_pos_ned += _vel_ned * h
	_ground_contact(h)


## Spool each motor toward its commanded throttle (channels 1..N) and return the
## per-motor thrust (N).
func _update_motors(h: float) -> PackedFloat32Array:
	var k := 1.0 - exp(-h / maxf(config.motor_time_const, 1e-4))
	var thrusts := PackedFloat32Array()
	thrusts.resize(_motor.size())
	for i in _motor.size():
		_motor[i] = lerpf(_motor[i], clampf(control_norm(i + 1), 0.0, 1.0), k)
		thrusts[i] = _motor[i] * _per_motor_max
	return thrusts


## Body moment (N·m): r × thrust from each motor's offset (roll/pitch) plus the
## prop reaction torque about yaw.
func _frame_moment(thrusts: PackedFloat32Array) -> Vector3:
	var m := Vector3.ZERO
	for i in thrusts.size():
		var t := thrusts[i]
		m.x += -_pos[i].y * t                          # roll  = -(py)·T
		m.y += _pos[i].x * t                           # pitch =  (px)·T
		m.z += _yawdir[i] * t * config.yaw_torque_coeff  # yaw reaction
	return m


## Rigid-body angular update with the full inertia tensor + gyroscopic coupling:
## omega_dot = I⁻¹ (M − omega × (I·omega)).
func _integrate_omega(moment: Vector3, dt: float) -> void:
	var iom := Vector3(_inertia.x * _omega.x, _inertia.y * _omega.y, _inertia.z * _omega.z)
	var gyro := _omega.cross(iom)
	var acc := Vector3(
		(moment.x - gyro.x) / _inertia.x,
		(moment.y - gyro.y) / _inertia.y,
		(moment.z - gyro.z) / _inertia.z)
	_omega += acc * dt
	var ang := _omega.length() * dt
	if ang > 1e-9:
		_dcm = (_dcm * Basis(_omega.normalized(), ang)).orthonormalized()


## Ground contact: rest at gear height, stop downward motion, brake horizontal
## sliding, and detect liftoff / hard touchdown — WITHOUT touching attitude (the
## autopilot owns that). Lifts off naturally once thrust > weight pushes it up.
func _ground_contact(_h: float) -> void:
	var gear_z := _ground_down - spawn_altitude   # NED-down at rest (z is down)
	if _pos_ned.z < gear_z:                        # higher than rest -> airborne
		if _on_ground:
			_on_ground = false
			took_off.emit()
		return
	# At or below gear height: in contact with the ground.
	if not _on_ground:                             # just touched down this step
		var att := GWCoordConvert.dcm_to_ned_attitude(_dcm)
		var slope := acos(clampf(_ground_normal.dot(Vector3(0, 0, -1)), -1.0, 1.0))
		if _vel_ned.z > CRASH_SINK_RATE or absf(att[0]) > CRASH_BANK \
				or absf(att[1]) > CRASH_PITCH or slope > CRASH_SLOPE:
			_enter_crash()
			return
		landed.emit()
	_on_ground = true
	_pos_ned.z = gear_z
	if _vel_ned.z > 0.0:
		_vel_ned.z = 0.0           # gear stops the descent
	_vel_ned.x *= clampf(1.0 - _h * 6.0, 0.0, 1.0)   # feet grip: brake sliding
	_vel_ned.y *= clampf(1.0 - _h * 6.0, 0.0, 1.0)
