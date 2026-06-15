## Fixed-wing 6-DOF flight dynamics — a faithful port of ArduPilot SIM_Plane's
## update loop, layered on the shared GWVehicleBody (SITL lockstep, ground/terrain,
## crash/ragdoll, wind, rendering, state). Moments are angular accelerations (no
## inertia tensor / gyroscopic term) exactly as SIM_Plane integrates them.
##
## Channels 1-4 = aileron / elevator / throttle / rudder.
class_name GWFlightBody
extends GWVehicleBody

const V_ROTATE := 9.0       ## airspeed (m/s) at which the aircraft leaves the gear
const ROLL_FRICTION := 0.04   ## kinetic rolling resistance once rolling
const STATIC_FRICTION := 0.3  ## stiction: thrust must exceed mu*m*g to start rolling
const STEER_RATE := 0.8       ## max nose-wheel yaw rate (rad/s) at speed
const LEVEL_TAU := 0.3        ## time constant (s) to null roll/pitch on the gear

## Default config shipped with the addon; used only if `config` is left unset.
const DEFAULT_CONFIG_PATH := "res://addons/godotwings/aircraft/Skywalker.tres"

@export var config: GWAircraftConfig

# Fixed-wing subsystems.
var _surfaces: GWControlSurfaces
var _prop: GWPropulsionModel
var _wing: GWWingModel
var _moment: GWMomentModel


func _setup_vehicle() -> bool:
	if config == null and ResourceLoader.exists(DEFAULT_CONFIG_PATH):
		config = load(DEFAULT_CONFIG_PATH)
	if config == null:
		push_error("GWFlightBody: no AircraftConfig assigned (set the `config` property).")
		return false
	_wing = GWWingModel.new(config)
	_moment = GWMomentModel.new(config)
	return true


func _reset_dynamics() -> void:
	# Recreate actuators so their lag state resets too.
	_surfaces = GWControlSurfaces.new(config)
	_prop = GWPropulsionModel.new(config)


func _vehicle_mass() -> float:
	return config.mass


func _default_hull_size() -> Vector3:
	# Render frame: X=span, Y=height, Z=length.
	return Vector3(config.b, 0.3, maxf(config.c * 4.0, 0.8))


## One sub-step of fixed-wing dynamics: actuators, then ground roll-out or airborne
## 6-DOF (controls come from the latest SITL command).
func _integrate(h: float) -> void:
	var aileron: float = _cmd.get("aileron", 0.0)
	var elevator: float = _cmd.get("elevator", 0.0)
	var throttle: float = _cmd.get("throttle", 0.0)
	var rudder: float = _cmd.get("rudder", 0.0)
	_surfaces.update(h, aileron, elevator, rudder)
	_prop.update(h, throttle)
	if _on_ground:
		_step_ground(h, rudder)
	else:
		_step_air(h)


## Airborne dynamics in NED/FRD — SIM_Plane's getForce / getTorque. Moments are
## angular accelerations (the coefficients bake in 1/inertia).
func _step_air(dt: float) -> void:
	var v_air_ned := _vel_ned - _wind_ned()
	var v_air_body := _dcm.transposed() * v_air_ned # NED -> FRD
	_airspeed = v_air_ned.length()

	var rho := _atmos.density(-_pos_ned.z)
	var alpha := 0.0
	var beta := 0.0
	if _airspeed > 0.01:
		alpha = atan2(v_air_body.z, v_air_body.x)
		beta = atan2(v_air_body.y, v_air_body.x)
	var da := _surfaces.aileron
	var de := _surfaces.elevator
	var dr := _surfaces.rudder
	var p := _omega.x
	var q := _omega.y
	var r := _omega.z

	# --- linear: specific force (FRD) = (thrust + aero) / mass ----------------
	var f_aero := _wing.force(_airspeed, alpha, beta, p, q, r, da, de, dr, rho)
	var accel_body := (Vector3(_prop.thrust(), 0, 0) + f_aero) / config.mass
	var accel_ned := _dcm * accel_body + Vector3(0, 0, G) # add gravity (down=+z)
	_vel_ned += accel_ned * dt
	_pos_ned += _vel_ned * dt

	# --- angular: rot_accel integrated directly (SIM_Plane convention) --------
	var rot_accel := _moment.torque(_airspeed, alpha, beta, p, q, r, da, de, dr, f_aero, rho)
	_omega += rot_accel * dt
	var ang := _omega.length() * dt
	if ang > 1e-9:
		_dcm = (_dcm * Basis(_omega.normalized(), ang)).orthonormalized()

	# Ground contact: within gear clearance while descending, or penetrating it.
	var agl := _agl()
	if agl < spawn_altitude and (_vel_ned.z > 0.0 or agl <= 0.0):
		_on_contact()


## Simplified ground roll-out: gear holds the aircraft, motion is along heading,
## rudder steers. Roll/pitch are nulled SMOOTHLY (over LEVEL_TAU) via real body
## rates so the reported gyro stays consistent — no teleport. Releases at V_ROTATE.
func _step_ground(dt: float, rudder: float) -> void:
	var att := GWCoordConvert.dcm_to_ned_attitude(_dcm)
	var roll: float = att[0]
	var pitch: float = att[1]
	var yaw: float = att[2]
	var fwd_h := Vector3(cos(yaw), sin(yaw), 0.0) # horizontal heading in NED

	# Longitudinal motion along the heading. Static friction holds the aircraft put
	# until thrust exceeds mu*m*g (no creep from a phantom throttle); once rolling,
	# only kinetic rolling resistance applies. Friction never reverses it (>= 0).
	var fwd_speed := maxf(_vel_ned.dot(fwd_h), 0.0)
	var thrust := _prop.thrust()
	var rho := _atmos.density(spawn_altitude)
	var drag := 0.5 * rho * fwd_speed * fwd_speed * config.s * config.c_drag_p
	if fwd_speed < 0.05 and thrust <= config.mass * G * STATIC_FRICTION:
		fwd_speed = 0.0 # static friction holds it put (no creep)
	else:
		var mu := STATIC_FRICTION if fwd_speed < 0.05 else ROLL_FRICTION
		var net := thrust - drag - config.mass * G * mu
		fwd_speed = maxf(fwd_speed + (net / config.mass) * dt, 0.0)

	# Attitude: body rates that null roll/pitch over LEVEL_TAU and steer in yaw.
	var steer := rudder * STEER_RATE * clampf(fwd_speed / 5.0, 0.0, 1.0)
	_omega = Vector3(-roll / LEVEL_TAU, -pitch / LEVEL_TAU, steer)
	var ang := _omega.length() * dt
	if ang > 1e-9:
		_dcm = (_dcm * Basis(_omega.normalized(), ang)).orthonormalized()

	_vel_ned = fwd_h * fwd_speed
	_pos_ned += _vel_ned * dt
	_pos_ned.z = _ground_down - spawn_altitude  # ride at gear height over the terrain
	_airspeed = fwd_speed

	if fwd_speed >= V_ROTATE:
		_on_ground = false # release; elevator now rotates the aircraft
		took_off.emit()


## The aircraft descends through the runway: a gentle, roughly level touchdown
## transitions to a ground roll; anything harder (sink/bank/pitch/slope) is a crash.
func _on_contact() -> void:
	var att := GWCoordConvert.dcm_to_ned_attitude(_dcm)
	var slope := acos(clampf(_ground_normal.dot(Vector3(0, 0, -1)), -1.0, 1.0))
	var hard := _vel_ned.z > CRASH_SINK_RATE \
			or absf(att[0]) > CRASH_BANK or absf(att[1]) > CRASH_PITCH \
			or slope > CRASH_SLOPE
	if hard:
		_enter_crash() # ragdoll or scripted settle, per crash_mode
	else:
		_pos_ned.z = _ground_down - spawn_altitude # gear rests on the terrain
		_on_ground = true
		if _vel_ned.z > 0.0:
			_vel_ned.z = 0.0 # gear absorbs the (small) descent
		landed.emit()
