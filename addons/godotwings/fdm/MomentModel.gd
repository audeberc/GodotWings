## Aerodynamic moment model — a faithful port of ArduPilot SIM_Plane::getTorque.
##
## Returns ANGULAR ACCELERATION (rad/s^2) in the body FRD frame, as (roll-rate,
## pitch-rate, yaw-rate) derivatives. SIM_Plane integrates `gyro += rot_accel*dt`
## with no inertia tensor, so its c_l/c_m/c_n coefficients already bake in
## 1/inertia. Do NOT divide the result by an inertia tensor.
class_name GWMomentModel
extends RefCounted

var _config: GWAircraftConfig


func _init(config: GWAircraftConfig) -> void:
	_config = config


## Body-frame angular acceleration (rad/s^2), (l, m, n) about (roll, pitch, yaw).
##   airspeed : true airspeed (m/s)
##   alpha    : angle of attack (rad)
##   beta     : sideslip (rad)
##   p, q, r  : body rates (rad/s) FRD
##   da/de/dr : normalized aileron/elevator/rudder [-1, 1]
##   force    : aerodynamic force (N, body FRD) from GWWingModel — for CG coupling
##   rho      : air density (kg/m^3)
func torque(airspeed: float, alpha: float, beta: float,
		p: float, q: float, r: float,
		da: float, de: float, dr: float,
		force: Vector3, rho: float) -> Vector3:
	var c := _config
	var la := 0.0
	var ma := 0.0
	var na := 0.0
	if airspeed >= 0.01:
		var qbar := 0.5 * rho * airspeed * airspeed * c.s
		var two_v := 2.0 * airspeed
		la = qbar * c.b * (c.c_l_0 + c.c_l_b * beta + c.c_l_p * c.b * p / two_v \
				+ c.c_l_r * c.b * r / two_v + c.c_l_deltaa * da + c.c_l_deltar * dr)
		ma = qbar * c.c * (c.c_m_0 + c.c_m_a * alpha + c.c_m_q * c.c * q / two_v \
				+ c.c_m_deltae * de)
		na = qbar * c.b * (c.c_n_0 + c.c_n_b * beta + c.c_n_p * c.b * p / two_v \
				+ c.c_n_r * c.b * r / two_v + c.c_n_deltaa * da + c.c_n_deltar * dr)

	# Torque from aero-force misalignment with the CG (r x F).
	var cg := c.cg_offset
	la += cg.y * force.z - cg.z * force.y
	ma += -cg.x * force.z + cg.z * force.x
	na += -cg.y * force.x + cg.x * force.y

	return Vector3(la, ma, na)
