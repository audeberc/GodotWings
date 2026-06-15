## Aerodynamic force model — a faithful port of ArduPilot SIM_Plane's
## liftCoeff / dragCoeff / getForce. Returns the aerodynamic force in the body
## FRD frame (x=forward, y=right, z=down), in Newtons.
class_name GWWingModel
extends RefCounted

var _config: GWAircraftConfig


func _init(config: GWAircraftConfig) -> void:
	_config = config


## Lift coefficient with sigmoid stall blend (SIM_Plane::liftCoeff).
func lift_coeff(alpha_in: float) -> float:
	var c := _config
	var alpha0 := c.alpha_stall
	var m := c.mcoeff
	# Clamp alpha near the stall point to avoid exp() overflow.
	var alpha := alpha_in
	var max_delta := 0.8
	if alpha - alpha0 > max_delta:
		alpha = alpha0 + max_delta
	elif alpha0 - alpha > max_delta:
		alpha = alpha0 - max_delta
	var sigmoid := (1.0 + exp(-m * (alpha - alpha0)) + exp(m * (alpha + alpha0))) \
			/ (1.0 + exp(-m * (alpha - alpha0))) / (1.0 + exp(m * (alpha + alpha0)))
	var linear := (1.0 - sigmoid) * (c.c_lift_0 + c.c_lift_a * alpha)
	var flat_plate := sigmoid * (2.0 * signf(alpha) * sin(alpha) * sin(alpha) * cos(alpha))
	return linear + flat_plate


## Drag coefficient — parabolic polar on the LINEAR lift (SIM_Plane::dragCoeff).
func drag_coeff(alpha: float) -> float:
	var c := _config
	var cl_lin := c.c_lift_0 + c.c_lift_a * alpha
	return c.c_drag_p + (cl_lin * cl_lin) / (PI * c.oswald * c.aspect_ratio())


## Aerodynamic force in body FRD (N). Mirrors SIM_Plane::getForce.
##   airspeed : true airspeed magnitude (m/s)
##   alpha    : angle of attack (rad), atan2(w, u)
##   beta     : sideslip (rad), atan2(v, u)
##   p, q, r  : body rates (rad/s) in FRD
##   da/de/dr : normalized aileron/elevator/rudder [-1, 1]
##   rho      : air density (kg/m^3)
func force(airspeed: float, alpha: float, beta: float,
		p: float, q: float, r: float,
		da: float, de: float, dr: float, rho: float) -> Vector3:
	if airspeed < 0.01:
		return Vector3.ZERO
	var c := _config
	var cl_a := lift_coeff(alpha)
	var cd_a := drag_coeff(alpha)
	var ca := cos(alpha)
	var sa := sin(alpha)

	# Lift/drag coefficients rotated into the body frame.
	var c_x_a := -cd_a * ca + cl_a * sa
	var c_x_q := -c.c_drag_q * ca + c.c_lift_q * sa
	var c_z_a := -cd_a * sa - cl_a * ca
	var c_z_q := -c.c_drag_q * sa - c.c_lift_q * ca

	var qbar := 0.5 * rho * airspeed * airspeed * c.s
	var two_v := 2.0 * airspeed

	var ax := qbar * (c_x_a + c_x_q * c.c * q / two_v \
			- c.c_drag_deltae * ca * absf(de) + c.c_lift_deltae * sa * de)
	var ay := qbar * (c.c_y_0 + c.c_y_b * beta + c.c_y_p * c.b * p / two_v \
			+ c.c_y_r * c.b * r / two_v + c.c_y_deltaa * da + c.c_y_deltar * dr)
	var az := qbar * (c_z_a + c_z_q * c.c * q / two_v \
			- c.c_drag_deltae * sa * absf(de) - c.c_lift_deltae * ca * de)

	return Vector3(ax, ay, az)
