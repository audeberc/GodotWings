## Per-aircraft flight model parameters.
##
## These mirror ArduPilot's SITL reference plane (libraries/SITL/SIM_Plane.{h,cpp})
## 1:1 — field names and default values match its `coefficient` struct (the
## "last_letter skywalker_2013" model). Matching ArduPilot's own model is what
## makes the aircraft flyable under ArduPilot's tuned controllers.
##
## IMPORTANT: the moment coefficients (c_l_*, c_m_*, c_n_*) yield ANGULAR
## ACCELERATION directly (SIM_Plane integrates `gyro += rot_accel*dt` with no
## inertia tensor); they already bake in 1/inertia. Do not divide by inertia.
class_name GWAircraftConfig
extends Resource

# --- Geometry / mass ---------------------------------------------------------
@export var s: float = 0.45        ## wing area, m^2
@export var b: float = 1.88        ## wingspan, m
@export var c: float = 0.24        ## mean aerodynamic chord, m
@export var mass: float = 2.0      ## kg

# --- Lift --------------------------------------------------------------------
@export var c_lift_0: float = 0.56
@export var c_lift_a: float = 6.9      ## lift-curve slope, per rad
@export var c_lift_deltae: float = 0.0
@export var c_lift_q: float = 0.0
@export var mcoeff: float = 50.0       ## stall sigmoid sharpness
@export var oswald: float = 0.9
@export var alpha_stall: float = 0.4712  ## rad (~27 deg)

# --- Drag --------------------------------------------------------------------
@export var c_drag_q: float = 0.0
@export var c_drag_deltae: float = 0.0
@export var c_drag_p: float = 0.1

# --- Side force --------------------------------------------------------------
@export var c_y_0: float = 0.0
@export var c_y_b: float = -0.98
@export var c_y_p: float = 0.0
@export var c_y_r: float = 0.0
@export var c_y_deltaa: float = 0.0
@export var c_y_deltar: float = -0.2

# --- Roll moment (angular accel) ---------------------------------------------
@export var c_l_0: float = 0.0
@export var c_l_p: float = -1.0        ## roll damping
@export var c_l_b: float = -0.12       ## dihedral
@export var c_l_r: float = 0.14
@export var c_l_deltaa: float = 0.25   ## aileron
@export var c_l_deltar: float = -0.037

# --- Pitch moment (angular accel) --------------------------------------------
@export var c_m_0: float = 0.045
@export var c_m_a: float = -0.7        ## static pitch stability
@export var c_m_q: float = -20.0       ## pitch damping
@export var c_m_deltae: float = 1.0    ## elevator (note: POSITIVE)

# --- Yaw moment (angular accel) ----------------------------------------------
@export var c_n_0: float = 0.0
@export var c_n_b: float = 0.25        ## weathervane
@export var c_n_p: float = 0.022
@export var c_n_r: float = -1.0        ## yaw damping
@export var c_n_deltaa: float = 0.0
@export var c_n_deltar: float = 0.1    ## rudder (note: POSITIVE)

# --- CG offset (m, FRD) — couples aero force into moments (pitch trim) --------
@export var cg_offset: Vector3 = Vector3(-0.15, 0.0, -0.05)

# --- Propulsion --------------------------------------------------------------
## Full-throttle thrust = mass * g / hover_throttle (SIM_Plane convention).
## Lower hover_throttle = more thrust. 0.5 -> ~39 N, thrust/weight ~2.0 (lively,
## easy manual takeoff). Raise toward 0.7 for a tamer trainer (~28 N, T/W ~1.4).
@export var hover_throttle: float = 0.5
@export var motor_time_const: float = 0.15  ## throttle spool lag, s

# --- Actuators ---------------------------------------------------------------
@export var servo_time_const: float = 0.05  ## control-surface lag, s
@export var max_deflection: float = 0.3491  ## rad, full-stick (deltaX_max)

const G := 9.80665

## Full-throttle thrust in Newtons (matches SIM_Plane's thrust_scale).
func thrust_scale() -> float:
	return mass * G / hover_throttle

## Aspect ratio.
func aspect_ratio() -> float:
	return (b * b) / s
