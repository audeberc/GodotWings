## Quadcopter (multirotor) parameters — the copter analog of GWAircraftConfig.
##
## Drives GWMultirotorBody, whose dynamics follow ArduPilot's SITL multicopter
## frame: per-motor thrust + reaction torque, summed into body force/moment, then
## integrated with a real inertia tensor (+ gyroscopic term). Geometry is a quad X.
@tool
class_name GWMultirotorConfig
extends Resource

const G := 9.80665

@export var mass: float = 0.7            ## kg (all-up weight)

@export_group("Inertia (kg·m²)")
## Leave at 0 to auto-estimate from mass + arm length (point-mass model). Set
## explicitly if you have a measured/CAD inertia tensor.
@export var Ixx: float = 0.0   ## roll
@export var Iyy: float = 0.0   ## pitch
@export var Izz: float = 0.0   ## yaw

@export_group("Geometry")
## Motor distance from the centre (m). Smaller = lower inertia = snappier rotation.
@export var arm_length: float = 0.13
## Motor count (quad X assumed; 4).
@export var motor_count: int = 4

@export_group("Propulsion")
## Full-throttle total thrust ÷ weight. ~5 = snappy FPV freestyle, ~2 = cinematic,
## 1.5 = heavy lifter. NB: hover throttle ≈ 1/this, so keep MOT_THST_HOVER in sync.
@export var thrust_to_weight: float = 5.0
## First-order motor spool lag (s). Lower = crisper throttle response.
@export var motor_time_const: float = 0.02
## Reaction (yaw) torque produced per Newton of motor thrust (m). Sets yaw
## authority. NEGATE this if the copter spins up uncontrollably in SITL (it flips
## the assumed CW/CCW sense — the one thing not validatable without ArduCopter).
@export var yaw_torque_coeff: float = 0.06

@export_group("Aero")
## Lumped translational drag (N per m/s). Higher = more damping / lower top speed.
@export var drag_coeff: float = 0.25


## Full-throttle total thrust (N).
func max_thrust_total() -> float:
	return thrust_to_weight * mass * G


## Inertia tensor (kg·m²), auto-estimating any axis left at 0.
func inertia() -> Vector3:
	var i := Vector3(Ixx, Iyy, Izz)
	var est := _estimate_inertia()
	if i.x <= 0.0: i.x = est.x
	if i.y <= 0.0: i.y = est.y
	if i.z <= 0.0: i.z = est.z
	return i


## Lumped estimate: mass at the arm radius, with a fill factor for the central
## battery/FC vs. tip motors. Roll≈pitch for a symmetric quad; yaw is larger (mass
## spread in the plane). Erring high keeps the rate loop stable on a stock tune;
## measure/CAD the tensor for fidelity.
func _estimate_inertia() -> Vector3:
	var r2 := arm_length * arm_length
	return Vector3(mass * r2 * 0.5, mass * r2 * 0.5, mass * r2 * 0.9)
