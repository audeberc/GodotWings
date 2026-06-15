## Control-surface actuator model: commanded deflections -> actual deflections
## with first-order servo lag.
##
## Inputs/outputs are NORMALIZED (aileron/elevator/rudder in [-1, 1]) — the form
## GWMomentModel's derivatives expect (matching ArduPilot SIM_Plane, whose
## coefficients are per unit input). Convert to radians for visual surface
## animation via [method deflection_rad].
class_name GWControlSurfaces
extends RefCounted

var _config: GWAircraftConfig

# Actual (lagged) deflections.
var aileron := 0.0
var elevator := 0.0
var rudder := 0.0


func _init(config: GWAircraftConfig) -> void:
	_config = config


func update(dt: float, aileron_cmd: float, elevator_cmd: float, rudder_cmd: float) -> void:
	var k := 1.0 - exp(-dt / maxf(_config.servo_time_const, 1e-4))
	aileron = lerpf(aileron, clampf(aileron_cmd, -1.0, 1.0), k)
	elevator = lerpf(elevator, clampf(elevator_cmd, -1.0, 1.0), k)
	rudder = lerpf(rudder, clampf(rudder_cmd, -1.0, 1.0), k)


## Physical deflection (radians) for a normalized surface value — for visuals.
func deflection_rad(normalized: float) -> float:
	return normalized * _config.max_deflection
