## Propulsion: throttle -> thrust along the body forward axis (linear in throttle),
## with first-order spool lag on the motor.
class_name GWPropulsionModel
extends RefCounted

var _config: GWAircraftConfig

## Actual (lagged) throttle, [0, 1].
var throttle := 0.0


func _init(config: GWAircraftConfig) -> void:
	_config = config


func update(dt: float, throttle_cmd: float) -> void:
	var k := 1.0 - exp(-dt / maxf(_config.motor_time_const, 1e-4))
	throttle = lerpf(throttle, clampf(throttle_cmd, 0.0, 1.0), k)


## Thrust magnitude (N) at the current spooled throttle.
func thrust() -> float:
	return throttle * _config.thrust_scale()
