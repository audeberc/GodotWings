class_name GWChannelSwitch
extends Node

## Drop-in bridge from a SITL control channel to Godot events — flipping an RC/aux
## switch in your GCS triggers scene logic (drop payload, lights, gear, …). Add it
## under a GWVehicleBody, set `channel`, connect the signals.
##
## Note SITL carries ArduPilot's *servo outputs*, not raw RC, so to surface an aux
## switch on channel 9 set e.g. SERVO9_FUNCTION = 59 (RCIN9 passthrough). See README.

## Emitted when the channel rises through high_threshold (switch ON).
signal activated
## Emitted when the channel falls through low_threshold (switch OFF).
signal deactivated
## Emitted whenever the normalised channel value changes (0..1).
signal value_changed(value: float)

## Source aircraft. Empty = auto-find the nearest GWVehicleBody ancestor.
@export var flight_body_path: NodePath
## Channel to watch, 1-based (1..16). 1-4 are flight controls; 5+ are free.
@export_range(1, 16) var channel: int = 7
## Hysteresis: rise above high to turn ON, fall below low to turn OFF.
@export_range(0.0, 1.0) var high_threshold: float = 0.6
@export_range(0.0, 1.0) var low_threshold: float = 0.4
## Treat a low channel as ON (e.g. a switch that idles high).
@export var invert: bool = false

var _body: GWVehicleBody
var _on := false
var _last_value := -1.0


func _ready() -> void:
	_body = _resolve_body()
	if _body == null:
		push_warning("GWChannelSwitch: no GWVehicleBody found; will stay idle.")
		return
	_body.controls_received.connect(_on_controls)


func _resolve_body() -> GWVehicleBody:
	if not flight_body_path.is_empty():
		return get_node_or_null(flight_body_path) as GWVehicleBody
	var n := get_parent()
	while n != null:
		if n is GWVehicleBody:
			return n
		n = n.get_parent()
	return null


func _on_controls(channels: PackedFloat32Array) -> void:
	var i := channel - 1
	if i < 0 or i >= channels.size():
		return
	var v := channels[i]
	if invert:
		v = 1.0 - v
	if not is_equal_approx(v, _last_value):
		_last_value = v
		value_changed.emit(v)
	# Hysteresis edge detection.
	if not _on and v >= high_threshold:
		_on = true
		activated.emit()
	elif _on and v <= low_threshold:
		_on = false
		deactivated.emit()


## Current debounced switch state.
func is_on() -> bool:
	return _on
