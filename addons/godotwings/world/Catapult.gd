@tool
class_name GWCatapult
extends Node3D

## Drop-in catapult launcher. Shows the launcher model and fires a held vehicle off
## it when a SITL aux channel goes high (RC6 by default, on the rising edge).
##
## Use: drop a GWCatapult in the world, place a GWAircraft / GWMulticopter on the
## rail, and tick the vehicle's `hold_until_launch` so it waits there. Flip the RC
## channel and it launches along its nose.
##
## RC trigger: Godot sees ArduPilot's servo outputs, not RC inputs, so the switch
## must be routed to an output — docker/sitl-defaults.parm already passes RCIN5-16
## through to channels 5-16. Drive RC<trigger_channel> high to fire.

const DEFAULT_MODEL := preload("res://addons/godotwings/aircraft/catapult.glb")

## Vehicle to launch. Empty = auto-find the nearest GWVehicleBody in the scene.
@export var aircraft_path: NodePath
## Launch speed off the rail (m/s). Must clear the wing's rotate speed (~9 m/s).
@export var launch_speed: float = 22.0
## SITL aux channel that fires the catapult on its rising edge.
@export_range(1, 16) var trigger_channel: int = 6
@export_range(0.0, 1.0) var trigger_threshold: float = 0.6
## Show the bundled launcher model (untick to parent your own mesh instead).
@export var show_model: bool = true:
	set(v):
		show_model = v
		_refresh_model()

var _vehicle: GWVehicleBody
var _ch_high := false
var _armed := false  # only fire after the channel has been seen low (ignore boot-high)


func _ready() -> void:
	_refresh_model()
	if Engine.is_editor_hint():
		return
	_vehicle = _resolve_vehicle()
	if _vehicle == null:
		push_warning("GWCatapult: no GWVehicleBody found to launch (set aircraft_path).")
		return
	_vehicle.controls_received.connect(_on_controls)


func _refresh_model() -> void:
	if not is_inside_tree():
		return
	var existing := get_node_or_null("Model")
	if show_model and existing == null:
		var wrapper := Node3D.new()
		wrapper.name = "Model"
		wrapper.add_child(DEFAULT_MODEL.instantiate())
		add_child(wrapper)
	elif not show_model and existing != null:
		existing.free()


func _resolve_vehicle() -> GWVehicleBody:
	if not aircraft_path.is_empty():
		return get_node_or_null(aircraft_path) as GWVehicleBody
	return _find_vehicle(get_tree().current_scene if get_tree() else null)


func _find_vehicle(n: Node) -> GWVehicleBody:
	if n == null:
		return null
	if n is GWVehicleBody:
		return n
	for c in n.get_children():
		var f := _find_vehicle(c)
		if f != null:
			return f
	return null


func _on_controls(channels: PackedFloat32Array) -> void:
	if trigger_channel < 1 or trigger_channel > channels.size():
		return
	var high := channels[trigger_channel - 1] >= trigger_threshold
	if not high:
		_armed = true                       # channel low -> arm; ignores a boot-high switch
	elif _armed and not _ch_high:           # deliberate low -> high edge
		_vehicle.launch(launch_speed)
		_armed = false
		print("GWCatapult: launched %s at %.0f m/s" % [_vehicle.name, launch_speed])
	_ch_high = high
