@icon("res://addons/godotwings/sensors/camera_icon.svg")
class_name GWViewCamera
extends Camera3D

## In-game spectator camera with selectable modes (NOT the streamed GWCamera).
## Drop it in a scene, point `target_path` at the aircraft (or leave empty to
## auto-find the nearest GWVehicleBody), and fly.
##
## Modes (cycle with `cycle_key`): CHASE (smoothed follow), ORBIT (rigid orbit),
## GROUND (ground-level spectator, WASD-walkable). Hold `orbit_button` to swing the
## angle, wheel to zoom.

enum Mode { CHASE, ORBIT, GROUND }

@export var mode := Mode.CHASE:
	set(v):
		mode = v
		if is_inside_tree():
			_enter_mode()
@export var target_path: NodePath

@export_group("Rig")
@export var distance := 18.0
@export var min_distance := 4.0
@export var max_distance := 300.0
## Elevation angle (deg) above the target for chase/orbit.
@export var pitch_deg := 18.0
@export var chase_smoothing := 5.0

@export_group("Input")
@export var enable_input := true
@export var cycle_key := KEY_C
@export var orbit_button := MOUSE_BUTTON_RIGHT
@export var orbit_sensitivity := 0.006
@export var zoom_step := 2.0

@export_group("Ground mode")
@export var allow_ground_move := true
@export var ground_move_speed := 25.0
@export var ground_eye_height := 2.0
@export_flags_3d_physics var ground_mask := 1

var _target: Node3D
var _yaw := 0.0
var _pitch := 0.0
var _orbiting := false
var _ground_pos := Vector3.ZERO


func _ready() -> void:
	_pitch = deg_to_rad(pitch_deg)
	_target = _resolve_target()
	current = true
	_enter_mode()


func _resolve_target() -> Node3D:
	if not target_path.is_empty():
		return get_node_or_null(target_path) as Node3D
	var root := get_tree().current_scene if get_tree() else null
	return _find_flight_body(root) if root else null


func _find_flight_body(n: Node) -> GWVehicleBody:
	if n is GWVehicleBody:
		return n
	for c in n.get_children():
		var f := _find_flight_body(c)
		if f != null:
			return f
	return null


func _enter_mode() -> void:
	if mode == Mode.GROUND:
		# Anchor the spectator at the camera's current spot, dropped onto the ground.
		_ground_pos = global_position
		_ground_pos.y = _ground_height(_ground_pos) + ground_eye_height


func _unhandled_input(event: InputEvent) -> void:
	if not enable_input:
		return
	if event is InputEventKey and event.pressed and not event.echo and event.keycode == cycle_key:
		mode = (mode + 1) % Mode.size()
	elif event is InputEventMouseButton:
		if event.button_index == orbit_button:
			_orbiting = event.pressed
		elif event.button_index == MOUSE_BUTTON_WHEEL_UP:
			distance = clampf(distance - zoom_step, min_distance, max_distance)
		elif event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			distance = clampf(distance + zoom_step, min_distance, max_distance)
	elif event is InputEventMouseMotion and _orbiting:
		_yaw -= event.relative.x * orbit_sensitivity
		_pitch = clampf(_pitch - event.relative.y * orbit_sensitivity, deg_to_rad(-5.0), deg_to_rad(85.0))


func _process(dt: float) -> void:
	if _target == null:
		_target = _resolve_target()
		return
	var t := _target.global_position
	match mode:
		Mode.CHASE:
			global_position = global_position.lerp(t + _orbit_offset(), clampf(chase_smoothing * dt, 0.0, 1.0))
			look_at(t, Vector3.UP)
		Mode.ORBIT:
			global_position = t + _orbit_offset()
			look_at(t, Vector3.UP)
		Mode.GROUND:
			if allow_ground_move and enable_input:
				_walk_ground(dt)
			global_position = _ground_pos
			if global_position.distance_to(t) > 0.5:
				look_at(t, Vector3.UP)


## Camera offset from the target for the given azimuth (yaw), elevation (pitch),
## and distance.
func _orbit_offset() -> Vector3:
	var cp := cos(_pitch)
	return Vector3(sin(_yaw) * cp, sin(_pitch), cos(_yaw) * cp) * distance


func _walk_ground(dt: float) -> void:
	var input := Vector3.ZERO
	if Input.is_physical_key_pressed(KEY_W): input.z -= 1.0
	if Input.is_physical_key_pressed(KEY_S): input.z += 1.0
	if Input.is_physical_key_pressed(KEY_A): input.x -= 1.0
	if Input.is_physical_key_pressed(KEY_D): input.x += 1.0
	if input == Vector3.ZERO:
		return
	# Move on the horizontal plane relative to the viewing direction.
	var fwd := _target.global_position - _ground_pos
	fwd.y = 0.0
	fwd = fwd.normalized() if fwd.length() > 0.01 else Vector3.FORWARD
	var right := fwd.cross(Vector3.UP).normalized()
	_ground_pos += (right * input.x - fwd * input.z) * ground_move_speed * dt
	_ground_pos.y = _ground_height(_ground_pos) + ground_eye_height


func _ground_height(p: Vector3) -> float:
	if not is_inside_tree():
		return 0.0
	var space := get_world_3d().direct_space_state
	if space == null:
		return 0.0
	var hit := space.intersect_ray(PhysicsRayQueryParameters3D.create(
			Vector3(p.x, p.y + 1000.0, p.z), Vector3(p.x, p.y - 2000.0, p.z), ground_mask))
	return hit.position.y if not hit.is_empty() else 0.0
