## Direct manual-control command source — fly a GodotWings vehicle straight from
## the keyboard or a joypad / USB RC controller, with NO SITL in the loop.
##
## It is a drop-in alternative to GWSITLBridge: it implements the same command
## contract (has_command / take_command / post_state), so GWVehicleBody drives the
## FDM through it unchanged. Each physics tick it synthesises a 16-channel PWM
## command from the current stick/key state, in ArduPilot's AETR layout
## (1=aileron, 2=elevator, 3=throttle, 4=rudder by default).
##
## The mapping is deliberately vehicle-agnostic and unstabilised: the fixed-wing
## FDM reads these as direct control-surface + throttle deflections (a flyable RC
## "manual" mode), while a multirotor reads channels 1-4 as RAW per-motor servos
## exactly as it would from SITL — hand-flying a quad means layering your own
## mixer / flight-mode sim on top of these raw channels.
##
## Enable it by setting `control_source = MANUAL` on the vehicle (GWAircraft /
## GWMulticopter auto-add one), or drop a GWManualInput under any GWVehicleBody.
class_name GWManualInput
extends Node

## Default input actions, auto-registered (only if absent) so the node works out
## of the box yet stays rebindable in Project Settings > Input Map. Keyboard uses
## physical keycodes (layout-independent). Joypad axes follow RC "mode 2":
## right stick = pitch/roll, left stick = throttle/yaw.
const _ACTION_DEFS := {
	# action: [ [physical_keycode, ...], [ [joy_axis, axis_value], ... ] ]
	"gw_roll_right":    [[KEY_RIGHT], [[JOY_AXIS_RIGHT_X, 1.0]]],
	"gw_roll_left":     [[KEY_LEFT],  [[JOY_AXIS_RIGHT_X, -1.0]]],
	"gw_pitch_up":      [[KEY_UP],    [[JOY_AXIS_RIGHT_Y, -1.0]]],
	"gw_pitch_down":    [[KEY_DOWN],  [[JOY_AXIS_RIGHT_Y, 1.0]]],
	"gw_yaw_right":     [[KEY_D],     [[JOY_AXIS_LEFT_X, 1.0]]],
	"gw_yaw_left":      [[KEY_A],     [[JOY_AXIS_LEFT_X, -1.0]]],
	"gw_throttle_up":   [[KEY_W],     [[JOY_AXIS_LEFT_Y, -1.0]]],
	"gw_throttle_down": [[KEY_S],     [[JOY_AXIS_LEFT_Y, 1.0]]],
	"gw_reset":         [[KEY_R],     []],
}

## Turn the whole source off (vehicle then receives no commands, FDM idles).
@export var enabled: bool = true
## Auto-create the default input actions above if they don't already exist. Turn
## off if you define your own gw_* actions in Project Settings and don't want the
## node touching the InputMap.
@export var auto_register_actions: bool = true

@export_group("Channel mapping")
## 1-based channels the four axes drive (AETR by default). Other channels rest at
## neutral (1500 µs) — bind a GWChannelSwitch to them as usual for aux functions.
@export_range(1, 16) var roll_channel: int = 1
@export_range(1, 16) var pitch_channel: int = 2
@export_range(1, 16) var throttle_channel: int = 3
@export_range(1, 16) var yaw_channel: int = 4

@export_group("Feel")
## Flip a control if it responds backwards for your airframe / stick.
@export var invert_roll: bool = false
@export var invert_pitch: bool = false
@export var invert_yaw: bool = false
## Throttle is sticky: up/down ramp it and it holds (suits both keyboard and a
## self-centering gamepad stick). This is the change in throttle per second at
## full deflection (0..1 range, so 0.5 = ~2 s stick-to-full).
@export_range(0.0, 5.0) var throttle_ramp: float = 0.5
## Ignore axis deflection below this (also the registered action deadzone).
@export_range(0.0, 0.5) var deadzone: float = 0.1

# --- live input state (sampled each frame) -----------------------------------
var _roll := 0.0
var _pitch := 0.0
var _yaw := 0.0
var _throttle := 0.0
var _reset_pending := false
var _reset_held := false


func _ready() -> void:
	if Engine.is_editor_hint():
		set_process(false)
		return
	if auto_register_actions:
		_register_default_actions()


func _process(delta: float) -> void:
	_sample(delta)


# -----------------------------------------------------------------------------
# Command-source contract (mirrors GWSITLBridge) — GWVehicleBody calls these.
# -----------------------------------------------------------------------------

## Manual control is not lockstepped: a fresh command is always available, so the
## FDM advances every host physics tick (real time).
func has_command() -> bool:
	return enabled


## Build the current 16-channel command. Same shape as GWSITLBridge.take_command().
func take_command() -> Dictionary:
	var cmd := _build_command(_roll, _pitch, _throttle, _yaw)
	cmd["reset"] = _reset_pending
	_reset_pending = false
	return cmd


## No autopilot to reply to — nothing to do with the produced state.
func post_state(_state: Dictionary) -> void:
	pass


# -----------------------------------------------------------------------------
# Input sampling + command construction.
# -----------------------------------------------------------------------------

## Read the current stick/key state into the live axes. Throttle is integrated
## (sticky); roll/pitch/yaw are instantaneous. Split out from _process so tests
## can drive it deterministically after Input.action_press().
func _sample(delta: float) -> void:
	_roll = _axis("gw_roll_left", "gw_roll_right", invert_roll)
	_pitch = _axis("gw_pitch_down", "gw_pitch_up", invert_pitch)
	_yaw = _axis("gw_yaw_left", "gw_yaw_right", invert_yaw)
	var thr_in := Input.get_action_strength("gw_throttle_up") \
			- Input.get_action_strength("gw_throttle_down")
	_throttle = clampf(_throttle + thr_in * throttle_ramp * delta, 0.0, 1.0)
	# Rising-edge reset (tracked ourselves so it's frame-independent / testable).
	var reset_now := Input.is_action_pressed("gw_reset")
	if reset_now and not _reset_held:
		_reset_pending = true
	_reset_held = reset_now


## Signed axis from a low/high action pair, deadzoned and optionally inverted.
func _axis(neg: String, pos: String, invert: bool) -> float:
	var v := Input.get_action_strength(pos) - Input.get_action_strength(neg)
	if absf(v) < deadzone:
		v = 0.0
	return clampf(-v if invert else v, -1.0, 1.0)


## Assemble the command dict from normalised axes (roll/pitch/yaw in -1..1,
## throttle in 0..1). Pure — no Input access — so it's directly unit-testable.
func _build_command(roll: float, pitch: float, throttle: float, yaw: float) -> Dictionary:
	var pwm := PackedInt32Array()
	pwm.resize(16)
	pwm.fill(1500)  # neutral; aux channels stay centred
	pwm[roll_channel - 1] = _surface_pwm(roll)
	pwm[pitch_channel - 1] = _surface_pwm(pitch)
	pwm[yaw_channel - 1] = _surface_pwm(yaw)
	pwm[throttle_channel - 1] = _throttle_pwm(throttle)
	return {
		"pwm": pwm,
		"aileron": clampf(roll, -1.0, 1.0),
		"elevator": clampf(pitch, -1.0, 1.0),
		"throttle": clampf(throttle, 0.0, 1.0),
		"rudder": clampf(yaw, -1.0, 1.0),
		"frame_count": 0,
		"reset": false,
	}


## Control-surface channel: -1..1 -> 1000..2000 µs, centred at 1500.
func _surface_pwm(v: float) -> int:
	return int(roundf(1500.0 + clampf(v, -1.0, 1.0) * 500.0))


## Throttle channel: 0..1 -> 1000..2000 µs.
func _throttle_pwm(t: float) -> int:
	return int(roundf(1000.0 + clampf(t, 0.0, 1.0) * 1000.0))


## Create the default gw_* actions (and their key/joypad events) if they are not
## already defined, so a freshly-dropped node is immediately controllable.
func _register_default_actions() -> void:
	for action in _ACTION_DEFS:
		if InputMap.has_action(action):
			continue
		InputMap.add_action(action, deadzone)
		var defs: Array = _ACTION_DEFS[action]
		for keycode in defs[0]:
			var ke := InputEventKey.new()
			ke.physical_keycode = keycode
			InputMap.action_add_event(action, ke)
		for joy in defs[1]:
			var je := InputEventJoypadMotion.new()
			je.axis = joy[0]
			je.axis_value = joy[1]
			InputMap.action_add_event(action, je)
