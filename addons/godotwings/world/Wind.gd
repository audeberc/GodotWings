@tool
class_name GWWind
extends Node3D

## Atmospheric wind for the flight model. Drop ONE in the world and every
## GWFlightBody / GWAircraft auto-finds it — the wind shifts the airspeed the aero
## sees (so the aircraft crabs, drifts and gets buffeted) and is reported to
## ArduPilot's windvane.
##
## Convention: `wind_from_deg` is the compass bearing the wind blows FROM (0 = N,
## 90 = E), like a METAR. Internally that becomes the air's velocity in NED. Three
## optional layers: mean wind, altitude shear, and turbulence (see the exports).

## Mean wind speed (m/s).
@export var wind_speed: float = 0.0:
	set(v): wind_speed = maxf(v, 0.0)
## Direction the wind blows FROM (deg, compass: 0 = N, 90 = E, 180 = S, 270 = W).
@export_range(0.0, 360.0) var wind_from_deg: float = 0.0

@export_group("Altitude shear")
## Power-law gradient: speed scales as (height / reference_height) ^ exponent.
## 0 = uniform with altitude; ~0.14 ≈ a neutral atmospheric boundary layer.
@export var shear_exponent: float = 0.0
## Height (m, AGL≈MSL here) at which `wind_speed` applies.
@export var reference_height: float = 10.0

@export_group("Turbulence")
## RMS gust speed (m/s) added on top of the mean, per axis. 0 = smooth air.
## Rough feel: 1-3 light, 4-6 moderate, 7+ severe. Synthesised as a sum of
## sinusoids across [gust_low_hz, gust_high_hz] — cheap, believable, and
## reproducible (seeded). Not a spectral Dryden model; good enough for
## disturbance-rejection testing.
@export var turbulence: float = 0.0:
	set(v): turbulence = maxf(v, 0.0)
## Slowest gust component (Hz) — long swells.
@export var gust_low_hz: float = 0.1
## Fastest gust component (Hz) — sharp jitter.
@export var gust_high_hz: float = 2.0
## Seed for reproducible gusts (tests / replays).
@export var seed: int = 12345

const _MODES := 6
var _w := PackedFloat32Array()       # angular frequencies (rad/s)
var _phase_n := PackedFloat32Array()
var _phase_e := PackedFloat32Array()
var _phase_d := PackedFloat32Array()
var _amp := 0.0                      # per-mode amplitude so each axis has RMS 1


func _ready() -> void:
	_build_modes()


func _build_modes() -> void:
	var rng := RandomNumberGenerator.new()
	rng.seed = seed
	_w.resize(_MODES); _phase_n.resize(_MODES); _phase_e.resize(_MODES); _phase_d.resize(_MODES)
	var lo := maxf(gust_low_hz, 0.001)
	var hi := maxf(gust_high_hz, lo * 1.01)
	for k in _MODES:
		var frac := float(k) / float(_MODES - 1)
		_w[k] = TAU * (lo * pow(hi / lo, frac))   # log-spaced over the gust band
		_phase_n[k] = rng.randf() * TAU
		_phase_e[k] = rng.randf() * TAU
		_phase_d[k] = rng.randf() * TAU
	# Sum of M equal sinusoids has RMS = amp·sqrt(M/2); pick amp so each axis ~ RMS 1.
	_amp = sqrt(2.0 / float(_MODES))


## Mean air velocity (NED, m/s), ignoring shear and turbulence.
func mean_wind_ned() -> Vector3:
	var f := deg_to_rad(wind_from_deg)
	# Blows FROM f, so the air moves toward f+180.
	return wind_speed * Vector3(-cos(f), -sin(f), 0.0)


## Instantaneous air velocity (NED, m/s) at a position and sim-time.
func sample(pos_ned: Vector3, t: float) -> Vector3:
	var w := mean_wind_ned()
	if shear_exponent > 0.0 and reference_height > 0.0:
		var h := maxf(-pos_ned.z, 0.1)   # height above MSL
		w *= pow(h / reference_height, shear_exponent)
	if turbulence > 0.0:
		if _w.is_empty():
			_build_modes()
		var g := Vector3.ZERO
		for k in _w.size():
			g.x += sin(_w[k] * t + _phase_n[k])
			g.y += sin(_w[k] * t + _phase_e[k])
			g.z += sin(_w[k] * t + _phase_d[k])
		w += g * (_amp * turbulence)
	return w


## ArduPilot windvane report: bearing the MEAN wind blows FROM (rad) + speed (m/s).
func windvane() -> Dictionary:
	return {"direction": deg_to_rad(wind_from_deg), "speed": wind_speed}
