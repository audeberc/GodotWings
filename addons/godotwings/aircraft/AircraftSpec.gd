## Physical aircraft description that derives a full GWAircraftConfig from real,
## measurable quantities — no invented "feel" knobs.
##
## SIM_Plane's moment coefficients are angular accelerations (an aero derivative
## divided by a moment of inertia; see GWMomentModel), so the one physical input
## beyond geometry the dynamics need is INERTIA — auto-estimated from mass+geometry
## here, or set explicitly. Calibration uses the validated Skywalker as the
## reference: its geometry/mass reproduces Skywalker, and changing mass, size,
## inertia, or static margin scales the coefficients by the correct relations.
## For total control, assign a GWAircraftConfig directly, or tick `bake_to_config`
## to write this spec's config out as a .tres and hand-tune it.
@tool
class_name GWAircraftSpec
extends Resource

# Validated baseline = the shipped Skywalker (its coefficients are the reference
# real aero derivatives, before dividing by inertia).
const BASE_MASS := 2.0
const BASE_B := 1.88
const BASE_S := 0.45
const SM_REF := 0.10       ## reference static margin the baseline c_m_a corresponds to
const RHO_SL := 1.225      ## sea-level air density, kg/m^3
const G := 9.80665

@export_group("Geometry & mass")
@export var mass: float = 2.0          ## kg
@export var wingspan: float = 1.88     ## m, tip to tip
@export var wing_area: float = 0.45    ## m^2 (planform)

@export_group("Performance")
## Intended level-cruise airspeed (m/s). Sets zero-alpha lift so it trims here.
@export var cruise_speed: float = 18.0
## Maximum (full-throttle) static thrust, Newtons. Weight is mass*9.81 for reference.
@export var max_thrust_n: float = 29.4

@export_group("Stability & aero")
## Static margin as a fraction of mean chord (distance CG->neutral point / MAC).
## Positive = pitch-stable. Typical 0.05-0.20; higher = stiffer in pitch.
@export var static_margin: float = 0.10
## Zero-lift (parasitic) drag coefficient Cd0: ~0.03 clean, ~0.06 typical, ~0.1 draggy.
@export var parasitic_drag: float = 0.06
@export var stall_angle_deg: float = 27.0

@export_group("Inertia (kg·m²)")
## Leave at 0 to auto-estimate from mass + geometry (radius-of-gyration model).
## Set explicitly if you have a CAD/measured inertia tensor.
@export var Ixx: float = 0.0   ## roll
@export var Iyy: float = 0.0   ## pitch
@export var Izz: float = 0.0   ## yaw

@export_group("Advanced")
## Aerodynamic-center offset from CG (m, FRD). Negative x = AC behind CG.
@export var cg_offset: Vector3 = Vector3(-0.15, 0.0, -0.05)

@export_group("Editor")
## Where bake_to_config writes the derived GWAircraftConfig.
@export_file("*.tres") var bake_path: String = ""
## Tick in the editor to save build() to `bake_path` (or next to this resource)
## as a full GWAircraftConfig you can then open and hand-tune.
@export var bake_to_config: bool = false:
	set(v):
		bake_to_config = false
		if v and Engine.is_editor_hint():
			_bake()


func _bake() -> void:
	var path := bake_path
	if path == "":
		path = (resource_path.get_basename() + "_config.tres") if resource_path != "" \
				else "res://aircraft_config.tres"
	var err := ResourceSaver.save(build(), path)
	if err == OK:
		print("GWAircraftSpec: baked GWAircraftConfig -> ", path)
	else:
		push_error("GWAircraftSpec: bake failed (err %d) writing %s" % [err, path])


## Crude but consistent inertia estimate (kg·m²) from mass and span.
## Roll uses semi-span, pitch uses an estimated fuselage half-length, yaw is the
## planar sum (perpendicular-axis approximation). Used for BOTH the baseline and
## the target, so only the RATIO matters — absolute accuracy is not required.
static func _estimate_inertia(m: float, b: float) -> Vector3:
	var length := 0.7 * b                  # fuselage length ~ 0.7 * span (conventional)
	var r_roll := 0.30 * (b * 0.5)         # roll: mass spread along the span
	var r_pitch := 0.30 * (length * 0.5)   # pitch: mass spread fore-aft
	var ixx := m * r_roll * r_roll
	var iyy := m * r_pitch * r_pitch
	return Vector3(ixx, iyy, ixx + iyy)    # Izz ~ Ixx + Iyy for a roughly planar body


static func _lift_slope(ar: float) -> float:
	var base_ar := (BASE_B * BASE_B) / BASE_S
	var s := TAU * ar / (ar + 2.0)
	var s_base := TAU * base_ar / (base_ar + 2.0)
	return clampf(6.9 * s / s_base, 3.0, 7.5)


## Derive a full GWAircraftConfig from this spec.
func build() -> GWAircraftConfig:
	var cfg := GWAircraftConfig.new()

	# --- Geometry / mass (direct) ---
	cfg.mass = mass
	cfg.b = wingspan
	cfg.s = wing_area
	cfg.c = wing_area / maxf(wingspan, 0.01)
	var ar := (wingspan * wingspan) / maxf(wing_area, 0.001)

	# --- Lift ---
	cfg.c_lift_a = _lift_slope(ar)
	cfg.oswald = 0.9
	cfg.alpha_stall = deg_to_rad(stall_angle_deg)
	var cl_cruise := (mass * G) / (0.5 * RHO_SL * cruise_speed * cruise_speed * maxf(wing_area, 0.001))
	cfg.c_lift_0 = clampf(cl_cruise, 0.05, 0.9)

	# --- Drag ---
	cfg.c_drag_p = parasitic_drag

	# --- Thrust: full-throttle thrust = max_thrust_n. thrust_scale = m*g/hover_throttle. ---
	cfg.hover_throttle = clampf((mass * G) / maxf(max_thrust_n, 0.1), 0.05, 5.0)

	# --- Moments: real derivative / inertia, calibrated to the validated baseline. ---
	# Each baseline coeff IS a real derivative / baseline-inertia, so scaling by
	# (base_I / I) recovers the real derivative and re-divides by this aircraft's I.
	var base_i := _estimate_inertia(BASE_MASS, BASE_B)
	var this_i := _estimate_inertia(mass, wingspan)
	if Ixx > 0.0: this_i.x = Ixx
	if Iyy > 0.0: this_i.y = Iyy
	if Izz > 0.0: this_i.z = Izz
	var roll_r := base_i.x / this_i.x
	var pitch_r := base_i.y / this_i.y
	var yaw_r := base_i.z / this_i.z
	# Static margin scales pitch stiffness linearly (Cm_a ∝ -CL_a·SM).
	var sm_r := static_margin / SM_REF

	# Roll.
	cfg.c_l_p = -1.0 * roll_r
	cfg.c_l_b = -0.12 * roll_r
	cfg.c_l_r = 0.14 * roll_r
	cfg.c_l_deltaa = 0.25 * roll_r
	cfg.c_l_deltar = -0.037 * roll_r

	# Pitch (stiffness also scaled by static margin).
	cfg.c_m_0 = 0.045 * pitch_r
	cfg.c_m_a = -0.7 * pitch_r * sm_r
	cfg.c_m_q = -20.0 * pitch_r
	cfg.c_m_deltae = 1.0 * pitch_r

	# Yaw.
	cfg.c_n_b = 0.25 * yaw_r
	cfg.c_n_p = 0.022 * yaw_r
	cfg.c_n_r = -1.0 * yaw_r
	cfg.c_n_deltar = 0.1 * yaw_r

	# Side force is a force, not an angular accel: no inertia scaling.
	cfg.c_y_b = -0.98
	cfg.c_y_deltar = -0.2

	cfg.cg_offset = cg_offset
	return cfg


# --- Preset shelf (also available as .tres in aircraft/presets/) ------------

## Docile high-wing trainer: stable, modest power.
static func trainer() -> GWAircraftSpec:
	var s := GWAircraftSpec.new()
	s.mass = 2.0; s.wingspan = 1.88; s.wing_area = 0.45
	s.cruise_speed = 16.0; s.max_thrust_n = 17.6
	s.static_margin = 0.15; s.parasitic_drag = 0.08
	return s

## Light foam park flyer: small, light, draggy.
static func foamie() -> GWAircraftSpec:
	var s := GWAircraftSpec.new()
	s.mass = 0.9; s.wingspan = 1.2; s.wing_area = 0.24
	s.cruise_speed = 13.0; s.max_thrust_n = 11.5
	s.static_margin = 0.10; s.parasitic_drag = 0.1
	return s

## FPV flying wing: fast, slick, low static margin.
static func fpv_wing() -> GWAircraftSpec:
	var s := GWAircraftSpec.new()
	s.mass = 1.4; s.wingspan = 1.0; s.wing_area = 0.22
	s.cruise_speed = 22.0; s.max_thrust_n = 24.7
	s.static_margin = 0.07; s.parasitic_drag = 0.04
	s.stall_angle_deg = 22.0
	return s
