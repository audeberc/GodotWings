## Conversions between ArduPilot's aerospace frames and Godot's render frame.
##
## DESIGN: the flight dynamics are integrated NATIVELY in NED / body-FRD (the
## frames ArduPilot's JSON protocol and the aero coefficients are defined in).
## That keeps the SITL state output conversion-free and avoids frame-handedness
## bugs. These helpers exist only to (a) extract Euler attitude from the
## body->NED DCM for the JSON packet, and (b) map state into Godot's world for
## rendering.
##
## Frames:
##   NED   : x=North, y=East, z=Down            (right-handed)
##   FRD   : x=forward(nose), y=right, z=down    (right-handed, aircraft body)
##   Godot : x=East, y=Up, z=South (North=-z)    (right-handed render world)
##
## The NED<->Godot map Psi(v) = (v.y, -v.z, -v.x) is a proper rotation (det +1).
class_name GWCoordConvert
extends RefCounted

## NED vector -> Godot world vector (proper rotation). Used for position.
static func ned_to_world(n: Vector3) -> Vector3:
	return Vector3(n.y, -n.z, -n.x)

## Godot world vector -> NED vector (inverse of [method ned_to_world]).
static func world_to_ned(w: Vector3) -> Vector3:
	return Vector3(-w.z, w.x, -w.y)

## Extract ArduPilot attitude [roll, pitch, yaw] (rad) from the body->NED DCM.
## `dcm` columns are the body axes expressed in NED (x=fwd, y=right, z=down).
##   roll  > 0 : right wing down,  pitch > 0 : nose up,  yaw : 0=North,+toward East
static func dcm_to_ned_attitude(dcm: Basis) -> Array:
	var fwd := dcm.x
	var right := dcm.y
	var down := dcm.z
	var pitch := asin(clampf(-fwd.z, -1.0, 1.0))
	var yaw := atan2(fwd.y, fwd.x)
	var roll := atan2(right.z, down.z)
	return [roll, pitch, yaw]

## Build a body->NED DCM from Euler roll/pitch/yaw (rad). Columns are the body
## axes (fwd, right, down) in NED. Exact inverse of [method dcm_to_ned_attitude].
static func attitude_to_dcm(roll: float, pitch: float, yaw: float) -> Basis:
	var cf := cos(roll)
	var sf := sin(roll)
	var ct := cos(pitch)
	var st := sin(pitch)
	var cp := cos(yaw)
	var sp := sin(yaw)
	var fwd := Vector3(ct * cp, ct * sp, -st)
	var right := Vector3(sf * st * cp - cf * sp, sf * st * sp + cf * cp, sf * ct)
	var down := Vector3(cf * st * cp + sf * sp, cf * st * sp - sf * cp, cf * ct)
	return Basis(fwd, right, down) # columns: x=fwd, y=right, z=down

## Convert a body->NED DCM into a standard Godot render Basis (nose along -Z,
## up +Y, right +X) so conventional Godot meshes display correctly.
static func dcm_to_render_basis(dcm: Basis) -> Basis:
	var gx := ned_to_world(dcm.y)   # body right  -> Godot +X
	var gy := -ned_to_world(dcm.z)  # body up (= -down) -> Godot +Y
	var gz := -ned_to_world(dcm.x)  # Godot +Z is backward, so -forward
	return Basis(gx, gy, gz)


## Inverse of dcm_to_render_basis: recover the body->NED DCM from a Godot render
## Basis (used to read an externally-simulated pose, e.g. a ragdoll RigidBody,
## back into the NED state). Columns: x=forward, y=right, z=down.
static func render_basis_to_dcm(b: Basis) -> Basis:
	var fwd := -world_to_ned(b.z)   # invert gz = -ned_to_world(dcm.x)
	var right := world_to_ned(b.x)  # invert gx =  ned_to_world(dcm.y)
	var down := -world_to_ned(b.y)  # invert gy = -ned_to_world(dcm.z)
	return Basis(fwd, right, down).orthonormalized()
