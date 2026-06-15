## Atmosphere: air density vs altitude.
##
## Owned by GWFlightBody. Density uses the ISA exponential approximation. Wind
## lives in a separate drop-in GWWind node (see GWWind.gd) so it's configurable
## from the scene and shared across aircraft.
class_name GWAtmosphereModel
extends RefCounted

const RHO_SEA_LEVEL := 1.225   ## kg/m^3
const SCALE_HEIGHT := 8500.0   ## m


## Air density at the given altitude above sea level (m).
func density(altitude: float) -> float:
	return RHO_SEA_LEVEL * exp(-maxf(altitude, 0.0) / SCALE_HEIGHT)
