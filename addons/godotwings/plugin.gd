## GodotWings SITL — editor plugin entry point.
##
## The library's nodes/resources are exposed via `class_name` (GWFlightBody,
## GWSITLBridge, GWAircraftConfig, ...), so they're usable whether or not this
## plugin is enabled. Enabling it just marks the addon as active and is the
## standard packaging convention; no editor UI is registered here yet.
@tool
extends EditorPlugin
