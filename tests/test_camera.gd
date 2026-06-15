extends SceneTree

# Headless smoke test for GWCamera. Verifies the script parses, the class
# registers, the node sets up its SubViewport/Camera/TCP server, auto-resolves
# the GWFlightBody, and tears down cleanly. Rendering + get_image() cannot run
# under --headless (dummy rasterizer), so frame-grab is not exercised here.

func _initialize() -> void:
	var ok := true

	var body := GWFlightBody.new()
	body.config = load("res://addons/godotwings/aircraft/Skywalker.tres")
	get_root().add_child(body)

	var cam := GWCamera.new()
	cam.launch_ffmpeg = false       # no ffmpeg on CI; just host the raw TCP server
	cam.raw_tcp_port = 5599
	cam.metadata_port = 5602
	cam.fps = 10.0
	body.add_child(cam)

	# Let _ready() + a couple of _process ticks run.
	await process_frame
	await process_frame

	if cam._viewport == null or cam._cam == null:
		push_error("FAIL: viewport/camera not created"); ok = false
	if cam._body != body:
		push_error("FAIL: GWFlightBody not auto-resolved"); ok = false
	if not cam._server.is_listening():
		push_error("FAIL: raw-frame TCP server not listening"); ok = false

	# A client can connect to the raw-frame server (stands in for ffmpeg).
	var client := StreamPeerTCP.new()
	if client.connect_to_host("127.0.0.1", 5599) != OK:
		push_error("FAIL: could not dial raw TCP server"); ok = false
	await process_frame
	await process_frame
	cam._process(0.0)  # let the server accept
	if cam._peer == null:
		push_error("FAIL: server did not accept frame consumer"); ok = false

	cam.queue_free()
	body.queue_free()
	await process_frame

	print("test_camera: ", "PASS" if ok else "FAIL")
	quit(0 if ok else 1)
