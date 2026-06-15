#!/usr/bin/env python3
"""Reference GodotWings camera client: H.264 video in, MAVLink commands out.

Closes the loop the library is built for:

    GodotWings (GWCamera) --RTP/H.264--> OpenCV --(detect)--> pymavlink --> ArduPilot SITL
                          --metadata UDP-> pose/sim_time correlation

This is intentionally minimal and dependency-light. It is NOT imported by the
addon; copy it into your companion project and replace the detect() stub with
your real CV.

Run GodotWings first (it must be listening so ArduPilot's lockstep can start),
then:

    pip install opencv-python pymavlink numpy
    python gw_camera_client.py --video user://godotwings_cam.sdp \
        --mavlink udp:127.0.0.1:14550

Video source options (match GWCamera's `protocol` export):
  * RTP_H264    -> pass the .sdp path GWCamera prints on startup.
                   (Set env OPENCV_FFMPEG_CAPTURE_OPTIONS as below for RTP.)
  * MPEGTS_H264 -> pass  udp://127.0.0.1:5600   (no SDP needed; simplest).
  * RTSP        -> pass  rtsp://127.0.0.1:8554/godotwings
"""

import argparse
import json
import os
import socket
import sys

# RTP over UDP needs these protocols whitelisted in the ffmpeg backend.
os.environ.setdefault(
    "OPENCV_FFMPEG_CAPTURE_OPTIONS", "protocol_whitelist;file,rtp,udp"
)

import numpy as np  # noqa: E402
import cv2  # noqa: E402


def open_video(src: str) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(src, cv2.CAP_FFMPEG)
    if not cap.isOpened():
        sys.exit(f"Could not open video source: {src!r}")
    return cap


def detect(frame: np.ndarray):
    """Replace with real CV. Returns (cx, cy) pixel target or None.

    Stub: find the largest bright blob as a stand-in 'target'."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, 220, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < 25:
        return None
    m = cv2.moments(c)
    if m["m00"] == 0:
        return None
    return int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"])


def make_metadata_socket(port: int) -> socket.socket:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", port))
    s.setblocking(False)
    return s


def latest_metadata(sock: socket.socket):
    """Drain the metadata socket, return the freshest packet (or None)."""
    latest = None
    while True:
        try:
            data, _ = sock.recvfrom(4096)
            latest = json.loads(data.decode("utf-8"))
        except (BlockingIOError, ValueError):
            break
    return latest


def send_landing_target(mav, cx, cy, w, h, fov_deg, distance=10.0):
    """Send a LANDING_TARGET in body angular offsets, from a pixel detection."""
    import math

    fov = math.radians(fov_deg)
    f = (w / 2) / math.tan(fov / 2)  # focal length in pixels (horizontal)
    angle_x = math.atan2(cx - w / 2, f)
    angle_y = math.atan2(cy - h / 2, f)
    mav.mav.landing_target_send(
        0, 0, 0,  # time_usec, target_num, frame
        angle_x, angle_y, distance,
        0.0, 0.0,  # size_x, size_y
    )


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", required=True,
                    help="SDP path, udp://host:port, or rtsp://...")
    ap.add_argument("--meta-port", type=int, default=5601)
    ap.add_argument("--mavlink", default="",
                    help="pymavlink connection string, e.g. udp:127.0.0.1:14550. "
                         "Empty disables the command path.")
    ap.add_argument("--show", action="store_true", help="cv2.imshow the feed.")
    args = ap.parse_args()

    cap = open_video(args.video)
    meta_sock = make_metadata_socket(args.meta_port)

    mav = None
    if args.mavlink:
        from pymavlink import mavutil
        mav = mavutil.mavlink_connection(args.mavlink)
        print("Waiting for MAVLink heartbeat...")
        mav.wait_heartbeat()
        print(f"Heartbeat from system {mav.target_system}.")

    print("Streaming. Ctrl-C to stop.")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            meta = latest_metadata(meta_sock)
            target = detect(frame)

            if target is not None:
                cx, cy = target
                if args.show:
                    cv2.circle(frame, (cx, cy), 8, (0, 0, 255), 2)
                if mav is not None:
                    h, w = frame.shape[:2]
                    fov = meta.get("fov_deg", 70.0) if meta else 70.0
                    send_landing_target(mav, cx, cy, w, h, fov)

            if meta and target is not None:
                print(f"frame {meta['frame_id']} t={meta['sim_time']:.2f}s "
                      f"pos_ned={meta['pos_ned']} target_px={target}")

            if args.show:
                cv2.imshow("GodotWings", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        if args.show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
