#!/usr/bin/env python3
"""Minimal ArduPilot JSON-SITL emulator for testing the GodotWings bridge.

Sends the binary PWM packet ArduPilot would send (magic 18458, little-endian)
to UDP 127.0.0.1:9002 at ~50 Hz and prints the JSON state Godot replies with.

This is a test fixture, NOT a flight controller — it just holds fixed stick
inputs so you can confirm the round trip and watch the placeholder aircraft move.

    python3 tools/fake_ardupilot.py                 # 70% throttle, neutral elevator (climbs out)
    python3 tools/fake_ardupilot.py --throttle 0    # idle
    python3 tools/fake_ardupilot.py --selfcheck     # offline: pack+unpack a packet, no Godot
"""
import argparse
import json
import socket
import struct
import sys
import time

MAGIC = 18458  # ArduPilot JSON servo-packet magic (0x481A), verified on the wire
PORT = 9002
# uint16 magic, uint16 frame_rate, uint32 frame_count, uint16 pwm[16]
PACKET_FMT = "<HHI16H"
PACKET_SIZE = struct.calcsize(PACKET_FMT)  # 40 bytes


def build_packet(frame_count: int, frame_rate: int, pwm: list[int]) -> bytes:
    assert len(pwm) == 16
    return struct.pack(PACKET_FMT, MAGIC, frame_rate, frame_count, *pwm)


def parse_packet(data: bytes) -> dict:
    fields = struct.unpack(PACKET_FMT, data)
    return {"magic": fields[0], "frame_rate": fields[1],
            "frame_count": fields[2], "pwm": list(fields[3:])}


def selfcheck() -> int:
    pwm = [1500, 1500, 2000, 1500] + [1500] * 12
    pkt = build_packet(7, 50, pwm)
    print(f"packet size = {len(pkt)} bytes (expected {PACKET_SIZE})")
    back = parse_packet(pkt)
    ok = back["magic"] == MAGIC and back["frame_count"] == 7 and back["pwm"] == pwm
    print("round-trip:", json.dumps(back))
    print("OK" if ok else "FAILED")
    return 0 if ok else 1


def _pwm_for(throttle, elevator, aileron, rudder):
    pwm = [1500] * 16
    pwm[0] = int(1500 + aileron * 500)
    pwm[1] = int(1500 + elevator * 500)
    pwm[2] = int(1000 + throttle * 1000)
    pwm[3] = int(1500 + rudder * 500)
    return pwm


def run(host: str, throttle: float, elevator: float, aileron: float,
        rudder: float, rate: int, phase2_after: float = 0.0,
        p2_throttle: float = 0.0, p2_elevator: float = -1.0) -> int:
    # Normalized [-1..1] / [0..1] -> PWM microseconds.
    pwm = _pwm_for(throttle, elevator, aileron, rudder)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2.0)
    dest = (host, PORT)
    print(f"sending {rate} Hz PWM to {dest}: "
          f"ail={pwm[0]} ele={pwm[1]} thr={pwm[2]} rud={pwm[3]}  (Ctrl-C to stop)")

    period = 1.0 / rate
    frame = 0
    last_print = 0.0
    timeouts = 0
    start = time.time()
    phase2 = False
    try:
        while True:
            if phase2_after > 0 and not phase2 and time.time() - start >= phase2_after:
                phase2 = True
                pwm = _pwm_for(p2_throttle, p2_elevator, aileron, rudder)
                print("--- phase 2: throttle=%.2f elevator=%.2f (dive) ---" % (p2_throttle, p2_elevator))
            sock.sendto(build_packet(frame, rate, pwm), dest)
            try:
                data, _ = sock.recvfrom(65535)
                timeouts = 0
                now = time.time()
                if now - last_print >= 0.5:
                    last_print = now
                    state = json.loads(data.decode("utf-8").strip())
                    pos = state.get("position", [0, 0, 0])
                    att = state.get("attitude", [0, 0, 0])
                    print(f"frame {frame:6d}  "
                          f"NED=({pos[0]:7.1f},{pos[1]:7.1f},{pos[2]:7.1f})  "
                          f"rpy=({att[0]:5.2f},{att[1]:5.2f},{att[2]:5.2f})  "
                          f"V={state.get('airspeed', 0):5.1f}")
            except socket.timeout:
                timeouts += 1
                if timeouts in (1, 3):
                    print("...no reply (is Godot running with Main.tscn?)")
            frame += 1
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nstopped.")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--throttle", type=float, default=0.7)
    # +elevator = nose up (matches ArduPilot SIM_Plane, c_m_deltae > 0). Neutral
    # climbs out on excess thrust; use a small positive value to rotate sooner.
    ap.add_argument("--elevator", type=float, default=0.0)
    ap.add_argument("--aileron", type=float, default=0.0)
    ap.add_argument("--rudder", type=float, default=0.0)
    ap.add_argument("--rate", type=int, default=50)
    ap.add_argument("--phase2-after", type=float, default=0.0,
                    help="after N seconds, switch to phase-2 controls (e.g. a dive)")
    ap.add_argument("--p2-throttle", type=float, default=0.0)
    ap.add_argument("--p2-elevator", type=float, default=-1.0)
    ap.add_argument("--selfcheck", action="store_true",
                    help="offline pack/unpack test, no network or Godot needed")
    args = ap.parse_args()

    if args.selfcheck:
        return selfcheck()
    return run(args.host, args.throttle, args.elevator, args.aileron,
               args.rudder, args.rate, args.phase2_after,
               args.p2_throttle, args.p2_elevator)


if __name__ == "__main__":
    sys.exit(main())
