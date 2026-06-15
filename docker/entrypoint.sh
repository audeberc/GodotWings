#!/usr/bin/env bash
# Launch ArduPlane SITL wired to GodotWings (JSON physics) and a GCS (MAVLink).
#
# Env vars (all optional, sensible defaults):
#   GODOT_HOST     host running GodotWings' SITL bridge (listens on UDP 9002)
#   GCS_HOST       host running QGroundControl / Mission Planner
#   GCS_PORT       MAVLink UDP port the GCS listens on (default 14550)
#   HOME_LOCATION  spawn lat,lon,alt,heading
#   SPEEDUP        sim speed multiplier (lockstep with Godot pins this to ~1)
#   MAVPROXY_DAEMON  "true" = headless (no console); default "false" = interactive
#                    MAVProxy console (attach with `docker attach godotwings-sitl`)
#   VEHICLE        ArduPlane (default) or ArduCopter — both binaries are built in
#                  the image. Match the Godot side (GWAircraft vs a multirotor).
#   NUM_VEHICLES   how many aircraft to spawn (default 1). Vehicle i uses ArduPilot
#                  instance -I i -> JSON physics on 9002+10*i (match a GWAircraft
#                  with sitl_instance=i) and MAVLink out to GCS_PORT+10*i.
#   PARAM_FILE     extra param file applied on boot (default /sitl-defaults.parm)
#   EXTRA_ARGS     appended verbatim to sim_vehicle.py
set -euo pipefail

GODOT_HOST="${GODOT_HOST:-host.docker.internal}"
GCS_HOST="${GCS_HOST:-host.docker.internal}"
GCS_PORT="${GCS_PORT:-14550}"
HOME_LOCATION="${HOME_LOCATION:--35.363261,149.165230,584,353}"
SPEEDUP="${SPEEDUP:-1}"
MAVPROXY_DAEMON="${MAVPROXY_DAEMON:-false}"
VEHICLE="${VEHICLE:-ArduPlane}"
NUM_VEHICLES="${NUM_VEHICLES:-1}"
PARAM_FILE="${PARAM_FILE:-/sitl-defaults.parm}"
EXTRA_ARGS="${EXTRA_ARGS:-}"

# Resolve a hostname to an IPv4 literal. ArduPilot's JSON frame is `:`-delimited
# (so an IPv6 address would be mis-parsed) and Godot's bridge / most GCS bind
# IPv4 — so force IPv4 (getent ahostsv4) rather than whatever getent hosts
# returns first (Docker's host.docker.internal often resolves to IPv6 too).
resolve_ipv4() {
    local host="$1"
    if [[ "$host" =~ ^[0-9.]+$ ]]; then
        echo "$host"; return
    fi
    local ip
    ip="$(getent ahostsv4 "$host" | awk '{print $1; exit}')" || true
    echo "${ip:-$host}"
}

GODOT_IP="$(resolve_ipv4 "$GODOT_HOST")"
GCS_IP="$(resolve_ipv4 "$GCS_HOST")"

# Build sim_vehicle.py args for ArduPilot instance $1. With -I i, ArduPilot offsets
# the JSON physics port to 9002+10*i (matching a GWAircraft with sitl_instance=i);
# we route that vehicle's MAVLink to GCS_PORT+10*i.
build_args() {
    local i="$1"
    local pfile="$2"
    local gcs_port=$((GCS_PORT + 10 * i))
    SIM_ARGS=(-v "${VEHICLE}" -I "${i}" -f "JSON:${GODOT_IP}" --no-rebuild \
        --speedup "${SPEEDUP}" -l "${HOME_LOCATION}" --out "udpout:${GCS_IP}:${gcs_port}")
    [ -n "${pfile}" ] && [ -f "${pfile}" ] && SIM_ARGS+=(--add-param-file="${pfile}")
}

echo "=========================================================="
echo " ${VEHICLE} SITL  (${NUM_VEHICLES} vehicle(s))"
echo "   JSON physics backend -> ${GODOT_IP}:9002  (+10 per vehicle)"
echo "   MAVLink GCS out       -> ${GCS_IP}:${GCS_PORT}  (+10 per vehicle)"
echo "   Home                  =  ${HOME_LOCATION}"
echo "   Params                =  ${PARAM_FILE}"
echo "=========================================================="

# Single vehicle: keep the interactive MAVProxy console (attachable).
if [ "${NUM_VEHICLES}" -le 1 ]; then
    build_args 0 "${PARAM_FILE}"
    if [ "${MAVPROXY_DAEMON}" = "true" ]; then
        SIM_ARGS+=(--mavproxy-args="--daemon")
    fi
    exec sim_vehicle.py "${SIM_ARGS[@]}" ${EXTRA_ARGS}
fi

# Multiple vehicles: launch each headless (no shared TTY), with a distinct sysid,
# and wait on them all.
pids=()
cleanup() { kill "${pids[@]}" 2>/dev/null || true; }
trap cleanup EXIT INT TERM
for ((i = 0; i < NUM_VEHICLES; i++)); do
    build_args "${i}" "${PARAM_FILE}"
    # --sysid gives each vehicle a distinct MAVLink system id (set at HAL init, so
    # it actually sticks — unlike a SYSID_THISMAV default, which the GCS otherwise
    # sees as identical and merges into one vehicle).
    SIM_ARGS+=(--sysid "$((i + 1))" --mavproxy-args="--daemon")
    echo "   vehicle ${i}: sysid $((i + 1))  JSON ${GODOT_IP}:$((9002 + 10 * i))  ->  GCS ${GCS_IP}:$((GCS_PORT + 10 * i))"
    sim_vehicle.py "${SIM_ARGS[@]}" ${EXTRA_ARGS} &
    pids+=("$!")
    sleep 3   # stagger so instances don't race on shared startup state
done
wait
