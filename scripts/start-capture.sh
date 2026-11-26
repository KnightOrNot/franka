#!/usr/bin/env bash
set -euo pipefail

usage() {
    printf 'Usage: %s [--simulate]\n' "${0##*/}"
}

simulate=false
case "$#:${1:-}" in
    0:) ;;
    1:--simulate) simulate=true ;;
    1:-h|1:--help) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
esac

repo="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
capture="$repo/.venv/bin/franka-capture"
if [[ ! -x $capture ]]; then
    printf 'franka-capture is not installed. Run %s/scripts/install-capture.sh first.\n' "$repo" >&2
    exit 1
fi

config="${FRANKA_CAPTURE_CONFIG:-$repo/config/site.env}"
[[ $config == /* ]] || config="$PWD/$config"
cd -- "$repo"

if "$simulate"; then
    socket="${XDG_RUNTIME_DIR:+$XDG_RUNTIME_DIR/franka-capture-sim}"
    socket="${socket:-/tmp/franka-capture-sim-$UID}/telemetry.sock"
    exec "$capture" serve --simulate "--socket=$socket" \
        --data-root=data/sim/raw --dataset-root=data/sim/lerobot --port=8079
fi

if [[ -e $config || -L $config || -n ${FRANKA_CAPTURE_CONFIG:-} ]]; then
    if [[ ! -f $config || ! -r $config ]]; then
        printf 'Cannot read capture configuration: %s\n' "$config" >&2
        exit 1
    fi
    # The site file is an operator-owned shell environment file also used by the bridge.
    source "$config"
fi

args=(serve)
add_option() {
    local value="${!1-}"
    [[ -z $value ]] || args+=("$2=$value")
}
add_option FRANKA_CAPTURE_SOCKET --socket
add_option FRANKA_CAPTURE_CAMERA_SERIAL --serial
add_option FRANKA_CAPTURE_WIDTH --width
add_option FRANKA_CAPTURE_HEIGHT --height
add_option FRANKA_CAPTURE_FPS --camera-fps
add_option FRANKA_CAPTURE_DATA_ROOT --data-root
add_option FRANKA_CAPTURE_DATASET_ROOT --dataset-root
add_option FRANKA_CAPTURE_HTTP_PORT --port
add_option FRANKA_CAPTURE_CONTROL_REVISION --control-revision
add_option FRANKA_CAPTURE_STALE_MS --stale-ms
add_option FRANKA_CAPTURE_MAX_SKEW_MS --max-skew-ms
add_option FRANKA_CAPTURE_MIN_FREE_MB --min-free-mb
add_option FRANKA_CAPTURE_MAX_SECONDS --max-seconds

exec "$capture" "${args[@]}"
