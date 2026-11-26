#!/usr/bin/env bash
set -euo pipefail

repo="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
python="$repo/.venv/bin/python"
if [[ ! -x $python ]]; then
    printf 'Run %s/scripts/install-capture.sh first.\n' "$repo" >&2
    exit 1
fi
config="${FRANKA_CAPTURE_CONFIG:-$repo/config/site.env}"
[[ $config == /* ]] || config="$PWD/$config"
if [[ ! -f $config || ! -r $config ]]; then
    printf 'Cannot read capture configuration: %s\n' "$config" >&2
    exit 1
fi
cd -- "$repo"
set -a
source "$config"
set +a
export FRANKA_CAPTURE_CONFIG="$config"
exec "$python" -m franka_capture.session "$@"
