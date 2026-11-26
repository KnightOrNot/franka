#!/usr/bin/env bash
set -euo pipefail

repo="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if ! uv_bin="$(command -v -- "${UV:-uv}")"; then
    printf 'Install uv and add it to PATH, or set UV to the path of an installed uv executable.\n' >&2
    exit 1
fi
[[ $uv_bin == /* ]] || uv_bin="$PWD/$uv_bin"
cd -- "$repo"
UV_PROJECT_ENVIRONMENT="$repo/.venv" exec "$uv_bin" sync --locked --no-dev --extra camera --extra export
