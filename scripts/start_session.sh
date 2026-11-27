#!/usr/bin/env bash
set -euo pipefail
scripts="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$scripts/start-session.sh" "$@"
