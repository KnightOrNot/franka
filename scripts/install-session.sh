#!/usr/bin/env bash
# Prepare separate opt-in entry points without changing the running controller.
set -euo pipefail
umask 077

repo="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
teleop="${FRANKA_TELEOP_ROOT:-$HOME/frankateleop}"
teleop="$(CDPATH= cd -- "$teleop" && pwd)"
poly=polymetis/polymetis/python/scripts
experiments=teleop/experiments
robot="$poly/launch_robot.py"
follow="$experiments/run_env.py"
robot_new="$poly/launch_robot_capture.py"
follow_new="$experiments/run_env_capture.py"
bridge="$experiments/franka_capture_session_bridge.py"

stage="$(mktemp -d)"
trap 'rm -r -- "$stage"' EXIT
mkdir -p "$stage/$poly" "$stage/$experiments"
cp -- "$teleop/$robot" "$stage/$robot"
cp -- "$teleop/$follow" "$stage/$follow"
cd -- "$stage"
sha256sum --check <<CHECKSUM
365fe395daa33c22d7537fc55038a5f670c0ef550cb06d46956b31ca78dbed80  $robot
65491d0a1197a846e10569cb26fd911f282e5bfd0ff37346bbb4f312aa9cd4ca  $follow
CHECKSUM
git apply --check "$repo/patches/polymetis-owned-shutdown.patch" \
    "$repo/patches/frankateleop-fr3-startup.patch"
git apply "$repo/patches/polymetis-owned-shutdown.patch" \
    "$repo/patches/frankateleop-fr3-startup.patch"
mv -- "$robot" "$robot_new"
mv -- "$follow" "$follow_new"
cp -- "$repo/src/franka_capture/bridge.py" "$bridge"

target="$teleop/$robot_new"
if [[ -L $target || ( -e $target && ! -f $target ) ]]; then
    printf 'Invalid session destination: %s\n' "$target" >&2
    exit 1
fi
if [[ ! -e $target ]]; then
    tmp="$(mktemp "${target}.XXXXXX")"
    cp -- "$robot_new" "$tmp"
    mv -n -- "$tmp" "$target"
fi
if ! cmp -s -- "$robot_new" "$target"; then
    printf 'Existing dedicated robot session file differs: %s\n' "$target" >&2
    exit 1
fi

# Keep the generated follow entry and bridge with the capture version that owns them.
# Older releases and the vendor teleop tree retain their existing follow files.
follow_dir="$repo/.native/follow"
mkdir -p -- "$follow_dir"
for name in "$follow_new" "$bridge"; do
    target="$follow_dir/${name##*/}"
    if ! cmp -s -- "$name" "$target"; then
        tmp="$(mktemp "${target}.XXXXXX")"
        cp -- "$name" "$tmp"
        mv -- "$tmp" "$target"
    fi
done
printf 'Dedicated FR3 session entries installed. No control or capture process was started.\n'
