#!/usr/bin/env bash

# No host display here, so we carry our own: Xvfb is the screen, x11vnc reads
# it, websockify turns that into the noVNC page the browser opens.

set -e

source /opt/ros/jazzy/setup.bash
source /sorting_arm_ws/install/setup.bash

Xvfb "${DISPLAY}" -screen 0 1600x900x24 &

# a fixed sleep here would hide a genuinely broken display, so we poll instead
display_number="${DISPLAY#:}"
for _ in $(seq 1 50); do
  [ -e "/tmp/.X11-unix/X${display_number}" ] && break
  sleep 0.1
done
[ -e "/tmp/.X11-unix/X${display_number}" ] || {
  echo "Xvfb did not come up on display ${DISPLAY}" >&2
  exit 1
}

openbox &
x11vnc -display "${DISPLAY}" -forever -shared -nopw -rfbport 5900 -quiet &
websockify --web=/usr/share/novnc 6080 localhost:5900 &

exec "$@"
