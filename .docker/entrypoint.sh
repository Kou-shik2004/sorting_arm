#!/usr/bin/env bash

# Wraps PID 1 and docker run. docker exec skips it, so bashrc repeats the sourcing

set -e

source /opt/ros/jazzy/setup.bash
source /opt/underlay/install/setup.bash
[ -f /sorting_arm_ws/install/setup.bash ] && source /sorting_arm_ws/install/setup.bash

# Hand off to whatever the container was told to run.
exec "$@"
