#!/usr/bin/env bash

set -e

source /opt/ros/jazzy/setup.bash
source /opt/underlay/install/setup.bash
source /sorting_arm_ws/install/setup.bash

exec "$@"
