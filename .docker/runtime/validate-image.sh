#!/usr/bin/env bash

set -euo pipefail

image="${1:?runtime image is required}"
expected_sha="${2:?expected commit marker is required}"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
runtime_skip_keys="$(
  grep -vE '^[[:space:]]*(#|$)' \
    "${script_dir}/packages/gui-skip-keys.txt" \
    | tr '\n' ' '
)"

test -n "${runtime_skip_keys}"
test "$(docker image inspect --format '{{.Config.User}}' "${image}")" = "ubuntu"
test "$(
  docker image inspect --format '{{json .Config.Entrypoint}}' "${image}"
)" = '["/usr/local/bin/sorting-arm-entrypoint"]'
test "$(
  docker image inspect --format '{{json .Config.Cmd}}' "${image}"
)" = '["ros2","launch","sorting_arm_bringup","sim.launch.xml","gui:=false"]'

docker run --rm --user root "${image}" bash -c '
  set -euo pipefail

  test ! -e /etc/ros/rosdep
  test ! -e /root/.ros/rosdep
  test ! -e /tmp/runtime-apt.txt
  test ! -e /tmp/sorting-arm-src
'

docker run --rm \
  --env EXPECTED_SHA="${expected_sha}" \
  --env RUNTIME_SKIP_KEYS="${runtime_skip_keys}" \
  "${image}" \
  bash -c '
    set -euo pipefail

    test "$(id -un)" = "ubuntu"
    test "$(id -u)" -ne 0
    test "${HOME}" = "/home/ubuntu"

    for directory in \
      "${GZ_HOMEDIR}" \
      "${ROS_LOG_DIR}" \
      "${XDG_CACHE_HOME}" \
      "${XDG_CACHE_HOME}/gz" \
      "${XDG_RUNTIME_DIR}"
    do
      test -d "${directory}"
      test -O "${directory}"
      test -w "${directory}"
    done
    test "$(stat -c %a "${XDG_RUNTIME_DIR}")" = "700"

    test "$(cat /sorting_arm_ws/.ci/test-passed.sha)" = "${EXPECTED_SHA}"
    ros2 pkg prefix sorting_arm_bringup

    launch_arguments="$(ros2 launch sorting_arm_bringup sim.launch.xml --show-args)"
    grep -A2 "gui" <<< "${launch_arguments}" \
      | grep -q "default:.*true"

    test ! -e /sorting_arm_ws/src
    test ! -e /sorting_arm_ws/build
    test ! -e /sorting_arm_ws/log
    test ! -e /etc/ros/rosdep
    test ! -e /home/ubuntu/.ros/rosdep
    test ! -e /tmp/runtime-apt.txt
    test ! -e /tmp/sorting-arm-src

    for tool in cc c++ gcc g++ colcon rosdep sudo cpplint; do
      if command -v "${tool}" >/dev/null; then
        echo "Excluded tool found in runtime image: ${tool}" >&2
        exit 1
      fi
    done

    for package in \
      build-essential \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-rosdep-modules \
      ros-dev-tools \
      sudo
    do
      status="$(
        dpkg-query -W -f=\${db:Status-Abbrev} "${package}" 2>/dev/null \
          || true
      )"
      if [ "${status}" = "ii " ]; then
        echo "Excluded apt package found in runtime image: ${package}" >&2
        exit 1
      fi
    done

    for package in ${RUNTIME_SKIP_KEYS}; do
      if ros2 pkg prefix "${package}" >/dev/null 2>&1; then
        echo "GUI package found in runtime image: ${package}" >&2
        exit 1
      fi
    done
  '
