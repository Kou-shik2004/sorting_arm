#!/usr/bin/env bash

# we keep build outputs outside the checkout so every run proves a clean build

set -eo pipefail

ci_workspace="${1:-/workspace}"
if [ "$#" -gt 1 ]; then
    echo "Usage: $0 [workspace]" >&2
    exit 2
fi

if [ ! -d "${ci_workspace}/src" ] ||
    [ ! -f "${ci_workspace}/.clang-format" ]; then
    echo "Invalid CI workspace: ${ci_workspace}" >&2
    exit 2
fi

ci_work_root="$(mktemp -d /tmp/sorting-arm-ci.XXXXXX)"
trap 'rm -rf "${ci_work_root}"' EXIT

source /opt/ros/jazzy/setup.bash
set -u

ci_dependency_types=(
    --dependency-types build
    --dependency-types build_export
    --dependency-types buildtool
    --dependency-types test
)

rosdep check \
    --from-paths "${ci_workspace}/src" \
    --ignore-src \
    "${ci_dependency_types[@]}"

find "${ci_workspace}/src" -type f \
    \( -name '*.cc' -o -name '*.cpp' -o -name '*.h' -o -name '*.hpp' \) \
    -print0 \
    | xargs -0 -r clang-format --dry-run --Werror

find "${ci_workspace}" \
    \( -path "${ci_workspace}/.git" \
       -o -path "${ci_workspace}/build" \
       -o -path "${ci_workspace}/install" \
       -o -path "${ci_workspace}/log" \
       -o -path "${ci_workspace}/context" \) -prune \
    -o -type f -name '*.sh' -print0 \
    | xargs -0 -r -n1 bash -n

ament_lint_cmake "${ci_workspace}/src"

find "${ci_workspace}/src" -type f -name '*.py' -print0 \
    | PYTHONPYCACHEPREFIX="${ci_work_root}/pycache" \
        xargs -0 -r python3 -m py_compile

find "${ci_workspace}/src" -type f \
    \( -name '*.xml' -o -name '*.xacro' -o -name '*.srdf' \) \
    -print0 \
    | xargs -0 -r xmllint --noout

colcon --log-base "${ci_work_root}/build-log" build \
    --base-paths "${ci_workspace}/src" \
    --build-base "${ci_work_root}/build" \
    --install-base "${ci_work_root}/install" \
    --event-handlers console_direct+ \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DPython3_EXECUTABLE=/usr/bin/python3

set +u
source "${ci_work_root}/install/setup.bash"
set -u

ci_xacro_entries=(
    "${ci_workspace}/src/sorting_arm_description/urdf/sorting_arm.urdf.xacro"
    "${ci_workspace}/src/sorting_arm_description/urdf/sorting_arm.mock.urdf.xacro"
    "${ci_workspace}/src/sorting_arm_description/urdf/sorting_arm.sim.urdf.xacro"
    "${ci_workspace}/src/sorting_arm_moveit/config/sorting_arm.urdf.xacro"
)

for ci_xacro_index in "${!ci_xacro_entries[@]}"; do
    ci_urdf="${ci_work_root}/expanded-${ci_xacro_index}.urdf"
    xacro "${ci_xacro_entries[ci_xacro_index]}" > "${ci_urdf}"
    check_urdf "${ci_urdf}" > /dev/null
done

ci_plain_urdf="${ci_work_root}/expanded-0.urdf"
ci_srdf="${ci_workspace}/src/sorting_arm_moveit/config/sorting_arm.srdf"

if [ "$(xmllint --xpath "string(/robot/link[@name='world']/@name)" \
    "${ci_plain_urdf}")" != "world" ]; then
    echo "Expanded URDF must contain the world link" >&2
    exit 1
fi

if [ "$(xmllint --xpath "string(/robot/link[@name='tcp']/@name)" \
    "${ci_plain_urdf}")" != "tcp" ]; then
    echo "Expanded URDF must contain the tcp link" >&2
    exit 1
fi

if [ "$(xmllint --xpath \
    "string(/robot/group[@name='arm']/chain/@base_link)" \
    "${ci_srdf}")" != "base_link" ] ||
    [ "$(xmllint --xpath \
    "string(/robot/group[@name='arm']/chain/@tip_link)" \
    "${ci_srdf}")" != "tcp" ]; then
    echo "SRDF arm group must define the base_link to tcp chain" >&2
    exit 1
fi

set +e
colcon --log-base "${ci_work_root}/test-log" test \
    --base-paths "${ci_workspace}/src" \
    --build-base "${ci_work_root}/build" \
    --install-base "${ci_work_root}/install" \
    --test-result-base "${ci_work_root}/build" \
    --return-code-on-test-failure \
    --event-handlers console_direct+
ci_test_status=$?

colcon --log-base "${ci_work_root}/test-result-log" test-result \
    --test-result-base "${ci_work_root}/build" \
    --verbose
ci_result_status=$?
set -e

if [ "${ci_test_status}" -ne 0 ]; then
    exit "${ci_test_status}"
fi

exit "${ci_result_status}"
