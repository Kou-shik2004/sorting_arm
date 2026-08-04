# syntax=docker/dockerfile:1

# this image proves the installed workspace before we make a runtime artifact

FROM ros:jazzy-ros-base-noble AS build-deps

ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN rm -f /etc/apt/apt.conf.d/docker-clean \
 && printf 'Binary::apt::APT::Keep-Downloaded-Packages "true";\n' \
      > /etc/apt/apt.conf.d/keep-cache

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    printf 'APT::Install-Recommends "0";\nAPT::Install-Suggests "0";\n' \
      > /etc/apt/apt.conf.d/99sorting-arm-runtime \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      cmake \
      python3-colcon-common-extensions \
      python3-rosdep

COPY src/sorting_arm_bringup/package.xml /tmp/sorting-arm-src/sorting_arm_bringup/
COPY src/sorting_arm_description/package.xml /tmp/sorting-arm-src/sorting_arm_description/
COPY src/sorting_arm_interfaces/package.xml /tmp/sorting-arm-src/sorting_arm_interfaces/
COPY src/sorting_arm_moveit/package.xml /tmp/sorting-arm-src/sorting_arm_moveit/
COPY src/sorting_arm_perception/package.xml /tmp/sorting-arm-src/sorting_arm_perception/
COPY src/sorting_arm_skills/package.xml /tmp/sorting-arm-src/sorting_arm_skills/

# Installing rosdep does not guarantee that its system source list exists.

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update \
 && rosdep update --rosdistro jazzy \
 && rosdep install \
      --from-paths /tmp/sorting-arm-src \
      --ignore-src \
      --rosdistro jazzy \
      --dependency-types build \
      --dependency-types build_export \
      --dependency-types buildtool \
      --dependency-types buildtool_export \
      --dependency-types test \
      --as-root apt:false \
      -y

ENV AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1

FROM build-deps AS workspace-builder

WORKDIR /sorting_arm_ws

COPY src/ src/

RUN source /opt/ros/jazzy/setup.bash \
 && colcon --log-base log build \
      --base-paths src \
      --build-base build \
      --install-base install \
      --event-handlers console_direct+ \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=ON \
        -DPython3_EXECUTABLE=/usr/bin/python3

FROM workspace-builder AS test

ARG GIT_COMMIT_SHA
ARG GITHUB_RUN_ID

RUN test -n "${GIT_COMMIT_SHA}" \
 && test -n "${GITHUB_RUN_ID}" \
 && printf '%s\n' "${GITHUB_RUN_ID}" > /tmp/github-run-id \
 && source /opt/ros/jazzy/setup.bash \
 && source /sorting_arm_ws/install/setup.bash \
 && colcon --log-base test-log test \
      --base-paths src \
      --build-base build \
      --install-base install \
      --test-result-base build \
      --return-code-on-test-failure \
      --event-handlers console_direct+ \
 && colcon --log-base test-result-log test-result \
      --test-result-base build \
      --verbose \
 && mkdir -p /sorting_arm_ws/.ci \
 && printf '%s\n' "${GIT_COMMIT_SHA}" > /sorting_arm_ws/.ci/test-passed.sha

# runtime starts fresh so our build toolchain cannot leak from the builder

FROM ros:jazzy-ros-core-noble AS runtime-base

ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN rm -f /etc/apt/apt.conf.d/docker-clean \
 && printf 'Binary::apt::APT::Keep-Downloaded-Packages "true";\n' \
      > /etc/apt/apt.conf.d/keep-cache \
 && printf 'APT::Install-Recommends "0";\nAPT::Install-Suggests "0";\n' \
      > /etc/apt/apt.conf.d/99sorting-arm-runtime

# ubuntu owns the non-root runtime account, so we verify it before using it

RUN test "$(id -u ubuntu)" -ne 0 \
 && test "$(getent passwd ubuntu | cut -d: -f6)" = "/home/ubuntu" \
 && test -d /home/ubuntu \
 && test "$(stat -c %u:%g /home/ubuntu)" = "$(id -u ubuntu):$(id -g ubuntu)"

FROM runtime-base AS runtime-dependency-resolver

COPY src/sorting_arm_bringup/package.xml /tmp/sorting-arm-src/sorting_arm_bringup/
COPY src/sorting_arm_description/package.xml /tmp/sorting-arm-src/sorting_arm_description/
COPY src/sorting_arm_interfaces/package.xml /tmp/sorting-arm-src/sorting_arm_interfaces/
COPY src/sorting_arm_moveit/package.xml /tmp/sorting-arm-src/sorting_arm_moveit/
COPY src/sorting_arm_perception/package.xml /tmp/sorting-arm-src/sorting_arm_perception/
COPY src/sorting_arm_skills/package.xml /tmp/sorting-arm-src/sorting_arm_skills/

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update \
 && apt-get install -y --no-install-recommends python3-rosdep \
 && command -v rosdep >/dev/null \
 && if [ ! -s /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
      rosdep init; \
    fi \
 && test -s /etc/ros/rosdep/sources.list.d/20-default.list

ENV HOME=/home/ubuntu

USER ubuntu

RUN --mount=type=bind,source=.docker/runtime/resolve-dependencies.sh,target=/tmp/resolve-dependencies.sh,readonly \
    --mount=type=bind,source=.docker/runtime/packages/gui-skip-keys.txt,target=/tmp/gui-skip-keys.txt,readonly \
    rosdep update --rosdistro jazzy \
 && bash /tmp/resolve-dependencies.sh \
      /tmp/sorting-arm-src \
      /tmp/gui-skip-keys.txt \
      /tmp/runtime-apt.txt

FROM runtime-base AS runtime-deps

RUN --mount=type=bind,from=runtime-dependency-resolver,source=/tmp/runtime-apt.txt,target=/tmp/runtime-apt.txt,readonly \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    runtime_packages="$(tr '\n' ' ' < /tmp/runtime-apt.txt)" \
 && test -n "${runtime_packages}" \
 && apt-get update \
 && apt-get install -y --no-install-recommends ${runtime_packages}

# we verify the installed dependencies with rosdep in a throwaway stage

FROM runtime-deps AS runtime-deps-check

ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

COPY src/sorting_arm_bringup/package.xml /tmp/sorting-arm-src/sorting_arm_bringup/
COPY src/sorting_arm_description/package.xml /tmp/sorting-arm-src/sorting_arm_description/
COPY src/sorting_arm_interfaces/package.xml /tmp/sorting-arm-src/sorting_arm_interfaces/
COPY src/sorting_arm_moveit/package.xml /tmp/sorting-arm-src/sorting_arm_moveit/
COPY src/sorting_arm_perception/package.xml /tmp/sorting-arm-src/sorting_arm_perception/
COPY src/sorting_arm_skills/package.xml /tmp/sorting-arm-src/sorting_arm_skills/

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update \
 && apt-get install -y --no-install-recommends python3-rosdep \
 && command -v rosdep >/dev/null \
 && if [ ! -s /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
      rosdep init; \
    fi \
 && test -s /etc/ros/rosdep/sources.list.d/20-default.list

ENV HOME=/home/ubuntu

USER ubuntu

RUN --mount=type=bind,source=.docker/runtime/packages/gui-skip-keys.txt,target=/tmp/gui-skip-keys.txt,readonly \
    skip_keys="$(grep -vE '^[[:space:]]*(#|$)' /tmp/gui-skip-keys.txt | tr '\n' ' ')" \
 && test -n "${skip_keys}" \
 && rosdep update --rosdistro jazzy \
 && rosdep check \
      --from-paths /tmp/sorting-arm-src \
      --ignore-src \
      --rosdistro jazzy \
      --dependency-types exec \
      --skip-keys="${skip_keys}"

FROM runtime-deps AS runtime-headless

ENV GZ_HOMEDIR=/home/ubuntu/.gz \
    HOME=/home/ubuntu \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_LOG_DIR=/home/ubuntu/.ros/log \
    XDG_CACHE_HOME=/home/ubuntu/.cache \
    XDG_RUNTIME_DIR=/tmp/sorting-arm-runtime

RUN install -d -o ubuntu -g ubuntu -m 0755 \
      "${GZ_HOMEDIR}" \
      /home/ubuntu/.ros \
      "${ROS_LOG_DIR}" \
      "${XDG_CACHE_HOME}" \
      "${XDG_CACHE_HOME}/gz" \
 && install -d -o ubuntu -g ubuntu -m 0700 "${XDG_RUNTIME_DIR}"

COPY --from=workspace-builder /sorting_arm_ws/install/ /sorting_arm_ws/install/
COPY --from=test /sorting_arm_ws/.ci/test-passed.sha /sorting_arm_ws/.ci/test-passed.sha
COPY --chmod=0755 .docker/runtime/entrypoint.sh /usr/local/bin/sorting-arm-entrypoint

WORKDIR /sorting_arm_ws
USER ubuntu

ENTRYPOINT ["/usr/local/bin/sorting-arm-entrypoint"]
CMD ["ros2", "launch", "sorting_arm_bringup", "sim.launch.xml", "gui:=false"]

FROM runtime-headless AS smoke

USER root

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update \
 && apt-get install -y --no-install-recommends ros-jazzy-launch-testing

RUN install -d -o root -g root -m 0755 /opt/sorting_arm_smoke

COPY --chown=root:root --chmod=0444 \
  .docker/runtime/smoke/test_headless.py \
  /opt/sorting_arm_smoke/test_headless.py

RUN test "$(stat -c %a /opt/sorting_arm_smoke)" = "755" \
 && test "$(stat -c %a /opt/sorting_arm_smoke/test_headless.py)" = "444"

USER ubuntu

CMD ["launch_test", "--verbose", "/opt/sorting_arm_smoke/test_headless.py"]
