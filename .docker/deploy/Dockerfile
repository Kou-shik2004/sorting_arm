# syntax=docker/dockerfile:1

# Ships app.launch.xml to someone who only has Docker. Not the dev image, not
# the CI image - see docs/container/deployment.md before reordering anything.

FROM ros:jazzy-ros-base-noble AS workspace-builder

ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN rm -f /etc/apt/apt.conf.d/docker-clean \
 && printf 'Binary::apt::APT::Keep-Downloaded-Packages "true";\n' \
      > /etc/apt/apt.conf.d/keep-cache

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    printf 'APT::Install-Recommends "0";\nAPT::Install-Suggests "0";\n' \
      > /etc/apt/apt.conf.d/99sorting-arm-deploy \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      cmake \
      python3-colcon-common-extensions \
      python3-rosdep

COPY src/sorting_arm_bringup/package.xml /tmp/sorting-arm-src/sorting_arm_bringup/
COPY src/sorting_arm_description/package.xml /tmp/sorting-arm-src/sorting_arm_description/
COPY src/sorting_arm_executive/package.xml /tmp/sorting-arm-src/sorting_arm_executive/
COPY src/sorting_arm_interfaces/package.xml /tmp/sorting-arm-src/sorting_arm_interfaces/
COPY src/sorting_arm_moveit/package.xml /tmp/sorting-arm-src/sorting_arm_moveit/
COPY src/sorting_arm_perception/package.xml /tmp/sorting-arm-src/sorting_arm_perception/
COPY src/sorting_arm_skills/package.xml /tmp/sorting-arm-src/sorting_arm_skills/

# manifests before source, so an edit to src/ never busts this dependency layer

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
      --as-root apt:false \
      -y

ENV AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1

WORKDIR /sorting_arm_ws

COPY src/ src/

# no BUILD_TESTING - proving the tests pass is CI's job, not this image's

RUN source /opt/ros/jazzy/setup.bash \
 && colcon --log-base log build \
      --base-paths src \
      --build-base build \
      --install-base install \
      --event-handlers console_direct+ \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF \
        -DPython3_EXECUTABLE=/usr/bin/python3

# runtime starts fresh so the compiler toolchain cannot leak into what ships

FROM ros:jazzy-ros-core-noble AS runtime-dependency-resolver

ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

COPY src/sorting_arm_bringup/package.xml /tmp/sorting-arm-src/sorting_arm_bringup/
COPY src/sorting_arm_description/package.xml /tmp/sorting-arm-src/sorting_arm_description/
COPY src/sorting_arm_executive/package.xml /tmp/sorting-arm-src/sorting_arm_executive/
COPY src/sorting_arm_interfaces/package.xml /tmp/sorting-arm-src/sorting_arm_interfaces/
COPY src/sorting_arm_moveit/package.xml /tmp/sorting-arm-src/sorting_arm_moveit/
COPY src/sorting_arm_perception/package.xml /tmp/sorting-arm-src/sorting_arm_perception/
COPY src/sorting_arm_skills/package.xml /tmp/sorting-arm-src/sorting_arm_skills/

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update \
 && apt-get install -y --no-install-recommends python3-rosdep \
 && rosdep init

RUN --mount=type=bind,source=.docker/runtime/resolve-dependencies.sh,target=/tmp/resolve-dependencies.sh,readonly \
    --mount=type=bind,source=.docker/deploy/packages/skip-keys.txt,target=/tmp/skip-keys.txt,readonly \
    rosdep update --rosdistro jazzy \
 && bash /tmp/resolve-dependencies.sh \
      /tmp/sorting-arm-src \
      /tmp/skip-keys.txt \
      /tmp/runtime-apt.txt

# the apt novnc package hard-depends on nodejs for a launcher we never call,
# so we take the static client straight from upstream instead

FROM ros:jazzy-ros-base-noble AS novnc-fetch

ARG DEBIAN_FRONTEND=noninteractive
ARG NOVNC_VERSION=1.7.0

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update \
 && apt-get install -y --no-install-recommends ca-certificates curl

RUN curl -fsSL "https://github.com/novnc/noVNC/archive/refs/tags/v${NOVNC_VERSION}.tar.gz" \
      -o /tmp/novnc.tar.gz \
 && mkdir -p /opt/novnc \
 && tar -xzf /tmp/novnc.tar.gz -C /opt/novnc --strip-components=1 \
 && rm /tmp/novnc.tar.gz

# noVNC ships vnc.html with no redirect at "/" - autoconnect+scale so opening
# the bare URL lands straight in a full-window session, no click needed

RUN printf '<!doctype html>\n<meta http-equiv="refresh" content="0; url=vnc.html?autoconnect=1&amp;resize=scale">\n' \
      > /opt/novnc/index.html

FROM ros:jazzy-ros-core-noble AS deployment

ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN rm -f /etc/apt/apt.conf.d/docker-clean \
 && printf 'Binary::apt::APT::Keep-Downloaded-Packages "true";\n' \
      > /etc/apt/apt.conf.d/keep-cache \
 && printf 'APT::Install-Recommends "0";\nAPT::Install-Suggests "0";\n' \
      > /etc/apt/apt.conf.d/99sorting-arm-deploy

COPY .docker/deploy/packages/gui.txt /tmp/gui.txt

# gui.txt is our display stack, runtime-apt.txt is everything rosdep resolved
# off package.xml - nothing else goes in this layer

RUN --mount=type=bind,from=runtime-dependency-resolver,source=/tmp/runtime-apt.txt,target=/tmp/runtime-apt.txt,readonly \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    gui_packages="$(grep -vE '^[[:space:]]*(#|$)' /tmp/gui.txt | tr '\n' ' ')" \
 && runtime_packages="$(tr '\n' ' ' < /tmp/runtime-apt.txt)" \
 && test -n "${gui_packages}" \
 && test -n "${runtime_packages}" \
 && apt-get update \
 && apt-get install -y --no-install-recommends ${runtime_packages} ${gui_packages}

ENV DISPLAY=:99 \
    GALLIUM_DRIVER=llvmpipe \
    GZ_HOMEDIR=/home/ubuntu/.gz \
    HOME=/home/ubuntu \
    LIBGL_ALWAYS_SOFTWARE=1 \
    QT_X11_NO_MITSHM=1 \
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
COPY --from=novnc-fetch /opt/novnc/ /usr/share/novnc/
COPY --chmod=0755 .docker/deploy/entrypoint.sh /usr/local/bin/sorting-arm-deploy-entrypoint

WORKDIR /sorting_arm_ws
USER ubuntu

ENTRYPOINT ["/usr/local/bin/sorting-arm-deploy-entrypoint"]
CMD ["ros2", "launch", "sorting_arm_bringup", "app.launch.xml"]
