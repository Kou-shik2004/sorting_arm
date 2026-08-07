# syntax=docker/dockerfile:1

# Above line is a parser directive : it ensures that we use the
# latest docker syntax (To use New features like BuildKit) to
# parse this file regardless of the docker version
# makes it portable
# should be in the first line

FROM osrf/ros:jazzy-desktop-full

LABEL maintainer = "Koushik S"
LABEL description = "sorting_arm public dev environment"

ARG DEBIAN_FRONTEND=noninteractive # uses default answers
ARG USERNAME=rosdev # override at build time: --build-arg USERNAME=<you>
ARG USER_UID=1000
ARG USER_GID=1000

# docker-clean deleted as it removes all .deb files after installation
# we need it to be stored in the BuildKit cache
# keep-cache keeps the .deb after installation (Just to Confirm our need)

RUN rm -f /etc/apt/apt.conf.d/docker-clean \
 && echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' \
      > /etc/apt/apt.conf.d/keep-cache

# 1. 24.04 comes with a user ubuntu with userid 1000 and we dont want that
# 2. we add our custom user and its group and give sudo access with no password req
# 3. we make the sudoers file read only and we create a new dir for cmd history for easy reverse search
# 4. we chown commandhistory to USERNAME here, before compose can mount a named volume over it
# 5. ownership cannot be fixed after the named volume exists, so this has to happen at build time

RUN userdel -r ubuntu 2>/dev/null || true \
 && groupadd --gid $USER_GID $USERNAME \
 && useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME \
 && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
 && chmod 0440 /etc/sudoers.d/$USERNAME \
 && mkdir -p /commandhistory \
 && chown $USERNAME:$USERNAME /commandhistory

# mounting to make it persistent across builds
# inside container these are stored
# /var/cache/apt stores downloaded .deb files to reuse it and avoid downloads
# /var/lib/apt stores the package indexes for metadata like version name & dependencies
# Locking to prevent concurrent writes accross parallel builds to avoid cache corruption

# General packages

COPY .docker/packages/base.txt /tmp/base.txt
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    pkgs="$(grep -vE '^[[:space:]]*(#|$)' /tmp/base.txt)" \
 && [ -n "$pkgs" ] \
 && apt-get update && apt-get install -y --no-install-recommends $pkgs

# ROS packages
# Seperate Build as we can add packages easily without triggering entire triggers
# It Rebuilds regardless as BuildKit cache layers and docker image layers are diff

COPY .docker/packages/ros.txt /tmp/ros.txt
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    pkgs="$(grep -vE '^[[:space:]]*(#|$)' /tmp/ros.txt)" \
 && [ -n "$pkgs" ] \
 && apt-get update && apt-get install -y --no-install-recommends $pkgs

# Underlay to install source pkgs

# The repos file stays in the image as a record of what was built.
# --packages-select keeps the upstream samples out.

COPY .docker/underlay.repos /opt/underlay.repos
RUN mkdir -p /opt/underlay/src \
 && vcs import /opt/underlay/src < /opt/underlay.repos \
 && . /opt/ros/jazzy/setup.sh \
 && colcon build --base-paths /opt/underlay/src \
      --build-base /opt/underlay/build \
      --install-base /opt/underlay/install \
      --merge-install \
      --packages-select btcpp_ros2_interfaces behaviortree_ros2 \
 && rm -rf /opt/underlay/build /opt/underlay/log /opt/underlay/src

COPY .docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# User Environment

USER $USERNAME
WORKDIR /sorting_arm_ws

# vscode-server dir, user owned, so the opt-in dev volume in compose.dev.yaml
# mounts with the right ownership instead of root's

RUN mkdir -p /home/${USERNAME}/.vscode-server
ENV PATH="/home/${USERNAME}/.local/bin:${PATH}"

# ENV not export, so docker exec and hook processes get it too, not just shells

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# CI's runtime image already sets this - without it here, colcon test silently
# skips cppcheck and only CI catches what it would have found

ENV AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1

RUN rosdep update

# Extra installs

# Cheapest layer in the file - nothing expensive below it

COPY .docker/packages/extra.txt /tmp/extra.txt
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    pkgs="$(grep -vE '^[[:space:]]*(#|$)' /tmp/extra.txt)" \
 && [ -n "$pkgs" ] \
 && sudo apt-get update && sudo apt-get install -y --no-install-recommends $pkgs

# Bash config
# Sourced straight off the bind mount, so a shell tweak needs no rebuild

RUN echo '[ -f /sorting_arm_ws/.docker/bashrc.sh ] && source /sorting_arm_ws/.docker/bashrc.sh' \
    >> /home/${USERNAME}/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
