#!/usr/bin/env bash

# Shared plumbing for build/start/enter/demo. Sourced, not run directly.

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER_NAME="sorting_arm"

# Fails loud and early rather than letting a docker command produce a
# confusing error three steps later.
require_docker() {
    if ! command -v docker >/dev/null 2>&1; then
        echo "Docker isn't installed. Get it here: https://docs.docker.com/engine/install/" >&2
        exit 1
    fi

    if docker info >/dev/null 2>&1; then
        DOCKER_CMD="docker"
        return
    fi

    if docker info 2>&1 | grep -qi "permission denied"; then
        echo "Your user can't reach the Docker daemon without sudo."
        echo "Fix it for good with: sudo usermod -aG docker \$USER (then log out and back in)"
        echo "For now, the next command runs as: sudo docker ..."
        DOCKER_CMD="sudo docker"
        return
    fi

    echo "Docker doesn't seem to be running. Try: sudo systemctl start docker" >&2
    exit 1
}

# Local-socket X11 clients only, not network-exposed. Missing xhost is a
# warning, not a stop - some WSLg setups dont need it at all.
allow_x11() {
    if command -v xhost >/dev/null 2>&1; then
        xhost +local: >/dev/null 2>&1 || true
    else
        echo "xhost not found - RViz/Gazebo windows may not display. Install x11-xserver-utils if that happens."
    fi
}

has_nvidia_runtime() {
    $DOCKER_CMD info --format '{{json .Runtimes}}' 2>/dev/null | grep -q nvidia
}

# Intel/AMD integrated graphics render node - nvidia has its own path above,
# this is the fallback so llvmpipe isnt the only option on a laptop GPU.
has_dri() {
    [ -e /dev/dri ]
}

# Reads --dev / --gpu / --no-gpu off "$@", leaves the rest in REMAINING_ARGS.
parse_common_flags() {
    SORTING_ARM_DEV=0
    SORTING_ARM_GPU=auto
    REMAINING_ARGS=()
    for arg in "$@"; do
        case "$arg" in
            --dev) SORTING_ARM_DEV=1 ;;
            --gpu) SORTING_ARM_GPU=on ;;
            --no-gpu) SORTING_ARM_GPU=off ;;
            *) REMAINING_ARGS+=("$arg") ;;
        esac
    done
}

# Fills the COMPOSE_ARGS array. Call after require_docker (GPU detection
# needs a working docker) and parse_common_flags. nvidia beats dri beats
# neither - --no-gpu drops both and leaves rendering to llvmpipe.
compose_args() {
    COMPOSE_ARGS=(-f docker-compose.yaml)
    if [ "${SORTING_ARM_GPU:-auto}" = "on" ] || { [ "${SORTING_ARM_GPU:-auto}" = "auto" ] && has_nvidia_runtime; }; then
        COMPOSE_ARGS+=(-f .docker/compose.gpu.yaml)
    elif [ "${SORTING_ARM_GPU:-auto}" = "auto" ] && has_dri; then
        COMPOSE_ARGS+=(-f .docker/compose.dri.yaml)
    fi
    if [ "${SORTING_ARM_DEV:-0}" = "1" ]; then
        COMPOSE_ARGS+=(-f .docker/compose.dev.yaml)
    fi
}

container_running() {
    [ -n "$($DOCKER_CMD ps --filter "name=^/${CONTAINER_NAME}\$" --filter "status=running" -q)" ]
}

# Brings the container up if it's down. Safe to call when it's already up.
ensure_up() {
    if ! container_running; then
        echo "container isn't up - starting it"
        $DOCKER_CMD compose "${COMPOSE_ARGS[@]}" up -d --build
    fi
}

workspace_built() {
    $DOCKER_CMD exec "$CONTAINER_NAME" test -f /sorting_arm_ws/install/setup.bash
}
