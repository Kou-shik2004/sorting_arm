#!/usr/bin/env bash

# Interactive-shell setup for the sorting_arm dev container, sourced from
# ~/.bashrc. Non-interactive shells return immediately.

case $- in
    *i*) ;;
      *) return ;;
esac

# source bash-completion 
if ! shopt -oq posix; then
    if [ -f /usr/share/bash-completion/bash_completion ]; then
        . /usr/share/bash-completion/bash_completion
    elif [ -f /etc/bash_completion ]; then
        . /etc/bash_completion
    fi
fi

# ROS 2
source /opt/ros/jazzy/setup.bash
source /opt/underlay/install/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
[ -f /sorting_arm_ws/install/setup.bash ] && source /sorting_arm_ws/install/setup.bash

# Track colcon's home with repo to avoid wiping off
export COLCON_HOME=/sorting_arm_ws/.docker/colcon

# Install what the package.xml files ask for, off the mounted src.
cdeps() {
    ( cd /sorting_arm_ws && rosdep install --from-paths src --ignore-src -r -y "$@" )
}

# Build the workspace from anywhere.
cbuild() {
    ( cd /sorting_arm_ws && colcon build --symlink-install "$@" )
}

# Run tests and show the failures, not just the counts.
ctest_all() {
    ( cd /sorting_arm_ws && colcon test "$@" && colcon test-result --verbose )
}

# history

if [ -d /commandhistory ]; then
    export HISTFILE=/commandhistory/.bash_history
fi

export HISTSIZE=100000
export HISTFILESIZE=200000
export HISTCONTROL=ignoreboth       # skip duplicates and lines starting with a space
shopt -s histappend                 # append on exit instead of overwriting HISTFILE
shopt -s cmdhist                    # keep a multi-line command as one history entry


case "$PROMPT_COMMAND" in
    *'history -a'*) ;;
    *) PROMPT_COMMAND="history -a${PROMPT_COMMAND:+; $PROMPT_COMMAND}" ;;
esac

# cosmetic
force_color_prompt=yes
PS1="\[\e[1;32m\]\u@sorting_arm\[\e[0m\]:\[\e[1;34m\]\w\[\e[0m\]\$ "
