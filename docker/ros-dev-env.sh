#!/usr/bin/env bash
# Shared by the container entrypoint and every interactive VS Code terminal.
source /opt/ros/humble/setup.bash
if [[ -f "${HOME}/metro_ws/install/local_setup.bash" ]]; then
    source "${HOME}/metro_ws/install/local_setup.bash"
fi
