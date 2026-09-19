#!/usr/bin/env bash
# Shared by the container entrypoint and every interactive VS Code terminal.
source /etc/metro-gui-env.sh
source /opt/ros/humble/setup.bash
if [[ -f /ws/install/local_setup.bash ]]; then
    source /ws/install/local_setup.bash
fi
