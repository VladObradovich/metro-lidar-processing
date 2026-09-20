#!/usr/bin/env bash
set -e
source /etc/metro-gui-env.sh
source /opt/ros/humble/setup.bash
source /opt/metro/install/local_setup.bash
exec "$@"
