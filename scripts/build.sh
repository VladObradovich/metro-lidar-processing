#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash

workspace="${HOME}/metro_ws"
if [[ ! -d "${workspace}/src/metro-lidar-processing" ]]; then
    echo 'Source mount missing: open the repository in its Metro LiDAR dev container.' >&2
    exit 1
fi
cd "${workspace}"
colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    "$@"

echo "Build complete. In an existing terminal: source ${workspace}/install/local_setup.bash"
