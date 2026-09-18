#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash

workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${workspace}"
colcon build --base-paths "${workspace}"/metro_perception_* --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    "$@"

echo "Build complete. In an existing terminal: source ${workspace}/install/local_setup.bash"
