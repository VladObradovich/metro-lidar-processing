#!/usr/bin/env bash
set -eo pipefail
source /etc/ros-dev-env.sh
cd "${HOME}/ros_ws"
if [[ ! -f install/local_setup.bash ]]; then
    echo 'Build the workspace first: bash scripts/build.sh' >&2
    exit 1
fi
test_status=0
colcon test --return-code-on-test-failure "$@" || test_status=$?
colcon test-result --verbose || test_status=$?
exit "${test_status}"
