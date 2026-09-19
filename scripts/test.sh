#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${workspace}"
if [[ ! -f install/local_setup.bash ]]; then
    echo 'Build the workspace first: bash scripts/build.sh' >&2
    exit 1
fi
source install/local_setup.bash
test_status=0
python3 -m pytest scripts/test || test_status=$?
colcon test --base-paths "${workspace}"/metro_perception_* \
    --return-code-on-test-failure "$@" || test_status=$?
colcon test-result --verbose || test_status=$?
exit "${test_status}"
