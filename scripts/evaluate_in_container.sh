#!/usr/bin/env bash
# Build the mounted checkout out of tree and run evaluate_all.py on it, so that a Q1 run is tied
# to one commit, one image and hashed binaries. Run from the repository root on the host:
#
#   image=metro-lidar:dev   # docker build -f docker/Dockerfile --target universal -t "$image" docker
#   docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp \
#     -e IMAGE_ID="$(docker image inspect "$image" --format '{{.Id}}')" \
#     -v "$PWD":/repo:ro -v "$PWD/rosbags":/data:ro -v "$PWD/results":/results \
#     "$image" bash /repo/scripts/evaluate_in_container.sh NAME [evaluate_all.py arguments]
#
# Results go to /results/NAME; the build log to /results/NAME-build.log.
set -eo pipefail
name=${1:?usage: evaluate_in_container.sh NAME [evaluate_all.py arguments]}
shift
source /opt/ros/humble/setup.bash
git config --global --add safe.directory /repo
cd /tmp
build=(colcon build --base-paths /repo --build-base /tmp/metro-build --install-base /tmp/metro-install
       --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF)
"${build[@]}" > "/results/${name}-build.log" 2>&1
source /tmp/metro-install/setup.bash
python3 /repo/scripts/evaluate_all.py --dataset-root /data --output-dir "/results/${name}" \
  --image-id "${IMAGE_ID:-not_recorded}" --build-info "${build[*]}" "$@"
