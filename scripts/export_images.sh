#!/usr/bin/env bash
# Save the built runtime images for a stand without internet access, where they cannot be
# built (the build needs Docker Hub, apt and pip). On the stand:
#   docker load -i metro-lidar-local.tar.gz      # or gunzip -c FILE | docker load
#
#   bash scripts/export_images.sh [OUTPUT_DIR]   # default results/images
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
out="${1:-$project_dir/results/images}"
mkdir -p "$out"
for image in metro-lidar:local metro-lidar:desktop; do
    if ! docker image inspect "$image" >/dev/null 2>&1; then
        echo "Image $image is missing; build it first (see README)." >&2
        exit 1
    fi
    file="$out/${image//:/-}.tar.gz"
    echo "Saving $image -> $file"
    docker save "$image" | gzip -1 > "$file"
    (cd "$out" && sha256sum "$(basename "$file")" > "$(basename "$file").sha256")
done
ls -lh "$out"
