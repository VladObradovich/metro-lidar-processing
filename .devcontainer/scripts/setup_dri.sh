#!/usr/bin/env bash
set -euo pipefail

# Docker cannot express an optional --device=/dev/dri. Recreate only the DRM
# nodes exposed by the host kernel; on WSL or a machine without DRM this is a
# no-op and Mesa falls back to software rendering.
shopt -s nullglob
drm_devices=(/sys/class/drm/card*/dev /sys/class/drm/render*/dev)
(( ${#drm_devices[@]} )) || exit 0

sudo mkdir -p /dev/dri
for device_file in "${drm_devices[@]}"; do
    device_name="$(basename "$(dirname "$device_file")")"
    IFS=: read -r major minor < "$device_file"
    target="/dev/dri/$device_name"
    if [[ ! -e "$target" ]]; then
        sudo mknod "$target" c "$major" "$minor"
    fi
    sudo chown root:dev "$target"
    sudo chmod 0660 "$target"
done
