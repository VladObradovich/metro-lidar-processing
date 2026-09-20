#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export METRO_UID="$(id -u)"
export METRO_GID="$(id -g)"
compose=(docker compose --project-directory "$project_dir" -f "$project_dir/compose.yaml")
proxy=(python3 "$project_dir/.devcontainer/scripts/x11_proxy.py"
       --runtime-dir "$project_dir/.devcontainer/.runtime/compose")

shopt -s nullglob
render_devices=(/dev/dri/renderD*)
if (( ${#render_devices[@]} )); then
    export METRO_RENDER_GID="$(stat -c '%g' "${render_devices[0]}")"
    compose+=(-f "$project_dir/docker/compose.gpu.yaml")
fi

action="${1:-up}"
if (( $# )); then shift; fi
case "$action" in
    up)
        docker compose version >/dev/null
        mkdir -p "$project_dir/rosbags" "$project_dir/results"
        "${proxy[@]}" start
        "${compose[@]}" up -d --build "$@"
        ;;
    down)
        "${compose[@]}" down "$@"
        "${proxy[@]}" stop
        ;;
    shell)
        "${compose[@]}" exec desktop /usr/local/bin/metro-entrypoint bash "$@"
        ;;
    exec)
        "${compose[@]}" exec desktop /usr/local/bin/metro-entrypoint "$@"
        ;;
    build|logs|ps|config)
        "${compose[@]}" "$action" "$@"
        ;;
    *)
        echo 'Usage: bash scripts/desktop.sh {up|down|shell|exec COMMAND...|build|logs|ps|config}' >&2
        exit 2
        ;;
esac
