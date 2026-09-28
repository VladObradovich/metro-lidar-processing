#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export METRO_UID="$(id -u)"
export METRO_GID="$(id -g)"
export METRO_PROJECT_DIR="$project_dir"
export METRO_X11_DIR="$project_dir/.devcontainer/.runtime/compose/x11"

compose=(docker compose --project-directory "$project_dir" -f "$project_dir/compose.yaml")
proxy=(python3 "$project_dir/.devcontainer/scripts/x11_proxy.py"
       --runtime-dir "$project_dir/.devcontainer/.runtime/compose")

action="${1:-up}"
if (( $# )); then shift; fi

source_mount=false
nvidia=auto
compose_args=()
for arg in "$@"; do
    case "$arg" in
        --source)
            source_mount=true
            ;;
        --nvidia)
            nvidia=true
            ;;
        --no-nvidia)
            nvidia=false
            ;;
        *)
            compose_args+=("$arg")
            ;;
    esac
done

if [[ "$source_mount" == true ]]; then
    compose+=(-f "$project_dir/docker/compose.source.yaml")
fi

shopt -s nullglob
render_devices=(/dev/dri/renderD*)
if (( ${#render_devices[@]} )); then
    export METRO_RENDER_GID="$(stat -c '%g' "${render_devices[0]}")"
    compose+=(-f "$project_dir/docker/compose.gpu.yaml")
fi
if [[ "$nvidia" == auto ]]; then
    if command -v nvidia-smi >/dev/null && command -v nvidia-container-cli >/dev/null; then
        nvidia=true
    else
        nvidia=false
    fi
fi
if [[ "$nvidia" == true ]]; then
    compose+=(-f "$project_dir/docker/compose.nvidia.yaml")
fi

case "$action" in
    up)
        docker compose version >/dev/null
        mkdir -p "$project_dir/rosbags" "$project_dir/results"
        project_name="$("${compose[@]}" config --format json \
            | python3 -c 'import json, sys; print(json.load(sys.stdin)["name"])')"
        "${proxy[@]}" start \
            --watch-label "com.docker.compose.project=$project_name" \
            --watch-label "com.docker.compose.service=desktop"
        if [[ "$source_mount" == true ]]; then
            echo "Source mount: $project_dir -> /ws"
        fi
        "${compose[@]}" up -d --build "${compose_args[@]}"
        ;;
    down)
        "${compose[@]}" down "${compose_args[@]}"
        "${proxy[@]}" stop
        ;;
    shell)
        "${compose[@]}" exec desktop /usr/local/bin/metro-entrypoint bash "${compose_args[@]}"
        ;;
    exec)
        "${compose[@]}" exec desktop /usr/local/bin/metro-entrypoint "${compose_args[@]}"
        ;;
    build|logs|ps|config)
        "${compose[@]}" "$action" "${compose_args[@]}"
        ;;
    *)
        echo 'Usage: bash scripts/desktop.sh {up|down|shell|exec COMMAND...|build|logs|ps|config}' \
             '[--source] [--nvidia|--no-nvidia]' >&2
        exit 2
        ;;
esac
