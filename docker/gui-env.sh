#!/usr/bin/env bash
# Shared by runtime entrypoint and interactive devcontainer terminals.
# Only read a numeric value; never execute a host-generated shell fragment.
if [[ -n "${DISPLAY:-}" ]]; then
    metro_scale="${METRO_QT_SCALE_FACTOR:-${QT_SCALE_FACTOR:-}}"
    if [[ -z "$metro_scale" && -r /tmp/.X11-unix/metro-scale ]]; then
        read -r metro_scale < /tmp/.X11-unix/metro-scale
    fi
    if [[ "$metro_scale" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
        # Preserve the requested size when both entrypoint and .bashrc source us.
        export METRO_QT_SCALE_FACTOR="$metro_scale"
        export QT_AUTO_SCREEN_SCALE_FACTOR=0
        export QT_ENABLE_HIGHDPI_SCALING=0
        export QT_SCALE_FACTOR_ROUNDING_POLICY=PassThrough
        if [[ "${METRO_QT_SCALING_MODE:-font}" == full ]]; then
            export QT_SCALE_FACTOR="$metro_scale"
            export QT_FONT_DPI=96
        else
            # Humble/OGRE can flicker with devicePixelRatio > 1 (rviz issue #1052).
            # Scale font-based widgets while keeping the GL render window at 1:1.
            export QT_SCALE_FACTOR=1
            export QT_SCREEN_SCALE_FACTORS=1
            export QT_FONT_DPI
            QT_FONT_DPI=$(awk -v scale="$metro_scale" 'BEGIN {printf "%.0f", 96 * scale}')
        fi
    fi
    unset metro_scale
fi
