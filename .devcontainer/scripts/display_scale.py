"""Detect host desktop scale without guessing from physical monitor dimensions."""

import json
import math
import os
import re
import subprocess


def run(*command):
    try:
        return subprocess.check_output(
            command, text=True, stderr=subprocess.DEVNULL, timeout=3
        )
    except (OSError, subprocess.SubprocessError):
        return ''


def json_output(*command):
    try:
        return json.loads(run(*command))
    except ValueError:
        return None


def valid_scale(value):
    try:
        scale = float(value)
    except (TypeError, ValueError):
        raise ValueError('Display scale must be a number between 0.5 and 4') from None
    if not math.isfinite(scale) or not 0.5 <= scale <= 4:
        raise ValueError('Display scale must be a number between 0.5 and 4')
    return scale


def choose_output(outputs, preferred=None):
    """Use an explicit/focused output, or a common scale if unambiguous."""
    if preferred:
        matches = [o for o in outputs if o['name'] == preferred]
        if not matches:
            raise ValueError(f'Active output {preferred!r} was not found')
        return valid_scale(matches[0]['scale'])
    focused = [o for o in outputs if o.get('focused')]
    if focused:
        return valid_scale(focused[0]['scale'])
    scales = {valid_scale(o['scale']) for o in outputs}
    return scales.pop() if len(scales) == 1 else None


def detect_scale(explicit=None, output=None):
    for value, source in (
        (explicit, '--scale'),
        (os.environ.get('METRO_QT_SCALE_FACTOR'), 'METRO_QT_SCALE_FACTOR'),
        (os.environ.get('QT_SCALE_FACTOR'), 'QT_SCALE_FACTOR'),
    ):
        if value:
            return valid_scale(value), source

    if os.environ.get('NIRI_SOCKET'):
        data = json_output('niri', 'msg', '-j', 'outputs')
        workspaces = json_output('niri', 'msg', '-j', 'workspaces') or []
        focused = next((w.get('output') for w in workspaces if w.get('is_focused')), None)
        if isinstance(data, dict):
            outputs = [dict(name=name, scale=o['logical']['scale'], focused=name == focused)
                       for name, o in data.items() if o.get('logical')]
            scale = choose_output(outputs, output)
            if scale is not None:
                return scale, f'Niri ({output or focused or "outputs"})'

    for available, command, source in (
        (os.environ.get('SWAYSOCK'), ('swaymsg', '-t', 'get_outputs', '-r'), 'Sway'),
        (os.environ.get('HYPRLAND_INSTANCE_SIGNATURE'), ('hyprctl', 'monitors', '-j'), 'Hyprland'),
    ):
        if available:
            data = json_output(*command)
            if isinstance(data, list):
                outputs = [o for o in data if o.get('active', True)
                           and not o.get('disabled', False) and 'scale' in o]
                scale = choose_output(outputs, output)
                if scale is not None:
                    return scale, source

    # Honour a GTK session scale when compositor IPC is unavailable.
    if os.environ.get('GDK_SCALE'):
        return valid_scale(os.environ['GDK_SCALE']), 'GDK_SCALE'
    match = re.search(r'^Xft\.dpi:\s*([\d.]+)', run('xrdb', '-query'), re.MULTILINE)
    if match:
        return valid_scale(float(match.group(1)) / 96), 'Xft.dpi'
    return 1.0, 'fallback; use --scale if the UI is too small'
