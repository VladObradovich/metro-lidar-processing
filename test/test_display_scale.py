"""Host scale detection tests; no ROS or running desktop required."""

import importlib.util
import os
import subprocess
from pathlib import Path
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location(
    'display_scale', Path(__file__).resolve().parents[1] / '.devcontainer/scripts/display_scale.py'
)
scale = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scale)


class DisplayScaleTests(unittest.TestCase):
    def test_font_mode_is_idempotent_and_full_mode_is_opt_in(self):
        helper = Path(__file__).resolve().parents[1] / 'docker/gui-env.sh'
        for mode, expected in [('font', '1 192'), ('full', '2 96')]:
            result = subprocess.check_output(
                ['bash', '-c', 'source "$1"; source "$1"; '
                 'echo "$QT_SCALE_FACTOR $QT_FONT_DPI"', 'check', str(helper)],
                env={'PATH': os.defpath, 'DISPLAY': ':test',
                     'METRO_QT_SCALE_FACTOR': '2', 'METRO_QT_SCALING_MODE': mode},
                text=True,
            )
            self.assertEqual(result.strip(), expected)

    def test_fractional_and_invalid_values(self):
        self.assertEqual(scale.valid_scale('1.5'), 1.5)
        for value in ('nan', 'inf', '0', '-1', 'shell code', '8'):
            with self.assertRaises(ValueError):
                scale.valid_scale(value)

    @patch.dict('os.environ', {'METRO_QT_SCALE_FACTOR': '1.5', 'QT_SCALE_FACTOR': '2'}, clear=True)
    def test_override_precedence(self):
        self.assertEqual(scale.detect_scale('1')[0], 1)
        self.assertEqual(scale.detect_scale()[0], 1.5)

    @patch.dict('os.environ', {'NIRI_SOCKET': '/test'}, clear=True)
    def test_niri_uses_focused_monitor_not_first(self):
        outputs = {'DP-1': {'logical': {'scale': 1}},
                   'eDP-1': {'logical': {'scale': 2}}, 'off': {'logical': None}}
        workspaces = [{'output': 'eDP-1', 'is_focused': True}]
        with patch.object(scale, 'json_output', side_effect=[outputs, workspaces]):
            self.assertEqual(scale.detect_scale()[0], 2)
        with patch.object(scale, 'json_output', side_effect=[outputs, workspaces]):
            self.assertEqual(scale.detect_scale(output='DP-1')[0], 1)

    def test_ambiguous_outputs_do_not_guess(self):
        self.assertIsNone(scale.choose_output([
            {'name': 'a', 'scale': 1}, {'name': 'b', 'scale': 2}]))

    @patch.dict('os.environ', {'SWAYSOCK': '/test'}, clear=True)
    def test_sway_fractional(self):
        with patch.object(scale, 'json_output', return_value=[
            {'name': 'a', 'scale': 1.5, 'focused': True, 'active': True}
        ]):
            self.assertEqual(scale.detect_scale()[0], 1.5)

    @patch.dict('os.environ', {}, clear=True)
    def test_xft_dpi_and_missing_desktop(self):
        with patch.object(scale, 'run', return_value='Xft.dpi:\t144\n'):
            self.assertEqual(scale.detect_scale()[0], 1.5)
        with patch.object(scale, 'run', return_value=''):
            self.assertEqual(scale.detect_scale()[0], 1)


if __name__ == '__main__':
    unittest.main()
