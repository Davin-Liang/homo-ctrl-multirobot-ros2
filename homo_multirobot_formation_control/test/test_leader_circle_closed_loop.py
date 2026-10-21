import importlib.util
import math
from pathlib import Path
import sys

import numpy as np


SCRIPT_PATH = (
    Path(__file__).parents[1] / 'scripts' / 'leader_circle_closed_loop.py')
CMAKE_PATH = Path(__file__).parents[1] / 'CMakeLists.txt'
SPEC = importlib.util.spec_from_file_location('leader_circle_closed_loop', SCRIPT_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_reference_heading_keeps_fixed_heading_by_default():
    assert MODULE.reference_heading('fixed', 0.73, np.array([0.0, 0.2])) == 0.73


def test_reference_heading_faces_ccw_and_cw_tangent_directions():
    ccw = MODULE.reference_heading('tangent', 0.0, np.array([-0.2, 0.0]))
    cw = MODULE.reference_heading('tangent', 0.0, np.array([0.2, 0.0]))

    assert math.isclose(ccw, math.pi)
    assert math.isclose(cw, 0.0)


def test_node_declares_and_uses_heading_mode():
    source = SCRIPT_PATH.read_text(encoding='utf-8')

    assert "self.declare_parameter('heading_mode', 'fixed')" in source
    assert 'self.heading_mode = str(self.get_parameter(' in source
    assert 'reference_heading(self.heading_mode, self.heading,' in source


def test_ctest_registers_leader_circle_closed_loop_pytest():
    cmake_source = CMAKE_PATH.read_text(encoding='utf-8')

    assert 'add_test(NAME test_leader_circle_closed_loop' in cmake_source
