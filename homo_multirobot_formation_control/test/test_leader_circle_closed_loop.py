import importlib.util
import math
from pathlib import Path
import sys

import numpy as np


SCRIPT_PATH = (
    Path(__file__).parents[1] / 'scripts' / 'leader_circle_closed_loop.py')
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
