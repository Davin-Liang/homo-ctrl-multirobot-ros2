import importlib.util
import sys
from pathlib import Path
import numpy as np

PATH = Path(__file__).resolve().parents[1] / "scripts" / "sim_6d_disc_artstein_hocbf_compare.py"

def load():
    spec = importlib.util.spec_from_file_location("coupled", PATH)
    m = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = m
    spec.loader.exec_module(m)
    return m

def test_governor_enters_bypass_and_returns_to_normal():
    m = load()
    g = m.LocalReferenceGovernor(activation_radius=1.2, release_radius=1.5,
                                 return_tolerance=0.05)
    target = np.array([3.0, 0.0])
    assert g.update(np.array([0.0, 0.0]), np.array([1.0, 0.0]), target).state == "BYPASS"
    assert g.update(np.array([3.0, 0.0]), np.array([1.0, 0.0]), target).state == "RETURN"
    assert g.update(np.array([3.0, 0.0]), np.array([1.0, 0.0]), target).state == "NORMAL"
