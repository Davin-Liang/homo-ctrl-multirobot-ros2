import importlib.util
from pathlib import Path
import unittest

import numpy as np


SCRIPT_PATH = Path(__file__).parents[1] / "scripts" / "sim_6d_map_hpc_artstein_compare.py"
SPEC = importlib.util.spec_from_file_location("sim_6d_map_hpc_artstein_compare", SCRIPT_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class MapFrameModelTest(unittest.TestCase):
    def test_map_error_is_zero_at_fixed_map_offset(self):
        leader = np.array([1.0, -2.0, 0.4, 0.2, -0.1, 0.3])
        offset = np.array([-1.0, 0.5])
        follower = np.array([0.0, -1.5, leader[2], 0.0, 0.0, 0.3])
        follower[3:5] = MODULE.rot(follower[2]).T @ (
            MODULE.rot(leader[2]) @ leader[3:5]
        )
        np.testing.assert_allclose(
            MODULE.map_error(leader, follower, offset), np.zeros(6), atol=1e-12
        )

    def test_nominal_homogeneous_identities_are_machine_precision(self):
        values = MODULE.verify_nominal_identities(2.0, 1.0, -0.25, 1.2, 2.0)
        self.assertEqual(values["controllability_rank"], 6)
        self.assertLess(
            max(value for key, value in values.items() if key != "controllability_rank"),
            1e-10,
        )


if __name__ == "__main__":
    unittest.main()
