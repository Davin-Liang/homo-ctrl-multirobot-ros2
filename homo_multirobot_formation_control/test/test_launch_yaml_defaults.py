import re
import unittest
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]
CONFIG_DIR = PACKAGE_DIR / "config"
LAUNCH_DIR = PACKAGE_DIR / "launch"
CMAKE_FILE = PACKAGE_DIR / "CMakeLists.txt"
PACKAGE_XML = PACKAGE_DIR / "package.xml"

LAUNCH_TO_YAML = {
    "formation_single_follower.launch.py": "formation_single_follower.yaml",
    "formation_single_follower_4d_artstein.launch.py": (
        "formation_single_follower_4d_artstein.yaml"
    ),
    "formation_single_follower_6d_disc.launch.py": (
        "formation_single_follower_6d_disc.yaml"
    ),
    "formation_single_follower_6d_artstein_disc_hocbf.launch.py": (
        "formation_single_follower_6d_artstein_disc_hocbf.yaml"
    ),
    "formation_single_follower_6d_map_hpc_artstein.launch.py": (
        "formation_single_follower_6d_map_hpc_artstein.yaml"
    ),
}


def yaml_parameter_names(path):
    return {
        line.split(":", 1)[0].strip()
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.startswith("    ") and not line.lstrip().startswith("#")
    }


def declared_argument_names(path):
    source = path.read_text(encoding="utf-8")
    if "for name, value in defaults.items()" in source:
        return None
    return set(re.findall(r'DeclareLaunchArgument\("([^\"]+)"', source))


class TestLaunchYamlDefaults(unittest.TestCase):
    def test_yaml_keys_match_declared_launch_arguments(self):
        for launch_name, yaml_name in LAUNCH_TO_YAML.items():
            with self.subTest(launch=launch_name):
                yaml_path = CONFIG_DIR / yaml_name
                self.assertTrue(yaml_path.is_file())
                declared_names = declared_argument_names(LAUNCH_DIR / launch_name)
                if declared_names is None:
                    source = (LAUNCH_DIR / launch_name).read_text(encoding="utf-8")
                    self.assertIn("for name, value in defaults.items()", source)
                    self.assertIn(
                        "{name: LaunchConfiguration(name) for name in defaults}", source)
                else:
                    self.assertEqual(declared_names, yaml_parameter_names(yaml_path))

    def test_launch_uses_yaml_for_defaults_before_overrides(self):
        for launch_name, yaml_name in LAUNCH_TO_YAML.items():
            with self.subTest(launch=launch_name):
                source = (LAUNCH_DIR / launch_name).read_text(encoding="utf-8")
                self.assertIn("yaml.safe_load", source)
                self.assertIn("_launch_default(defaults[name])", source)
                self.assertLess(source.index(yaml_name), source.index("parameters=["))

    def test_runtime_dependencies_and_ctest_registration_are_declared(self):
        self.assertIn("<exec_depend>python3-yaml</exec_depend>",
                      PACKAGE_XML.read_text(encoding="utf-8"))
        self.assertIn("add_test(NAME test_launch_yaml_defaults",
                      CMAKE_FILE.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
