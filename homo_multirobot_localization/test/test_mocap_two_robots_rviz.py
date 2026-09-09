from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "launch" / "mocap_two_robots.launch.py"
RVIZ = ROOT / "rviz" / "mocap_two_robots.rviz"
CMAKE = ROOT / "CMakeLists.txt"


def test_mocap_launch_starts_rviz_with_packaged_config():
    launch_source = LAUNCH.read_text()
    assert 'package="rviz2"' in launch_source
    assert 'executable="rviz2"' in launch_source
    assert '"-d"' in launch_source
    assert '"rviz", "mocap_two_robots.rviz"' in launch_source


def test_mocap_rviz_config_is_packaged_and_targets_two_robots():
    config_source = RVIZ.read_text()
    assert "Fixed Frame: map" in config_source
    assert "/robot1/robot_description" in config_source
    assert "/robot2/robot_description" in config_source
    assert "DIRECTORY launch config rviz" in CMAKE.read_text()
