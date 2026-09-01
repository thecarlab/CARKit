from pathlib import Path

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from sensor_msgs.msg import CompressedImage

from carkit_intro2av.control_algorithm import (
    ControlConfig,
    compute_command,
)
from carkit_intro2av.perception_algorithm import (
    PerceptionConfig,
    process_image,
)
from carkit_intro2av.planning_algorithm import (
    PlanningConfig,
    compute_path,
)


def test_each_algorithm_has_an_explicit_student_todo():
    package = Path(__file__).parents[1] / "carkit_intro2av"
    for name in (
        "planning_algorithm.py",
        "control_algorithm.py",
        "perception_algorithm.py",
    ):
        source = (package / name).read_text(encoding="utf-8")
        assert "TODO(Intro2AV)" in source


def test_control_boilerplate_defaults_to_zero_commands():
    source = (
        Path(__file__).parents[1]
        / "carkit_intro2av"
        / "control_algorithm.py"
    ).read_text(encoding="utf-8")
    assert "speed_mps: float = 0.0" in source
    assert "steering_angle_rad: float = 0.0" in source


def test_node_wrappers_are_substantial_and_parameterized():
    package = Path(__file__).parents[1] / "carkit_intro2av"
    expected = {
        "planning.py": ("OccupancyGrid", "planning_rate_hz", "_validated_path"),
        "control.py": ("input_timeout_sec", "maximum_steering_rad", "math.isfinite"),
        "perception.py": ("CameraInfo", "max_inference_rate_hz", "_validate_result"),
    }
    for name, markers in expected.items():
        source = (package / name).read_text(encoding="utf-8")
        assert len(source.splitlines()) >= 100
        assert all(marker in source for marker in markers)


def test_package_contains_reference_style_launch_and_configs():
    root = Path(__file__).parents[1]
    assert (root / "launch" / "algorithms.launch.py").is_file()
    for component in ("planning", "control", "perception"):
        assert (root / "config" / f"{component}.yaml").is_file()


def test_unimplemented_algorithms_return_safe_typed_results():
    plan = compute_path(
        OccupancyGrid(),
        Odometry(),
        PoseStamped(),
        PlanningConfig(65, False, 0.25, 0.10, 0.15),
    )
    command = compute_command(
        Odometry(),
        RosPath(),
        ControlConfig(0.325, 0.55, 0.45, 1.0, 0.34, 0.15),
    )
    perception = process_image(
        CompressedImage(), None, PerceptionConfig(0.2, 448, "")
    )
    assert plan == []
    assert command.speed_mps == 0.0
    assert command.steering_angle_rad == 0.0
    assert perception.detections == []
    assert perception.traffic_lights == []
