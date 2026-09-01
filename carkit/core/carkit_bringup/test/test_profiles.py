from pathlib import Path
import unittest

import yaml


PROFILE_DIR = Path(__file__).parents[1] / "config" / "profiles"
LAUNCH_FILE = Path(__file__).parents[1] / "launch" / "carkit.launch.py"
COMPONENTS = {"chassis", "sensors", "planning", "control", "perception", "behavior"}
ALGORITHMS = {"planning", "control", "perception"}


def profiles():
    return {
        path.stem: yaml.safe_load(path.read_text(encoding="utf-8"))
        for path in PROFILE_DIR.glob("*.yaml")
    }


class TestProfiles(unittest.TestCase):
    def test_bringup_uses_native_web_bridge(self):
        launch_source = LAUNCH_FILE.read_text(encoding="utf-8")
        self.assertIn('package="carkit_web_bridge"', launch_source)
        self.assertIn('executable="web_bridge_node"', launch_source)
        self.assertNotIn('package="rosbridge_server"', launch_source)

    def test_expected_profiles_exist(self):
        self.assertEqual(
            set(profiles()), {"reference", "ada_high_school", "intro2av"}
        )

    def test_all_profiles_have_the_same_contract(self):
        for profile in profiles().values():
            self.assertEqual(set(profile["components"]), COMPONENTS)
            self.assertEqual(set(profile["implementations"]), ALGORITHMS)

    def test_profiles_select_explicit_course_packages(self):
        loaded = profiles()
        self.assertEqual(
            set(loaded["intro2av"]["implementations"].values()),
            {"intro2av_python"},
        )
        self.assertEqual(
            set(loaded["ada_high_school"]["implementations"].values()),
            {"ada_academy"},
        )
        self.assertEqual(
            set(loaded["reference"]["implementations"].values()),
            {"reference"},
        )
