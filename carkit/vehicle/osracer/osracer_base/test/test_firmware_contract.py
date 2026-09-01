import json
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONTRACT_PATH = ROOT / "test/fixtures/proto_1_1/firmware_contract.json"


class PublicFirmwareContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.contract = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))

    def test_contract_is_minimal_and_sanitized(self):
        self.assertEqual(
            set(self.contract),
            {"schema_version", "protocol", "command", "vehicle_capability"},
        )
        self.assertEqual(self.contract["schema_version"], 1)
        self.assertEqual(self.contract["protocol"], "1.1")
        self.assertEqual(
            self.contract["command"],
            {
                "name": "v",
                "linear_velocity_unit": "m/s",
                "steering_angle_unit": "deg",
            },
        )
        capability = self.contract["vehicle_capability"]
        self.assertEqual(capability["request"], "vehicle get\n")
        self.assertEqual(capability["contract"], 1)
        self.assertEqual(
            capability["response_fields"],
            [
                "Contract",
                "Profile",
                "Schema",
                "WheelbaseMm",
                "ForwardMaxMmps",
                "ReverseMaxMmps",
                "SteeringMaxMdeg",
                "BatteryMinMv",
                "BatteryMaxMv",
            ],
        )
        self.assertEqual(
            capability["units"],
            {
                "WheelbaseMm": "mm",
                "ForwardMaxMmps": "mm/s",
                "ReverseMaxMmps": "mm/s",
                "SteeringMaxMdeg": "millidegree",
                "BatteryMinMv": "mV",
                "BatteryMaxMv": "mV",
            },
        )

        serialized = json.dumps(self.contract, sort_keys=True).lower()
        for private_field in (
            "gpio",
            "encoder",
            "hardware",
            "manufacturer",
            "nvs",
            "pid",
            "pwm",
            "product_name",
            "wheel_radius",
        ):
            self.assertNotIn(private_field, serialized)

    def test_contract_contains_no_vehicle_mapping_or_physical_values(self):
        serialized = json.dumps(self.contract, sort_keys=True).lower()
        for vehicle_name in ("neo", "red", "blue"):
            self.assertNotRegex(serialized, rf"\b{vehicle_name}\b")
        self.assertNotIn("profiles", self.contract)


if __name__ == "__main__":
    unittest.main()
