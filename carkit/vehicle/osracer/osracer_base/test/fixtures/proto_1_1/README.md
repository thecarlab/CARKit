# Proto 1.1 Public Fixtures

These fixtures describe public host-visible Proto 1.1 behavior only.

The machine-readable fixture records its exact behavior anchor for deterministic
parser regression. That provenance field is test metadata, not a runtime
dependency or installation instruction.

- All identifiers and numeric telemetry values are synthetic.
- No firmware source, device serial number, calibration value, or site log is
  included.
- The samples cover framing, command order, sensor units, covariance defaults,
  capability validation, and timeout behavior; they are not vehicle
  configurations.
- Capability values and identifiers are anonymous synthetic test data.
- Geometry, directional operating limits, steering range, and battery display
  range are read from the controller for each serial connection.
- The synchronized telemetry frame has exactly 18 whitespace-separated fields.
- `firmware_contract.json` defines the public protocol version, command units,
  and vehicle-capability response fields used by host compatibility tests.
