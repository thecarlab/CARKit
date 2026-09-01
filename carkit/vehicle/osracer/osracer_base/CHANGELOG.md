# Changelog

<!-- markdownlint-configure-file {"MD024": {"siblings_only": true}} -->

This changelog records user-visible changes to OSRacer Base.

## [Unreleased]

## [0.3.0] - 2026-08-15

### Added

- Reads and strictly validates the controller's vehicle capability contract on
  every serial connection before enabling motion.

### Changed

- Uses controller-reported wheelbase, separate forward and reverse speed
  limits, steering limit, and battery display range for the current connection.
- Revalidates firmware, profile, and vehicle identity after every reconnect.

### Removed

- Removes duplicate ROS vehicle-physics configuration files and the associated
  profile, geometry, speed, steering, and battery-range launch parameters.

## [0.2.0] - 2026-08-12

### Added

- Explicit vehicle configuration files with schema validation.
- Firmware-interface identity checks before motion commands are enabled.
- Machine-readable public serial-interface contract and protocol fixtures.
- Serial reconnection, connection-state diagnostics, device checks, and udev
  installation utilities.
- ROS 2 Humble and Jazzy build and test coverage.

### Changed

- Centralizes the ROS values required by the chassis driver: wheelbase,
  steering limit, speed limit, frame names, and battery display range.
- Publishes synchronized odometry and inertial data with one shared timestamp.
- Keeps velocity and Ackermann command interfaces active through one driver
  implementation and one command-timeout safety path.

### Fixed

- Rejects non-finite synchronized-motion, magnetic-field, and battery values
  before publishing ROS messages.
- Rate-limits repeated invalid-telemetry diagnostics.

## [0.1.0] - 2026-08-07

- Introduced the standalone ROS 2 chassis package, launch files, udev rule,
  velocity and Ackermann command interfaces, odometry and sensor publication,
  and the initial Humble/Jazzy CI workflow.

[0.3.0]: https://github.com/osrbot/osracer_base/releases/tag/v0.3.0
[0.2.0]: https://github.com/osrbot/osracer_base/releases/tag/v0.2.0
[0.1.0]: https://github.com/osrbot/osracer_base/releases/tag/v0.1.0
