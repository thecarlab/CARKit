# Intro2AV algorithms

This is the Python implementation selectable as **Intro2AV · Python** for
planning, control, and perception. Each component keeps its ROS interface and
safe runtime behavior while leaving the core algorithm as student work. The
parallel `carkit_intro2av_cpp` package has the same contract in C++.

The package deliberately has the same layers as the reference stack:

- `planning.py`, `control.py`, `perception.py`: complete ROS node wrappers,
  including parameters, QoS, timers, state, validation, and safe publication
- `*_algorithm.py`: the only student-owned algorithm implementations
- `config/*.yaml`: the same kind of runtime tuning surface as the reference
  packages
- `launch/algorithms.launch.py`: standalone launch entry point

Search for `TODO(Intro2AV)` in the three algorithm files. Planning receives
the occupancy grid, odometry, and goal. Control receives the current odometry
and complete path. Perception receives compressed images, calibration, typed
outputs, and a WebUI preview path. Until implemented, planning returns an
empty path, perception returns empty detections, and control returns a stop.
All commands still pass through `carkit_control_center`.
