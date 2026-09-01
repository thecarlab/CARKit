# Intro2AV C++ boilerplates

This package mirrors the production shape of the reference stack and the
Python `carkit_intro2av` package:

- `src/planning.cpp`, `control.cpp`, `perception.cpp`: complete `rclcpp` node
  wrappers with parameters, SensorDataQoS, state, timers, validation, and
  fail-safe publication
- `src/*_algorithm.cpp`: the only intentionally empty student algorithms
- `include/carkit_intro2av_cpp/*_algorithm.hpp`: explicit algorithm contracts
- `config/*.yaml` and `launch/algorithms.launch.py`: reference-style runtime
  configuration and standalone bringup

Search for `TODO(Intro2AV C++)` in the algorithm sources. Select C++
independently for planning, control, and perception in the WebUI, save here,
and use the Compile tab before launching it. The unfinished controller always
returns a stop and remains behind `carkit_control_center`.
