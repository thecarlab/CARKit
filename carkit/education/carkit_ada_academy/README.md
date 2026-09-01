# ADA Academy algorithms

This ROS 2 package is selected for planning, control, and perception whenever
the WebUI course is **ADA Academy**. It is intentionally separate from the
protected reference packages.

Student-facing files:

- `carkit_ada_academy/planning.py`: guided goal-to-path generation
- `carkit_ada_academy/control.py`: guided pure-pursuit controller
- `carkit_ada_academy/perception.py`: confidence/class filtering around the
  reference TensorRT detector
- `carkit_ada_academy/math_utils.py`: small algorithms and tuning seams that
  can be tested without starting ROS

Stable outputs are `/plan`, `/drive`, and `/yolo/detections_2d`. `/drive`
always passes through `carkit_control_center`; this package never publishes
directly to the chassis command topic.
