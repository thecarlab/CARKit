# Navigation Cleanup

CARKit now uses one supported autonomous navigation layout:

- Nav2 localization, mapping, and planning live under `carkit/navigation/`.
- All occupancy maps live under the repository's top-level `map/` folder.
- Retired map/localization artifacts and unused SLAM message interfaces were
  removed.
- The supported mapping workflow uses SLAM Toolbox.
- The supported localization and planning workflow uses AMCL and Nav2.
- Human control uses the joystick-only `carkit_human_control` launch.
- The standalone keyboard and CARKit pure-pursuit control workflows were
  removed.
- The obsolete shared launcher module was removed. Perception, mapping,
  localization, and planning now own their RViz configurations.

ROS package names were kept stable so existing Nav2 launch commands continue
to use `carkit_navigation`, `carkit_amcl`, and `carkit_slam`.
