# Localization

Package: `carkit_lidar_localization`

NDT LiDAR localization against a PCD map.

## Launch

```bash
ros2 launch carkit_lidar_localization lidar_localization.launch.py
```

This opens RViz by default. Use the `2D Pose Estimate` tool in RViz to publish
`/initialpose`; localization starts publishing `/pcl_pose` after both the map
and initial pose are available.

RViz shows the saved map on `/map` and `/initial_map`. The live `/cloud_in`
display is shown in the sensor frame until the first initial pose is set; after
localization publishes `map -> base_link`, the cloud can be compared against
the map in the `map` frame.

Use a custom map:

```bash
ros2 launch carkit_lidar_localization lidar_localization.launch.py map_path:=/path/to/map.pcd
```

Use the previous root-level generated map from the CARKit Docker workspace:

```bash
ros2 launch carkit_lidar_localization lidar_localization.launch.py \
  map_path:=/workspaces/CARKit/map.pcd
```

Use the newer saved-map location:

```bash
ros2 launch carkit_lidar_localization lidar_localization.launch.py \
  map_path:=/workspaces/CARKit/map/map.pcd
```

Disable RViz for headless tests:

```bash
ros2 launch carkit_lidar_localization lidar_localization.launch.py use_rviz:=false
```

## Test

Start SLLiDAR and `carkit_sensor_transforms` first, then:

```bash
ros2 topic echo /cloud_in --once
ros2 topic echo /pcl_pose --once
ros2 topic echo /initial_map --once
```

Inputs:

- `/cloud_in` (`sensor_msgs/PointCloud2`)
- `/initialpose` (`geometry_msgs/PoseWithCovarianceStamped`)
- `/map` (`sensor_msgs/PointCloud2`) when not using `map_path`

Outputs:

- `/pcl_pose` (`geometry_msgs/PoseWithCovarianceStamped`)
- `/path` (`nav_msgs/Path`)
- `/initial_map` (`sensor_msgs/PointCloud2`)
