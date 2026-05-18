# Localization

Package: `carkit_lidar_localization`

NDT LiDAR localization against a PCD map.

## Launch

```bash
ros2 launch carkit_lidar_localization lidar_localization.launch.py
```

Use a custom map:

```bash
ros2 launch carkit_lidar_localization lidar_localization.launch.py map_path:=/path/to/map.pcd
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
