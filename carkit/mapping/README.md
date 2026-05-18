# Mapping

Mapping uses CARKit wrappers around LiDAR scan matching and graph-based SLAM. The `ndt_omp_ros2` package is vendored directly under this folder without its upstream `.git` metadata because CARKit carries local integration changes for mapping.

Packages:

- `carkit_scanmatcher`
- `carkit_graph_based_slam`
- `carkit_lidarslam`
- `ndt_omp_ros2`

## Third-Party NDT

`carkit/mapping/ndt_omp_ros2` is already part of this repository. Keep its upstream license and README in place when changing it.

## Launch Full Mapping

```bash
ros2 launch carkit_lidarslam lidarslam.launch.py
```

## Launch Parts

```bash
ros2 launch carkit_scanmatcher mapping_car.launch.py
ros2 launch carkit_graph_based_slam graphbasedslam.launch.py
```

## Test

Start LiDAR and `/cloud_in`, then:

```bash
ros2 topic echo /current_pose --once
ros2 topic echo /map --once
ros2 topic echo /map_array --once
ros2 service call /map_save std_srvs/srv/Empty
```

Inputs:

- `/input_cloud`, usually remapped from `/cloud_in` (`sensor_msgs/PointCloud2`)
- `/imu_transformed` (`sensor_msgs/Imu`, optional)
- `/initial_pose` (`geometry_msgs/PoseStamped`, optional)

Outputs:

- `/current_pose` (`geometry_msgs/PoseStamped`)
- `/map` (`sensor_msgs/PointCloud2`)
- `/map_array` (`carkit_lidarslam_msgs/MapArray`)
- `/path` (`nav_msgs/Path`)
- `/modified_map` (`sensor_msgs/PointCloud2`)
- `/modified_path` (`nav_msgs/Path`)
