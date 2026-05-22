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

Start the LiDAR and publish `/cloud_in` first:

```bash
ros2 launch sllidar_ros2 sllidar_s2_launch.py
ros2 run carkit_sensor_transforms lidar_transformer_node
```

Then launch the full mapping stack. This starts scan matching, graph-based SLAM,
and RViz with the mapping view loaded:

```bash
ros2 launch carkit_lidarslam lidarslam.launch.py
```

In RViz, use `map` as the fixed frame and watch `/cloud_in`, `/map`, `/path`,
and `/modified_map`. Drive or push the car slowly through the environment; the
map point cloud should grow as new scans arrive.

Save the generated map:

```bash
ros2 service call /map_save std_srvs/srv/Empty
```

The save service writes `map.pcd` and `pose_graph.g2o` in the directory where
the mapping launch process was started.

## Launch Parts

```bash
ros2 launch carkit_scanmatcher mapping_car.launch.py
ros2 launch carkit_graph_based_slam graphbasedslam.launch.py
```

## Test

Start LiDAR and `/cloud_in`, then:

```bash
ros2 topic hz /cloud_in
ros2 topic echo /current_pose --once
ros2 topic echo /map --once
ros2 topic echo /map_array --once
ros2 service list | grep map_save
ros2 service call /map_save std_srvs/srv/Empty
ls -lh map.pcd pose_graph.g2o
```

If RViz is blank, first confirm `/cloud_in` is publishing and that RViz fixed
frame is `map`. If `/map_save` prints `initial map is not received`, move the
car until `/map_array` publishes at least once.

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
