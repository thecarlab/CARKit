# Interfaces

Package: `carkit_lidarslam_msgs`

Custom messages used by CARKit mapping.

## Build Just Interfaces

```bash
colcon build --symlink-install --packages-select carkit_lidarslam_msgs
source install/setup.bash
```

## Test

```bash
ros2 interface show carkit_lidarslam_msgs/msg/SubMap
ros2 interface show carkit_lidarslam_msgs/msg/MapArray
```

Messages:

- `SubMap.msg`
- `MapArray.msg`
