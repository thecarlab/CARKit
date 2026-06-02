# CARKit Navigation

Package: `carkit_navigation`

Bringup orchestration layer for the Nav2 AV workflow. Combines `carkit_slam` (mapping) and `carkit_amcl` (localization + navigation) via a single launch entry point.

## Launch

```bash
# Mapping
ros2 launch carkit_bringup carkit_nav2_av.launch.py mode:=mapping

# Navigation
ros2 launch carkit_bringup carkit_nav2_av.launch.py \
  mode:=navigation \
  map:=/workspaces/CARKit/carkit/mapping/carkit_slam/maps/map.yaml
```

See `carkit/mapping/README.md` and `carkit/localization/README.md` for full workflow details.

## Package Structure

- `launch/bringup.launch.py` — orchestrates `carkit_slam` and `carkit_amcl` based on `mode` argument
