# Map Path Editor

Lightweight web editor for loading Nav2 occupancy maps and aligning reusable path shapes in the `map` frame.

## Quick Start

From the Jetson host or inside the CARKit container:

```bash
cd map_path_editor
./run.sh
```

Open `http://localhost:8010` in a browser.

Environment variables:

```bash
MAP_PATH_EDITOR_HOST=0.0.0.0 \
MAP_PATH_EDITOR_PORT=8010 \
MAP_PATH_EDITOR_MAP_DIR=/workspaces/CARKit/map \
./run.sh
```

Or pass CLI flags directly:

```bash
python3 server.py --host 0.0.0.0 --port 8010 --map-dir ../map
```

## What It Supports

- Lists `.yaml` maps from the configured map directory and loads their referenced `.pgm` images.
- Loads a local YAML/PGM pair from the browser without writing files to disk.
- Aligns dynamic path templates:
  - straight line
  - circle
  - half circle
  - track: straight, half circle, straight, half circle
- Lets the user drag the path on the map, rotate it, and adjust dimensions in meters.
- Exports JSON with the shape parameters and sampled path points in the `map` frame.

## Controls

- Drag on the canvas to place or move the path.
- Shift-drag or middle-drag to pan the map.
- Use the mouse wheel or Zoom slider to zoom.
- Use Fit map to reset the viewport.

The editor uses the map YAML `origin` and `resolution` fields to convert between image pixels and world coordinates. Exported points are in meters and use `frame_id: "map"`.

## Requirements

- Python 3 standard library only.
- A Nav2-style occupancy map YAML plus PGM image.

The backend does not publish to ROS 2. Use the exported JSON as an input to a ROS node or converter when you want to publish a `nav_msgs/Path`.
