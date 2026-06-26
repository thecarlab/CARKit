# Docker

CARKit uses `ariiees/carkit:latest` as the single Jetson runtime image. The
image contains ROS 2 Humble, Nav2, RViz, Foxglove Bridge, build tools,
RealSense SDK, TensorRT/CUDA Python ML dependencies, and the system
dependencies needed to build the mounted CARKit workspace.

The image does not contain a baked copy of this repository. `run_jetson.sh`
mounts the checkout at `/workspaces/CARKit`.

## User Flow

On the Jetson host:

```bash
git clone https://github.com/thecarlab/CARKit.git
cd CARKit
docker pull ariiees/carkit:latest
./docker/run_jetson.sh
```

Inside Docker:

```bash
./docker/build_workspace.sh
source install/setup.bash
```

`run_jetson.sh` pulls `ariiees/carkit:latest` only when it is missing locally.
Use `PULL_IMAGE=always ./docker/run_jetson.sh` to force a Docker Hub refresh,
or `PULL_IMAGE=never ./docker/run_jetson.sh` when testing a local image that
should not be refreshed from Docker Hub.

`./docker/run_jetson.sh` opens a shell in the mounted workspace. Build the
workspace, source it, then start the launches you need.

Before starting the container command, `run_jetson.sh` verifies that the
selected image contains the Nav2 and Foxglove runtime packages used by CARKit.
It also starts the container with host networking, `/dev`, `/dev/shm`, X11
display mounts, and NVIDIA runtime support when the runtime is registered.

Foxglove Bridge binds to `0.0.0.0:8765` when started by CARKit launches. From
another computer on the same network, connect Foxglove to:

```text
ws://<jetson-ip>:8765
```

The container runs as root by default so hardware access and repeated ROS
terminals stay consistent on the Jetson.

Overrides:

```bash
# Run as your host UID/GID instead of root.
CARKIT_RUN_AS_ROOT=0 ./docker/run_jetson.sh

# Temporarily skip the Nav2/Foxglove image preflight check.
CARKIT_REQUIRE_RUNTIME=0 ./docker/run_jetson.sh

# Do not repair old generated-file ownership when running as host UID/GID.
CARKIT_FIX_PERMISSIONS_ON_START=0 ./docker/run_jetson.sh
```

## Workspace Build

Inside Docker:

```bash
./docker/build_workspace.sh
source install/setup.bash
```

`build_workspace.sh`:

1. Fetches vendored source repos with `carkit/setup_vendor_repos.sh`.
2. Sources `/opt/ros/humble/setup.bash`.
3. Runs `rosdep install` for packages under `carkit/`.
4. Builds with `colcon build --symlink-install`.
5. Prints available `carkit` packages.

It defaults to `BUILD_JOBS=1` and `PARALLEL_WORKERS=1` to avoid out-of-memory
failures on an 8 GB Jetson Orin Nano.

## Scripts

- `run_jetson.sh`: pulls/runs the runtime image and mounts this checkout.
- `build_workspace.sh`: fetches vendor repos, installs ROS dependencies, and
  builds the workspace.
- `publish_image.sh`: maintainer helper to build, check, and push
  `ariiees/carkit:latest`.
- `test_workspace_in_docker.sh`: pulls/runs the image, builds this checkout,
  and checks launch arguments.

## Maintainer Publish

```bash
docker login
./docker/publish_image.sh
```

To build and run the image preflight check without pushing:

```bash
PUSH_IMAGE=0 ./docker/publish_image.sh
```

After a successful local publish, the local `ariiees/carkit:latest` image is
already the image that was pushed. Other machines should pull it with:

```bash
docker pull ariiees/carkit:latest
```

Never put Docker Hub credentials in this repository.

## Python ML Packages

`docker/Dockerfile.jetson` installs NVIDIA's JetPack PyTorch wheel, builds a
matching torchvision from source, installs ONNX and Ultralytics with pinned
constraints, keeps NumPy compatible with ROS 2 Humble, and keeps
`setuptools<80` so `colcon-core` remains compatible.

## Rosdep Notes

`build_workspace.sh` refreshes apt indexes before running `rosdep install`
because the Docker image removes apt lists to keep the image smaller. It skips
the `librealsense2` rosdep key by default because CARKit builds and installs
the native RealSense SDK in the Docker image.

Override skipped keys if needed:

```bash
ROSDEP_SKIP_KEYS="" ./docker/build_workspace.sh
```
