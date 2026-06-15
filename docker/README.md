# Docker

CARKit uses `ariiees/carkit:latest` as the single environment image. The image contains ROS 2 Humble, build tools, RViz, Python tools, RealSense SDK, and common CARKit system dependencies. It also includes the ROS dependencies needed to build the vendored F1TENTH/ADA control stack. It does not contain a baked copy of the CARKit workspace.

The image builds and installs the native `librealsense2` SDK from source for Jetson, then the mounted workspace builds `realsense2_camera` from the cloned `realsense-ros` source.

The former CARKit control container (`ariiees/ada:foxy-f1tenth`) is no longer used. Its F1TENTH/ADA source is vendored in this repository under `carkit/vehicle/f1tenth_system` and builds inside this same Humble image.

## User Flow

```bash
git clone https://github.com/thecarlab/CARKit.git
cd CARKit
docker pull ariiees/carkit:latest
./docker/run_jetson.sh
```

`run_jetson.sh` pulls `ariiees/carkit:latest` by default before launching. Use `PULL_IMAGE=never ./docker/run_jetson.sh` when testing a local image that should not be refreshed from Docker Hub.

Before opening the interactive shell, `run_jetson.sh` verifies that the selected
image contains the Nav2 packages used by `carkit_navigation`, including
`nav2_bringup`, `nav2_smac_planner`, `nav2_regulated_pure_pursuit_controller`,
and `slam_toolbox`. This prevents accidentally launching an older image that
cannot run CARKit Nav2 navigation.

The container runs commands as root by default. This keeps hardware access,
ROS graph ownership, and repeated CARKit terminals consistent on the
Jetson.

Overrides:

```bash
# Run as your host UID/GID instead of root.
CARKIT_RUN_AS_ROOT=0 ./docker/run_jetson.sh

# Temporarily skip the Nav2 image preflight check.
CARKIT_REQUIRE_NAV2=0 ./docker/run_jetson.sh

# Do not repair old generated-file ownership when running as host UID/GID.
CARKIT_FIX_PERMISSIONS_ON_START=0 ./docker/run_jetson.sh
```

Inside Docker:

```bash
./docker/build_workspace.sh
source install/setup.bash
ros2 launch carkit_navigation navigation.launch.py
```

`build_workspace.sh` defaults to `BUILD_JOBS=1` and `PARALLEL_WORKERS=1` to avoid out-of-memory failures on an 8GB Jetson Orin Nano. Increase those environment variables only on systems with more memory.

## Scripts

- `run_jetson.sh`: refreshes the image by default, then starts the container with host networking, NVIDIA runtime when registered, `/dev`, `/dev/shm`, X11 display mounts, and root ROS runtime ownership by default.
- `build_workspace.sh`: clones external source packages, runs `rosdep`, builds with `colcon`, and lists CARKit packages.
- `publish_image.sh`: maintainer-only helper to build and push `ariiees/carkit:latest`.
- `test_workspace_in_docker.sh`: pulls/runs the image, builds this checkout, and checks launch arguments.

## Maintainer Publish

```bash
docker login
./docker/publish_image.sh
```

To build and run the Nav2 image preflight check without pushing:

```bash
PUSH_IMAGE=0 ./docker/publish_image.sh
```

Never put Docker Hub credentials in this repository.

## Python ML Packages

The Jetson Dockerfile pins the ML stack used to build and run the FP16
TensorRT engine:

- NVIDIA JetPack 6.1 PyTorch
  `2.5.0a0+872d972e41.nv24.08` with CUDA 12.6
- Torchvision `0.20.0`, built from source inside the image against NVIDIA's
  PyTorch wheel
- Ultralytics `8.4.54`
- NumPy `1.26.1`
- ONNX `1.17.0` for the temporary PyTorch-to-ONNX export step

Ultralytics is installed with `--no-deps` after its non-Torch dependencies.
This prevents pip from replacing the NVIDIA PyTorch build with a generic CUDA
wheel. Torchvision uses `FORCE_CUDA=1` and `MAX_JOBS=1`; the latter limits
memory pressure while compiling on the Jetson. The image also keeps
`setuptools<80` for `colcon-core` compatibility.

The Docker build can validate imports and compiled torchvision operations, but
CUDA device availability must be checked when the container runs with the
NVIDIA runtime:

```bash
python3 - <<'PY'
import torch
import torchvision

print(torch.__version__)
print(torch.version.cuda)
print(torch.cuda.is_available())
print(torchvision.__version__)
print(torchvision.extension._has_ops())
PY
```

## Rosdep Notes

`build_workspace.sh` refreshes apt indexes before running `rosdep install` because the Docker image removes apt lists to keep the image smaller. It also skips the `librealsense2` rosdep key by default since `ros-humble-librealsense2` is not consistently available on Jetson ROS apt repositories.

Override skipped keys if needed:

```bash
ROSDEP_SKIP_KEYS="" ./docker/build_workspace.sh
```
