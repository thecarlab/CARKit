#!/usr/bin/env bash
# Called by the VESC profile image and installer; leave OSRacer untouched.
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
mode=${1:---ensure}
if [[ "$mode" != --ensure && "$mode" != --check ]]; then
    echo "Usage: $0 [--ensure|--check]" >&2
    exit 2
fi

build_root=$(mktemp -d /tmp/carkit-image-transport.XXXXXX)
trap 'rm -rf "$build_root"' EXIT

# Use CMake's normal discovery, exactly as the camera build does. Compare
# actual resolved library paths, not Python cv2's unrelated wheel version.
mkdir "$build_root/probe"
cat > "$build_root/probe/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.14)
project(carkit_opencv_probe LANGUAGES CXX)
find_package(OpenCV 4 REQUIRED CONFIG COMPONENTS opencv_core opencv_imgproc opencv_imgcodecs)
file(WRITE "${CMAKE_BINARY_DIR}/opencv-version" "${OpenCV_VERSION}")
file(WRITE "${CMAKE_BINARY_DIR}/opencv-dir" "${OpenCV_DIR}")
file(GENERATE OUTPUT "${CMAKE_BINARY_DIR}/opencv-core" CONTENT "$<TARGET_FILE:opencv_core>")
CMAKE
cmake -S "$build_root/probe" -B "$build_root/probe-build" > "$build_root/probe.log" 2>&1 || {
    cat "$build_root/probe.log" >&2
    exit 1
}
opencv_version=$(cat "$build_root/probe-build/opencv-version")
opencv_dir=$(cat "$build_root/probe-build/opencv-dir")
opencv_core=$(cat "$build_root/probe-build/opencv-core")
echo "CARKit image transport: detected OpenCV $opencv_version ($opencv_dir)"

compatible_libraries() {
    python3 - "$opencv_core" <<'PY'
import pathlib, re, subprocess, sys
expected = pathlib.Path(sys.argv[1]).resolve()
valid = True
for name in ('libcv_bridge.so', 'libcompressed_image_transport.so'):
    library = pathlib.Path('/opt/ros/humble/lib') / name
    result = subprocess.run(['ldd', str(library)], capture_output=True, text=True)
    cores = {pathlib.Path(p).resolve() for p in re.findall(
        r'libopencv_core\S*\s+=>\s+(\S+)', result.stdout)}
    matches = (result.returncode == 0 and 'not found' not in result.stdout
               and cores == {expected})
    print(f'{name}: {"compatible" if matches else "rebuild required"}')
    valid &= matches
sys.exit(0 if valid else 1)
PY
}

if compatible_libraries; then
    echo 'CARKit image transport: already compatible; skipping rebuild.'
    exit 0
fi
[[ "$mode" != --check ]] || exit 1
if [[ $EUID -ne 0 ]]; then
    echo 'CARKit image transport repair requires root inside the container.' >&2
    exit 1
fi

fetch_source() {
    local repository=$1 commit=$2 destination=$3
    git init -q "$destination"
    git -C "$destination" fetch -q --depth 1 "$repository" "$commit"
    git -C "$destination" checkout -q --detach FETCH_HEAD
}

# Pin the ROS Humble sources so new machines get the tested interfaces.
fetch_source https://github.com/ros-perception/vision_opencv.git \
    9800f67cea477c44cfb64e349854bcb6a09dc9ce "$build_root/vision_opencv"
fetch_source https://github.com/ros-perception/image_transport_plugins.git \
    d4691b2fff53a9b575c140dee1f9264dcfec438a "$build_root/image_transport_plugins"

for package in vision_opencv/cv_bridge image_transport_plugins/compressed_image_transport; do
    package_name=${package##*/}
    cmake -S "$build_root/$package" -B "$build_root/build/$package_name" \
        -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
        -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/opt/ros/humble \
        -DOpenCV_DIR="$opencv_dir" -DPython3_EXECUTABLE=/usr/bin/python3
    cmake --build "$build_root/build/$package_name" --parallel 2
    # Install package artifacts, preserving ROS's top-level setup scripts.
    cmake --install "$build_root/build/$package_name"
done
ldconfig

compatible_libraries
echo "CARKit image transport: rebuilt and verified against OpenCV $opencv_version."
