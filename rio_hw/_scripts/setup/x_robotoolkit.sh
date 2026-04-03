#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

uv pip install cmake pybind11 setuptools
echo "[OK] cmake $(cmake --version | head -1)"
# Point CMake at pybind11's cmake config so find_package(pybind11) succeeds
export CMAKE_PREFIX_PATH="$(python -m pybind11 --cmakedir)"
echo "[OK] pybind11 cmake dir: $CMAKE_PREFIX_PATH"

# Clone pybind wrapper
mkdir -p "$REPO_ROOT/third_party"
XRT_DIR="$REPO_ROOT/third_party/XRoboToolkit-PC-Service-Pybind"
if [ ! -d "$XRT_DIR" ]; then
    git clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind.git "$XRT_DIR"
fi

# Build PXREARobotSDK from upstream source
echo "[INFO] Building PXREARobotSDK"
XRT_TMP="$XRT_DIR/tmp"
mkdir -p "$XRT_TMP"
if [ ! -d "$XRT_TMP/XRoboToolkit-PC-Service" ]; then
    git clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service.git "$XRT_TMP/XRoboToolkit-PC-Service"
fi
bash "$XRT_TMP/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/build.sh"
mkdir -p "$XRT_DIR/lib" "$XRT_DIR/include"
cp "$XRT_TMP/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/PXREARobotSDK.h" \
    "$XRT_DIR/include/"
cp -r "$XRT_TMP/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/nlohmann" \
    "$XRT_DIR/include/nlohmann/"
cp "$XRT_TMP/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/build/libPXREARobotSDK.so" \
    "$XRT_DIR/lib/"
rm -rf "$XRT_TMP"
echo "[OK] PXREARobotSDK library built and installed"

uv pip install --no-build-isolation -e "$REPO_ROOT/third_party/XRoboToolkit-PC-Service-Pybind/"
