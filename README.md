<p align="center">
  <a href="https://github.com/astral-sh/uv">
  <img src="https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/uv/main/assets/badge/v0.json" /></a>
  <a href="https://github.com/astral-sh/ruff">
  <img src="https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/ruff/main/assets/badge/v2.json" /></a>
  <a href="https://github.com/astral-sh/ty">
  <img src="https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/ty/main/assets/badge/v0.json" /></a>
</p>

# RIO-HW

RIO-HW 🎛️ is a real-time control library for cross-embodiment robot hardware.

# Setup

Tested on: Ubuntu 22.04 LTS (optional: real-time kernel patch). See [`docs/ubuntu.md`](docs/ubuntu.md) for setup instructions.

```bash
# install uv
curl -LsSf https://astral.sh/uv/install.sh | sh

# create venv
uv venv --python 3.10
source .venv/bin/activate
uv sync --all-extras
```

> See [[Hardware Guide (Google Docs)]](https://docs.google.com/document/d/1_NbHk4z9HABPnaqow-VP-srbQBz42kBXCbvbyiJrW74) for hardware BOM and example robot stations.

### Cameras

Realsense: [`docs/cameras/realsense.md`](docs/cameras/realsense.md)
```bash
sudo ./scripts/setup/realsense.sh
# verify install
realsense-viewer
# Note: if you do not need L515 support, then you can use newer versions of librealsense2/pyrealsense2 >2.54.2
```

Record3d (iOS device): Download the [Record3D app](https://record3d.app/) and keep the app running in the background. You may need to pay for both "Full Version" and "Wi-Fi Streaming & RGBD Videos" in-app purchases or restore purchases.
```bash
sudo ./scripts/setup/record3d.sh
# Enable "Settings -> Higher-quality LiDAR recording" to capture data at higher resolution.
```

Zed: To install ZED SDK for the first time, see [Stereolabs Docs: Install the ZED Python API](https://www.stereolabs.com/docs/development/python/install).
```bash
# (re)install pyzed
python /usr/local/zed/get_python_api.py
# verify install
/usr/local/zed/tools/ZED_Explorer
# fix Error: "Can't save calibration file.
sudo chmod -R 777 /usr/local/zed/settings
```

### Interfaces

AVP Stream (Apple Vision Pro): Download the [Tracking Streamer app](https://apps.apple.com/us/app/tracking-streamer/id6478969032). Run the app and click `Start` to stream tracking data over WiFI network. Click the digital crown to stop streaming.

Gamepad: [`docs/interfaces/gamepad.md`](docs/interfaces/gamepad.md)

Gello: [`docs/interfaces/gello.md`](docs/interfaces/gello.md)

Oculus Reader (Quest 2): [`rail-berkeley/oculus_reader/README.md`](https://github.com/rail-berkeley/oculus_reader?tab=readme-ov-file#setup-of-the-adb)

Spacemouse:
```bash
sudo ./scripts/setup/spacemouse.sh
# verify install
systemctl status spacenavd
```

Vuer: [`docs/interfaces/vuer.md`](docs/interfaces/vuer.md)

XRoboToolkit: [`docs/interfaces/x_robotoolkit.md`](docs/interfaces/x_robotoolkit.md)

### Robots

#### Arms

Franka: [`docs/robots/franka.md`](docs/robots/franka.md)

Kinova: [`docs/robots/kinova.md`](docs/robots/kinova.md)

UR: [`docs/robots/ur.md`](docs/robots/ur.md)

XArm [`docs/robots/xarm.md`](docs/robots/xarm.md)

#### Humanoids

Unitree G1: [`docs/robots/unitree_g1.md`](docs/robots/unitree_g1.md)

# Usage

A `Node` dynamically inherits from any given `Middleware` to handle automatically message passing, and factory functions produce `Server` and `Client` nodes. Each `Node` publishes data and handles requests, in separate loops or in the same loop.
1. `Node.pub()` only publishes data.
2. `Node.req()` only handles requests.
3. `Node.pubreq()` both publishes data and handles requests.
4. `Node.req()` only handles requests, and `Node.pub()` publishes data in a separate worker.
5. `Node.pub()` only publishes data, and `Node.req()` handles requests in a separate worker.

Users only need to implement "pub/req" behavior in nodes through `Node.pub() / Node.req() / Node.pubreq()`. A "pub" loop should call `ring_buffer.put()` to publish data, and a "req" loop should call `request_queue.get()` to handle requests. A `Server` runs "pub/req" ("publish"/"request") and a `Client` resolves "sub/rep" ("subscribe"/"reply") automatically. See [`template.py`](rio_hw/_template/template.py) for an example outline of a `Node`.

### Conventions

- SI units: `seconds`, `meters`, `kilograms`, `radians`
- Quaternion ordering: `(x, y, z, w)`
- SE3 matrix: `[R t; 0 1]` where `R` is 3x3 rotation matrix and `t` is 3x1 translation vector.
- Linear velocity: COM, world frame
- Angular velocity: world frame
- Naming:
  - `*_frame`: coordinate system
  - `a_b_tf`: rigid-body transform $T_{a}^{b}$ from frame $a$ to frame $b$ (frame $a$ in coordinates of frame $b$)
  - `*_pose`: pose of an entity in a frame
- Coordinate system: `Z` up, right-handed
- Other conventions:
  - Images: `(height, width, channel)`, RGB channel order
  - Robot base frame: `+X` front, `+Y` left, `+Z` up
  - Robot eef frame: at mounting flange / tool base
  - Gripper: `[0, 1]` -> `[closed, open]`

# Acknowledgements
- [`real-stanford/DexUMI`](https://github.com/real-stanford/DexUMI)
- [`real-stanford/universal_manipulation_interface`](https://github.com/real-stanford/universal_manipulation_interface)
- [`real-stanford/diffusion_policy`](https://github.com/real-stanford/diffusion_policy)
