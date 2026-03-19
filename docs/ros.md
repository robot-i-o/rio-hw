# ROS

We use Roslibpy, Rosbridge, and Robostack to support message passing between (system-less) ROS.

Tested: ROS 2 Jazzy (Robostack) on Ubuntu 22.04 LTS.

Reference: [Roslibpy](https://roslibpy.readthedocs.io/en/latest/), [Robostack](https://robostack.github.io/GettingStarted.html).

```bash
# install pixi
curl -fsSL https://pixi.sh/install.sh | bash

pixi init robostack
cd robostack/

# configure `pixi.toml`
# [feature.jazzy.dependencies]
# ros-jazzy-desktop = "*"
# ros-jazzy-rosbridge-server = "*"
vim pixi.toml
pixi install

# enter ROS shell (noetic, humble, jazzy, kilted)
pixi shell -e jazzy
# should see (robostack:jazzy) shell prefix

ros2 launch rosbridge_server rosbridge_websocket_launch.xml
# now you can import roslibpy in your Python script to connect to ROS nodes

# checking roundtrip message latency
# https://roslibpy.readthedocs.io/en/latest/examples/02_check_latency.html

# OPTIONAL: install custom ROS packages
mkdir -p ros2_ws/src
cp {PKG_PATH} ros2_ws/src/
cd ros2_ws/
colcon build --symlink-install

# source custom environment
source install/setup.bash
ros2 run {PKG_NAME} {PKG_NODE}
```

## FAQ
- You will need to use `pip install ...` instead of `uv pip install ...`. You can also use pixi to install packages: `pixi add package` or `pixi add --pypi "package @ file://${PWD}"` for local installation.

- `realsense2` in `pixi.toml`:
```bash
# ros-jazzy-librealsense2 = "*"
# ros-jazzy-realsense2-camera = "*"
# ros-jazzy-realsense2-camera-msgs = "*"
# ros-jazzy-realsense2-description = "*"
```
