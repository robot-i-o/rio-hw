<p align="center">
  <a href="https://github.com/astral-sh/uv">
  <img src="https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/uv/main/assets/badge/v0.json" /></a>
  <a href="https://github.com/astral-sh/ruff">
  <img src="https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/ruff/main/assets/badge/v2.json" /></a>
</p>

# Recontrol

Recontrol 🎛️ is a real-time control library for cross-embodiment robot manipulation.

# Setup

Tested on: Ubuntu 22.04 LTS w/ real-time kernel patch. See [`docs/ubuntu.md`](docs/ubuntu.md) for setup instructions.

```bash
# install uv
curl -LsSf https://astral.sh/uv/install.sh | sh

# create venv
uv venv --python 3.10
source .venv/bin/activate
uv sync --all-extras
```

### Cameras

Realsense:
```bash
sudo ./scripts/setup/realsense.sh
# verify install
realsense-viewer
```

### Interfaces

Spacemouse:
```bash
sudo ./scripts/setup/spacemouse.sh
# verify install
systemctl status spacenavd
```

### Robots
Arms:
  - [`docs/ur.md`](docs/ur.md)
  - [`docs/xarm.md`](docs/xarm.md)

See [[Hardware Guide (Google Docs)]](https://docs.google.com/document/d/1_NbHk4z9HABPnaqow-VP-srbQBz42kBXCbvbyiJrW74) for hardware and example robot stations.

# Usage

A `Node` dynamically inherits from any given `Middleware` parent, and factory functions automatically create `Server` and `Client` nodes. Each `Node` publishes data and handles requests, in separate loops or in the same loop.
1. `Node.run()` only publishes data.
2. `Node.run()` only handles requests.
3. `Node.run()` both publishes data and handles requests. (NOT RECOMMENDED)
4. `Node.run()` only handles requests, and `Node.pub()` publishes data in a separate worker.
5. `Node.run()` only publishes data, and `Node.req()` handles requests in a separate worker.

Users only need to implement "pub/req" behavior in nodes. A "pub" loop should call `ring_buffer.put()` to publish data, and a "req" loop should call `request_queue.get()` to handle requests. A `Server` runs "pub/req" ("publish"/"request") and a `Client` resolves "sub/rep" ("subscribe"/"reply") automatically.

### Conventions

- Quaternion ordering: `(x, y, z, w)`
- Coordinate system: `Z` up, right-handed
- Linear velocity: COM, world frame
- Angular velocity: world frame
- Images: `(height, width, channel)`, RGB channel order
- SI units: `seconds`, `meters`, `kilograms`, `radians`

# Acknowledgements
- [`real-stanford/DexUMI`](https://github.com/real-stanford/DexUMI)
- [`real-stanford/universal_manipulation_interface`](https://github.com/real-stanford/universal_manipulation_interface)
- [`real-stanford/diffusion_policy`](https://github.com/real-stanford/diffusion_policy)
