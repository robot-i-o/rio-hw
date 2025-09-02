<p align="center">
  <a href="https://github.com/astral-sh/uv">
  <img src="https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/uv/main/assets/badge/v0.json" /></a>
  <a href="https://github.com/astral-sh/ruff">
  <img src="https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/ruff/main/assets/badge/v2.json" /></a>
</p>

# Recontrol

Recontrol 🎛️ is a real-time control library for cross-embodiment robot manipulation.

# Setup

Tested on: Ubuntu 22.04 LTS *jammy* w/ real-time kernel patch.

```bash
# install uv
curl -LsSf https://astral.sh/uv/install.sh | sh

# create venv
uv venv --python 3.10
source .venv/bin/activate
uv sync --all-extras
```

### Peripherals

```bash
# SpaceMouse
sudo ./scripts/setup/spacemouse.sh
# verify install
systemctl status spacenavd
```

# Usage

### Robot arms
- [`docs/xarm.md`](docs/xarm.md)

# Acknowledgements
- [`real-stanford/DexUMI`](https://github.com/real-stanford/DexUMI)
- [`real-stanford/universal_manipulation_interface`](https://github.com/real-stanford/universal_manipulation_interface)
- [`real-stanford/diffusion_policy`](https://github.com/real-stanford/diffusion_policy)
