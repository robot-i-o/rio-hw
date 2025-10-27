# GELLO

Tested: GELLO Franka, GELLO xArm7

Reference: [`gello_software`](https://github.com/wuphilipp/gello_software) and [`gello_mechanical`](https://github.com/wuphilipp/gello_mechanical)

```bash
# 1. Update Motor IDs (see `gello_software/README.md` Hardware Configuration)

# 2. Generate YAML config
# See GELLO orientation in https://github.com/wuphilipp/gello_software?tab=readme-ov-file#1-manual-gello_agent-setup
python -m scripts.setup.gello.generate_config

    # UR5
    --start-joints 0 -1.57 1.57 -1.57 -1.57 0 \
    --joint-signs 1 1 -1 1 1 1 \
    --output-path examples/station_cfgs/data/gello_ur5_left.yaml

    # Franka
    --start-joints 0 0 0 -1.57 0 1.57 0 \
    --joint-signs 1 -1 1 1 1 -1 1 \
    --output-path examples/station_cfgs/data/gello_franka_left.yaml

    # xArm7
    --start-joints 0 0 0 1.57 0 1.57 0 \
    --joint-signs 1 -1 1 1 1 1 1 \
    --output-path examples/station_cfgs/data/gello_xarm7_left.yaml

    # I2RT YAM
    --start-joints 0 0 0 0 0 0 \
    --joint-signs 1 -1 -1 -1 1 1 \
    --output-path examples/station_cfgs/data/gello_yam_left.yaml

# fix error detecting offsets
sudo usermod -aG dialout $USER
sudo chmod -R 755 /dev/serial/by-id/
```
