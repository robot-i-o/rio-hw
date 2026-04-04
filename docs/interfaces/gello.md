# GELLO

Tested: GELLO Franka, GELLO xArm7, GELLO UR5

Reference: [`gello_software`](https://github.com/wuphilipp/gello_software) and [`gello_mechanical`](https://github.com/wuphilipp/gello_mechanical)

> Dynamixel Wizard 2.0: [Ubuntu](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/), [MacOS](https://apps.apple.com/us/app/dynamixel-wizard-2-0/id1471288434).

```bash
# 1. Update Motor IDs (see `gello_software/README.md` Hardware Configuration)

# 2. Generate YAML config
# See GELLO orientation in https://github.com/wuphilipp/gello_software?tab=readme-ov-file#1-manual-gello_agent-setup
python -m scripts.setup_gello.generate_config \

    # UR5
    --start-joints 0 -1.57 1.57 -1.57 -1.57 0 \
    --joint-signs 1 1 -1 1 1 1 \
    --output-path examples/station_cfgs/data/gello_ur5.yaml

    # Franka
    --start-joints 0 0 0 -1.57 0 1.57 0 \
    --joint-signs 1 -1 1 1 1 -1 1 \
    --output-path examples/station_cfgs/data/gello_franka.yaml

    # xArm7
    --start-joints 0 0 0 1.57 0 1.57 0 \
    --joint-signs 1 -1 1 1 1 1 1 \
    --output-path examples/station_cfgs/data/gello_xarm7.yaml

    # Lite6
    --start-joints 0 0 0 1.57 0 1.57 0 \
    --joint-signs 1 -1 1 1 1 1 1 \
    --output-path examples/station_cfgs/data/gello_lite6.yaml

    # YAM
    --start-joints 0 0 0 0 0 0 \
    --joint-signs 1 -1 -1 -1 1 1 \
    --output-path examples/station_cfgs/data/gello_yam.yaml

# fix error detecting offsets
sudo usermod -aG dialout $USER
newgrp dialout
sudo chmod -R 777 /dev/serial/by-id/

# Set latency timer to 1ms
sudo sh -c 'echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer'
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
# in Dynamixel Wizard 2.0:
# Set Return Delay Time (Control Table Register 9) from 250 µs to 0 µs.
```

## FAQ

- GELLOs are kinematically scaled by $\alpha=0.5$, so try to match scaling when positioning for a bimanual setup.
