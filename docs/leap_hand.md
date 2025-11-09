# LEAP Hand

Tested: LEAP Hand V1 (Full).

Reference: [`LEAP_Hand_API`](https://github.com/leap-hand/LEAP_Hand_API).

> Dynamixel Wizard 2.0: [Ubuntu](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/), [MacOS](https://apps.apple.com/us/app/dynamixel-wizard-2-0/id1471288434).

```bash
sudo usermod -aG dialout $USER
sudo chmod 777 /dev/ttyUSB0

# Set latency timer to 1ms
sudo sh -c 'echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer'
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
# in Dynamixel Wizard 2.0:
# Set Return Delay Time (Control Table Register 9) from 250 µs to 0 µs.
```
