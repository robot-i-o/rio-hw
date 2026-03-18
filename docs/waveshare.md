# Waveshare USB-CAN Adapter

Tested: Waveshare USB to CAN Adapter Model A, STM32.

Reference: [Waveshare docs](https://www.waveshare.com/wiki/USB-CAN-A#Working_with_Linux).

```bash
sudo systemctl disable brltty-udev.service brltty.service 2>/dev/null
sudo apt purge brltty
# conflicts with waveshare CAN-USB-A adapter

sudo modprobe option
sudo sh -c 'echo "1a86 7523" > /sys/bus/usb-serial/drivers/option1/new_id'

lsusb
# check that "QinHeng Electronics CH340 serial converter" is listed
sudo dmesg | tail -n 20

# Add user to dialout group
sudo usermod -aG dialout $USER
newgrp dialout
sudo chmod -R 777 /dev/serial/by-id/
```
