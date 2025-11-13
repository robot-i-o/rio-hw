# XHand

Tested: RobotEra XHand1.

Reference: [RobotEra docs](https://www.robotera.com/en/download1.html).

```bash
# Install dependencies
sudo ./scripts/setup/xhand.sh
sudo ./scripts/setup/ethercat.sh

# Install xhand web GUI
cd "上位机(ubuntu)/"
sudo dpkg -i xhand_v1.1.16.1_release_20250728.deb
cd -

# Open xhand web GUI
sudo pkill -SIGTERM -f xhand
sudo gtk-launch xhand
http://127.0.0.1:1888/?lang=en

# Install xhand python package
cd xhand_control_sdk_py_v118/
unzip xhand_control_sdk_py_x86_64_v118.zip
cd xhand_control_sdk_py/
pip install xhand_controller-1.1.8-cp310-*.whl

# RS485
sudo chmod 666 /dev/ttyUSB0

# EtherCAT
nmcli device status
# for second hand, use MASTER1_DEVICE
export MASTER0_DEVICE="enx*" && export DEVICE_MODULES="generic"
sudo sed -i "s/\(MASTER0_DEVICE=\).*/\1\"$MASTER0_DEVICE\"/" /etc/ethercat.conf
sudo sed -i "s/\(DEVICE_MODULES=\).*/\1\"$DEVICE_MODULES\"/" /etc/ethercat.conf
sudo systemctl restart ethercat

# Run example script
python xhand_control_example.py
```

# FAQ

- XHand will perform an initial calibration movement that will spread the fingers when powered on. Make sure the area around the hand is clear.
