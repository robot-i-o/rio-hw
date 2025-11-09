# XHand

Tested: RobotEra XHand1.

Reference: [RobotEra docs](https://www.robotera.com/en/download1.html).

```bash
# Install dependencies
sudo ./scripts/setup/xhand.sh
sudo ./scripts/setup/ethercat.sh

# Install xhand python package
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
sudo sed -i "s/\(MASTER1_DEVICE=\).*/\1\"$MASTER1_DEVICE\"/" /etc/ethercat.conf
sudo sed -i "s/\(DEVICE_MODULES=\).*/\1\"$DEVICE_MODULES\"/" /etc/ethercat.conf
sudo systemctl restart ethercat

# Run example script
python xhand_control_example.py
```

# FAQ

- XHand will perform an initial calibration movement that will spread the fingers when powered on. Make sure the area around the hand is clear.
