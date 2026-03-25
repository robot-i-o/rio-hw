# xArm

Tested: xArm 7/850 and Lite6. (Software/Firmware version: 2.7.0)

Reference: [UFactory docs](https://docs.ufactory.cc/).

```bash
# get ethernet device name enx* for robot
nmcli device status

# set (temporary) ip to ethernet network interface (use host id - 1 of robot ip)
ip addr add 192.168.1.110/24 dev enx*

# ping robot ip (check control box)
ping 192.168.1.111

# open web-based GUI
http://192.168.1.111:18333/
# `ssh -L 18333:192.168.1.111:18333 USER@SERVER`
http://localhost:18333/

# (optional) Dual arm on same machine
# Settings -> My Device -> Network -> IP Address
# change IP address, then Reboot
```

## FAQ

- Change Settings -> General -> Manual Mode Sensitivity to 5, to make it easier to move the robot during Manual Mode.

- Force-Torque sensor: Settings -> Externals -> Torque Sensor, Enable and do Payload Identification.
