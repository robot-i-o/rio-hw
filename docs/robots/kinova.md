# Kinova

Tested: Kinova Gen3 6DoF

Reference: [Kinova docs](https://www.kinovarobotics.com/product/gen3-robots), [Kortex API](https://github.com/Kinovarobotics/kortex)

```bash
# get ethernet device name for robot
# macOS
networksetup -listallhardwareports
# Linux
nmcli device status

# set (temporary) ip on same subnet as robot (check robot IP label)
# macOS
sudo ifconfig en5 192.168.1.8 netmask 255.255.255.0
# Linux
sudo ip addr add 192.168.1.8/24 dev enx*

# ping robot ip
ping 192.168.1.9

# open web GUI (admin/admin)
http://<robot_ip>/
```

## FAQ

- **Startup:** Press silver button until blue LED (< 5 seconds, longer resets to factory).
- **Ready:** Solid green light indicates arm is ready.
- **Gripper:** Ensure not blocked during startup (opens/closes during initialization).
- **E-stop:** Red button must be unlocked (twisted out) for operation.
- **BaseCyclic API:** Linux only, 125Hz default (up to 500Hz with PREEMPT_RT kernel).
