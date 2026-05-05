# Unitree G1

Tested: Unitree G1 (EDU Plus 29-DOF).

Reference: [Unitree docs](https://support.unitree.com/home/en/G1_developer).

## Setup

Install Homosoma `unitree_interface` Python bindings for `unitree_sdk2`:

```bash
pip install unitree_sdk2 --no-index --find-links "https://github.com/amazon-far/unitree_sdk2/releases/expanded_assets/0.1.3"
python -c "import unitree_interface.unitree_interface as m; print(m.__file__)"
```

## Network Setup

Connect the robot's Ethernet port to your machine. Set a static IP on the same subnet as the robot (default: `192.168.123.xxx`):

```bash
# find the robot-facing ethernet interface (typically enp* or eth0)
ifconfig

# assign an IP in the 192.168.123.x range (e.g., .244)
sudo ip addr add 192.168.123.244/24 dev <INTERFACE>

# verify connectivity to robot development computer
ping 192.168.123.164
```

## FAQ

- The robot takes ~30–60 seconds to fully initialize after power-on. Wait for all status LEDs to turn solid before running code.

- If you aren't able to ping the robot, try the following debugging steps

```
# Remove the eth profile through the GUI,
# Settings -> Network -> USB Ethernet -> Settings: Remove Connection 
sudo ip addr flush dev <INTERFACE>
sudo systemctl restart NetworkManager
```
