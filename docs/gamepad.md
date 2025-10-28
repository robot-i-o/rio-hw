# Gamepad

Supports Xbox, PlayStation, and generic USB/Bluetooth gamepads via Linux `evdev`.

> **Recommended**: Use 2.4 GHz wireless dongles. Plug-and-play with no pairing issues.

### Setup

```bash
# Install dependencies
sudo ./scripts/setup/gamepad.sh

# Optional: Install xpadneo for Xbox controllers over Bluetooth
sudo ./scripts/setup/gamepad.sh true

# Add user to input group
sudo usermod -a -G input $USER
# Log out and back in for changes to take effect
```

### Connect Controller

**2.4 GHz Dongle (Preferred):**
1. Plug in USB dongle
2. Turn on controller (auto-pairs)

> NOTE: using Bluetooth GUI is more reliable than finding specific bluetooth id with scanning.

**Bluetooth (First time pairing)**
```bash
bluetoothctl
power on
scan on
# Xbox: Hold pair button until LED flashes
# PlayStation: Hold Share + PS until light flashes
pair XX:XX:XX:XX:XX:XX
connect XX:XX:XX:XX:XX:XX
trust XX:XX:XX:XX:XX:XX
exit
```

**Bluetooth (Connecting)**
```bash
bluetoothctl
power on
connect XX:XX:XX:XX:XX:XX
exit
```
