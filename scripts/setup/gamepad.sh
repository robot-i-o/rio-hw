#!/bin/bash

# Ensure the script is run with sudo
set -e
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root (use sudo)." 1>&2
   exit 1
fi

_INSTALL_XPADNEO=${1:-false}

ACTUAL_USER=${SUDO_USER:-$USER}
ACTUAL_HOME=$(eval echo ~$ACTUAL_USER)

apt update
apt install -y dkms linux-headers-$(uname -r) git libusb-dev joystick
modprobe uhid
echo "uhid" > /etc/modules-load.d/uhid.conf
if [ "$_INSTALL_XPADNEO" != "true" ]; then
    echo "Skipping xpadneo installation. To install, run the script with 'true' as the first argument."
    exit 0
fi

cd "$ACTUAL_HOME"
if [ -d "$ACTUAL_HOME/xpadneo" ]; then
   ./xpadneo/uninstall.sh
   rm -rf "$ACTUAL_HOME/xpadneo"
fi

echo "Installing xpadneo driver temporarily to $ACTUAL_HOME/xpadneo"
[ -d "xpadneo" ] && cd xpadneo && sudo -u $ACTUAL_USER git pull || sudo -u $ACTUAL_USER git clone -b v0.9.7 https://github.com/atar-axis/xpadneo && cd xpadneo
./install.sh
chown -R $ACTUAL_USER:$ACTUAL_USER "$ACTUAL_HOME/xpadneo"

echo "Done. Pair controller with: bluetoothctl"
