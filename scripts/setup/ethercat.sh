#!/bin/bash

# Ensure the script is run with sudo
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root (use sudo)." 1>&2
   exit 1
fi

# https://icube-robotics.github.io/ethercat_driver_ros2/quickstart/installation.html
# https://etherlab.org/en_GB/getting-started
# https://techoverflow.net/2025/01/24/how-to-install-etherlab-ethercat-master-on-ubuntu-24-04/

# Install dependencies
sudo apt install -y git autoconf libtool pkg-config make build-essential net-tools

curl -fsSL https://download.opensuse.org/repositories/science:/EtherLab/xUbuntu_$(lsb_release -rs)/Release.key | gpg --dearmor | sudo tee "/usr/share/keyrings/etherlab.gpg" >/dev/null
sudo echo "deb [signed-by=/usr/share/keyrings/etherlab.gpg] https://download.opensuse.org/repositories/science:/EtherLab/xUbuntu_$(lsb_release -rs)/ ./" | sudo tee /etc/apt/sources.list.d/etherlab.list
sudo apt update && sudo apt install -y \
    ethercat-master \
    libethercat-dev

sudo sh -c 'echo KERNEL==\"EtherCAT[0-9]*\", MODE=\"0660\", GROUP=\"ethercat\" > /lib/udev/rules.d/99-EtherCAT.rules'
