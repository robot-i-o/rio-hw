#!/bin/bash

# Ensure the script is run with sudo
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root (use sudo)." 1>&2
   exit 1
fi

sudo apt install -y libspnav-dev spacenavd
sudo systemctl start spacenavd
