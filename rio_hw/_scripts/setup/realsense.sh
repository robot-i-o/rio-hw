#!/bin/bash

# Ensure the script is run with sudo
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root (use sudo)." 1>&2
   exit 1
fi

# NOTE: Make sure secure boot is disabled in BIOS if you run into issues with DKMS
sudo mokutil --sb-state | grep -q "SecureBoot disabled" || { echo "ERROR: Secure Boot must be disabled in BIOS"; exit 1; }

# https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages

# Register the server's public key
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc | \
gpg --dearmor | sudo tee /etc/apt/keyrings/librealsenseai.gpg > /dev/null

# Make sure apt HTTPS support is installed
sudo apt-get install -y apt-transport-https gnupg

# Add the server to the list of repositories
echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update

# Install the libraries
# specify older version for L515 support https://github.com/IntelRealSense/librealsense/issues/13638#issuecomment-2564363960
apt-cache madison librealsense2
sudo apt-get install -y --allow-downgrades \
  librealsense2=2.54.2-0~realsense.10773 \
  librealsense2-utils=2.54.2-0~realsense.10773 \
  librealsense2-gl=2.54.2-0~realsense.10773 \
  librealsense2-dkms
# pip install "pyrealsense2==2.54.2.5684"

# Verify that the kernel is updated
modinfo uvcvideo | grep "version:"
