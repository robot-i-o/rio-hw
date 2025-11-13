#!/bin/bash

# Ensure the script is run with sudo
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root (use sudo)." 1>&2
   exit 1
fi

# Install dependencies (web GUI)
sudo apt install -y \
    openjdk-8-jdk \
    python3-pip \
    libboost-filesystem-dev \
    nlohmann-json3-dev

# Install dependencies (python package)
sudo apt install -y \
    cmake \
    g++ \
    libcurl4-openssl-dev \
    libssl-dev \
    nlohmann-json3-dev
