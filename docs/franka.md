# Franka

Tested: Franka FR3. (robot system version: 5.8.1, libfranka version: >=0.15.x)

Reference: [Franka docs](https://frankarobotics.github.io/docs/index.html).

```bash
# ping robot fci-ip (should be less than 1ms)
# https://frankarobotics.github.io/docs/troubleshooting.html#simple-ping-tests
sudo ping 172.16.0.2 -i 0.001 -D -c 10000 -s 1200

# open web-based GUI
https://172.16.0.2/desk/
# `ssh -L 8443:172.16.0.2:443 USER@SERVER`
https://localhost:8443/desk/

# (optional) Dual arm on the same machine
# Settings -> Network -> C2 - Shop Floor network
# change Address and Gateway, such as:
# Address: 172.16.1.2
# Gateway: 172.16.1.1
# then click Apply
```

# Building libfranka

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake git libpoco-dev libeigen3-dev libfmt-dev
sudo apt-get install -y lsb-release curl
sudo mkdir -p /etc/apt/keyrings
curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | sudo tee /etc/apt/keyrings/robotpkg.asc
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | sudo tee /etc/apt/sources.list.d/robotpkg.list
sudo apt-get update
sudo apt-get install -y robotpkg-pinocchio

sudo apt-get remove "*libfranka*"
git clone --recurse-submodules https://github.com/frankarobotics/libfranka.git --branch 0.17.0
cd libfranka
mkdir build/
cd build/
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/opt/openrobots/lib/cmake -DBUILD_TESTS=OFF ..
make
cpack -G DEB
sudo dpkg -i libfranka*.deb
```

# FAQ

- Do **not** use robot system version 5.8.0, see [here](https://github.com/facebookresearch/fairo/issues/1426#issuecomment-3167067195).

- Guide Mode: Joints -> Unlock, Guiding Mode -> Free Move, Operations -> Programming, Base Light -> White. Then gently press the two side buttons on the Pilot-Grip.
