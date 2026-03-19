# XRoboToolkit

Tested: Pico 4 Ultra, Quest 3.

Reference: [XRoboToolkit](https://github.com/XR-Robotics)

## Setup

On desktop:

```bash
# Install XRoboToolkit PC service
curl -f https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
sudo dpkg -i XRoboToolkit-PC-Service_1.0.0_ubuntu_22.04_amd64.deb
rm XRoboToolkit-PC-Service_1.0.0_ubuntu_22.04_amd64.deb
# Install dependencies
bash ./scripts/setup/x_robotoolkit.sh
```

On headset:

1. Enable Developer Mode in (Settings -> Developer)
2. Open the browser and go to https://github.com/XR-Robotics, and scroll down until you find the APK download link
3. Download the .apk file, then click to open the download, and select Install.

## Deployment

1. Connect your headset and workstation to the same Wi-Fi
2. Launch the XRoboToolkit application inside the headset, in the "Unknown" section.
3. Find the Wi-Fi address of your workstation, and enter it into the "PC Service:" field on the headset. If it works, you should see WORKING next to the status.
4. Under the "Tracking" section, enable  "Head" and "Controller", set the Pico Motion Tracker mode to "Full-Body", and enable "Send".
5. On the bottom bar, click on "Motion Trackers" to calibrate your motion trackers.
