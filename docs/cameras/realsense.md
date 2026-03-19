# Realsense

Tested: L515, D405, D415, D435.

Reference: [RealSense docs](https://dev.realsenseai.com/docs/docs-get-started)

```bash
sudo ./scripts/setup/realsense.sh
# verify install
realsense-viewer
# Note: if you do not need L515 support, then you can use newer versions of librealsense2/pyrealsense2 >2.54.2
```

## FAQ

- Ensure cameras are running at USB 3.2 speeds. Using high quality USB-C cables is recommended.

- Make sure `realsense-viewer` is not running when streaming video.

- Try to plug cameras in separate USB buses to avoid saturation when streaming from multiple cameras.

- If you run into DKMS troubles, try reloading uvcvideo `sudo modprobe uvcvideo`. If this fails, **make sure secure boot is disabled**:
```bash
sudo mokutil --sb-state | grep -q "SecureBoot disabled" || { echo "ERROR: Secure Boot must be disabled in BIOS"; exit 1; }
```
