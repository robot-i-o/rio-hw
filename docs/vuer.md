# Vuer

Tested: Pico 4 Ultra, Quest 3, Quest 2.

Reference: [TeleVuer](https://github.com/unitreerobotics/televuer)

```bash
# create certificate files for Vuer
mkdir -p ~/.cache/vuer
cd ~/.cache/vuer
openssl req -x509 -nodes -days 365 -newkey rsa:2048 -keyout key.pem -out cert.pem
cd -

# allow firewall access
sudo ufw allow 8012/tcp
```

## Deployment

1. Connect your headset and workstation to the same Wi-Fi, and get the IP address of your workstation
2. Start the teleoperation script
3. On the headset, go to browser and enter https://vuer.ai?ws=wss://[HOST_IP]:8012

Optionally, you can also set up [ADB Reverse Port Forward](https://github.com/unitreerobotics/xr_teleoperate/wiki/XR_Device#2-adb-reverse-port-forwarding-for-quest-or-pico) to reduce latency, e.g. for streaming
