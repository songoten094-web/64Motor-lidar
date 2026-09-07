# REAL MID-360 connection — Step 1

This build keeps the vendor Livox SDK2 source tree unchanged and connects the HMI through `LivoxHmiBridge`.

## 1. Physical network

- Connect the MID-360 to the Windows PC Ethernet port directly or through an Ethernet switch.
- Use a dedicated Ethernet adapter if possible; avoid relying on Wi-Fi for the LiDAR data path.
- The included SDK2 configuration uses host IP `192.168.1.5` and the MID-360 ports 56100–56500 on the lidar side and 56101–56501 on the host side. This matches the official Livox SDK2 MID-360 quick-start configuration.

## 2. Windows Ethernet IPv4

Set the Ethernet adapter to:

- IP address: `192.168.1.5`
- Subnet mask: `255.255.255.0`
- Default gateway: leave blank for a dedicated link
- DNS: leave blank

If your network uses a different subnet, edit `config/mid360_config.json` so `host_ip` matches the PC Ethernet address.

## 3. Firewall

Allow the HMI executable and native Livox bridge through Windows Defender Firewall for the private Ethernet network. If Windows asks for permission when the HMI first starts, allow it.

## 4. Connect

1. Power the MID-360.
2. Wait for it to finish booting.
3. Start the HMI.
4. Press **Connect MID-360**.
5. Watch the status panel:
   - `SYSTEM: LIVox RUNNING`
   - `Waiting for point cloud`
   - then `Points: ...`, `Frame: ...`, and packet counters should increase.

## 5. If it stays at NO FRAME > 2 s

Check in this order:

1. Windows Ethernet adapter is enabled and has `192.168.1.5`.
2. Ethernet cable/link is up.
3. The LiDAR and PC are on the same subnet.
4. Windows Firewall is not blocking the HMI.
5. Another Livox application is not already using the same UDP ports.
6. Confirm the actual MID-360 network configuration before changing the SDK config.

The HMI does not modify the vendor SDK. The bridge only consumes SDK2 callbacks and publishes a bounded latest-frame buffer to C#.
