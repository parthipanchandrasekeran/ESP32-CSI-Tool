# Fork Changes — Room Sense

This document lists changes made in this fork on top of the upstream
[StevenMHernandez/ESP32-CSI-Tool](https://github.com/StevenMHernandez/ESP32-CSI-Tool).

The upstream project is the source of all CSI extraction code (the
`csi_callback`, the CSV serialization format, the active/passive/AP
project structure). This fork adds networking, scanning, and
multi-node orchestration on top of that foundation.

License: MIT (unchanged). Original copyright © 2020 Steven M. Hernandez.

---

## Additions

### Wireless UDP streaming
The original tool writes CSI data to the serial port (and optionally to an
SD card). This fork adds a UDP transmitter so boards can stream wirelessly
to any host on the LAN. Boards no longer need to be USB-tethered to a PC.

- New globals: `udp_csi_fd`, target host/port via `menuconfig`
- Each CSI line is prefixed with the board's MAC address so a single
  receiver can dispatch packets from many boards.

### BLE scanner task (NimBLE)
A FreeRTOS task that periodically scans for Bluetooth LE advertisements
and emits one UDP line per observation:

```
<board_mac> BLE <addr> <rssi> [name]
```

Used by the backend to localize phones and detect unfamiliar devices.

### WiFi neighbor scanner task
Periodically scans 2.4 GHz channels for nearby APs and emits:

```
<board_mac> WIFI <bssid> <rssi> <channel> <auth> <ssid>
```

Hardened with:
- Waits for STA-up before first scan
- `esp_wifi_scan_stop()` before each new scan (recovers from stuck state)
- Error counter with 30 s recovery delay on repeated failures
- Periodic `WIFI_HB` heartbeat lines so the host can detect a wedged task

### CPU temperature publishing
The on-chip temperature sensor is polled and reported as:

```
<board_mac> TEMP <celsius>
```

Useful for detecting overheating boards or correlating environmental
conditions with sensor performance.

### `csi_traffic_task` — self-traffic generator
In low-traffic home environments, CSI events can dry up because the
ESP32 only receives a CSI callback when frames are received. This task
sends a small UDP keepalive packet to the gateway at ~10 Hz, which
provokes ACK frames from the AP and keeps CSI events flowing.

Without this, boards in quiet rooms would report near-zero motion
samples even when motion was clearly happening.

### Multi-board dispatch protocol
All outbound packets are prefixed with the sending board's MAC address
so a single receiver process can de-multiplex many boards into separate
per-node state machines on the host side.

---

## Files added / modified

- `_components/csi_component.h` — main location of fork additions
  (UDP socket setup, scanner tasks, traffic generator, temp publisher)
- `active_sta/` — used as the deployable firmware for Room Sense nodes

---

## Upstream sync

Upstream is tracked as a separate remote:

```
git remote -v
origin    https://github.com/parthipanchandrasekeran/ESP32-CSI-Tool.git
upstream  https://github.com/StevenMHernandez/ESP32-CSI-Tool.git
```

To pull future upstream changes:

```
git fetch upstream
git merge upstream/master
```
