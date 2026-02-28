# USB (ROVI): topology, bandwidth, troubleshooting

ROVI uses multiple USB peripherals at once (camera, depth sensor, LiDAR, Wi‑Fi, serial devices). When you start a camera-heavy stack (for example `mapping`), overall **USB bandwidth + power + CPU/interrupt load** can change enough to surface issues that look like “random” serial disconnects.

This page is intentionally **table-first**: fill the tables, then run the commands to confirm.

## Tables (start here)

### ROVI device roles + stable symlinks

`tools/rovi_usb_setup.py` creates stable udev symlinks for the robot’s USB serial devices:

| Role | Typical chipset / VID:PID | Kernel node | Stable symlink |
|---|---|---|---|
| Control board (Rosmaster) | CH340 (`1a86:7523`) | `/dev/ttyUSB*` | `/dev/robot_control` |
| LiDAR | CH340 (`1a86:7523`) | `/dev/ttyUSB*` | `/dev/robot_lidar` |
| Serial display | ESP32‑S3 (CDC ACM) (`303a:1001`) | `/dev/ttyACM*` | `/dev/robot_display` |

Important detail: if **two CH340** devices are connected, `rovi_usb_setup.py` identifies the Rosmaster by actively probing `Rosmaster.get_version()`; the “other” CH340 becomes the LiDAR.

### USB inventory worksheet (copy/paste and edit)

Fill this in after you run `lsusb`, `lsusb -t`, and `usb-devices`:

| Role | VID:PID | Driver | `/dev/...` | `lsusb -t` path + speed | Notes |
|---|---|---|---|---|---|
| Display | `303a:1001` | `cdc_acm` | `/dev/ttyACM?` | `… 12M` | Should map to `/dev/robot_display` |
| Rosmaster | `1a86:7523` | `ch341` | `/dev/ttyUSB?` | `… 12M` | Should map to `/dev/robot_control` |
| LiDAR | `1a86:7523` | `ch341` | `/dev/ttyUSB?` | `… 12M` | Should map to `/dev/robot_lidar` |
| Wi‑Fi adapter | (varies) | (varies) | (N/A) | `… 480M/5000M` | Prefer its own port/controller |
| RGB camera | (varies) | `uvcvideo` | (N/A) | `… 480M/5000M` | High bandwidth |
| Depth sensor | (varies) | (varies) | (N/A) | `… 480M/5000M` | High bandwidth |

### Theoretical bandwidth budget (current defaults)

Assumptions:
- “MB/s” is **decimal** MB (`1 MB = 1,000,000 bytes`).
- Values are **payload only** (USB framing + ROS middleware overhead not included).
- Camera defaults come from:
  - `src/rovi_bringup/rovi_bringup/launch_lib/camera_args.py` (RGB size, `depth_mode`)
  - `src/rovi_bringup/launch/camera.launch.py` (RGB `pixel_format=YUYV`, `output_encoding=rgb8`)

| Stream | Default (ROVI) | Payload formula | Assumed rate | Expected MB/s (payload) | How to measure (recommended) |
|---|---|---:|---:|---:|---|
| RGB on USB (camera → host) | `640x480`, `YUYV` | `w*h*2` | ~`30 Hz` | `18.4` | USB: `sudo usbtop` (see below). ROS rate: `ros2 topic hz /camera/color/image` |
| RGB on ROS (host → ROS2) | `640x480`, `rgb8` | `w*h*3` | ~`30 Hz` | `27.6` | `ros2 topic bw /camera/color/image` |
| Depth on USB/ROS (sensor → host → ROS2) | `640x400`, `16-bit` | `w*h*2` | `30 Hz` (from `ORBBEC_640x400_30Hz`) | `15.4` | ROS: `ros2 topic bw /camera/depth/image` · USB: `sudo usbtop` |
| LiDAR scan | `/scan` (`LaserScan`) | `len(ranges)*4 (+ intensities*4)` | ~`10 Hz` | typically `<0.1` | `ros2 topic bw /scan` |
| UI status to display | gRPC status (`period_s: 3`) + serial @ `256000` baud | `bytes_per_msg * (1/period_s)` | `0.33 Hz` | ~`0.0001–0.001` | `timeout 10 grpcurl -plaintext -import-path ${ROVI_ROS_WS_DIR}/src/ros_ui_bridge/proto -proto ui_bridge.proto localhost:50051 roblibs.ui_bridge.v1.UiBridge/StreamStatus | wc -c` |

Rule of thumb: **USB2 High‑Speed (480M)** often tops out around **~35–40 MB/s sustained** in practice. One uncompressed camera can be “fine”; two on the same USB2 hub path can be enough to cause resets/stalls.

Quick sums (payload only, assuming ~30 Hz):
- RGB (USB YUYV) + depth (16-bit) ≈ `18.4 + 15.4 = 33.8 MB/s`
- RGB (ROS rgb8) + depth (ROS 16-bit) ≈ `27.6 + 15.4 = 43.0 MB/s`

### Command cheat‑sheet (most useful first)

| Question | Command |
|---|---|
| “Who shares which hub/controller?” | `lsusb -t` |
| “What VID:PID is this?” | `lsusb` |
| “Show speed/driver per device” | `usb-devices` |
| “Map `/dev/robot_*` to physical port” | `udevadm info -q path -n /dev/robot_display` *(or `_control`, `_lidar`)* |
| “Did USB reset/disconnect under load?” | `sudo journalctl -k -f \| rg -i "usb\|xhci\|reset\|disconnect\|cdc_acm\|ttyacm\|uvcvideo\|openni"` |
| “Did a tty device re-enumerate?” | `sudo udevadm monitor --udev --subsystem-match=tty` |
| “What’s using the serial port?” | `sudo lsof /dev/ttyACM0` *(or `/dev/robot_display`)* |
| “How much ROS bandwidth is this topic?” | `ros2 topic bw /camera/color/image` *(and `/camera/depth/image`, `/scan`)* |

## Commands (copy/paste)

### 1) Show USB topology (who shares a hub/controller)
```bash
lsusb -t
```

What to look for:
- The **speed** on each branch (`12M`, `480M`, `5000M`).
- Whether “important” devices share the same hub/controller path.

### 2) Identify devices (VID:PID)
```bash
lsusb
```

### 3) Per-device details + speed (verbose)
```bash
usb-devices
```

### 4) Map `/dev/...` nodes back to topology paths
```bash
udevadm info -q path -n /dev/robot_control
udevadm info -q path -n /dev/robot_lidar
udevadm info -q path -n /dev/robot_display
```

### 5) Watch for USB resets/disconnects (kernel log = smoking gun)
```bash
sudo journalctl -k -f | rg -i "usb|xhci|reset|disconnect|cdc_acm|ttyacm|uvcvideo|openni"
```

### 6) Watch for serial re-enumeration (udev)
```bash
sudo udevadm monitor --udev --subsystem-match=tty
```

### 7) Serial port sanity
```bash
ls -la /dev/robot_display /dev/ttyACM0 2>/dev/null || true
sudo lsof /dev/ttyACM0 2>/dev/null || true
```

### 8) CPU + USB controller interrupt pressure
```bash
top -o %CPU
cat /proc/interrupts | rg -i "xhci|usb"
```

### 9) Measure ROS bandwidth/rates (quick reality check)
```bash
ros2 topic hz /camera/color/image
ros2 topic bw /camera/color/image

ros2 topic hz /camera/depth/image
ros2 topic bw /camera/depth/image

ros2 topic hz /scan
ros2 topic bw /scan
```

### 10) `usbtop` (throughput per bus/device)
If `usbtop` can’t capture, it usually needs usbmon/debugfs:
```bash
sudo modprobe usbmon
sudo mount -t debugfs none /sys/kernel/debug 2>/dev/null || true
ls /sys/kernel/debug/usb/usbmon 2>/dev/null || true
sudo usbtop
```

## Reading `tools/rovi_usb_setup.py` output (what it means)

Run:
```bash
sudo python3 tools/rovi_usb_setup.py
```

Key lines to look for:
- `[scan] CH340 devices (N): ...` lists detected CH340s and their `id_path`/`by_path`.
- `[identify] ... get_version -> <number>` indicates which CH340 is the Rosmaster (valid versions look like `3.5`).
- `[setup] Done. Verified symlinks exist: /dev/robot_control /dev/robot_lidar /dev/robot_display`

After running it, verify:
```bash
ls -l /dev/robot_control /dev/robot_lidar /dev/robot_display
```

Common output patterns:

| Script output snippet | Usually means | Next command / fix |
|---|---|---|
| `sniff: error: device reports readiness to read but returned no data` | Device reset mid-read, or another process has the port open | `sudo lsof /dev/ttyUSB0 /dev/ttyUSB1 2>/dev/null || true` |
| `[error] Expected exactly 2 CH340 devices ...` | One of (control board, lidar) is missing or not enumerated as CH340 | `lsusb \| rg -n \"1a86:7523\"` · replug cables/hub |
| `[error] Expected exactly 1 display device ...` | Display missing, or different VID/PID than expected | `lsusb \| rg -n \"303a:1001\"` · `sudo dmesg -T \| tail -n 80` · rerun with `--allow-missing-display` if you just want rules installed |
| `get_version -> -1.0` (or non-positive) | That CH340 is probably not the Rosmaster board | Let the script probe the other CH340 (this is normal if you have 2 devices) |

## Interpreting `lsusb -t` (common confusion)

### USB 3 hubs often appear twice
Many USB 3 hubs enumerate as:
- a **USB 3 (SuperSpeed)** hub (under a `5000M` root hub), and
- a **USB 2 companion** hub (under a `480M` root hub).

So seeing “your hub” under `480M` does *not* automatically mean you plugged into a USB 2 port; it can simply be the USB 2 side of a USB 3 hub.

### Device speed is per-device, not per-port
Even on a USB 3 port/hub, a given device can still enumerate at `12M` or `480M` if it’s USB2‑only (or the cable/negotiation falls back).

## Wiring strategy (practical)

- **High bandwidth** (RGB/depth cameras): prefer a dedicated port/hub path.
- **Timing/stability critical** (control board, LiDAR): avoid sharing with cameras when possible.
- **Bursty/power-hungry** (Wi‑Fi adapter): prefer its own port/controller if available.
- **Low bandwidth** (keyboard dongle, BT adapter): safe to share.
- **Serial display** (`cdc_acm` / `ttyACM*`): keep off the camera hub path if it ever resets under camera load.

## ROVI recipe: “Serial display blanks when starting `mapping`”

1) Fast isolation (is the camera stack the trigger?):
```bash
mapping camera_enabled:=false
```

2) If it only blanks when cameras are enabled, look for USB/tty resets:
```bash
sudo udevadm monitor --udev --subsystem-match=tty
sudo journalctl -k -f | rg -i "usb|reset|disconnect|ttyacm|cdc_acm"
```

3) Confirm the UI gateway is still responding (rules out “gateway died”):
```bash
grpcurl -plaintext -import-path ${ROVI_ROS_WS_DIR}/src/ros_ui_bridge/proto -proto ui_bridge.proto \
  localhost:50051 roblibs.ui_bridge.v1.UiBridge/GetStatus | head
```

## See also

- `docs/lidar.md` — LiDAR (`/scan`) troubleshooting (USB → ROS → gRPC).
- `docs/usb_topology.md` — hub wiring + stack-load experiments and decoded `lsusb -t`.
