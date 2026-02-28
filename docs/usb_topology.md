# USB hubs vs stack load (ROVI) — experiments + decoded topology

This page documents a specific failure pattern observed on the robot:

- Cameras can run at ~30 FPS and the serial display is fine.
- Starting heavier stacks (`mapping`, `localization`) makes the **serial display stop updating**, even when the camera is on its own powered hub.
- When a PC subscribes to camera topics (RViz / viewer), camera FPS can collapse to ~3–4 FPS.

Goal: make the observations + USB topology **human readable**, and provide a **repeatable triage checklist** to decide whether this is:
- (A) USB/hub/bandwidth/power,
- (B) CPU/scheduling pressure,
- (C) gRPC/UI bridge issues,
- (D) serial display firmware/payload handling.

## Physical wiring (as described)

Raspberry Pi ports:
- **Blue (left/top)**: powered hub #1 (“camera hub”) → Orbbec camera only *(depth + color over one USB cable)*.
- **Blue (left/bottom)**: powered hub #2 (“robot IO hub”) → motors control, Wi‑Fi, serial display, LiDAR.
- **Black (right)**: unused.

## Experiment log (current evidence)

| Scenario | CPU (approx) | Camera FPS (depth,color) | PC viewer / RViz | Serial display | Notes |
|---|---:|---:|---|---|---|
| `camera` | 27% | 29, 30 | none | OK | Baseline |
| `camera` + PC RViz/view | 18% | 3, 3 | enabled | OK | Subscribing to raw images can backpressure/saturate CPU/network |
| `mapping` (no topology, no PC viewer) | 23% | 20, 20 | none | **Stops** | Stops on stack start |
| `mapping` (topology, no PC) | 24% | 16, 15 | gRPC viewer shows low FPS | **Stops** | Stops on stack start |
| `mapping` (topology, PC RViz/View) | 27% | 3, 3 | gRPC viewer shows low FPS | **Stops** | Stops on stack start |
| `localization` | 25% | 15, 20 | none | **Stops** | Stops on stack start |

Note : when Display stops on stack start, and when gRPC viewer is active the gRPC is still working visible on web UI client.
Takeaway (so far): separating hubs reduces “pure USB contention” suspicion, but the failure still correlates with **starting compute-heavy stacks**.

## Decoding `lsusb -t` (your current snapshot)

Note: bus/device numbers can change after replug/reboot; focus on **drivers** (`uvcvideo`, `cdc_acm`, `ch341`) and **VID:PID** from `lsusb` when matching roles.

Raw output (for reference):
```text
/:  Bus 001.Port 001: Dev 001, Class=root_hub, Driver=dwc2/1p, 480M
/:  Bus 002.Port 001: Dev 001, Class=root_hub, Driver=xhci-hcd/2p, 480M
    |__ Port 001: Dev 002, If 0, Class=Hub, Driver=hub/4p, 480M
        |__ Port 001: Dev 010, If 0, Class=Vendor Specific Class, Driver=ch341, 12M
        |__ Port 002: Dev 011, If 0, Class=Vendor Specific Class, Driver=rtl8852bu, 480M
        |__ Port 003: Dev 008, If 0, Class=Communications, Driver=cdc_acm, 12M
        |__ Port 003: Dev 008, If 1, Class=CDC Data, Driver=cdc_acm, 12M
        |__ Port 003: Dev 008, If 2, Class=Vendor Specific Class, Driver=[none], 12M
        |__ Port 004: Dev 009, If 0, Class=Vendor Specific Class, Driver=ch341, 12M
/:  Bus 003.Port 001: Dev 001, Class=root_hub, Driver=xhci-hcd/1p, 5000M
    |__ Port 001: Dev 002, If 0, Class=Hub, Driver=hub/4p, 5000M
/:  Bus 004.Port 001: Dev 001, Class=root_hub, Driver=xhci-hcd/2p, 480M
    |__ Port 001: Dev 002, If 0, Class=Hub, Driver=hub/4p, 480M
        |__ Port 004: Dev 005, If 0, Class=Hub, Driver=hub/2p, 480M
            |__ Port 001: Dev 006, If 0, Class=Video, Driver=uvcvideo, 480M
            |__ Port 001: Dev 006, If 1, Class=Video, Driver=uvcvideo, 480M
/:  Bus 005.Port 001: Dev 001, Class=root_hub, Driver=xhci-hcd/1p, 5000M
    |__ Port 001: Dev 002, If 0, Class=Hub, Driver=hub/4p, 5000M
        |__ Port 004: Dev 003, If 0, Class=Hub, Driver=hub/2p, 5000M
            |__ Port 002: Dev 006, If 0, Class=Vendor Specific Class, Driver=usbfs, 5000M
```

### Human-readable “who is where” view

| physical USB receptacle | Hub group | Bus| speed | What’s on it |
|---|---|---|---|---|
| Blue left/top | Acer-Camera | `004` | `480M` | `uvcvideo` (color camera interface) |
| Blue left/top | Acer-Camera | `005` | `5000M` | `usbfs` vendor-specific device (depth sensor) |
| Blue left-bottom | Acer-IO | `002` | `480M` | `ch341` (ttyUSB motors), `cdc_acm` (ttyACM display), `rtl8852bu` (Wi‑Fi), `ch341` (ttyUSB lidar) |
| Blue left-bottom | Acer-IO | `003` | `5000M` | - |

### Decoded tree (annotated)

```text
Robot IO hub (blue left/bottom)
  xhci-hcd USB2 root hub (Bus 002 @ 480M)
    Hub (480M)
      ch341 (12M)         -> ttyUSB*  (Rosmaster or LiDAR)
      rtl8852bu (480M)    -> Wi‑Fi adapter
      cdc_acm (12M)       -> ttyACM*  (ESP32 display)
      ch341 (12M)         -> ttyUSB*  (Rosmaster or LiDAR)
  xhci-hcd USB3 root hub (Bus 003 @ 5000M)
    Hub (5000M)
      (no SuperSpeed devices in this snapshot)

Camera hub (blue left/top)
  xhci-hcd USB2 root hub (Bus 004 @ 480M)
    Hub (480M) -> Hub (480M)
      uvcvideo (480M)     -> color camera interface
  xhci-hcd USB3 root hub (Bus 005 @ 5000M)
    Hub (5000M) -> Hub (5000M)
      usbfs (5000M)       -> depth sensor interface
```

Important note: USB3 hubs often appear as **two hubs** (USB2 companion + USB3 SuperSpeed). Seeing a hub under `480M` does not mean “plugged into USB2”; it can be the USB2 side of a USB3 hub.

## What this suggests (without guessing)

The “USB-only” theory got weaker because:
- The camera is isolated on its own hub, yet `mapping`/`localization` still stop the display.
- The camera FPS drop to ~3–4 FPS happens with a **PC subscriber** (RViz/viewer), which is consistent with CPU/network/DDS backpressure, not just USB.

The next step is to identify **which link in the chain** fails when the display blanks:

1) **USB/serial reset?** (ESP32 disappears/re-enumerates)
2) **Serial write stall?** (`serial_display` blocked or errors on `/dev/robot_display`)
3) **gRPC stream disruption?** (`serial_display` loses `StreamStatus` even though `GetStatus` still works)
4) **UI payload change triggers firmware bug?** (more fields / longer lines when stacks start)

## Fast triage checklist (run when it “stops”)

| Check | Command | What it tells you |
|---|---|---|
| Display still enumerated? | `ls -l /dev/robot_display /dev/ttyACM*` | If missing/changed → USB reset/disconnect |
| Is display port owned? | `sudo lsof /dev/robot_display 2>/dev/null \|\| true` | Confirms a process still holds the port |
| Any tty remove/add events? | `sudo udevadm monitor --udev --subsystem-match=tty` | Shows live re-enumeration |
| Kernel saw resets? | `sudo journalctl -k -n 200 --no-pager \| rg -i \"usb|reset|disconnect|ttyacm|cdc_acm\"` | USB-level evidence |
| Is UI bridge alive? | `grpcurl -plaintext -import-path ${ROVI_ROS_WS_DIR}/src/ros_ui_bridge/proto -proto ui_bridge.proto localhost:50051 roblibs.ui_bridge.v1.UiBridge/GetStatus \| head` | If this fails, it’s not “just the display” |
| Is streaming alive? | `timeout 10 grpcurl -plaintext -import-path ${ROVI_ROS_WS_DIR}/src/ros_ui_bridge/proto -proto ui_bridge.proto localhost:50051 roblibs.ui_bridge.v1.UiBridge/StreamStatus \| head -n 40` | Detects stream cancellation / stalls |
| Who is using CPU now? | `top -o %CPU` | Confirms load spikes during stack startup |
| ROS image bandwidth | `ros2 topic bw /camera/color/image` | Real bytes/s on ROS side (can imply backpressure) |

If you want one “single capture” command during a reproduction, run these in 3 terminals:
- `sudo journalctl -k -f | rg -i "usb|reset|disconnect|ttyacm|cdc_acm"`
- `journalctl -u rovi-gateway.service -f | rg -i "robot_serial_display|serial_display|gRPC stream error|UNAVAILABLE|Serial write failed"`
- `top -o %CPU`

## See also

- `docs/usb.md` — USB commands + bandwidth budgeting tables.
- `docs/troubleshooting.md` — general runtime troubleshooting.
