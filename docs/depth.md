# ROS

## Preferred driver: `openni2_camera` (Jazzy apt)

This camera works reliably via OpenNI2 on this robot, so the preferred ROS path is `openni2_camera`.

### Install

```bash
sudo apt update
sudo apt install -y ros-jazzy-openni2-camera
```

### Configure OpenNI2 (Orbbec SDK)

`openni2_camera` links against `libOpenNI2.so.0`. The Orbbec OpenNI2 SDK ships `libOpenNI2.so` (no `.so.0`), so add a symlink and prefer this directory via `LD_LIBRARY_PATH`.

```bash
source /opt/ros/$ROS_DISTRO/setup.bash

export OPENNI2_REDIST=$HOME/OpenNI/OpenNI_2.3.0/tools/NiViewer
ln -sf "$OPENNI2_REDIST/libOpenNI2.so" "$OPENNI2_REDIST/libOpenNI2.so.0"
export LD_LIBRARY_PATH=$OPENNI2_REDIST:$LD_LIBRARY_PATH
```

Sanity-check device discovery:
```bash
ros2 run openni2_camera list_devices
```

If it still shows `Found 0 devices`, confirm which OpenNI2 library is being used:
```bash
ldd /opt/ros/$ROS_DISTRO/lib/openni2_camera/list_devices | rg 'OpenNI2'
```

### Run (headless)

Use the packaged launch:
```bash
ros2 launch openni2_camera camera_only.launch.py namespace:=camera
```

For this specific camera (Astra Stereo S U3), the OpenNI2 sensor modes are `640x400` and `320x200` (not `640x480`/`320x240`). If you see “Unsupported * video mode … 640x480 …” and the images are black, run with the Orbbec-specific modes:

```bash
ros2 run openni2_camera openni2_camera_driver
```

Note: this repo carries a small `openni2_camera` overlay patch (in `src/openni2_camera`) to add the `ORBBEC_640x400_*` / `ORBBEC_320x200_*` mode names and default to them; run `build` after pulling changes.

Verify:
```bash
ros2 topic list | rg -i 'openni|depth|ir|camera_info'
```

Notes:
- Depth + IR come from OpenNI2. RGB may not be provided via OpenNI2 on this device (it presents RGB as a separate UVC camera), so treat RGB as a separate V4L2 camera if needed.
- If the driver prints “Unsupported * video mode …”, it’s usually non-fatal if topics are streaming. Confirm with:
  - `ros2 topic hz /depth_raw/image`
  - `ros2 topic echo /depth_raw/image --once` (check `encoding`, `width`, `height`)
- This workspace’s `openni2_camera` overlay defaults to depth-only publishing (`enable_ir:=false`, `enable_color:=false`) to keep the topic list clean.
- `device_id:="#1"` means “first OpenNI2 enumerated device”. If you ever plug multiple OpenNI2 cameras, set `device_id` using the URI from `ros2 run openni2_camera list_devices`.

# Native
## Install
* install depedencies
```bash
sudo apt update
sudo apt install -y freeglut3-dev libglu1-mesa libgl1 unrar
```

If NiViewer complains `libglut.so.3` missing, create the symlink once:
```bash
sudo ln -sf /lib/aarch64-linux-gnu/libglut.so.3.12 /lib/aarch64-linux-gnu/libglut.so.3
sudo ldconfig
```

* download, unzip, unrar then move to `~/OpenNI/OpenNI_2.3.0`
https://github.com/orbbec/OpenNI_SDK/releases/download/v2.3.0.86-beat6/OpenNI_2.3.0.86_202210111155_4c8f5aa4_beta6_arm64.zip

```bash
mkdir -p ~/OpenNI && cd ~/OpenNI
unzip OpenNI_2.3.0.86_202210111155_4c8f5aa4_beta6_arm64.zip
unrar x 066797_OpenNI_2.3.0.86_202210111155_4c8f5aa4_beta6_a311d.rar
```

* the applicable rules are the following extracted from `~/OpenNI/OpenNI_2.3.0/rules/orbbec-usb.rules`
```conf
SUBSYSTEM=="usb", ATTR{idProduct}=="0614", ATTR{idVendor}=="2bc5", MODE:="0666", OWNER:="root", GROUP:="video", SYMLINK+="gemini"
SUBSYSTEM=="usb", ATTR{idProduct}=="0511", ATTR{idVendor}=="2bc5", MODE:="0666", OWNER:="root", GROUP:="video", SYMLINK+="gemini_rgb"
```
* install rules
```bash
cd ~/OpenNI/OpenNI_2.3.0/rules
sudo chmod +x install.sh
sudo ./install.sh
sudo udevadm control --reload-rules
sudo udevadm trigger
# unplug + replug camera
```
* run
```bash
~/OpenNI/OpenNI_2.3.0/tools/NiViewer
export OPENNI2_REDIST=$PWD
export LD_LIBRARY_PATH=$PWD:$LD_LIBRARY_PATH
./NiViewer
```

## sanity checks

* color
```bash
mkdir -p output/cam_snapshot
ffmpeg -hide_banner -loglevel error -y -f v4l2 -i /dev/video0 -frames:v 1 output/cam_snapshot/video0.png
```

* depth
```bash
cd ~/OpenNI/OpenNI_2.3.0/tools/NiViewer
(rovi-ros-ws) wass@rovi:~/OpenNI/OpenNI_2.3.0/tools/NiViewer$ ./SimpleRead
Warning: USB events thread - failed to set priority. This might cause loss of data...
ReadBinaryFile, 63
[width height fx fy cx cy baseline]640 400 945.028 945.028 640 400 40
[00000000]     1228
[00030726]        0
[00064006]        0
[00097287]        0
[00130567]        0
[00163848]     1923
[00197128]        0
[00230408]        0
[00263689]        0
[00330250]     1218
[00363530]        0
[00396810]        0
[00430091]        0
[00463371]        0
[00496652]     1948
[00529932]        0
[00563212]        0
[00596493]        0
[00629773]        0
[00696334]     1258
[00729614]     1258
[00762895]     1248
[00796175]     1258
```

# Listing
## usb

```bash
wass@rovi:~$ lsusb
Bus 002 Device 009: ID 2bc5:0511 Orbbec 3D Technology International, Inc USB 2.0 Camera
Bus 003 Device 006: ID 2bc5:0614 Orbbec 3D Technology International, Inc ORBBEC Depth Sensor
```

```bash
wass@rovi:~$ ls -l /dev/v4l/by-id/
total 0
lrwxrwxrwx 1 root root 12 Jan 24 11:07 usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_AY2W35300N8-video-index0 -> ../../video0
lrwxrwxrwx 1 root root 12 Jan 24 11:07 usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_AY2W35300N8-video-index1 -> ../../video1
```
## devices
```bash
wass@rovi:~$ v4l2-ctl --list-devices
pispbe (platform:1000880000.pisp_be):
        /dev/video20
        /dev/video21
        /dev/video22
        /dev/video23
        /dev/video24
        /dev/video25
        /dev/video26
        /dev/video27
        /dev/video28
        /dev/video29
        /dev/video30
        /dev/video31
        /dev/video32
        /dev/video33
        /dev/video34
        /dev/video35
        /dev/video36
        /dev/video37
        /dev/media0
        /dev/media2

rpivid (platform:rpivid):
        /dev/video19
        /dev/media1

USB 2.0 Camera: USB Camera (usb-xhci-hcd.0-1.3.1):
        /dev/video0
        /dev/video1
        /dev/media3
```

## formats
```bash
wass@rovi:~$ v4l2-ctl -d /dev/video0 --list-formats-ext
ioctl: VIDIOC_ENUM_FMT
        Type: Video Capture

        [0]: 'MJPG' (Motion-JPEG, compressed)
                Size: Discrete 640x480
                        Interval: Discrete 0.017s (60.000 fps)
                        Interval: Discrete 0.033s (30.000 fps)
                        Interval: Discrete 0.040s (25.000 fps)
                        Interval: Discrete 0.050s (20.000 fps)
                        Interval: Discrete 0.067s (15.000 fps)
                        Interval: Discrete 0.100s (10.000 fps)
                        Interval: Discrete 0.200s (5.000 fps)
                        Interval: Discrete 0.017s (60.000 fps)
                        Interval: Discrete 0.033s (30.000 fps)
                        Interval: Discrete 0.040s (25.000 fps)
                        Interval: Discrete 0.050s (20.000 fps)
                        Interval: Discrete 0.067s (15.000 fps)
                        Interval: Discrete 0.100s (10.000 fps)
                        Interval: Discrete 0.200s (5.000 fps)
                Size: Discrete 2592x1944
                        Interval: Discrete 0.040s (25.000 fps)
                        Interval: Discrete 0.050s (20.000 fps)
                        Interval: Discrete 0.067s (15.000 fps)
                        Interval: Discrete 0.100s (10.000 fps)
                        Interval: Discrete 0.200s (5.000 fps)
                Size: Discrete 2560x1440
                        Interval: Discrete 0.033s (30.000 fps)
                        Interval: Discrete 0.050s (20.000 fps)
                        Interval: Discrete 0.067s (15.000 fps)
                        Interval: Discrete 0.100s (10.000 fps)
                        Interval: Discrete 0.200s (5.000 fps)
                Size: Discrete 1920x1080
                        Interval: Discrete 0.033s (30.000 fps)
                        Interval: Discrete 0.040s (25.000 fps)
                        Interval: Discrete 0.050s (20.000 fps)
                        Interval: Discrete 0.067s (15.000 fps)
                        Interval: Discrete 0.100s (10.000 fps)
                        Interval: Discrete 0.200s (5.000 fps)
                Size: Discrete 1280x960
                        Interval: Discrete 0.033s (30.000 fps)
                        Interval: Discrete 0.040s (25.000 fps)
                        Interval: Discrete 0.050s (20.000 fps)
                        Interval: Discrete 0.067s (15.000 fps)
                        Interval: Discrete 0.100s (10.000 fps)
                        Interval: Discrete 0.200s (5.000 fps)
                Size: Discrete 1280x720
                        Interval: Discrete 0.033s (30.000 fps)
                        Interval: Discrete 0.040s (25.000 fps)
                        Interval: Discrete 0.050s (20.000 fps)
                        Interval: Discrete 0.067s (15.000 fps)
                        Interval: Discrete 0.100s (10.000 fps)
                        Interval: Discrete 0.200s (5.000 fps)
                Size: Discrete 320x240
                        Interval: Discrete 0.017s (60.000 fps)
                        Interval: Discrete 0.033s (30.000 fps)
                        Interval: Discrete 0.040s (25.000 fps)
                        Interval: Discrete 0.050s (20.000 fps)
                        Interval: Discrete 0.067s (15.000 fps)
                        Interval: Discrete 0.100s (10.000 fps)
                        Interval: Discrete 0.200s (5.000 fps)
                Size: Discrete 640x480
                        Interval: Discrete 0.017s (60.000 fps)
                        Interval: Discrete 0.033s (30.000 fps)
                        Interval: Discrete 0.040s (25.000 fps)
                        Interval: Discrete 0.050s (20.000 fps)
                        Interval: Discrete 0.067s (15.000 fps)
                        Interval: Discrete 0.100s (10.000 fps)
                        Interval: Discrete 0.200s (5.000 fps)
                        Interval: Discrete 0.017s (60.000 fps)
                        Interval: Discrete 0.033s (30.000 fps)
                        Interval: Discrete 0.040s (25.000 fps)
                        Interval: Discrete 0.050s (20.000 fps)
                        Interval: Discrete 0.067s (15.000 fps)
                        Interval: Discrete 0.100s (10.000 fps)
                        Interval: Discrete 0.200s (5.000 fps)
        [1]: 'YUYV' (YUYV 4:2:2)
                Size: Discrete 640x480
                        Interval: Discrete 0.033s (30.000 fps)
                        Interval: Discrete 0.040s (25.000 fps)
                        Interval: Discrete 0.050s (20.000 fps)
                        Interval: Discrete 0.067s (15.000 fps)
                        Interval: Discrete 0.100s (10.000 fps)
                        Interval: Discrete 0.200s (5.000 fps)
                        Interval: Discrete 0.033s (30.000 fps)
                        Interval: Discrete 0.040s (25.000 fps)
                        Interval: Discrete 0.050s (20.000 fps)
                        Interval: Discrete 0.067s (15.000 fps)
                        Interval: Discrete 0.100s (10.000 fps)
                        Interval: Discrete 0.200s (5.000 fps)
                Size: Discrete 2592x1944
                        Interval: Discrete 0.500s (2.000 fps)
                Size: Discrete 2560x1440
                        Interval: Discrete 0.333s (3.000 fps)
                Size: Discrete 1920x1080
                        Interval: Discrete 0.200s (5.000 fps)
                Size: Discrete 1280x960
                        Interval: Discrete 0.200s (5.000 fps)
                Size: Discrete 1280x720
                        Interval: Discrete 0.200s (5.000 fps)
                Size: Discrete 320x240
                        Interval: Discrete 0.017s (60.000 fps)
                        Interval: Discrete 0.033s (30.000 fps)
                        Interval: Discrete 0.040s (25.000 fps)
                        Interval: Discrete 0.050s (20.000 fps)
                        Interval: Discrete 0.067s (15.000 fps)
                        Interval: Discrete 0.100s (10.000 fps)
                        Interval: Discrete 0.200s (5.000 fps)
                Size: Discrete 640x480
                        Interval: Discrete 0.033s (30.000 fps)
                        Interval: Discrete 0.040s (25.000 fps)
                        Interval: Discrete 0.050s (20.000 fps)
                        Interval: Discrete 0.067s (15.000 fps)
                        Interval: Discrete 0.100s (10.000 fps)
                        Interval: Discrete 0.200s (5.000 fps)
                        Interval: Discrete 0.033s (30.000 fps)
                        Interval: Discrete 0.040s (25.000 fps)
                        Interval: Discrete 0.050s (20.000 fps)
                        Interval: Discrete 0.067s (15.000 fps)
                        Interval: Discrete 0.100s (10.000 fps)
                        Interval: Discrete 0.200s (5.000 fps)
```

```bash
wass@rovi:~$ v4l2-ctl -d /dev/video1 --list-formats-ext


```
