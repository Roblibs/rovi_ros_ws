

```bash
g++ -std=c++17 ob_list.cpp \
  -I/opt/ros/jazzy/include \
  -L/opt/ros/jazzy/lib -lOrbbecSDK -Wl,-rpath,/opt/ros/jazzy/lib \
  -o ob_list
```
## ob_list

Quick sanity-check for whether OpenOrbbecSDK (used by `ros-jazzy-orbbec-camera`) can see any devices.

Build:

`g++ -std=c++17 ob_list.cpp -I/opt/ros/jazzy/include -L/opt/ros/jazzy/lib -lOrbbecSDK -Wl,-rpath,/opt/ros/jazzy/lib -o ob_list`

Run (uses the same config file the ROS node uses):

`./ob_list`

Optional: pass a different SDK config path as arg1:

`./ob_list /path/to/OrbbecSDKConfig.xml`

## ros_snapshot

Save a single snapshot from ROS image topics (no OpenCV; writes `.pgm`/`.ppm`/`.pfm`).

Default topics:
- `/depth_raw/image`
- `/ir/image_raw`

Run:

`python3 tools/depth/ros_snapshot.py`

Custom topics/output:

`python3 tools/depth/ros_snapshot.py --topic /depth_raw/image --out-dir output/cam_snapshot_ros`

Note: the saved depth image is 16-bit (`.pgm` with maxval 65535). Many image viewers display it as black; use the generated `*_viz.pgm` (8-bit scaled) for quick inspection.

## openni2_list_modes

List the OpenNI2 device’s supported video modes (resolution/fps/pixel format) for depth/IR/color.

Build (uses the Orbbec OpenNI2 SDK headers/libs):

`g++ -std=c++17 openni2_list_modes.cpp -I$HOME/OpenNI/OpenNI_2.3.0/sdk/Include -L$HOME/OpenNI/OpenNI_2.3.0/tools/NiViewer -lOpenNI2 -Wl,-rpath,$HOME/OpenNI/OpenNI_2.3.0/tools/NiViewer -o openni2_list_modes`

Run:

`OPENNI2_REDIST=$HOME/OpenNI/OpenNI_2.3.0/tools/NiViewer LD_LIBRARY_PATH=$HOME/OpenNI/OpenNI_2.3.0/tools/NiViewer:$LD_LIBRARY_PATH ./openni2_list_modes`

runlog
```bash
(rovi-ros-ws) wass@rovi:~/dev/rovi_ros_ws/tools/depth$ ./ob_list
OpenOrbbecSDK version: 2.5.5 (open-source-beta)
Config path: /opt/ros/jazzy/share/orbbec_camera/config/OrbbecSDKConfig_v2.0.xml
[01/25 15:32:08.223134][error][26169][UsbEnumeratorLibusb.cpp:157] Invalid descriptor index: 0
[01/25 15:32:08.223201][error][26169][UsbEnumeratorLibusb.cpp:403] Failed to query USB device serial number
deviceCount=0
```
