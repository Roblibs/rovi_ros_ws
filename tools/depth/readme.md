

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

runlog
```bash
(rovi-ros-ws) wass@rovi:~/dev/rovi_ros_ws/tools/depth$ ./ob_list
OpenOrbbecSDK version: 2.5.5 (open-source-beta)
Config path: /opt/ros/jazzy/share/orbbec_camera/config/OrbbecSDKConfig_v2.0.xml
[01/25 15:32:08.223134][error][26169][UsbEnumeratorLibusb.cpp:157] Invalid descriptor index: 0
[01/25 15:32:08.223201][error][26169][UsbEnumeratorLibusb.cpp:403] Failed to query USB device serial number
deviceCount=0
```