(rovi-ros-ws) wass@rovi:~/dev/rovi_ros_ws$ camera
[INFO] [launch]: All log files can be found below /home/wass/.ros/log/2026-02-07-16-43-22-265051-rovi-19542
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.user]: Using venv site-packages: /home/wass/dev/rovi_ros_ws/.venv/lib/python3.12/site-packages
[INFO] [launch.user]: [camera] rgb_video_device: /dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_AY2W35300N8-video-index0
[INFO] [robot_state_publisher-1]: process started with pid [19547]
[INFO] [rovi_odom_integrator_node-2]: process started with pid [19548]
[INFO] [rosmaster_driver_node-3]: process started with pid [19549]
[INFO] [rplidar_composition-4]: process started with pid [19550]
[INFO] [ui_bridge-5]: process started with pid [19551]
[INFO] [serial_display-6]: process started with pid [19552]
[INFO] [joy_node-7]: process started with pid [19553]
[INFO] [teleop_node-8]: process started with pid [19554]
[INFO] [twist_mux-9]: process started with pid [19555]
[INFO] [openni2_camera_driver-10]: process started with pid [19556]
[INFO] [v4l2_camera_node-11]: process started with pid [19558]
[rplidar_composition-4] [INFO] [1770479002.723966996] [rplidar_composition]: RPLIDAR running on ROS 2 package rplidar_ros. SDK Version: '1.12.0'
[robot_state_publisher-1] [INFO] [1770479002.755369296] [robot_state_publisher]: Robot initialized
[twist_mux-9] [INFO] [1770479002.801359310] [twist_mux]: Topic handler 'topics.joy' subscribed to topic 'cmd_vel_joy': timeout = 0.500000s , priority = 100.
[twist_mux-9] [INFO] [1770479002.806046165] [twist_mux]: Topic handler 'topics.keyboard' subscribed to topic 'cmd_vel_keyboard': timeout = 0.500000s , priority = 90.
[twist_mux-9] [INFO] [1770479002.807806574] [twist_mux]: Topic handler 'topics.nav' subscribed to topic 'cmd_vel_nav': timeout = 0.500000s , priority = 50.
[teleop_node-8] [INFO] [1770479002.956428271] [TeleopTwistJoy]: Teleop enable button 5.
[teleop_node-8] [INFO] [1770479002.956525956] [TeleopTwistJoy]: Turbo on button 4.
[teleop_node-8] [INFO] [1770479002.956536382] [TeleopTwistJoy]: Linear axis x on 4 at scale 0.700000.
[teleop_node-8] [INFO] [1770479002.956547753] [TeleopTwistJoy]: Turbo for linear axis x is scale 1.000000.
[teleop_node-8] [INFO] [1770479002.956554086] [TeleopTwistJoy]: Linear axis y on 3 at scale 0.700000.
[teleop_node-8] [INFO] [1770479002.956559086] [TeleopTwistJoy]: Turbo for linear axis y is scale -1.200000.
[teleop_node-8] [INFO] [1770479002.956564327] [TeleopTwistJoy]: Angular axis yaw on 0 at scale 3.200000.
[teleop_node-8] [INFO] [1770479002.956568753] [TeleopTwistJoy]: Turbo for angular axis yaw is scale 5.000000.
[openni2_camera_driver-10] [INFO] [1770479002.985391255] [openni2]: Device "2bc5/0614@3/6" found.
[v4l2_camera_node-11] [INFO] [1770479003.227068999] [v4l2_camera]: Driver: uvcvideo
[v4l2_camera_node-11] [INFO] [1770479003.227196221] [v4l2_camera]: Version: 395276
[v4l2_camera_node-11] [INFO] [1770479003.227203962] [v4l2_camera]: Device: USB 2.0 Camera: USB Camera
[v4l2_camera_node-11] [INFO] [1770479003.227208777] [v4l2_camera]: Location: usb-xhci-hcd.0-1.3.1
[v4l2_camera_node-11] [INFO] [1770479003.227212851] [v4l2_camera]: Capabilities:
[v4l2_camera_node-11] [INFO] [1770479003.227217073] [v4l2_camera]:   Read/write: NO
[v4l2_camera_node-11] [INFO] [1770479003.227222166] [v4l2_camera]:   Streaming: YES
[v4l2_camera_node-11] [INFO] [1770479003.227246462] [v4l2_camera]: Current pixel format: YUYV @ 640x480
[v4l2_camera_node-11] [INFO] [1770479003.227339462] [v4l2_camera]: Available pixel formats:
[v4l2_camera_node-11] [INFO] [1770479003.227350796] [v4l2_camera]:   MJPG - Motion-JPEG
[v4l2_camera_node-11] [INFO] [1770479003.227357240] [v4l2_camera]:   YUYV - YUYV 4:2:2
[v4l2_camera_node-11] [INFO] [1770479003.227362907] [v4l2_camera]: Available controls:
[v4l2_camera_node-11] [INFO] [1770479003.227379425] [v4l2_camera]:   Brightness (1) = 0
[v4l2_camera_node-11] [INFO] [1770479003.227388240] [v4l2_camera]:   Contrast (1) = 32
[v4l2_camera_node-11] [INFO] [1770479003.227395573] [v4l2_camera]:   Saturation (1) = 64
[v4l2_camera_node-11] [INFO] [1770479003.227401851] [v4l2_camera]:   Hue (1) = 0
[v4l2_camera_node-11] [INFO] [1770479003.227408166] [v4l2_camera]:   White Balance, Automatic (2) = 1
[v4l2_camera_node-11] [INFO] [1770479003.227414462] [v4l2_camera]:   Gamma (1) = 100
[v4l2_camera_node-11] [INFO] [1770479003.227421536] [v4l2_camera]:   Gain (1) = 0
[v4l2_camera_node-11] [INFO] [1770479003.227427759] [v4l2_camera]:   Power Line Frequency (3) = 1
[v4l2_camera_node-11] [INFO] [1770479003.227434925] [v4l2_camera]:   White Balance Temperature (1) = 4600 [inactive]
[v4l2_camera_node-11] [INFO] [1770479003.227441370] [v4l2_camera]:   Sharpness (1) = 3
[v4l2_camera_node-11] [INFO] [1770479003.227447851] [v4l2_camera]:   Backlight Compensation (1) = 1
[v4l2_camera_node-11] [ERROR] [1770479003.227462259] [v4l2_camera]: Failed getting value for control 10092545: Permission denied (13); returning 0!
[v4l2_camera_node-11] [INFO] [1770479003.227543481] [v4l2_camera]:   Camera Controls (6) = 0
[v4l2_camera_node-11] [INFO] [1770479003.227559518] [v4l2_camera]:   Auto Exposure (3) = 3
[v4l2_camera_node-11] [INFO] [1770479003.227566925] [v4l2_camera]:   Exposure Time, Absolute (1) = 157 [inactive]
[v4l2_camera_node-11] [INFO] [1770479003.227573722] [v4l2_camera]:   Exposure, Dynamic Framerate (2) = 0
[v4l2_camera_node-11] [WARN] [1770479003.233353429] [camera.color.v4l2_camera]: Control type not currently supported: 6, for control: Camera Controls
[v4l2_camera_node-11] [INFO] [1770479003.233852559] [v4l2_camera]: Starting camera
[serial_display-6] [INFO] [1770479003.280850370] [robot_serial_display]: Waiting for UI gateway at 127.0.0.1:50051 (failed to connect to all addresses; last error: UNKNOWN: ipv4:127.0.0.1:50051: Failed to connect to remote host: connect: Connection refused (111)); retrying in 2.0s
[rosmaster_driver_node-3] [INFO] [1770479003.684987729] [rosmaster_driver]: [init] Initializing hardware adapter ...
[rosmaster_driver_node-3] [INFO] [1770479003.685869526] [rosmaster_driver]: [init] Creating Rosmaster(com=/dev/robot_control, debug=False)
[rosmaster_driver_node-3] Rosmaster Serial Opened! Baudrate=115200
[rosmaster_driver_node-3] [INFO] [1770479003.704863799] [rosmaster_driver]: [init] Rosmaster instance created
[rosmaster_driver_node-3] [INFO] [1770479003.705421133] [rosmaster_driver]: [init] Starting receive thread
[rosmaster_driver_node-3] ----------------create receive threading--------------
[rosmaster_driver_node-3] [INFO] [1770479003.756563872] [rosmaster_driver]: [init] Receive thread started
[rosmaster_driver_node-3] [INFO] [1770479003.757304391] [rosmaster_driver]: [init] Hardware adapter ready
[rosmaster_driver_node-3] [INFO] [1770479003.771878013] [rosmaster_driver]: [init] Publishers and subscriptions created
[rosmaster_driver_node-3] [INFO] [1770479003.772986328] [rosmaster_driver]: [init] Timer created with period 0.100s
[rosmaster_driver_node-3] [INFO] [1770479003.774036496] [rosmaster_driver]: rosmaster_driver started (imu_link=imu_link, prefix='', rate=10.0 Hz, port=/dev/robot_control, debug=False, rot90=True)
[rosmaster_driver_node-3] [INFO] [1770479003.873202955] [rosmaster_driver]: [timer] First timer callback
[v4l2_camera_node-11] [WARN] [1770479003.964363037] [camera.color.v4l2_camera]: Image encoding not the same as requested output, performing possibly slow conversion: yuv422_yuy2 => rgb8
[v4l2_camera_node-11] [INFO] [1770479003.968405003] [camera.color.v4l2_camera]: using default calibration URL
[v4l2_camera_node-11] [INFO] [1770479003.968512744] [camera.color.v4l2_camera]: camera calibration URL: file:///home/wass/.ros/camera_info/usb_2.0_camera:_usb_camera.yaml
[ui_bridge-5] [INFO] [1770479004.158186656] [ui_bridge_metrics]: Subscribed for topic=/voltage type=std_msgs/msg/Float32 (configured)
[ui_bridge-5] [INFO] [1770479004.224206980] [ui_bridge_metrics]: Subscribed for topic=/odom_raw type=nav_msgs/msg/Odometry (configured)
[ui_bridge-5] [INFO] [1770479004.229116354] [ui_bridge_metrics]: Subscribed for topic=/scan type=sensor_msgs/msg/LaserScan (configured)
[ui_bridge-5] [INFO] [1770479004.271679717] [ui_bridge_robot_state]: Fixed frame resolved from session: odom (stack=camera)
[ui_bridge-5] [INFO] [1770479004.278365203] [ui_bridge_robot_state]: Robot state downsampling: disabled (forward on every odom)
[ui_bridge-5] [INFO] [1770479004.305463982] [ui_bridge_lidar]: Lidar downsampling: /scan -> /viz/scan disabled (forward all scans)
[ui_bridge-5] [INFO] [1770479004.327246053] [ui_bridge_map]: Map downsampling: /map @ 0.50 Hz cap
[ui_bridge-5] [INFO] [1770479004.345054824] [ros_ui_bridge]: ros_ui_bridge gRPC listening on 0.0.0.0:50051
[rplidar_composition-4] [INFO] [1770479005.238485454] [rplidar_composition]: RPLIDAR S/N: C1CAFA86C2E392D0A5E59FF7561E5C60
[rplidar_composition-4] [INFO] [1770479005.238571640] [rplidar_composition]: Firmware Ver: 1.29
[rplidar_composition-4] [INFO] [1770479005.238587917] [rplidar_composition]: Hardware Rev: 7
[rplidar_composition-4] [INFO] [1770479005.241834901] [rplidar_composition]: RPLidar health status : '0'
[rplidar_composition-4] [INFO] [1770479005.241875012] [rplidar_composition]: Start
[serial_display-6] [INFO] [1770479005.287182600] [robot_serial_display]: Connected to UI gateway at 127.0.0.1:50051
[serial_display-6] [INFO] [1770479005.289031879] [robot_serial_display]: Opened display serial on /dev/robot_display @ 256000 baud
[rplidar_composition-4] [INFO] [1770479005.802880408] [rplidar_composition]: current scan mode: Sensitivity, max_distance: 12.0 m, Point number: 7.9K , angle_compensate: 2, flip_x_axis 0
[openni2_camera_driver-10] Warning: USB events thread - failed to set priority. This might cause loss of data...