
## Troubleshooting: pose "jitter"/shaking in RViz
If the robot appears to twitch/rotate slightly in place (especially in `rovi_map.rviz`), isolate *which TF link* is moving first:

- **Is the jitter only in `map` frame?** Switch RViz Fixed Frame to `odom` (or run `rviz2 -d install/share/rovi_description/rviz/rovi.rviz`). If it’s stable in `odom` but not in `map`, the motion is coming from SLAM’s `map -> odom` correction (expected to move as scan-matching/loop-closure refines).
- **Is the jitter already in `odom` frame?** Then it’s coming from the odometry pipeline (`odom -> base_footprint`), i.e., wheels (`/odom_raw`) and/or IMU fusion (`/imu/data`).

Config knobs to try (in order):

1) **IMU frame consistency (most common):** if your driver already rotates IMU axes into REP-103, publish IMU data in `base_footprint` to avoid an extra TF rotation from `imu_link`.
   - File: `rovi_ros_ws/src/rovi_bringup/config/rosmaster_driver.yaml` (`rosmaster_driver.ros__parameters.imu_link`)

2) **Don’t over-trust IMU orientation:** ensure `/imu/data` has a non-zero `orientation_covariance`.
   - File: `rovi_ros_ws/src/rovi_localization/config/imu_filter_madgwick.yaml` (`imu_filter.ros__parameters.orientation_stddev`)

3) **Reduce IMU influence in EKF:** if yaw-rate is noisy (and covariance is effectively “too perfect”), fuse IMU yaw *without* IMU yaw-rate first.
   - File: `rovi_ros_ws/src/rovi_localization/config/ekf_odom_imu.yaml` (`imu0_config`)

If the jitter is from SLAM (`map -> odom`) and you want it visually calmer, tune `slam_toolbox` scan processing cadence:
- File: `rovi_ros_ws/src/rovi_slam/config/slam_toolbox_mapping.yaml` (`minimum_time_interval`, `throttle_scans`, `transform_publish_period`, loop-closure settings)
