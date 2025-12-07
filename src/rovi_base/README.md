# rovi_base

Odometry node that integrates `vel_raw` (Twist) feedback from the Rosmaster driver and publishes `odom_raw` plus an optional TF `odom -> base_footprint`.

## Topics
- Subscribes: `vel_raw` (`geometry_msgs/Twist`, defaults to SensorDataQoS).
- Publishes: `odom_raw` (`nav_msgs/Odometry`, reliable QoS).
- TF: optional `odom_frame -> base_frame` transform.

## Parameters
- `odom_frame` (string, default `odom`): parent frame id.
- `base_frame` (string, default `base_footprint`): child frame id.
- `vel_topic` (string, default `vel_raw`): input Twist topic.
- `odom_topic` (string, default `odom_raw`): output Odometry topic.
- `linear_scale_x` / `linear_scale_y` (double, default 1.0): scale factors applied to incoming velocities.
- `publish_tf` (bool, default `true`): enable TF broadcast.
- `publish_rate` (double, default `10.0`): expected velocity feedback rate; `integrator_period` derives from this.
- `integrator_period` (double, default auto): optional override; if set and mismatched with `1/publish_rate` a warning is emitted.
- `drop_warn_factor` (double, default `3.0`): diagnostics warning threshold is `drop_warn_factor * integrator_period` since last vel_raw message.
- `diagnostics_period` (double, default `10.0` seconds): how often to publish diagnostics; set `<= 0` to disable.

## Timing and clocks
- Integration runs on a fixed-period timer using the latest received velocity; the period defaults to `1 / publish_rate` and can be overridden.
- Arrival intervals are still monitored (min/max/last) to spot bursts or gaps.
- Message and TF headers are stamped with the ROS clock (`get_clock()->now()`); consumers see ROS time on the outputs.

## Diagnostics
- Publishes a `timing` diagnostic: message count, last/min/max arrival `dt`, age of last message, and count of non-positive `dt`.
- Warns when no messages have arrived yet or when the last message is older than `drop_warn_factor * integrator_period` (possible drops).

## Example run
```bash
ros2 run rovi_base rovi_base_node --ros-args \
  -p publish_tf:=true \
  -p odom_frame:=odom \
  -p base_frame:=base_footprint \
  -p publish_rate:=10.0 \
  -p drop_warn_factor:=3.0
```

# Diagnostics
* diagnostics shows a healthy output steady at 100 message_count delta (10 Hz x 10 s) and arrival only jitters between ~ 0.91 and 0.1004

```bash
>ros2 topic echo /diagnostics
---
header:
  stamp:
    sec: 1765117525
    nanosec: 457757048
  frame_id: ''
status:
- level: "\0"
  name: 'rovi_base: timing'
  message: Timing OK
  hardware_id: rovi_base
  values:
  - key: message_count
    value: '5992'
  - key: last_arrival_dt
    value: '0.0998525'
  - key: last_age
    value: '0.0971692'
  - key: min_arrival_dt
    value: '0.0918049'
  - key: max_arrival_dt
    value: '0.100394'
  - key: dt_nonpositive_count
    value: '0'
```