ROVI_ENV=robot
# NOTE: systemd EnvironmentFile does not expand $HOME; keep this absolute.
ROVI_ROS_WS_DIR=/home/wass/dev/rovi_ros_ws

# DDS / discovery
ROS_DOMAIN_ID=0
ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# Odometry TF publisher choice for the always-on gateway:
# - 1: gateway publishes TF odom->base_footprint from the raw odom integrator (teleop-friendly default)
# - 0: disable integrator TF so EKF stacks (mapping/localization/nav) can publish TF without conflicts
ROVI_ODOM_INTEGRATOR_PUBLISH_TF=1
