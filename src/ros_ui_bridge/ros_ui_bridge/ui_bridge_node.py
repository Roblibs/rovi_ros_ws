#!/usr/bin/env python3
from __future__ import annotations

import argparse
import asyncio
import logging
import signal
import threading
from typing import Optional

import grpc
import psutil
import rclpy
from rclpy.executors import SingleThreadedExecutor

from .api import ui_bridge_pb2_grpc
from .config import UiBridgeConfig, load_config
from .grpc_gateway import UiBridgeService
from .robot_model_provider import RobotModelProvider
from .robot_state_node import UiBridgeRobotStateNode
from .robot_state_store import RobotStateBroadcaster
from .ros_metrics_node import UiBridgeRosNode
from .status_store import RateMetricSnapshot, SnapshotBroadcaster
from .voltage_state import VoltageState


async def _publish_loop(
    *,
    cfg: UiBridgeConfig,
    voltage_state: VoltageState,
    ros_node: UiBridgeRosNode,
    broadcaster: SnapshotBroadcaster,
    stop_event: asyncio.Event,
) -> None:
    while not stop_event.is_set():
        cpu_percent = float(psutil.cpu_percent(interval=None))
        voltage_v, _last_update = voltage_state.read()
        rates = [
            RateMetricSnapshot(id=metric_id, hz=hz, target_hz=target_hz)
            for metric_id, hz, target_hz in ros_node.sample_rate_metrics()
        ]
        await broadcaster.publish(cpu_percent=cpu_percent, voltage_v=voltage_v, rates=rates)
        await asyncio.sleep(cfg.update_period_s)


async def _publish_robot_state_loop(
    *,
    cfg: UiBridgeConfig,
    state_node: UiBridgeRobotStateNode,
    broadcaster: RobotStateBroadcaster,
    stop_event: asyncio.Event,
) -> None:
    period_s = 1.0 / float(cfg.robot_state.update_hz)
    if period_s <= 0.0:
        period_s = 0.1

    while not stop_event.is_set():
        pose_odom, pose_map, wheel_angles = state_node.sample()
        await broadcaster.publish(pose_odom=pose_odom, pose_map=pose_map, wheel_angles=wheel_angles)
        await asyncio.sleep(period_s)


async def _serve_grpc(
    *,
    bind: str,
    service: UiBridgeService,
    stop_event: asyncio.Event,
    logger: logging.Logger,
) -> None:
    server = grpc.aio.server()
    ui_bridge_pb2_grpc.add_UiBridgeServicer_to_server(service, server)
    server.add_insecure_port(bind)
    await server.start()
    logger.info("ros_ui_bridge gRPC listening on %s", bind)

    try:
        await stop_event.wait()
    finally:
        await server.stop(grace=1.0)


async def _run_async(
    *,
    cfg: UiBridgeConfig,
    voltage_state: VoltageState,
    ros_node: UiBridgeRosNode,
    state_node: UiBridgeRobotStateNode,
    logger: logging.Logger,
) -> None:
    stop_event = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, stop_event.set)
        except NotImplementedError:
            pass

    status_broadcaster = SnapshotBroadcaster()
    robot_state_broadcaster = RobotStateBroadcaster()

    model_provider = RobotModelProvider(glb_path=cfg.robot_model.glb_path)

    service = UiBridgeService(
        status_broadcaster=status_broadcaster,
        robot_state_broadcaster=robot_state_broadcaster,
        model_provider=model_provider,
        model_chunk_size_bytes=cfg.robot_model.chunk_size_bytes,
        odom_frame=cfg.robot_state.odom_frame,
        base_frame=cfg.robot_state.base_frame,
        map_frame=cfg.robot_state.map_frame,
        wheel_joint_names=cfg.robot_state.wheel_joint_names,
    )

    publisher_task = asyncio.create_task(
        _publish_loop(
            cfg=cfg,
            voltage_state=voltage_state,
            ros_node=ros_node,
            broadcaster=status_broadcaster,
            stop_event=stop_event,
        )
    )
    robot_state_task = asyncio.create_task(
        _publish_robot_state_loop(
            cfg=cfg,
            state_node=state_node,
            broadcaster=robot_state_broadcaster,
            stop_event=stop_event,
        )
    )
    grpc_task = asyncio.create_task(_serve_grpc(bind=cfg.grpc_bind, service=service, stop_event=stop_event, logger=logger))

    try:
        await asyncio.gather(publisher_task, robot_state_task, grpc_task)
    finally:
        for task in (publisher_task, robot_state_task, grpc_task):
            task.cancel()


def main(argv: Optional[list[str]] = None) -> None:
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger('ros_ui_bridge')

    parser = argparse.ArgumentParser(description='ROS UI bridge (gRPC).')
    parser.add_argument('--config', default=None, help='Path to YAML config (defaults to package config/default.yaml).')
    args, ros_args = parser.parse_known_args(argv)

    cfg = load_config(args.config)

    rclpy.init(args=ros_args)
    voltage_state = VoltageState()
    ros_node = UiBridgeRosNode(
        voltage_state=voltage_state,
        voltage_topic=cfg.voltage_topic,
        topic_rates=cfg.topic_rates,
        tf_rates=cfg.tf_rates,
    )
    state_node = UiBridgeRobotStateNode(
        odom_topic=cfg.robot_state.odom_topic,
        joint_states_topic=cfg.robot_state.joint_states_topic,
        odom_frame=cfg.robot_state.odom_frame,
        base_frame=cfg.robot_state.base_frame,
        map_frame=cfg.robot_state.map_frame,
        wheel_joint_names=cfg.robot_state.wheel_joint_names,
        map_tf_max_age_s=cfg.robot_state.map_tf_max_age_s,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(ros_node)
    executor.add_node(state_node)

    spin_thread = threading.Thread(target=executor.spin, name='ros_ui_bridge_ros_spin', daemon=True)
    spin_thread.start()

    # Prime psutil so the first value is meaningful.
    psutil.cpu_percent(None)

    try:
        asyncio.run(_run_async(cfg=cfg, voltage_state=voltage_state, ros_node=ros_node, state_node=state_node, logger=logger))
    finally:
        executor.shutdown()
        ros_node.destroy_node()
        state_node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
