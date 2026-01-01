#!/usr/bin/env python3
"""UI bridge main node: wires ROS nodes + gRPC service together."""
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
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor

from .api import ui_bridge_pb2_grpc
from .config import UiBridgeConfig, load_config
from .grpc_gateway import UiBridgeService
from .lidar_node import LidarScanData, UiBridgeLidarNode
from .robot_model_provider import RobotModelProvider
from .robot_state_node import RobotStateData, UiBridgeRobotStateNode
from .ros_metrics_node import UiBridgeRosNode
from .status_store import RateMetricSnapshot, SnapshotBroadcaster
from .throttled_forwarder import AsyncStreamBroadcaster
from .voltage_state import VoltageState


async def _publish_status_loop(
    *,
    cfg: UiBridgeConfig,
    voltage_state: VoltageState,
    ros_node: UiBridgeRosNode,
    broadcaster: SnapshotBroadcaster,
    stop_event: asyncio.Event,
) -> None:
    """Status stream still uses timer-based collection (legacy SnapshotBroadcaster)."""
    period_s = cfg.status_stream.period_s
    if period_s <= 0.0:
        period_s = 3.0

    while not stop_event.is_set():
        cpu_percent = float(psutil.cpu_percent(interval=None))
        voltage_v, _last_update = voltage_state.read()
        rates = [
            RateMetricSnapshot(id=metric_id, hz=hz, target_hz=target_hz)
            for metric_id, hz, target_hz in ros_node.sample_rate_metrics()
        ]
        await broadcaster.publish(cpu_percent=cpu_percent, voltage_v=voltage_v, rates=rates)
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
    robot_state_broadcaster: AsyncStreamBroadcaster[RobotStateData],
    lidar_broadcaster: Optional[AsyncStreamBroadcaster[LidarScanData]],
    logger: logging.Logger,
) -> None:
    stop_event = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, stop_event.set)
        except NotImplementedError:
            pass

    # Status still uses SnapshotBroadcaster (timer-based collection)
    status_broadcaster = SnapshotBroadcaster()

    model_provider = RobotModelProvider(glb_path=cfg.robot_model.glb_path)

    service = UiBridgeService(
        status_broadcaster=status_broadcaster,
        robot_state_broadcaster=robot_state_broadcaster,
        lidar_broadcaster=lidar_broadcaster,
        model_provider=model_provider,
        model_chunk_size_bytes=cfg.robot_model.chunk_size_bytes,
        odom_frame=cfg.robot_state_stream.odom_frame,
        base_frame=cfg.robot_state_stream.base_frame,
        map_frame=cfg.robot_state_stream.map_frame,
        wheel_joint_names=cfg.robot_state_stream.wheel_joint_names,
    )

    tasks: list[asyncio.Task[None]] = []

    # Only status uses a publish loop now; robot_state and lidar are event-driven
    tasks.append(asyncio.create_task(
        _publish_status_loop(
            cfg=cfg,
            voltage_state=voltage_state,
            ros_node=ros_node,
            broadcaster=status_broadcaster,
            stop_event=stop_event,
        )
    ))
    tasks.append(asyncio.create_task(_serve_grpc(bind=cfg.grpc_bind, service=service, stop_event=stop_event, logger=logger)))

    try:
        await asyncio.gather(*tasks)
    finally:
        for task in tasks:
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
        topic_rates=cfg.status_stream.rates,
        tf_rates=cfg.status_stream.tf_rates,
    )

    # Create broadcasters for queue-based gRPC streaming
    robot_state_broadcaster: AsyncStreamBroadcaster[RobotStateData] = AsyncStreamBroadcaster()
    lidar_broadcaster: Optional[AsyncStreamBroadcaster[LidarScanData]] = None

    state_node = UiBridgeRobotStateNode(
        odom_topic=cfg.robot_state_stream.odom_topic,
        joint_states_topic=cfg.robot_state_stream.joint_states_topic,
        odom_frame=cfg.robot_state_stream.odom_frame,
        base_frame=cfg.robot_state_stream.base_frame,
        map_frame=cfg.robot_state_stream.map_frame,
        wheel_joint_names=cfg.robot_state_stream.wheel_joint_names,
        map_tf_max_age_s=cfg.robot_state_stream.map_tf_max_age_s,
        period_s=cfg.robot_state_stream.period_s,
        grpc_broadcaster=robot_state_broadcaster,
    )

    lidar_node: Optional[UiBridgeLidarNode] = None
    if cfg.lidar_stream is not None:
        lidar_broadcaster = AsyncStreamBroadcaster()
        lidar_node = UiBridgeLidarNode(
            input_topic=cfg.lidar_stream.topic,
            output_topic=cfg.lidar_stream.output_topic,
            frame_id=cfg.lidar_stream.frame_id,
            period_s=cfg.lidar_stream.period_s,
            grpc_broadcaster=lidar_broadcaster,
        )

    executor = SingleThreadedExecutor()
    executor.add_node(ros_node)
    executor.add_node(state_node)
    if lidar_node is not None:
        executor.add_node(lidar_node)

    spin_thread = threading.Thread(target=executor.spin, name='ros_ui_bridge_ros_spin', daemon=True)
    spin_thread.start()

    # Prime psutil so the first value is meaningful.
    psutil.cpu_percent(None)

    try:
        try:
            asyncio.run(
                _run_async(
                    cfg=cfg,
                    voltage_state=voltage_state,
                    ros_node=ros_node,
                    robot_state_broadcaster=robot_state_broadcaster,
                    lidar_broadcaster=lidar_broadcaster,
                    logger=logger,
                )
            )
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
    finally:
        try:
            executor.shutdown()
        except Exception:
            pass

        try:
            ros_node.destroy_node()
        except Exception:
            pass

        try:
            state_node.destroy_node()
        except Exception:
            pass

        if lidar_node is not None:
            try:
                lidar_node.destroy_node()
            except Exception:
                pass

        try:
            rclpy.shutdown()
        except Exception:
            pass

        try:
            spin_thread.join(timeout=2.0)
        except Exception:
            pass


if __name__ == '__main__':
    main()
