#!/usr/bin/env python3
"""UI bridge main node: wires ROS nodes + gRPC service together."""
from __future__ import annotations

import argparse
import asyncio
import signal
import threading
from typing import Optional

import grpc
import psutil
import rclpy
from rclpy.logging import get_logger
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor

from .api import ui_bridge_pb2_grpc
from .config import UiBridgeConfig, load_config
from .grpc_gateway import UiBridgeService
from .floor_topology_node import FloorTopologyData, UiBridgeFloorTopologyNode
from .lidar_node import LidarScanData, UiBridgeLidarNode
from .map_node import MapData, UiBridgeMapNode
from .robot_model_provider import RobotModelProvider
from .robot_state_node import RobotStateData, UiBridgeRobotStateNode
from .ros_metrics_node import UiBridgeRosNode
from .status_store import StatusBroadcaster, StatusFieldMeta, StatusFieldValue
from .throttled_forwarder import AsyncStreamBroadcaster
from .session_info import SESSION_CURRENT_LAUNCH_REF_TOPIC, fixed_frame_from_stack, stack_from_launch_ref
from .conductor.systemd import process_is_running, systemd_is_active
from .conductor.net import iface_status_text

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


async def _publish_status_loop(
    *,
    cfg: UiBridgeConfig,
    ros_node: UiBridgeRosNode,
    broadcaster: StatusBroadcaster,
    stop_event: asyncio.Event,
    system_fields: list,
) -> None:
    """Collect status fields on a fixed cadence and publish when they change."""

    period_s = cfg.status_stream.period_s
    if period_s <= 0.0:
        period_s = 3.0
    debug_log = cfg.status_stream.debug_log
    always_publish = cfg.status_stream.always_publish

    last_signature: Optional[tuple[tuple[str, float, int, int], ...]] = None
    seq = 0

    while not stop_event.is_set():
        now_ros = ros_node.get_clock().now()

        values: list[StatusFieldValue] = []

        # System providers (e.g., cpu_percent) are computed here.
        for field in system_fields:
            if field.source.type == 'cpu_percent':
                cpu_percent = float(psutil.cpu_percent(interval=None))
                values.append(StatusFieldValue(id=field.id, value=cpu_percent, text=None, stamp=now_ros.to_msg()))
            elif field.source.type == 'service':
                service_name = (field.source.service or '').strip()
                text = systemd_is_active(service_name) if service_name else 'unknown'
                values.append(StatusFieldValue(id=field.id, value=0.0, text=text, stamp=now_ros.to_msg()))
            elif field.source.type == 'process':
                process_name = (field.source.process or '').strip()
                service_name = (field.source.service or '').strip() if field.source.service else None
                if process_name:
                    running = process_is_running(process_name, service=service_name)
                    text = 'running' if running else 'missing'
                else:
                    text = 'unknown'
                values.append(StatusFieldValue(id=field.id, value=0.0, text=text, stamp=now_ros.to_msg()))
            elif field.source.type == 'net_iface':
                iface = (field.source.iface or '').strip()
                text = iface_status_text(iface) if iface else 'unknown'
                values.append(StatusFieldValue(id=field.id, value=0.0, text=text, stamp=now_ros.to_msg()))

        # ROS-derived fields (topic values + rates).
        for field_id, value, stamp in ros_node.collect_ros_values(now_ros=now_ros):
            values.append(StatusFieldValue(id=field_id, value=value, text=None, stamp=stamp))

        signature = tuple(sorted((v.id, v.value, v.text or '', v.stamp.sec, v.stamp.nanosec) for v in values))

        # If we've never sent anything and have no values yet, stay quiet to avoid empty spam.
        if last_signature is None and not signature and not always_publish:
            await asyncio.sleep(period_s)
            continue

        if always_publish or signature != last_signature:
            seq += 1
            snapshot = broadcaster.build_snapshot(seq=seq, stamp=now_ros.to_msg(), values=values)
            await broadcaster.publish(snapshot)
            last_signature = signature
            if debug_log:
                ids = sorted({v.id for v in values})
                ros_node.get_logger().info(
                    f"Status publish seq={seq} ids={ids} values={len(values)}"
                )

        await asyncio.sleep(period_s)


async def _serve_grpc(
    *,
    bind: str,
    service: UiBridgeService,
    stop_event: asyncio.Event,
    logger,
) -> None:
    server = grpc.aio.server()
    ui_bridge_pb2_grpc.add_UiBridgeServicer_to_server(service, server)
    bound_port = server.add_insecure_port(bind)
    if not bound_port:
        # This is the typical symptom of a duplicate gateway plane (or another process)
        # already holding the gRPC port.
        logger.error(
            f"ros_ui_bridge failed to bind gRPC on {bind} (port already in use or invalid bind); exiting"
        )
        stop_event.set()
        return
    try:
        await server.start()
    except Exception as exc:  # noqa: BLE001
        logger.error(f"ros_ui_bridge gRPC failed to start on {bind}: {exc}")
        stop_event.set()
        return
    logger.info(f"ros_ui_bridge gRPC listening on {bind} (bound_port={bound_port})")

    try:
        await stop_event.wait()
    finally:
        await server.stop(grace=1.0)


async def _run_async(
    *,
    cfg: UiBridgeConfig,
    ros_node: UiBridgeRosNode,
    robot_state_broadcaster: AsyncStreamBroadcaster[RobotStateData],
    lidar_broadcaster: Optional[AsyncStreamBroadcaster[LidarScanData]],
    map_broadcaster: Optional[AsyncStreamBroadcaster[MapData]],
    floor_topology_broadcaster: Optional[AsyncStreamBroadcaster[FloorTopologyData]],
    logger,
) -> None:
    stop_event = asyncio.Event()
    loop = asyncio.get_running_loop()

    # Ensure broadcasters know which asyncio loop to publish onto.
    robot_state_broadcaster.set_loop(loop)
    if lidar_broadcaster is not None:
        lidar_broadcaster.set_loop(loop)
    if map_broadcaster is not None:
        map_broadcaster.set_loop(loop)
    if floor_topology_broadcaster is not None:
        floor_topology_broadcaster.set_loop(loop)

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, stop_event.set)
        except NotImplementedError:
            pass

    fields_meta = [
        StatusFieldMeta(id=f.id, unit=f.unit, value_type=f.value_type, min=f.min, max=f.max, target=f.target)
        for f in cfg.status_stream.fields
    ]
    system_fields = [f for f in cfg.status_stream.fields if f.source.provider == 'system']
    status_broadcaster = StatusBroadcaster(
        fields=fields_meta,
        current_launch_ref=None,
        stack=None,
        fixed_frame=None,
    )

    session_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

    def _on_session(msg: String) -> None:
        ref = str(msg.data or "").strip()
        if not ref:
            return
        stack = stack_from_launch_ref(ref)
        status_broadcaster.set_session(
            current_launch_ref=ref,
            stack=stack,
            fixed_frame=fixed_frame_from_stack(stack),
        )

    # Keep the subscription referenced so it stays alive for the executor thread.
    ros_node._session_sub = ros_node.create_subscription(String, SESSION_CURRENT_LAUNCH_REF_TOPIC, _on_session, session_qos)  # type: ignore[attr-defined]

    model_provider = RobotModelProvider(glb_path=cfg.robot_model.glb_path)

    service = UiBridgeService(
        status_broadcaster=status_broadcaster,
        robot_state_broadcaster=robot_state_broadcaster,
        lidar_broadcaster=lidar_broadcaster,
        map_broadcaster=map_broadcaster,
        floor_topology_broadcaster=floor_topology_broadcaster,
        model_provider=model_provider,
        model_chunk_size_bytes=cfg.robot_model.chunk_size_bytes,
        odom_frame=cfg.robot_state_stream.odom_frame,
        base_frame=cfg.robot_state_stream.base_frame,
        map_frame=cfg.robot_state_stream.map_frame,
        wheel_joint_names=cfg.robot_state_stream.wheel_joint_names,
        control=cfg.control,
    )

    tasks: list[asyncio.Task[None]] = []

    # Only status uses a publish loop now; robot_state and lidar are event-driven
    tasks.append(asyncio.create_task(
        _publish_status_loop(
            cfg=cfg,
            ros_node=ros_node,
            broadcaster=status_broadcaster,
            stop_event=stop_event,
            system_fields=system_fields,
        )
    ))
    tasks.append(asyncio.create_task(_serve_grpc(bind=cfg.grpc_bind, service=service, stop_event=stop_event, logger=logger)))

    try:
        await asyncio.gather(*tasks)
    finally:
        for task in tasks:
            task.cancel()


def main(argv: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(description='ROS UI bridge (gRPC).')
    parser.add_argument('--config', default=None, help='Path to YAML config (defaults to package config/default.yaml).')
    args, ros_args = parser.parse_known_args(argv)

    cfg = load_config(args.config)

    rclpy.init(args=ros_args)
    logger = get_logger('ros_ui_bridge')
    ros_node = UiBridgeRosNode(
        status_fields=cfg.status_stream.fields,
        tf_demux=cfg.tf_demux,
    )

    # Create broadcasters for queue-based gRPC streaming
    robot_state_broadcaster: AsyncStreamBroadcaster[RobotStateData] = AsyncStreamBroadcaster()
    lidar_broadcaster: Optional[AsyncStreamBroadcaster[LidarScanData]] = None
    map_broadcaster: Optional[AsyncStreamBroadcaster[MapData]] = None
    floor_topology_broadcaster: Optional[AsyncStreamBroadcaster[FloorTopologyData]] = None

    state_node = UiBridgeRobotStateNode(
        odom_topic=cfg.robot_state_stream.odom_topic,
        joint_states_topic=cfg.robot_state_stream.joint_states_topic,
        odom_frame=cfg.robot_state_stream.odom_frame,
        base_frame=cfg.robot_state_stream.base_frame,
        map_frame=cfg.robot_state_stream.map_frame,
        wheel_joint_names=cfg.robot_state_stream.wheel_joint_names,
        map_tf_max_age_s=cfg.robot_state_stream.map_tf_max_age_s,
        downsampling_period_s=cfg.robot_state_stream.downsampling_period_s,
        grpc_broadcaster=robot_state_broadcaster,
    )

    lidar_node: Optional[UiBridgeLidarNode] = None
    if cfg.lidar_stream is not None:
        lidar_broadcaster = AsyncStreamBroadcaster()
        lidar_node = UiBridgeLidarNode(
            input_topic=cfg.lidar_stream.topic,
            output_topic=cfg.lidar_stream.output_topic,
            frame_id=cfg.lidar_stream.frame_id,
            downsampling_period_s=cfg.lidar_stream.downsampling_period_s,
            grpc_broadcaster=lidar_broadcaster,
        )

    map_node: Optional[UiBridgeMapNode] = None
    if cfg.map_stream is not None:
        map_broadcaster = AsyncStreamBroadcaster()
        map_node = UiBridgeMapNode(
            topic=cfg.map_stream.topic,
            downsampling_period_s=cfg.map_stream.downsampling_period_s,
            grpc_broadcaster=map_broadcaster,
        )

    floor_topology_node: Optional[UiBridgeFloorTopologyNode] = None
    if cfg.floor_topology_stream is not None:
        floor_topology_broadcaster = AsyncStreamBroadcaster()
        floor_topology_node = UiBridgeFloorTopologyNode(
            topic=cfg.floor_topology_stream.topic,
            downsampling_period_s=cfg.floor_topology_stream.downsampling_period_s,
            grpc_broadcaster=floor_topology_broadcaster,
        )

    executor = SingleThreadedExecutor()
    executor.add_node(ros_node)
    executor.add_node(state_node)
    if lidar_node is not None:
        executor.add_node(lidar_node)
    if map_node is not None:
        executor.add_node(map_node)
    if floor_topology_node is not None:
        executor.add_node(floor_topology_node)

    spin_thread = threading.Thread(target=executor.spin, name='ros_ui_bridge_ros_spin', daemon=True)
    spin_thread.start()

    # Prime psutil so the first value is meaningful.
    psutil.cpu_percent(None)

    try:
        try:
            asyncio.run(
                _run_async(
                    cfg=cfg,
                    ros_node=ros_node,
                    robot_state_broadcaster=robot_state_broadcaster,
                    lidar_broadcaster=lidar_broadcaster,
                    map_broadcaster=map_broadcaster,
                    floor_topology_broadcaster=floor_topology_broadcaster,
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

        if map_node is not None:
            try:
                map_node.destroy_node()
            except Exception:
                pass

        if floor_topology_node is not None:
            try:
                floor_topology_node.destroy_node()
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
