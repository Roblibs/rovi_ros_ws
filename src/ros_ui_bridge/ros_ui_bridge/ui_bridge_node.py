#!/usr/bin/env python3
"""UI bridge main node: wires ROS nodes + gRPC service together."""
from __future__ import annotations

import argparse
import asyncio
import signal
import subprocess
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
from .lidar_node import LidarScanData, UiBridgeLidarNode
from .map_node import MapData, UiBridgeMapNode
from .robot_model_provider import RobotModelProvider
from .robot_state_node import RobotStateData, UiBridgeRobotStateNode
from .ros_metrics_node import UiBridgeRosNode
from .status_store import StatusBroadcaster, StatusFieldMeta, StatusFieldValue
from .throttled_forwarder import AsyncStreamBroadcaster
from .session_info import resolve_session


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
                text = _systemd_is_active(service_name) if service_name else 'unknown'
                values.append(StatusFieldValue(id=field.id, value=0.0, text=text, stamp=now_ros.to_msg()))
            elif field.source.type == 'process':
                process_name = (field.source.process or '').strip()
                service_name = (field.source.service or '').strip() if field.source.service else None
                if process_name:
                    running = _process_is_running(process_name, service=service_name)
                    text = 'running' if running else 'missing'
                else:
                    text = 'unknown'
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
    server.add_insecure_port(bind)
    await server.start()
    logger.info(f"ros_ui_bridge gRPC listening on {bind}")

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

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, stop_event.set)
        except NotImplementedError:
            pass

    session = resolve_session()
    fields_meta = [
        StatusFieldMeta(id=f.id, unit=f.unit, value_type=f.value_type, min=f.min, max=f.max, target=f.target)
        for f in cfg.status_stream.fields
    ]
    system_fields = [f for f in cfg.status_stream.fields if f.source.provider == 'system']
    status_broadcaster = StatusBroadcaster(
        fields=fields_meta,
        current_launch_ref=session.current_launch_ref,
        stack=session.stack,
        fixed_frame=session.fixed_frame,
    )

    model_provider = RobotModelProvider(glb_path=cfg.robot_model.glb_path)

    service = UiBridgeService(
        status_broadcaster=status_broadcaster,
        robot_state_broadcaster=robot_state_broadcaster,
        lidar_broadcaster=lidar_broadcaster,
        map_broadcaster=map_broadcaster,
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

    executor = SingleThreadedExecutor()
    executor.add_node(ros_node)
    executor.add_node(state_node)
    if lidar_node is not None:
        executor.add_node(lidar_node)
    if map_node is not None:
        executor.add_node(map_node)

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

        try:
            rclpy.shutdown()
        except Exception:
            pass

        try:
            spin_thread.join(timeout=2.0)
        except Exception:
            pass


def _systemd_is_active(service_name: str) -> str:
    unit = str(service_name).strip()
    if not unit:
        return 'unknown'
    if not unit.endswith('.service'):
        unit = unit + '.service'

    try:
        cp = subprocess.run(
            ['systemctl', 'is-active', unit],
            check=False,
            capture_output=True,
            text=True,
            timeout=1.5,
        )
    except Exception:  # noqa: BLE001
        return 'unknown'

    out = (cp.stdout or '').strip()
    if out:
        return out
    err = (cp.stderr or '').strip()
    return err or 'unknown'


def _process_is_running(process_name: str, *, service: str | None) -> bool:
    needle = str(process_name).strip()
    if not needle:
        return False

    if service:
        unit = str(service).strip()
        if not unit.endswith('.service'):
            unit = unit + '.service'
        try:
            cp = subprocess.run(
                ['systemctl', 'show', unit, '-p', 'ControlGroup', '--value'],
                check=False,
                capture_output=True,
                text=True,
                timeout=1.5,
            )
        except Exception:  # noqa: BLE001
            return False

        cgroup = (cp.stdout or '').strip()
        if not cgroup:
            return False

        procs_path = f"/sys/fs/cgroup{cgroup}/cgroup.procs"
        try:
            with open(procs_path, 'r', encoding='utf-8') as f:
                pids_text = f.read()
        except OSError:
            return False

        for line in pids_text.splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                pid = int(line)
            except ValueError:
                continue
            try:
                proc = psutil.Process(pid)
                cmdline = ' '.join(proc.cmdline())
            except Exception:  # noqa: BLE001
                continue
            if needle in cmdline:
                return True
        return False

    for proc in psutil.process_iter(['cmdline']):
        try:
            cmdline_list = proc.info.get('cmdline') or []
            cmdline = ' '.join(cmdline_list)
        except Exception:  # noqa: BLE001
            continue
        if needle in cmdline:
            return True
    return False


if __name__ == '__main__':
    main()
