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

from .api import ui_gateway_pb2_grpc
from .config import UiGatewayConfig, load_config
from .grpc_gateway import UiGatewayService
from .ros_voltage_subscriber import VoltageSubscriber
from .status_store import SnapshotBroadcaster
from .voltage_state import VoltageState


async def _publish_loop(
    *,
    cfg: UiGatewayConfig,
    voltage_state: VoltageState,
    broadcaster: SnapshotBroadcaster,
    stop_event: asyncio.Event,
) -> None:
    while not stop_event.is_set():
        cpu_percent = float(psutil.cpu_percent(interval=None))
        voltage_v, _last_update = voltage_state.read()
        await broadcaster.publish(cpu_percent=cpu_percent, voltage_v=voltage_v)
        await asyncio.sleep(cfg.update_period_s)


async def _serve_grpc(
    *,
    bind: str,
    broadcaster: SnapshotBroadcaster,
    stop_event: asyncio.Event,
    logger: logging.Logger,
) -> None:
    server = grpc.aio.server()
    ui_gateway_pb2_grpc.add_UiGatewayServicer_to_server(UiGatewayService(broadcaster), server)
    server.add_insecure_port(bind)
    await server.start()
    logger.info("rovi_ui_gateway gRPC listening on %s", bind)

    try:
        await stop_event.wait()
    finally:
        await server.stop(grace=1.0)


async def _run_async(*, cfg: UiGatewayConfig, voltage_state: VoltageState, logger: logging.Logger) -> None:
    stop_event = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, stop_event.set)
        except NotImplementedError:
            pass

    broadcaster = SnapshotBroadcaster()

    publisher_task = asyncio.create_task(
        _publish_loop(cfg=cfg, voltage_state=voltage_state, broadcaster=broadcaster, stop_event=stop_event)
    )
    grpc_task = asyncio.create_task(_serve_grpc(bind=cfg.grpc_bind, broadcaster=broadcaster, stop_event=stop_event, logger=logger))

    try:
        await asyncio.gather(publisher_task, grpc_task)
    finally:
        for task in (publisher_task, grpc_task):
            task.cancel()


def main(argv: Optional[list[str]] = None) -> None:
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger('rovi_ui_gateway')

    parser = argparse.ArgumentParser(description='Rovi UI gRPC gateway.')
    parser.add_argument('--config', default=None, help='Path to YAML config (defaults to package config/default.yaml).')
    args, ros_args = parser.parse_known_args(argv)

    cfg = load_config(args.config)

    rclpy.init(args=ros_args)
    voltage_state = VoltageState()
    voltage_node = VoltageSubscriber(voltage_state=voltage_state, topic=cfg.voltage_topic)
    executor = SingleThreadedExecutor()
    executor.add_node(voltage_node)

    spin_thread = threading.Thread(target=executor.spin, name='rovi_ui_gateway_ros_spin', daemon=True)
    spin_thread.start()

    # Prime psutil so the first value is meaningful.
    psutil.cpu_percent(None)

    try:
        asyncio.run(_run_async(cfg=cfg, voltage_state=voltage_state, logger=logger))
    finally:
        executor.shutdown()
        voltage_node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()

