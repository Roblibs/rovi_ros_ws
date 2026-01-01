#!/usr/bin/env python3
from __future__ import annotations

import argparse
import asyncio
import json
import signal
import traceback
from typing import Optional

import grpc
import rclpy
from rclpy.logging import get_logger
from serial import Serial, SerialException

from ros_ui_bridge.api import ui_bridge_pb2, ui_bridge_pb2_grpc

from .config import SerialDisplayConfig, load_config


class _SerialWriter:
    def __init__(self, *, port: str, baudrate: int, logger) -> None:
        self._port = port
        self._baudrate = baudrate
        self._logger = logger
        self._serial: Optional[Serial] = None
        self._last_open_error: Optional[str] = None

    def close(self) -> None:
        if self._serial is None:
            return
        try:
            self._serial.close()
        finally:
            self._serial = None

    def ensure_open(self) -> Optional[Serial]:
        if self._serial and self._serial.is_open:
            return self._serial

        try:
            self._serial = Serial(self._port, baudrate=self._baudrate, timeout=1)
            self._logger.info(f"Opened display serial on {self._port} @ {self._baudrate} baud")
            self._last_open_error = None
        except SerialException as exc:
            error = str(exc)
            if error != self._last_open_error:
                self._logger.warning(f"Failed to open display serial {self._port}: {exc}")
                self._last_open_error = error
            self._serial = None
        return self._serial

    def write_json_line(self, payload: list[dict]) -> None:
        ser = self.ensure_open()
        if ser is None:
            return

        line = json.dumps(payload, separators=(',', ':')) + '\n'
        try:
            ser.write(line.encode('utf-8'))
        except SerialException:
            self.close()
            raise


def _build_payload(update: ui_bridge_pb2.StatusUpdate, cfg: SerialDisplayConfig) -> list[dict]:
    cpu_int = int(round(update.cpu_percent))
    selected_ids = _dedupe_preserve_order(cfg.selected_ids)
    rate_by_id = {metric.id: metric for metric in update.rates}

    payload: list[dict] = []
    for metric_id in selected_ids:
        if metric_id == 'cpu':
            payload.append(
                {
                    'id': 'cpu',
                    'value': cpu_int,
                    'text': f"{cpu_int}%",
                }
            )
            continue

        if metric_id == 'voltage':
            if update.WhichOneof('voltage') == 'voltage_v':
                voltage_v = float(update.voltage_v)
                payload.append(
                    {
                        'id': 'voltage',
                        'value': voltage_v,
                        'text': f"{voltage_v:.1f}V",
                    }
                )
            continue

        rate_metric = rate_by_id.get(metric_id)
        if rate_metric is None:
            continue

        measured = int(round(rate_metric.hz))
        target = None
        if rate_metric.WhichOneof('target') == 'target_hz':
            target = int(round(rate_metric.target_hz))

        payload.append(
            {
                'id': metric_id,
                'value': measured,
                'text': f"{measured}/{target}Hz" if target is not None else f"{measured}Hz",
            }
        )

    return payload


def _dedupe_preserve_order(items: list[str]) -> list[str]:
    seen: set[str] = set()
    out: list[str] = []
    for item in items:
        if item in seen:
            continue
        seen.add(item)
        out.append(item)
    return out


async def _run(cfg: SerialDisplayConfig, *, stop_event: asyncio.Event, logger) -> None:
    serial_writer = _SerialWriter(port=cfg.serial_port, baudrate=cfg.baudrate, logger=logger)
    request = ui_bridge_pb2.StatusRequest()
    grpc_connected = False
    last_grpc_log_key: Optional[tuple[grpc.StatusCode, str]] = None

    while not stop_event.is_set():
        channel: Optional[grpc.aio.Channel] = None
        try:
            channel = grpc.aio.insecure_channel(cfg.gateway_address)
            stub = ui_bridge_pb2_grpc.UiBridgeStub(channel)

            async for update in stub.GetStatus(request):
                if stop_event.is_set():
                    break
                if not grpc_connected:
                    logger.info(f"Connected to UI gateway at {cfg.gateway_address}")
                    grpc_connected = True
                    last_grpc_log_key = None
                payload = _build_payload(update, cfg)
                if not payload:
                    continue
                try:
                    serial_writer.write_json_line(payload)
                except SerialException as exc:
                    logger.warning(f"Serial write failed: {exc}")

        except grpc.aio.AioRpcError as exc:
            code = exc.code()
            details = exc.details() or ""
            log_key = (code, details)

            if code == grpc.StatusCode.UNAVAILABLE and not grpc_connected:
                if log_key != last_grpc_log_key:
                    logger.info(
                        f"Waiting for UI gateway at {cfg.gateway_address} ({details}); "
                        f"retrying in {cfg.reconnect_delay_s:.1f}s"
                    )
                    last_grpc_log_key = log_key
            else:
                if grpc_connected:
                    grpc_connected = False
                if log_key != last_grpc_log_key:
                    code_name = getattr(code, 'name', str(code))
                    logger.warning(
                        f"gRPC stream error ({code_name}): {details}; retrying in {cfg.reconnect_delay_s:.1f}s"
                    )
                    last_grpc_log_key = log_key
        except Exception:  # noqa: BLE001
            if grpc_connected:
                grpc_connected = False
            last_grpc_log_key = None
            logger.error(
                "Unexpected error in serial display loop:\n" + traceback.format_exc()
            )
        finally:
            if channel is not None:
                await channel.close()

        if not stop_event.is_set():
            await asyncio.sleep(cfg.reconnect_delay_s)


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description='Robot serial display (gRPC client).')
    parser.add_argument('--config', default=None, help='Path to YAML config (defaults to package config/default.yaml).')
    args, ros_args = parser.parse_known_args(argv)

    cfg = load_config(args.config)

    rclpy.init(args=ros_args)
    logger = get_logger('robot_serial_display')

    stop_event = asyncio.Event()
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, stop_event.set)
        except NotImplementedError:
            pass

    try:
        loop.run_until_complete(_run(cfg, stop_event=stop_event, logger=logger))
    finally:
        loop.close()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
