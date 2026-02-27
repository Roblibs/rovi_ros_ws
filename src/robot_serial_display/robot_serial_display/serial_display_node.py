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
            # Important: set a write timeout so a stalled USB ACM device cannot block this process forever.
            # If the display resets or stops reading, writes can otherwise hang and the UI appears "dead"
            # until the gateway/display process is restarted.
            self._serial = Serial(self._port, baudrate=self._baudrate, timeout=1, write_timeout=1)
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

        line = _encode_payload(payload)
        try:
            ser.write(line.encode('utf-8'))
        except SerialException:
            self.close()
            raise


def _format_value(val: float, unit: str) -> str:
    if unit == '%':
        return f"{val:.0f}{unit}"
    if unit.lower() in ('hz',):
        return f"{val:.0f}{unit}"
    if unit.lower() in ('v', 'volt', 'voltage'):
        return f"{val:.1f}{unit}"
    if unit:
        return f"{val:.2f}{unit}"
    return f"{val:.2f}"


def _encode_payload(payload: list[dict]) -> str:
    return json.dumps(payload, separators=(',', ':')) + '\n'


def _build_payload(
    *,
    selected_ids: list[str],
    selected_scales: dict[str, float],
    meta_by_id: dict[str, ui_bridge_pb2.StatusFieldMeta],
    values_by_id: dict[str, float],
) -> list[dict]:
    payload: list[dict] = []
    for field_id in _dedupe_preserve_order(selected_ids):
        meta = meta_by_id.get(field_id)
        val = values_by_id.get(field_id)
        if meta is None or val is None:
            continue

        encoded_value: float | int
        display_val = val
        scale = selected_scales.get(field_id, 1.0)
        scaled_val = val * scale
        encoded_value = int(round(scaled_val))

        text = _format_value(display_val, meta.unit)
        if meta.HasField('target'):
            target_text = _format_value(meta.target, meta.unit)
            text = f"{text}/{target_text}"

        payload.append(
            {
                'id': field_id,
                'value': encoded_value,
                'text': text,
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


async def _run(cfg: SerialDisplayConfig, *, stop_event: asyncio.Event, logger, debug: bool) -> None:
    serial_writer = _SerialWriter(port=cfg.serial_port, baudrate=cfg.baudrate, logger=logger)
    request = ui_bridge_pb2.StatusRequest()
    grpc_connected = False
    last_grpc_log_key: Optional[tuple[grpc.StatusCode, str]] = None
    meta_by_id: dict[str, ui_bridge_pb2.StatusFieldMeta] = {}
    values_by_id: dict[str, float] = {}

    while not stop_event.is_set():
        channel: Optional[grpc.aio.Channel] = None
        try:
            channel = grpc.aio.insecure_channel(cfg.gateway_address)
            stub = ui_bridge_pb2_grpc.UiBridgeStub(channel)

            # Fetch metadata + latest values first (single response).
            snapshot = await stub.GetStatus(request)
            meta_by_id = {f.id: f for f in snapshot.fields}
            values_by_id = {v.id: float(v.value) for v in snapshot.values}
            if not grpc_connected:
                logger.info(f"Connected to UI gateway at {cfg.gateway_address}")
                grpc_connected = True
                last_grpc_log_key = None
            initial_payload = _build_payload(
                selected_ids=cfg.selected_ids,
                selected_scales=cfg.selected_scales,
                meta_by_id=meta_by_id,
                values_by_id=values_by_id,
            )
            if debug:
                logger.info(f"Initial status payload: {initial_payload or '(empty)'}")
            if initial_payload:
                if debug:
                    logger.info(f"Serial write line: {_encode_payload(initial_payload).strip()}")
                try:
                    serial_writer.write_json_line(initial_payload)
                except SerialException as exc:
                    logger.warning(f"Serial write failed: {exc}")

            async for update in stub.StreamStatus(request):
                if stop_event.is_set():
                    break

                # Update values map with the non-stale values received; missing ids are considered stale.
                values_by_id = {v.id: float(v.value) for v in update.values}

                payload = _build_payload(
                    selected_ids=cfg.selected_ids,
                    selected_scales=cfg.selected_scales,
                    meta_by_id=meta_by_id,
                    values_by_id=values_by_id,
                )
                if debug:
                    received_ids = sorted(values_by_id.keys())
                    logger.info(
                        f"Status update received (ids={received_ids}): {payload or '(empty)'}"
                    )
                if not payload:
                    continue
                if debug:
                    logger.info(f"Serial write line: {_encode_payload(payload).strip()}")
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
    parser.add_argument('--debug', action='store_true', help='Log each status update and payload.')
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
        loop.run_until_complete(_run(cfg, stop_event=stop_event, logger=logger, debug=args.debug))
    finally:
        loop.close()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
