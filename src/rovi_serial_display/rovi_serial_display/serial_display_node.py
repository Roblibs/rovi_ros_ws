#!/usr/bin/env python3
from __future__ import annotations

import argparse
import asyncio
import json
import logging
import signal
from typing import Optional

import grpc
from serial import Serial, SerialException

from rovi_ui_gateway.api import ui_gateway_pb2, ui_gateway_pb2_grpc

from .config import SerialDisplayConfig, load_config


class _SerialWriter:
    def __init__(self, *, port: str, baudrate: int, logger: logging.Logger) -> None:
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
            self._logger.info("Opened display serial on %s @ %s baud", self._port, self._baudrate)
            self._last_open_error = None
        except SerialException as exc:
            error = str(exc)
            if error != self._last_open_error:
                self._logger.warning("Failed to open display serial %s: %s", self._port, exc)
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


def _build_payload(update: ui_gateway_pb2.StatusUpdate, cfg: SerialDisplayConfig) -> list[dict]:
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


async def _run(cfg: SerialDisplayConfig, *, stop_event: asyncio.Event, logger: logging.Logger) -> None:
    serial_writer = _SerialWriter(port=cfg.serial_port, baudrate=cfg.baudrate, logger=logger)
    request = ui_gateway_pb2.StatusRequest()

    while not stop_event.is_set():
        channel: Optional[grpc.aio.Channel] = None
        try:
            channel = grpc.aio.insecure_channel(cfg.gateway_address)
            stub = ui_gateway_pb2_grpc.UiGatewayStub(channel)

            async for update in stub.GetStatus(request):
                if stop_event.is_set():
                    break
                payload = _build_payload(update, cfg)
                if not payload:
                    continue
                try:
                    serial_writer.write_json_line(payload)
                except SerialException as exc:
                    logger.warning("Serial write failed: %s", exc)

        except grpc.aio.AioRpcError as exc:
            logger.warning("gRPC stream error: %s", exc)
        except Exception:  # noqa: BLE001
            logger.exception("Unexpected error in serial display loop")
        finally:
            if channel is not None:
                await channel.close()

        if not stop_event.is_set():
            await asyncio.sleep(cfg.reconnect_delay_s)


def main(argv: list[str] | None = None) -> None:
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger('rovi_serial_display')

    parser = argparse.ArgumentParser(description='Rovi serial display (gRPC client).')
    parser.add_argument('--config', default=None, help='Path to YAML config (defaults to package config/default.yaml).')
    args, _unknown = parser.parse_known_args(argv)

    cfg = load_config(args.config)

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


if __name__ == '__main__':
    main()
