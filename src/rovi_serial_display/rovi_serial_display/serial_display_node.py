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
        except SerialException as exc:
            self._logger.warning("Failed to open display serial %s: %s", self._port, exc)
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
    payload: list[dict] = [
        {
            'id': cfg.cpu_id,
            'value': cpu_int,
            'text': f"{cpu_int}%",
        }
    ]

    if update.WhichOneof('voltage') == 'voltage_v':
        payload.append(
            {
                'id': cfg.voltage_id,
                'value': float(update.voltage_v),
                'text': f"{float(update.voltage_v):.1f}V",
            }
        )

    for metric in update.rates:
        measured = int(round(metric.hz))
        target = None
        if metric.WhichOneof('target') == 'target_hz':
            target = int(round(metric.target_hz))

        payload.append(
            {
                'id': metric.id,
                'value': measured,
                'text': f"{measured}/{target}Hz" if target is not None else f"{measured}Hz",
            }
        )

    return payload


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
