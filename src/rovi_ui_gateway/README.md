# rovi_ui_gateway

Low-rate UI gateway for Rovi: collects status data (CPU, voltage, …) and serves it to UI clients over gRPC.

## gRPC API

`GetStatus` is a request/stream RPC:

- Client sends: `StatusRequest{}`
- Server responds: stream of periodic `StatusUpdate` snapshots
  - `cpu_percent` is always set
  - `voltage_v` is only set after the gateway has received at least one voltage message from ROS (no “fake” voltage)
  - `rates[]` contains any configured topic/TF rates that have been observed

Proto: `proto/ui_gateway.proto`

## Metrics configuration

The gateway can also publish periodic rates (Hz) for arbitrary ROS topics and for specific TF transforms.

Example:

```yaml
metrics:
  rates:
    - id: "hz_driver"
      topic: "/voltage"
      target_hz: 10
  tf_rates:
    - id: "hz_slam"
      parent: "map"
      child: "odom"
      target_hz: 50
```

Rate metrics are only emitted once at least one message/transform has been received (to avoid “fake” values).

## Run

This node is started by `rovi_bringup/robot_bringup.launch.py` by default.

To run manually:

```bash
ros2 run rovi_ui_gateway ui_gateway -- --config /path/to/ui_gateway.yaml
```

Default config is installed at:

- `$(ros2 pkg prefix rovi_ui_gateway)/share/rovi_ui_gateway/config/default.yaml`

## Python client example

```python
import asyncio
import grpc

from rovi_ui_gateway.api import ui_gateway_pb2, ui_gateway_pb2_grpc


async def main() -> None:
    async with grpc.aio.insecure_channel("127.0.0.1:50051") as channel:
        stub = ui_gateway_pb2_grpc.UiGatewayStub(channel)
        async for update in stub.GetStatus(ui_gateway_pb2.StatusRequest()):
            print(f"cpu={update.cpu_percent:.0f}% seq={update.seq}")
            if update.WhichOneof("voltage") == "voltage_v":
                print(f"voltage={update.voltage_v:.1f}V")
            for metric in update.rates:
                name = metric.id
                hz = metric.hz
                if metric.WhichOneof("target") == "target_hz":
                    print(f"{name}={hz:.1f}/{metric.target_hz:.0f}Hz")
                else:
                    print(f"{name}={hz:.1f}Hz")


if __name__ == "__main__":
    asyncio.run(main())
```
