from __future__ import annotations

from rclpy.node import Node
from std_msgs.msg import Float32

from .voltage_state import VoltageState


class VoltageSubscriber(Node):
    def __init__(self, *, voltage_state: VoltageState, topic: str) -> None:
        super().__init__('rovi_ui_gateway_voltage')
        self._voltage_state = voltage_state
        self._subscription = self.create_subscription(Float32, topic, self._on_voltage, 10)

    def _on_voltage(self, msg: Float32) -> None:
        self._voltage_state.update(float(msg.data))
