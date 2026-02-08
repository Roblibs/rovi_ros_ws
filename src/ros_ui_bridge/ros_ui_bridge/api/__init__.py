from __future__ import annotations

# grpcio-tools generates `*_pb2_grpc.py` files that use absolute imports like:
#   import ui_bridge_pb2 as ui__bridge__pb2
# When these modules are installed under `ros_ui_bridge.api`, that absolute import
# fails unless we alias it into `sys.modules`.
#
# This keeps runtime stable without having to patch generated files.
import sys as _sys

from . import ui_bridge_pb2 as _ui_bridge_pb2

_sys.modules.setdefault("ui_bridge_pb2", _ui_bridge_pb2)
