from __future__ import annotations

# grpcio-tools generates `*_pb2_grpc.py` files that use absolute imports like:
#   import ui_bridge_pb2 as ui__bridge__pb2
# When these modules are installed under `ros_ui_bridge.api`, that absolute import
# fails unless we alias it into `sys.modules`.
#
# This keeps runtime stable without having to patch generated files.
import sys as _sys

try:
    from . import ui_bridge_pb2 as _ui_bridge_pb2
except Exception as exc:  # pragma: no cover
    raise ImportError(
        "ros_ui_bridge protobuf stubs are not present.\n"
        "They are generated at build time; ensure you have built the workspace.\n"
        "If building from source, install: python3-grpc-tools\n"
        "Then run: clean && build"
    ) from exc

_sys.modules.setdefault("ui_bridge_pb2", _ui_bridge_pb2)
