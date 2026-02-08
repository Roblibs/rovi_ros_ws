# calibration
- calibration file has no specific camera id, serial usage to be clarified.

# USB enumeration
- Keep RGB pinned to stable `/dev/v4l/by-id/...`.
- Pin depth to stable device identifier where supported; otherwise document bus-path risk and validate runtime selection.

# controls & config
- on sttartup control 10092545 is failing with an error
- list all needed controls and evaluate their usage
- check if any configuration needed for performance optimization
