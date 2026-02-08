from __future__ import annotations

import socket

import psutil


def iface_status_text(iface: str) -> str:
    name = str(iface).strip()
    if not name:
        return "unknown"

    stats = psutil.net_if_stats().get(name)
    if stats is None:
        return "missing"
    if not stats.isup:
        return "down"

    addrs = psutil.net_if_addrs().get(name, [])

    ipv4 = None
    for a in addrs:
        if a.family == socket.AF_INET:
            addr = (a.address or "").strip()
            if not addr:
                continue
            # Skip self-assigned IPv4.
            if addr.startswith("169.254."):
                continue
            ipv4 = addr
            break
    if ipv4:
        return f"ip:{ipv4}"

    ipv6 = None
    for a in addrs:
        if a.family == socket.AF_INET6:
            addr = (a.address or "").strip()
            if not addr:
                continue
            # Strip scope if present: "fe80::1%wlan0" -> "fe80::1"
            ipv6 = addr.split("%", 1)[0]
            break
    if ipv6:
        return f"ip6:{ipv6}"

    return "up:no_ip"

