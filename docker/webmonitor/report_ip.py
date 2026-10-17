#!/usr/bin/env python3
"""Publish this vehicle's routed IP address to the CARLab fleet monitor."""

from __future__ import annotations

import ipaddress
import json
import logging
import os
import socket
import ssl
import subprocess
import sys
import time
import urllib.error
import urllib.request
import urllib.parse


LOG = logging.getLogger("carkit-webmonitor")


def current_ip(probe_host: str = "1.1.1.1", probe_port: int = 443) -> str:
    """Return the source address selected by the default route without sending data."""
    interface = os.environ.get("CARKIT_REPORT_INTERFACE", "").strip()
    if interface:
        result = subprocess.run(
            ["ip", "-j", "-4", "address", "show", "dev", interface],
            check=True, capture_output=True, text=True, timeout=5,
        )
        for device in json.loads(result.stdout):
            if not {"UP", "LOWER_UP"}.issubset(device.get("flags", [])):
                continue
            for address in device.get("addr_info", []):
                if address.get("scope") != "global":
                    continue
                parsed = ipaddress.ip_address(address["local"])
                if not (parsed.is_unspecified or parsed.is_loopback or parsed.is_link_local):
                    return str(parsed)
        raise RuntimeError(f"report interface {interface} has no usable IPv4 address")
    family = socket.AF_INET6 if ":" in probe_host else socket.AF_INET
    with socket.socket(family, socket.SOCK_DGRAM) as connection:
        connection.connect((probe_host, probe_port))
        address = connection.getsockname()[0]
    parsed = ipaddress.ip_address(address)
    if parsed.is_unspecified or parsed.is_loopback or parsed.is_link_local:
        raise RuntimeError(f"default route selected an unusable address: {address}")
    return str(parsed)


def send_check_in(
    endpoint: str,
    token: str,
    vehicle_id: str,
    webui_port: int,
    address: str,
) -> None:
    body = json.dumps(
        {
            "vehicle_id": vehicle_id,
            "ip_address": address,
            "webui_port": webui_port,
        }
    ).encode("utf-8")
    request = urllib.request.Request(
        endpoint,
        data=body,
        method="POST",
        headers={
            "Authorization": f"Bearer {token}",
            "Content-Type": "application/json",
            "User-Agent": "CARKit-WebMonitor/1.0",
        },
    )
    context = ssl.create_default_context()
    with urllib.request.urlopen(request, timeout=20, context=context) as response:
        if response.status < 200 or response.status >= 300:
            raise RuntimeError(f"monitor returned HTTP {response.status}")


def required_env(name: str) -> str:
    value = os.environ.get(name, "").strip()
    if not value:
        raise RuntimeError(f"{name} is required")
    return value


def wait_for_boot_network(endpoint: str, timeout: float = 120) -> None:
    """Allow boot-time routing, DNS and the clock to settle before HTTP attempts.

    This bounded readiness wait does not consume the configured check-in attempts.
    TLS certificate verification remains enabled in send_check_in.
    """
    deadline = time.monotonic() + timeout
    hostname = urllib.parse.urlsplit(endpoint).hostname
    while True:
        try:
            current_ip()
            if time.time() < 1735689600:  # 2025-01-01; reject the Jetson epoch clock.
                raise RuntimeError("waiting for the system clock")
            socket.getaddrinfo(hostname, 443, type=socket.SOCK_STREAM)
            return
        except (OSError, RuntimeError, subprocess.SubprocessError, ValueError) as error:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                LOG.warning("boot readiness wait expired; using normal check-in retries: %s", error)
                return
            LOG.info("waiting for boot network and clock: %s", error)
            time.sleep(min(5, remaining))


def startup_ready(webui_port: int) -> bool:
    """Do not advertise a vehicle until its container, HTTP server and clock are ready."""
    try:
        synchronized = subprocess.run(
            ["timedatectl", "show", "--property=NTPSynchronized", "--value"],
            check=True, capture_output=True, text=True, timeout=5,
        ).stdout.strip()
        if synchronized != "yes":
            return False
        running = subprocess.run(
            ["docker", "inspect", "--format", "{{.State.Running}}", "carkit"],
            check=True, capture_output=True, text=True, timeout=5,
        ).stdout.strip()
        if running != "true":
            return False
        # The landing page is public even when the API requires an access key.
        with urllib.request.urlopen(f"http://127.0.0.1:{webui_port}/", timeout=5) as response:
            return response.status == 200
    except (OSError, subprocess.SubprocessError, urllib.error.URLError):
        return False


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    try:
        endpoint = required_env("CARKIT_MONITOR_ENDPOINT")
        token = required_env("CARKIT_REPORTER_TOKEN")
        vehicle_id = required_env("CARKIT_VEHICLE_ID").upper()
        port = int(os.environ.get("CARKIT_WEBUI_PORT", "8080"))
        success_interval = max(
            60, int(os.environ.get("CARKIT_REPORT_INTERVAL", "60"))
        )
        retry_interval = max(300, int(os.environ.get("CARKIT_REPORT_RETRY", "300")))
        max_tries = min(
            20, max(1, int(os.environ.get("CARKIT_REPORT_MAX_TRIES", "20")))
        )
    except (RuntimeError, ValueError) as error:
        LOG.error("invalid configuration: %s", error)
        return 2

    if not endpoint.startswith("https://"):
        LOG.error("CARKIT_MONITOR_ENDPOINT must use HTTPS")
        return 2
    if (
        not vehicle_id.startswith("ADA")
        or not vehicle_id[3:].isdigit()
        or int(vehicle_id[3:]) < 1
    ):
        LOG.error("CARKIT_VEHICLE_ID must look like ADA5")
        return 2
    if not 1 <= port <= 65535:
        LOG.error("CARKIT_WEBUI_PORT must be between 1 and 65535")
        return 2

    wait_for_boot_network(endpoint)
    consecutive_failures = 0
    waiting = False
    while True:
        if not startup_ready(port):
            if not waiting:
                LOG.info("waiting for CARKit container, WebUI, and synchronized host clock")
            waiting = True
            time.sleep(5)
            continue
        waiting = False
        try:
            address = current_ip()
            send_check_in(endpoint, token, vehicle_id, port, address)
            consecutive_failures = 0
            LOG.info("reported %s at %s", vehicle_id, address)
            time.sleep(success_interval)
        except (OSError, RuntimeError, subprocess.SubprocessError, ValueError, urllib.error.URLError) as error:
            consecutive_failures += 1
            if consecutive_failures >= max_tries:
                LOG.warning(
                    "check-in attempt %s/%s failed; stopping until the service "
                    "or vehicle restarts: %s",
                    consecutive_failures,
                    max_tries,
                    error,
                )
                return 0
            LOG.warning(
                "check-in attempt %s/%s failed; retrying in %ss: %s",
                consecutive_failures,
                max_tries,
                retry_interval,
                error,
            )
            time.sleep(retry_interval)


if __name__ == "__main__":
    sys.exit(main())
