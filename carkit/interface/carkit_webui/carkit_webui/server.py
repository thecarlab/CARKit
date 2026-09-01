#!/usr/bin/env python3

# CARKit learning annotation: implements the behavior described by this file's package and module.
"""Dependency-free HTTP API and static server for the CARKit dashboard."""

import argparse
import ast
from collections import deque
import difflib
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import threading
import time
from urllib.parse import parse_qs, urlparse


PROFILES = {"reference", "ada_high_school", "intro2av"}
CHASSIS = {"osracer", "f1tenth"}
IMPLEMENTATIONS = {
    "reference", "ada_academy", "intro2av_python", "intro2av_cpp", "off"
}
PROFILE_IMPLEMENTATIONS = {
    "ada_high_school": {
        "planning": "ada_academy",
        "control": "ada_academy",
        "perception": "ada_academy",
    },
    "intro2av": {
        "planning": "intro2av_python",
        "control": "intro2av_python",
        "perception": "intro2av_python",
    },
    "reference": {
        "planning": "reference",
        "control": "reference",
        "perception": "reference",
    },
}
PERCEPTION_MODELS = {"generic_coco", "traffic_signs", "combined", "custom"}
COMPONENTS = {"chassis", "sensors", "planning", "control", "perception", "behavior"}
BUILD_TARGETS = {
    "perception": ["carkit_perception_msgs"],
    "localization": ["carkit_amcl", "carkit_slam"],
    "control": [
        "carkit_control_center",
        "carkit_human_control",
    ],
    "planning": ["carkit_behavior"],
}
IMPLEMENTATION_BUILD_TARGETS = {
    "perception": {
        "reference": ["carkit_perception"],
        # ADA filtering requires the protected reference detector at runtime.
        "ada_academy": ["carkit_perception", "carkit_ada_academy"],
        "intro2av_python": ["carkit_intro2av"],
        "intro2av_cpp": ["carkit_intro2av_cpp"],
        "off": [],
    },
    "control": {
        # Reference control uses the cmd_vel-to-Ackermann bridge in carkit_amcl.
        "reference": ["carkit_amcl"],
        "ada_academy": ["carkit_ada_academy"],
        "intro2av_python": ["carkit_intro2av"],
        "intro2av_cpp": ["carkit_intro2av_cpp"],
        "off": [],
    },
    "planning": {
        "reference": ["carkit_navigation", "carkit_amcl"],
        "ada_academy": ["carkit_ada_academy"],
        "intro2av_python": ["carkit_intro2av"],
        "intro2av_cpp": ["carkit_intro2av_cpp"],
        "off": [],
    },
}
SAFE_PATH = re.compile(r"^[A-Za-z0-9_./-]+$")
EDITOR_FILES = {
    "ada_high_school": {
        component: Path(
            "carkit/education/carkit_ada_academy/"
            f"carkit_ada_academy/{component}.py"
        )
        for component in ("perception", "planning", "control")
    },
    "intro2av_python": {
        component: Path(
            "carkit/education/carkit_intro2av/"
            f"carkit_intro2av/{component}_algorithm.py"
        )
        for component in ("perception", "planning", "control")
    },
    "intro2av_cpp": {
        component: Path(
            "carkit/education/carkit_intro2av_cpp/"
            f"src/{component}_algorithm.cpp"
        )
        for component in ("perception", "planning", "control")
    },
}
EDITOR_ROOTS = {
    "ada_high_school": Path("carkit/education/carkit_ada_academy"),
    "intro2av_python": Path("carkit/education/carkit_intro2av"),
    "intro2av_cpp": Path("carkit/education/carkit_intro2av_cpp"),
}
EDITOR_EXTENSIONS = {
    ".cfg", ".cmake", ".cpp", ".h", ".hpp", ".json", ".md",
    ".py", ".txt", ".xml", ".yaml", ".yml",
}
EDITOR_FILENAMES = {"CMakeLists.txt"}


def resolve_build_packages(target, implementations=None):
    """Return the minimal workspace package set for one Compile-tab target."""
    if target not in BUILD_TARGETS:
        raise ValueError("invalid compile target")
    packages = list(BUILD_TARGETS[target])
    if target in IMPLEMENTATION_BUILD_TARGETS:
        implementation = (implementations or {}).get(target, "reference")
        if implementation not in IMPLEMENTATIONS:
            raise ValueError(f"invalid {target} implementation")
        packages.extend(
            IMPLEMENTATION_BUILD_TARGETS[target][implementation]
        )
    # Preserve the intentional dependency order while removing duplicates.
    return list(dict.fromkeys(packages))


def estimate_lipo_percentage(voltage):
    """Estimate charge for an unknown 2S-6S LiPo from pack voltage."""
    voltage = float(voltage)
    if not math.isfinite(voltage) or voltage <= 0:
        return None
    cells = max(2, min(6, math.ceil(voltage / 4.25)))
    empty_voltage = 3.6 * cells
    full_voltage = 4.2 * cells
    return max(
        0.0,
        min(1.0, (voltage - empty_voltage) / (full_voltage - empty_voltage)),
    )


class ChassisTelemetryMonitor:
    """Keep the latest battery sample available to HTTP-only clients."""

    def __init__(self):
        self.lock = threading.Lock()
        self.latest = None
        self.executor = None
        self.node = None
        self.thread = None
        self.rclpy = None
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import BatteryState

            rclpy.init(args=None)
            self.rclpy = rclpy
            self.node = Node("carkit_webui_telemetry")
            self.node.create_subscription(
                BatteryState,
                "/battery_state",
                self._battery_callback,
                qos_profile_sensor_data,
            )
            try:
                from vesc_msgs.msg import VescStateStamped

                self.node.create_subscription(
                    VescStateStamped,
                    "/sensors/core",
                    self._vesc_callback,
                    qos_profile_sensor_data,
                )
            except ImportError:
                pass
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.node)
            self.thread = threading.Thread(
                target=self.executor.spin,
                name="carkit-telemetry",
                daemon=True,
            )
            self.thread.start()
        except (ImportError, RuntimeError) as error:
            print(f"CARKit telemetry monitor unavailable: {error}")
            self.close()

    def _store(self, voltage, percentage, source):
        voltage = float(voltage)
        percentage = (
            None if percentage is None else float(percentage)
        )
        if not math.isfinite(voltage) or voltage <= 0:
            return
        if percentage is not None:
            if not math.isfinite(percentage) or percentage < 0:
                percentage = None
            else:
                percentage = max(0.0, min(1.0, percentage))
        with self.lock:
            self.latest = {
                "voltage": voltage,
                "percentage": percentage,
                "estimated": True,
                "source": source,
                "received_monotonic": time.monotonic(),
            }

    def _battery_callback(self, message):
        self._store(message.voltage, message.percentage, "battery_state")

    def _vesc_callback(self, message):
        voltage = message.state.voltage_input
        self._store(
            voltage,
            estimate_lipo_percentage(voltage),
            "vesc",
        )

    def snapshot(self):
        with self.lock:
            if self.latest is None:
                return None
            value = dict(self.latest)
        received = value.pop("received_monotonic")
        value["age_seconds"] = round(max(0.0, time.monotonic() - received), 1)
        value["fresh"] = value["age_seconds"] <= 5.0
        return value

    def close(self):
        if self.executor is not None:
            self.executor.shutdown(timeout_sec=1.0)
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.node is not None:
            self.node.destroy_node()
        if self.rclpy is not None and self.rclpy.ok():
            self.rclpy.shutdown()
        self.executor = None
        self.thread = None
        self.node = None
        self.rclpy = None


class RevisionConflict(RuntimeError):
    """Raised when a student tries to overwrite a newer saved revision."""


class CollaborativeDocument:
    """Authoritative text, operation history, and presence for one source file."""

    def __init__(self, content):
        self.content = content
        self.version = 0
        self.snapshots = {0: content}
        self.history = []
        self.users = {}
        self.diagnostics = []


class ProcessManager:
    def __init__(self, workspace):
        self.workspace = Path(workspace).resolve()
        self.process = None
        self.current_config = None
        self.job = None
        self.job_kind = None
        self.job_return_code = None
        self.job_packages = []
        self.logs = deque(maxlen=500)
        self.log_cursor = 0
        self.repeated_log_times = {}
        self.lock = threading.Lock()
        self.editor_lock = threading.RLock()
        self.metrics_lock = threading.Lock()
        self.metrics_cache = None
        self.metrics_cache_time = 0.0
        self.cpu_sample = self._cpu_totals()
        workload_usage = self._workload_cpu_usage_seconds()
        self.workload_cpu_sample = (
            (workload_usage, time.monotonic())
            if workload_usage is not None else None
        )
        self.telemetry_monitor = None
        self.collab_documents = {}

    def map_files(self):
        """List launchable occupancy-map YAML files from the workspace map folder."""
        map_root = (self.workspace / "map").resolve()
        if not map_root.is_dir():
            return []
        files = []
        for candidate in sorted(
            map_root.glob("*.yaml"), key=lambda path: path.name.lower()
        ):
            try:
                resolved = candidate.resolve(strict=True)
                resolved.relative_to(map_root)
            except (OSError, ValueError):
                continue
            if resolved.is_file():
                files.append({"name": candidate.name, "path": str(resolved)})
        return files

    @staticmethod
    def _cpu_totals():
        try:
            fields = Path("/proc/stat").read_text(
                encoding="utf-8"
            ).splitlines()[0].split()[1:]
            values = [int(field) for field in fields]
            return sum(values), values[3] + (values[4] if len(values) > 4 else 0)
        except (OSError, ValueError, IndexError):
            return None

    @staticmethod
    def _workload_cpu_usage_seconds():
        """Return CPU time charged to the current Docker cgroup."""
        try:
            values = dict(
                line.split()
                for line in Path("/sys/fs/cgroup/cpu.stat").read_text(
                    encoding="utf-8"
                ).splitlines()
            )
            return int(values["usage_usec"]) / 1_000_000.0
        except (OSError, ValueError, KeyError):
            pass
        try:
            nanoseconds = int(Path(
                "/sys/fs/cgroup/cpuacct/cpuacct.usage"
            ).read_text(encoding="utf-8"))
            return nanoseconds / 1_000_000_000.0
        except (OSError, ValueError):
            return None

    @staticmethod
    def _memory_metrics():
        try:
            values = {}
            for line in Path("/proc/meminfo").read_text(
                encoding="utf-8"
            ).splitlines():
                name, value = line.split(":", 1)
                values[name] = int(value.split()[0])
            total = values["MemTotal"] * 1024
            available = values["MemAvailable"] * 1024
            used = total - available
            return {
                "used_bytes": used,
                "total_bytes": total,
                "percent": round(100.0 * used / total, 1),
            }
        except (OSError, ValueError, KeyError, ZeroDivisionError):
            return None

    @staticmethod
    def _cpu_temperature():
        readings = []
        for zone in Path("/sys/class/thermal").glob("thermal_zone*"):
            try:
                kind = (zone / "type").read_text(encoding="utf-8").strip()
                raw = float((zone / "temp").read_text(encoding="utf-8").strip())
            except (OSError, ValueError):
                continue
            temperature = raw / 1000.0 if raw > 1000.0 else raw
            if 0.0 < temperature < 150.0:
                readings.append((kind, temperature))
        for preferred in ("cpu-thermal", "tj-thermal"):
            match = next((value for kind, value in readings if kind == preferred), None)
            if match is not None:
                return round(match, 1)
        return round(max((value for _, value in readings), default=0.0), 1) or None

    def system_metrics(self):
        now = time.monotonic()
        with self.metrics_lock:
            if self.metrics_cache is not None and now - self.metrics_cache_time < 1.0:
                return self.metrics_cache
            cpu_count = os.cpu_count() or 1
            cpu_capacity_percent = 100.0 * cpu_count
            current_cpu = self._cpu_totals()
            cpu_percent = None
            if current_cpu is not None and self.cpu_sample is not None:
                total_delta = current_cpu[0] - self.cpu_sample[0]
                idle_delta = current_cpu[1] - self.cpu_sample[1]
                if total_delta > 0:
                    cpu_percent = round(
                        min(
                            cpu_capacity_percent,
                            max(
                                0.0,
                                100.0
                                * cpu_count
                                * (total_delta - idle_delta)
                                / total_delta,
                            ),
                        ),
                        1,
                    )
            self.cpu_sample = current_cpu
            current_workload_usage = self._workload_cpu_usage_seconds()
            workload_cpu_percent = None
            if (
                current_workload_usage is not None
                and self.workload_cpu_sample is not None
            ):
                previous_usage, previous_time = self.workload_cpu_sample
                elapsed = now - previous_time
                if elapsed > 0:
                    workload_cpu_percent = round(
                        min(
                            cpu_capacity_percent,
                            max(
                                0.0,
                                100.0
                                * (current_workload_usage - previous_usage)
                                / elapsed,
                            ),
                        ),
                        1,
                    )
            self.workload_cpu_sample = (
                (current_workload_usage, now)
                if current_workload_usage is not None else None
            )
            try:
                load_1m, load_5m, load_15m = os.getloadavg()
            except OSError:
                load_1m = load_5m = load_15m = None
            try:
                uptime = float(Path("/proc/uptime").read_text(
                    encoding="utf-8"
                ).split()[0])
            except (OSError, ValueError, IndexError):
                uptime = None
            self.metrics_cache = {
                "cpu_count": cpu_count,
                "cpu_capacity_percent": cpu_capacity_percent,
                "cpu_percent": cpu_percent,
                "workload_cpu_percent": workload_cpu_percent,
                "memory": self._memory_metrics(),
                "cpu_temperature_c": self._cpu_temperature(),
                "load_average": [load_1m, load_5m, load_15m],
                "uptime_seconds": uptime,
            }
            self.metrics_cache_time = now
            return self.metrics_cache

    @staticmethod
    def _revision(content):
        return hashlib.sha256(content.encode("utf-8")).hexdigest()

    def _editor_path(self, profile, component):
        try:
            relative_path = EDITOR_FILES[profile][component]
        except KeyError as error:
            raise ValueError("invalid editable course or component") from error
        path = (self.workspace / relative_path).resolve()
        if self.workspace not in path.parents:
            raise ValueError("editor path escaped the workspace")
        if not path.is_file():
            raise RuntimeError(
                f"student source file is missing: {relative_path}"
            )
        return relative_path, path

    @staticmethod
    def _editor_language(path):
        if path.name == "CMakeLists.txt" or path.suffix == ".cmake":
            return "cmake"
        if path.suffix == ".py":
            return "python"
        if path.suffix in {".cpp", ".h", ".hpp"}:
            return "cpp"
        return path.suffix.lstrip(".") or "text"

    def _editor_workspace_path(self, profile, file_path):
        try:
            root_relative = EDITOR_ROOTS[profile]
        except KeyError as error:
            raise ValueError("invalid editable course") from error
        if not isinstance(file_path, str) or not file_path:
            raise ValueError("an editable file path is required")
        requested = Path(file_path)
        if requested.is_absolute() or ".." in requested.parts:
            raise ValueError("editor path escaped the course workspace")
        root = (self.workspace / root_relative).resolve()
        path = (root / requested).resolve()
        if root not in path.parents:
            raise ValueError("editor path escaped the course workspace")
        if not path.is_file():
            raise ValueError("editable course file does not exist")
        if path.suffix not in EDITOR_EXTENSIONS and path.name not in EDITOR_FILENAMES:
            raise ValueError("file type is not editable in the course workspace")
        return path.relative_to(self.workspace), path, requested.as_posix()

    def editor_tree(self, profile):
        try:
            root_relative = EDITOR_ROOTS[profile]
            defaults = EDITOR_FILES[profile]
        except KeyError as error:
            raise ValueError("invalid editable course") from error
        root = (self.workspace / root_relative).resolve()
        if not root.is_dir():
            raise RuntimeError(f"student package is missing: {root_relative}")
        files = []
        for path in sorted(root.rglob("*")):
            if not path.is_file() or any(part.startswith(".") for part in path.parts):
                continue
            if "__pycache__" in path.parts:
                continue
            if path.suffix not in EDITOR_EXTENSIONS and path.name not in EDITOR_FILENAMES:
                continue
            resolved = path.resolve()
            if root not in resolved.parents:
                continue
            files.append({
                "path": resolved.relative_to(root).as_posix(),
                "name": path.name,
                "language": self._editor_language(path),
            })
            if len(files) >= 256:
                break
        return {
            "profile": profile,
            "root": str(root_relative),
            "files": files,
            "defaults": {
                component: str(
                    (self.workspace / relative).resolve().relative_to(root)
                )
                for component, relative in defaults.items()
            },
        }

    def editor_manifest(self):
        return {
            "profiles": [
                {"id": "ada_high_school", "label": "ADA Academy · Python"},
                {"id": "intro2av_python", "label": "Intro2AV · Python"},
                {"id": "intro2av_cpp", "label": "Intro2AV · C++"},
            ],
            "components": ["perception", "planning", "control"],
            "max_users": 5,
        }

    def read_editor_file(self, profile, component):
        relative_path, path = self._editor_path(profile, component)
        with self.editor_lock:
            content = path.read_text(encoding="utf-8")
        return {
            "profile": profile,
            "component": component,
            "path": str(relative_path),
            "content": content,
            "revision": self._revision(content),
            "language": self._editor_language(path),
        }

    def save_editor_file(self, profile, component, content, revision):
        if not isinstance(content, str):
            raise ValueError("editor content must be text")
        if len(content.encode("utf-8")) > 64 * 1024:
            raise ValueError("editor content exceeds 64 KiB")
        if "\x00" in content:
            raise ValueError("editor content contains a null byte")
        relative_path, path = self._editor_path(profile, component)
        with self.editor_lock:
            current = path.read_text(encoding="utf-8")
            current_revision = self._revision(current)
            if revision != current_revision:
                raise RevisionConflict(
                    "This file changed after you opened it. "
                    "Reload it before saving."
                )
            temporary = path.with_name(
                f".{path.name}.{threading.get_ident()}.tmp"
            )
            try:
                temporary.write_text(content, encoding="utf-8")
                os.replace(temporary, path)
            finally:
                if temporary.exists():
                    temporary.unlink()
        syntax_error = None
        if path.suffix == ".py":
            try:
                ast.parse(content, filename=str(relative_path))
            except SyntaxError as error:
                syntax_error = {
                    "line": error.lineno,
                    "column": error.offset,
                    "message": error.msg,
                }
        return {
            "profile": profile,
            "component": component,
            "path": str(relative_path),
            "revision": self._revision(content),
            "syntax_error": syntax_error,
            "language": self._editor_language(path),
        }

    @staticmethod
    def _validate_editor_content(content):
        if not isinstance(content, str):
            raise ValueError("editor content must be text")
        if len(content.encode("utf-8")) > 64 * 1024:
            raise ValueError("editor content exceeds 64 KiB")
        if "\x00" in content:
            raise ValueError("editor content contains a null byte")

    @staticmethod
    def _cpp_diagnostics(content):
        pairs = {")": "(", "]": "[", "}": "{"}
        stack = []
        diagnostics = []
        line = 1
        column = 0
        index = 0
        quote = None
        block_comment = False
        while index < len(content):
            character = content[index]
            following = content[index + 1] if index + 1 < len(content) else ""
            column += 1
            if character == "\n":
                line += 1
                column = 0
                index += 1
                continue
            if block_comment:
                if character == "*" and following == "/":
                    block_comment = False
                    index += 2
                    column += 1
                else:
                    index += 1
                continue
            if quote:
                if character == "\\":
                    index += 2
                    column += 1
                    continue
                if character == quote:
                    quote = None
                index += 1
                continue
            if character == "/" and following == "/":
                newline = content.find("\n", index)
                if newline < 0:
                    break
                index = newline
                continue
            if character == "/" and following == "*":
                block_comment = True
                index += 2
                column += 1
                continue
            if character in {'"', "'"}:
                quote = character
            elif character in "([{":
                stack.append((character, line, column))
            elif character in pairs:
                if not stack or stack[-1][0] != pairs[character]:
                    diagnostics.append({
                        "line": line,
                        "column": column,
                        "message": f"unmatched '{character}'",
                        "severity": "error",
                    })
                else:
                    stack.pop()
            index += 1
        for character, open_line, open_column in stack[-8:]:
            diagnostics.append({
                "line": open_line,
                "column": open_column,
                "message": f"unclosed '{character}'",
                "severity": "error",
            })
        if quote:
            diagnostics.append({
                "line": line,
                "column": max(1, column),
                "message": "unclosed string literal",
                "severity": "error",
            })
        if block_comment:
            diagnostics.append({
                "line": line,
                "column": max(1, column),
                "message": "unclosed block comment",
                "severity": "error",
            })
        return diagnostics

    @classmethod
    def _source_diagnostics(cls, path, content):
        if path.suffix in {".cpp", ".h", ".hpp"}:
            return cls._cpp_diagnostics(content)
        if path.suffix != ".py":
            return []
        try:
            ast.parse(content, filename=str(path))
        except SyntaxError as error:
            return [{
                "line": error.lineno or 1,
                "column": error.offset or 1,
                "message": error.msg,
                "severity": "error",
            }]
        return []

    @staticmethod
    def _diff_operations(before, after):
        matcher = difflib.SequenceMatcher(None, before, after, autojunk=False)
        operations = []
        for tag, start, end, replacement_start, replacement_end in reversed(
            matcher.get_opcodes()
        ):
            if tag == "equal":
                continue
            operations.append({
                "start": start,
                "end": end,
                "text": after[replacement_start:replacement_end],
            })
        return operations

    @staticmethod
    def _transform_position(position, operation):
        start = operation["start"]
        end = operation["end"]
        inserted = len(operation["text"])
        if start == end:
            return position if position < start else position + inserted
        if position <= start:
            return position
        if position >= end:
            return position + inserted - (end - start)
        return start + inserted

    def _collab_document(self, profile, component, file_path=None):
        if file_path:
            relative_path, path, file_id = self._editor_workspace_path(
                profile, file_path
            )
        else:
            relative_path, path = self._editor_path(profile, component)
            root = (self.workspace / EDITOR_ROOTS[profile]).resolve()
            file_id = path.relative_to(root).as_posix()
        key = (profile, file_id)
        document = self.collab_documents.get(key)
        if document is None:
            document = CollaborativeDocument(path.read_text(encoding="utf-8"))
            document.diagnostics = self._source_diagnostics(path, document.content)
            self.collab_documents[key] = document
        return relative_path, path, file_id, document

    @staticmethod
    def _presence(document):
        cutoff = time.monotonic() - 12.0
        document.users = {
            client_id: user
            for client_id, user in document.users.items()
            if user["updated"] >= cutoff
        }
        return [
            {
                "client_id": client_id,
                "name": user["name"],
                "cursor": user["cursor"],
                "selection": user["selection"],
                "color": user["color"],
            }
            for client_id, user in document.users.items()
        ]

    @staticmethod
    def _update_presence(document, client_id, name, cursor, selection):
        if not re.fullmatch(r"[A-Za-z0-9_-]{8,64}", str(client_id or "")):
            raise ValueError("invalid editor client id")
        clean_name = re.sub(r"[^A-Za-z0-9 ._-]", "", str(name or "Student"))[:32]
        colors = ("#4aa3df", "#e67e5f", "#65b96e", "#b578d4", "#e2a93b")
        document.users[client_id] = {
            "name": clean_name or "Student",
            "cursor": max(0, min(int(cursor or 0), len(document.content))),
            "selection": max(0, min(int(selection or 0), len(document.content))),
            "color": colors[int(hashlib.sha256(client_id.encode()).hexdigest(), 16) % len(colors)],
            "updated": time.monotonic(),
        }

    def collaborative_snapshot(
        self, profile, component, client_id, name, cursor=0, selection=0,
        file_path=None,
    ):
        with self.editor_lock:
            relative_path, path, file_id, document = self._collab_document(
                profile, component, file_path
            )
            self._update_presence(document, client_id, name, cursor, selection)
            return {
                "profile": profile,
                "component": component,
                "file": file_id,
                "path": str(relative_path),
                "content": document.content,
                "version": document.version,
                "revision": self._revision(document.content),
                "language": self._editor_language(path),
                "diagnostics": document.diagnostics,
                "users": self._presence(document),
            }

    def synchronize_editor(
        self, profile, component, client_id, name, base_version,
        content, cursor=0, selection=0, file_path=None,
    ):
        self._validate_editor_content(content)
        with self.editor_lock:
            relative_path, path, file_id, document = self._collab_document(
                profile, component, file_path
            )
            try:
                base_version = int(base_version)
                base_content = document.snapshots[base_version]
            except (TypeError, ValueError, KeyError) as error:
                raise RevisionConflict(
                    "The collaboration history expired; reload the shared document."
                ) from error
            for operation in self._diff_operations(base_content, content):
                for remote in document.history:
                    if remote["version"] <= base_version:
                        continue
                    operation["start"] = self._transform_position(
                        operation["start"], remote
                    )
                    operation["end"] = self._transform_position(
                        operation["end"], remote
                    )
                operation["start"] = max(0, min(operation["start"], len(document.content)))
                operation["end"] = max(
                    operation["start"], min(operation["end"], len(document.content))
                )
                document.content = (
                    document.content[:operation["start"]]
                    + operation["text"]
                    + document.content[operation["end"]:]
                )
                document.version += 1
                operation["version"] = document.version
                operation["client_id"] = client_id
                document.history.append(operation.copy())
                document.snapshots[document.version] = document.content
            document.history = document.history[-200:]
            keep_versions = {document.version}
            keep_versions.update(item["version"] for item in document.history)
            if document.history:
                keep_versions.add(document.history[0]["version"] - 1)
            document.snapshots = {
                version: snapshot
                for version, snapshot in document.snapshots.items()
                if version in keep_versions
            }
            temporary = path.with_name(f".{path.name}.collab.tmp")
            try:
                temporary.write_text(document.content, encoding="utf-8")
                os.replace(temporary, path)
            finally:
                if temporary.exists():
                    temporary.unlink()
            document.diagnostics = self._source_diagnostics(path, document.content)
            self._update_presence(document, client_id, name, cursor, selection)
            return {
                "profile": profile,
                "component": component,
                "file": file_id,
                "path": str(relative_path),
                "content": document.content,
                "version": document.version,
                "revision": self._revision(document.content),
                "language": self._editor_language(path),
                "diagnostics": document.diagnostics,
                "users": self._presence(document),
            }

    def leave_editor(self, profile, component, client_id, file_path=None):
        with self.editor_lock:
            try:
                _, _, file_id = self._editor_workspace_path(profile, file_path)
            except (ValueError, KeyError):
                try:
                    _, path = self._editor_path(profile, component)
                    root = (self.workspace / EDITOR_ROOTS[profile]).resolve()
                    file_id = path.relative_to(root).as_posix()
                except (ValueError, KeyError):
                    return
            document = self.collab_documents.get((profile, file_id))
            if document:
                document.users.pop(client_id, None)

    def _append_log_locked(self, line):
        repeat_key = None
        if "Timed out waiting for transform from base_link to map" in line:
            repeat_key = "waiting_for_map_transform"
        elif "Please set the initial pose" in line:
            repeat_key = "waiting_for_initial_pose"
        if repeat_key:
            now = time.monotonic()
            last = self.repeated_log_times.get(repeat_key)
            if last is not None and now - last < 5.0:
                return
            self.repeated_log_times[repeat_key] = now
        self.log_cursor += 1
        self.logs.append(line)

    def _record_output(self, process, label):
        if process.stdout:
            for line in iter(process.stdout.readline, ""):
                with self.lock:
                    self._append_log_locked(f"[{label}] {line.rstrip()}")
        return_code = process.wait()
        with self.lock:
            if process is self.job:
                self.job_return_code = return_code
            self._append_log_locked(f"[{label}] exited with code {return_code}")

    def _spawn(self, command, label):
        process = subprocess.Popen(
            command,
            cwd=self.workspace,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
            bufsize=1,
        )
        threading.Thread(
            target=self._record_output, args=(process, label), daemon=True
        ).start()
        return process

    def install(self, chassis):
        if chassis not in CHASSIS:
            raise ValueError("chassis must be osracer or f1tenth")
        self.stop()
        with self.lock:
            if self.job and self.job.poll() is None:
                raise RuntimeError("an install/build job is already running")
            self._append_log_locked(f"[install] preparing CARKit for {chassis}")
            self.job_kind = "install"
            self.job_return_code = None
            self.job_packages = []
            self.job = self._spawn(
                [str(self.workspace / "docker" / "install_carkit.sh"), chassis],
                "install",
            )

    def compile(self, target, implementations=None):
        if target != "all" and target not in BUILD_TARGETS:
            raise ValueError("invalid compile target")
        if implementations is not None and not isinstance(implementations, dict):
            raise ValueError("implementations must be an object")
        if self.job and self.job.poll() is None:
            raise RuntimeError("an install/build job is already running")

        packages = (
            []
            if target == "all"
            else resolve_build_packages(target, implementations)
        )
        if target != "all" and not packages:
            raise ValueError(
                f"{target} implementation is off; no packages to compile"
            )

        # Never replace files in the install overlay while nodes are using it.
        self.stop()
        if target == "all":
            config_path = self.workspace / ".carkit" / "config.json"
            chassis = "osracer"
            if config_path.is_file():
                try:
                    selected = json.loads(
                        config_path.read_text(encoding="utf-8")
                    ).get("chassis")
                    if selected in CHASSIS:
                        chassis = selected
                except (OSError, json.JSONDecodeError):
                    pass
            command = [
                "bash",
                "-lc",
                (
                    f"CARKIT_CHASSIS={chassis} "
                    "exec ./docker/build_workspace.sh"
                ),
            ]
        else:
            package_names = " ".join(packages)
            command = [
                "bash",
                "-lc",
                "source /opt/ros/${ROS_DISTRO:-humble}/setup.bash && "
                "if [ -f install/setup.bash ]; then "
                "source install/setup.bash; fi && "
                "exec colcon build --symlink-install "
                f"--packages-up-to {package_names} "
                "--cmake-args -DCMAKE_BUILD_TYPE=Release "
                "--event-handlers console_cohesion+",
            ]

        with self.lock:
            package_summary = (
                "entire repository"
                if target == "all"
                else ", ".join(packages)
            )
            self._append_log_locked(
                f"[compile] starting target={target}; packages={package_summary}"
            )
            self.job_kind = f"compile:{target}"
            self.job_return_code = None
            self.job_packages = packages
            self.job = self._spawn(command, f"compile:{target}")
        return {"target": target, "packages": packages}

    def launch(self, request):
        if self.job and self.job.poll() is None:
            raise RuntimeError("wait for the install/build job to finish")
        profile = request.get("profile", "ada_high_school")
        chassis = request.get("chassis", "osracer")
        if profile not in PROFILES or chassis not in CHASSIS:
            raise ValueError("invalid profile or chassis")
        requested_implementations = request.get("implementations", {})
        implementations = {}
        for name in ("planning", "control", "perception"):
            implementation = requested_implementations.get(
                name,
                PROFILE_IMPLEMENTATIONS[profile][name],
            )
            if implementation not in IMPLEMENTATIONS:
                raise ValueError(f"invalid {name} implementation")
            implementations[name] = implementation
        enabled = request.get("components", {})
        if set(enabled) - COMPONENTS:
            raise ValueError("invalid component name")
        map_path = request.get("map", "/workspaces/CARKit/map/map_3f.yaml")
        if not SAFE_PATH.fullmatch(map_path):
            raise ValueError("invalid map path")
        try:
            selected_map = Path(map_path).resolve(strict=True)
            selected_map.relative_to((self.workspace / "map").resolve())
        except (OSError, ValueError) as error:
            raise ValueError(
                "map must be an available file in the CARKit map folder"
            ) from error
        if selected_map.suffix.lower() != ".yaml" or not selected_map.is_file():
            raise ValueError("map must be an available .yaml file")
        map_path = str(selected_map)
        perception_model = request.get("perception_model", "combined")
        if perception_model not in PERCEPTION_MODELS:
            raise ValueError("invalid perception model")
        custom_model_path = request.get("custom_perception_model_path", "")
        if custom_model_path and not SAFE_PATH.fullmatch(custom_model_path):
            raise ValueError("invalid custom perception model path")
        if perception_model == "custom":
            if not custom_model_path.endswith(".engine"):
                raise ValueError(
                    "custom perception model must be a TensorRT .engine file"
                )
            if not Path(custom_model_path).is_file():
                raise ValueError(
                    "custom perception model does not exist inside the container"
                )
        config_path = self.workspace / ".carkit" / "config.json"
        if not config_path.is_file():
            raise RuntimeError("install/build a chassis before starting CARKit")
        try:
            installed_chassis = json.loads(
                config_path.read_text(encoding="utf-8")
            ).get("chassis")
        except (OSError, json.JSONDecodeError) as error:
            raise RuntimeError("the CARKit installation selection is invalid") from error
        if installed_chassis != chassis:
            raise RuntimeError(
                f"{installed_chassis} is installed; install {chassis} before launching it"
            )

        self.stop()
        launch_args = [
            f"profile:={profile}",
            f"chassis:={chassis}",
            f"map:={map_path}",
            f"perception_model:={perception_model}",
        ]
        if custom_model_path:
            launch_args.append(
                f"custom_perception_model_path:={custom_model_path}"
            )
        for name in ("planning", "control", "perception"):
            launch_args.append(f"{name}:={implementations[name]}")
        for name in COMPONENTS:
            if name in enabled:
                value = "true" if bool(enabled[name]) else "false"
                launch_args.append(f"start_{name}:={value}")
        launch_args.extend([
            f"start_camera:={'true' if request.get('camera', True) else 'false'}",
            f"start_lidar:={'true' if request.get('lidar', True) else 'false'}",
            "web_bridge:=true",
        ])
        ros_command = " ".join([
            "source /opt/ros/${ROS_DISTRO:-humble}/setup.bash",
            "&& source install/setup.bash",
            "&& exec ros2 launch carkit_bringup carkit.launch.py",
            *launch_args,
        ])
        with self.lock:
            self._append_log_locked(f"[launch] profile={profile} chassis={chassis}")
            self.current_config = dict(request)
            self.current_config["implementations"] = implementations
            self.process = self._spawn(["bash", "-lc", ros_command], "carkit")

    def stop(self):
        process = self.process
        if not process or process.poll() is not None:
            self.process = None
            self.current_config = None
            return
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
        finally:
            self.process = None
            self.current_config = None

    def status(self, log_after=None):
        process = self.process
        job = self.job
        config_path = self.workspace / ".carkit" / "config.json"
        selected = None
        if config_path.is_file():
            try:
                selected = json.loads(config_path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                selected = None
        system = self.system_metrics()
        chassis_telemetry = (
            self.telemetry_monitor.snapshot()
            if self.telemetry_monitor is not None else None
        )
        with self.lock:
            logs = list(self.logs)
            log_start = self.log_cursor - len(logs)
            if log_after is not None:
                log_after = max(0, min(int(log_after), self.log_cursor))
                offset = max(0, log_after - log_start)
                logs = logs[offset:]
                log_start += offset
            return {
                "running": bool(process and process.poll() is None),
                "pid": process.pid if process and process.poll() is None else None,
                "job_running": bool(job and job.poll() is None),
                "job": self.job_kind,
                "job_packages": list(self.job_packages),
                "job_return_code": self.job_return_code,
                "installed": (
                    (self.workspace / "install" / "setup.bash").is_file()
                    and config_path.is_file()
                ),
                "selection": selected,
                "launch_config": self.current_config,
                "logs": logs,
                "log_start": log_start,
                "log_cursor": self.log_cursor,
                "time": time.time(),
                "system": system,
                "chassis_telemetry": chassis_telemetry,
            }


class ApiHandler(BaseHTTPRequestHandler):
    server_version = "CARKitWeb/0.1"

    def _json(self, status, value):
        body = json.dumps(value).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _request_json(self):
        """Convert request data into a JSON object for browser transport."""
        length = int(self.headers.get("Content-Length", "0"))
        if length > 128 * 1024:
            raise ValueError("request is too large")
        return json.loads(self.rfile.read(length) or b"{}")

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path
        if path == "/api/status":
            values = parse_qs(parsed.query).get("log_after", [])
            try:
                log_after = int(values[0]) if values else None
            except ValueError:
                self._json(400, {"error": "log_after must be an integer"})
                return
            self._json(200, self.server.manager.status(log_after))
            return
        if path == "/api/config":
            maps = self.server.manager.map_files()
            preferred_map = next(
                (
                    item["path"]
                    for item in maps
                    if item["name"] == "map_3f.yaml"
                ),
                maps[0]["path"] if maps else None,
            )
            self._json(200, {
                "profiles": [
                    {
                        "id": "ada_high_school",
                        "label": "ADA Academy",
                        "help": "Working guided algorithms for incremental lessons.",
                        "behavior": True,
                        "implementations": PROFILE_IMPLEMENTATIONS["ada_high_school"],
                    },
                    {
                        "id": "intro2av",
                        "label": "Intro2AV",
                        "help": "Safe ROS 2 boilerplates for ground-up implementation.",
                        "behavior": True,
                        "implementations": PROFILE_IMPLEMENTATIONS["intro2av"],
                    },
                    {
                        "id": "reference",
                        "label": "Reference",
                        "help": "Complete instructor implementation.",
                        "behavior": True,
                        "implementations": PROFILE_IMPLEMENTATIONS["reference"],
                    },
                ],
                "chassis": ["osracer", "f1tenth"],
                "implementations": [
                    "reference", "ada_academy", "intro2av_python",
                    "intro2av_cpp", "off"
                ],
                "perception_models": [
                    {
                        "id": "generic_coco",
                        "label": "Generic YOLO · COCO",
                        "help": "Fast single-model detection for common objects.",
                    },
                    {
                        "id": "traffic_signs",
                        "label": "Traffic signs only",
                        "help": "Fast course-specific model for signs and cones.",
                    },
                    {
                        "id": "combined",
                        "label": "COCO + traffic signs",
                        "help": "Runs COCO every frame and refreshes signs every other frame.",
                    },
                    {
                        "id": "custom",
                        "label": "Custom TensorRT engine",
                        "help": "Use a model exported for this Jetson and image size.",
                    },
                ],
                "maps": maps,
                "default_map": preferred_map,
                "web_bridge_port": 9090,
            })
            return
        if path == "/api/editor":
            self._json(200, self.server.manager.editor_manifest())
            return
        if path == "/api/editor/tree":
            values = parse_qs(parsed.query)
            try:
                value = self.server.manager.editor_tree(
                    values.get("profile", [None])[0]
                )
                self._json(200, value)
            except (ValueError, RuntimeError, OSError) as error:
                self._json(400, {"error": str(error)})
            return
        if path == "/api/editor/collab":
            values = parse_qs(parsed.query)
            try:
                value = self.server.manager.collaborative_snapshot(
                    values.get("profile", [None])[0],
                    values.get("component", [None])[0],
                    values.get("client_id", [None])[0],
                    values.get("name", ["Student"])[0],
                    values.get("cursor", [0])[0],
                    values.get("selection", [0])[0],
                    values.get("file", [None])[0],
                )
                self._json(200, value)
            except (ValueError, RuntimeError, OSError) as error:
                self._json(400, {"error": str(error)})
            return
        if path == "/api/editor/file":
            query = parse_qs(parsed.query)
            try:
                profile = query.get("profile", [""])[0]
                component = query.get("component", [""])[0]
                value = self.server.manager.read_editor_file(
                    profile,
                    component,
                )
                self._json(200, value)
            except (ValueError, RuntimeError, OSError) as error:
                self._json(400, {"error": str(error)})
            return
        self._serve_static(path)

    def do_POST(self):
        path = urlparse(self.path).path
        try:
            request = self._request_json()
            if path == "/api/install":
                self.server.manager.install(request.get("chassis", "osracer"))
            elif path == "/api/launch":
                self.server.manager.launch(request)
            elif path == "/api/compile":
                value = self.server.manager.compile(
                    request.get("target", "all"),
                    request.get("implementations"),
                )
                self._json(202, {"ok": True, **value})
                return
            elif path == "/api/stop":
                self.server.manager.stop()
            elif path == "/api/editor/save":
                value = self.server.manager.save_editor_file(
                    request.get("profile"),
                    request.get("component"),
                    request.get("content"),
                    request.get("revision"),
                )
                self._json(200, value)
                return
            elif path == "/api/editor/sync":
                value = self.server.manager.synchronize_editor(
                    request.get("profile"),
                    request.get("component"),
                    request.get("client_id"),
                    request.get("name", "Student"),
                    request.get("base_version"),
                    request.get("content"),
                    request.get("cursor", 0),
                    request.get("selection", 0),
                    request.get("file"),
                )
                self._json(200, value)
                return
            elif path == "/api/editor/leave":
                self.server.manager.leave_editor(
                    request.get("profile"),
                    request.get("component"),
                    request.get("client_id"),
                    request.get("file"),
                )
            else:
                self._json(404, {"error": "unknown endpoint"})
                return
            self._json(202, {"ok": True})
        except RevisionConflict as error:
            self._json(409, {"error": str(error), "conflict": True})
        except (ValueError, RuntimeError, OSError, json.JSONDecodeError) as error:
            self._json(400, {"error": str(error)})

    def do_HEAD(self):
        path = urlparse(self.path).path
        if path not in {"", "/", "/index.html", "/app.js", "/style.css"}:
            self.send_error(404)
            return
        name = "index.html" if path in {"", "/"} else path.lstrip("/")
        file_path = self.server.static_dir / name
        if not file_path.is_file():
            self.send_error(404)
            return
        mime = {
            ".html": "text/html",
            ".js": "text/javascript",
            ".css": "text/css",
        }[file_path.suffix]
        self.send_response(200)
        self.send_header("Content-Type", mime + "; charset=utf-8")
        self.send_header("Content-Length", str(file_path.stat().st_size))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()

    def _serve_static(self, path):
        name = "index.html" if path in {"", "/"} else path.lstrip("/")
        if "/" in name or name not in {"index.html", "app.js", "style.css"}:
            self.send_error(404)
            return
        file_path = self.server.static_dir / name
        if not file_path.is_file():
            self.send_error(404)
            return
        mime = {
            ".html": "text/html",
            ".js": "text/javascript",
            ".css": "text/css",
        }[file_path.suffix]
        body = file_path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", mime + "; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, message, *args):
        print(f"[web] {self.address_string()} {message % args}")


def find_static_dir(workspace):
    source = Path(workspace) / "carkit" / "interface" / "carkit_webui" / "static"
    if source.is_dir():
        return source
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("carkit_webui")) / "static"
    except ImportError as error:
        raise RuntimeError("CARKit WebUI static assets were not found") from error


def main(args=None):
    parser = argparse.ArgumentParser(description="CARKit browser dashboard")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument(
        "--workspace",
        default=os.environ.get("CARKIT_WORKSPACE", "/workspaces/CARKit"),
    )
    options = parser.parse_args(args)
    manager = ProcessManager(options.workspace)
    telemetry_monitor = ChassisTelemetryMonitor()
    manager.telemetry_monitor = telemetry_monitor
    server = ThreadingHTTPServer((options.host, options.port), ApiHandler)
    server.manager = manager
    server.static_dir = find_static_dir(options.workspace)
    print(f"CARKit WebUI: http://{options.host}:{options.port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        manager.stop()
        telemetry_monitor.close()
        server.server_close()


if __name__ == "__main__":
    main()
