#!/usr/bin/env python3
"""Display per-process CPU and memory usage for ROS 2 nodes.

ROS 2 does not expose a standard node-to-PID API.  This tool combines the
local ROS graph with process command lines.  Standalone nodes launched by ROS
2 normally carry an ``__node:=...`` remap, which gives an exact mapping.  A
fallback matches the executable name to the graph node name.  Nodes hosted by
a component container share one operating-system process and therefore cannot
have their CPU usage separated by the OS; the monitor reports that limitation
instead of inventing a per-component value.  Both a GUI and an SSH-friendly
continuously refreshed terminal view are available.
"""

from __future__ import annotations

import argparse
from collections import Counter
import csv
from dataclasses import dataclass
from datetime import datetime
import os
from pathlib import Path
import sys
import time
from typing import Iterable, Optional, Sequence, TextIO

import psutil
import rclpy
from rclpy.node import Node


TEXT_ONLY_FLAGS = {"--terminal", "--once", "--self-test", "--help", "-h"}
TEXT_ONLY_REQUESTED = any(flag in sys.argv[1:] for flag in TEXT_ONLY_FLAGS)
GUI_IMPORT_ERROR: Optional[ImportError] = None

if not TEXT_ONLY_REQUESTED:
    try:
        from PyQt5.QtCore import Qt, QTimer
        from PyQt5.QtGui import QColor, QFont
        from PyQt5.QtWidgets import (
            QApplication,
            QCheckBox,
            QComboBox,
            QHBoxLayout,
            QHeaderView,
            QLabel,
            QLineEdit,
            QMainWindow,
            QPlainTextEdit,
            QPushButton,
            QTableWidget,
            QTableWidgetItem,
            QTabWidget,
            QVBoxLayout,
            QWidget,
        )
    except ImportError as error:
        GUI_IMPORT_ERROR = error

if TEXT_ONLY_REQUESTED or GUI_IMPORT_ERROR is not None:
    class _QtStub:
        UserRole = 0

    class _WidgetStub:
        pass

    Qt = _QtStub()  # noqa: F811
    QTableWidgetItem = _WidgetStub  # noqa: F811
    QMainWindow = _WidgetStub  # noqa: F811
    QApplication = None  # noqa: F811
    QCheckBox = QComboBox = QHBoxLayout = QHeaderView = None  # noqa: F811
    QLabel = QLineEdit = QPlainTextEdit = QPushButton = None  # noqa: F811
    QTableWidget = QTabWidget = QVBoxLayout = QWidget = None  # noqa: F811
    QTimer = QColor = QFont = None  # noqa: F811


MONITOR_NODE_PREFIX = "carkit_node_cpu_monitor"
DETAIL_ROLE = Qt.UserRole + 1
CSV_TOP_NODE_COUNT = 20


@dataclass(frozen=True)
class ProcessRecord:
    """Stable process identity and command-line metadata."""

    pid: int
    create_time: float
    process_name: str
    executable_hint: str
    cmdline: tuple[str, ...]
    explicit_node_name: Optional[str]
    explicit_namespace: Optional[str]
    ros_candidate: bool

    @property
    def command(self) -> str:
        return " ".join(self.cmdline)


@dataclass(frozen=True)
class ProcessMetrics:
    cpu_percent: float
    memory_percent: float
    rss_bytes: int
    thread_count: int
    status: str


@dataclass(frozen=True)
class ProcessNodeMatch:
    process: ProcessRecord
    node_name: Optional[str]
    method: str


@dataclass(frozen=True)
class NodeRow:
    node_name: str
    process: Optional[ProcessRecord]
    metrics: Optional[ProcessMetrics]
    mapping: str


@dataclass(frozen=True)
class CpuCoreMetrics:
    index: int
    cpu_percent: float
    frequency_hz: Optional[int]
    online: bool


@dataclass(frozen=True)
class SystemCpuMetrics:
    cpu_percent: float
    cores: tuple[CpuCoreMetrics, ...]


def normalized_namespace(namespace: Optional[str]) -> str:
    if namespace is None:
        return ""
    stripped = namespace.strip().strip("/")
    return f"/{stripped}" if stripped else ""


def normalized_node_name(name: str, namespace: Optional[str] = None) -> str:
    name = name.strip()
    if not name:
        return "/"
    if name.startswith("/"):
        parts = [part for part in name.split("/") if part]
        return "/" + "/".join(parts)
    ns = normalized_namespace(namespace)
    return f"{ns}/{name}" if ns else f"/{name}"


def node_basename(full_name: str) -> str:
    return full_name.rstrip("/").split("/")[-1]


def node_is_hidden(full_name: str) -> bool:
    return any(part.startswith("_") for part in full_name.split("/") if part)


def remap_value(cmdline: Sequence[str], key: str) -> Optional[str]:
    marker = f"{key}:="
    for token in cmdline:
        if marker in token:
            value = token.split(marker, 1)[1].strip()
            if value:
                return value
    return None


def executable_hint(cmdline: Sequence[str], process_name: str) -> str:
    """Return the ROS executable name from native and Python commands."""

    if not cmdline:
        return process_name

    before_ros_args = list(cmdline)
    if "--ros-args" in before_ros_args:
        before_ros_args = before_ros_args[: before_ros_args.index("--ros-args")]

    first = Path(before_ros_args[0]).name if before_ros_args else process_name
    python_names = {"python", "python3", "python3.10", "pypy", "pypy3"}
    if first in python_names:
        for index, token in enumerate(before_ros_args[1:], start=1):
            if token == "-m" and index + 1 < len(before_ros_args):
                return before_ros_args[index + 1].split(".")[-1]
            if not token.startswith("-"):
                return Path(token).stem
        return process_name

    return Path(before_ros_args[0]).stem if before_ros_args else process_name


def is_ros_candidate(
    cmdline: Sequence[str],
    hint: str,
    explicit_node_name: Optional[str],
) -> bool:
    if explicit_node_name:
        return True

    excluded = {"bash", "sh", "zsh", "ros2", "python", "python3"}
    if hint in excluded:
        return False

    if "--ros-args" in cmdline:
        return True

    command = " ".join(cmdline)
    return (
        "/opt/ros/" in command and "/lib/" in command
    ) or (
        "/install/" in command and "/lib/" in command
    )


def discover_processes() -> list[ProcessRecord]:
    records: list[ProcessRecord] = []
    attributes = ["pid", "name", "cmdline", "create_time"]

    for process in psutil.process_iter(attributes):
        try:
            info = process.info
            cmdline = tuple(info.get("cmdline") or ())
            if not cmdline:
                continue
            name = str(info.get("name") or Path(cmdline[0]).name)
            node_name = remap_value(cmdline, "__node")
            namespace = remap_value(cmdline, "__ns")
            hint = executable_hint(cmdline, name)
            records.append(
                ProcessRecord(
                    pid=int(info["pid"]),
                    create_time=float(info.get("create_time") or 0.0),
                    process_name=name,
                    executable_hint=hint,
                    cmdline=cmdline,
                    explicit_node_name=node_name,
                    explicit_namespace=namespace,
                    ros_candidate=is_ros_candidate(cmdline, hint, node_name),
                )
            )
        except (psutil.AccessDenied, psutil.NoSuchProcess, psutil.ZombieProcess):
            continue

    return records


def graph_name_candidates(executable: str, graph_nodes: set[str]) -> list[str]:
    aliases = [executable]
    if executable.endswith("_node"):
        aliases.append(executable[: -len("_node")])

    exact = [name for name in graph_nodes if node_basename(name) == aliases[0]]
    if exact:
        return sorted(exact)

    return sorted(
        name for name in graph_nodes if node_basename(name) in set(aliases[1:])
    )


def match_processes_to_nodes(
    graph_nodes: Iterable[str],
    process_records: Iterable[ProcessRecord],
) -> tuple[list[NodeRow], list[ProcessNodeMatch]]:
    """Map local processes to graph nodes and retain duplicate processes."""

    normalized_graph = [normalized_node_name(name) for name in graph_nodes]
    graph_counts = Counter(normalized_graph)
    graph_set = set(normalized_graph)
    mapped_counts: Counter[str] = Counter()
    matches: list[ProcessNodeMatch] = []

    for process in process_records:
        if not process.ros_candidate:
            continue

        matched_name: Optional[str] = None
        method = "unmatched ROS process"

        if process.explicit_node_name:
            explicit = normalized_node_name(
                process.explicit_node_name,
                process.explicit_namespace,
            )
            if explicit in graph_set:
                matched_name = explicit
                method = "exact __node remap"

        if matched_name is None:
            candidates = graph_name_candidates(process.executable_hint, graph_set)
            if len(candidates) == 1:
                matched_name = candidates[0]
                method = "executable name"

        if matched_name is not None:
            mapped_counts[matched_name] += 1
        matches.append(ProcessNodeMatch(process, matched_name, method))

    rows: list[NodeRow] = []
    for match in matches:
        if match.node_name is not None:
            rows.append(
                NodeRow(
                    node_name=match.node_name,
                    process=match.process,
                    metrics=None,
                    mapping=match.method,
                )
            )

    for name, count in sorted(graph_counts.items()):
        missing_count = max(0, count - mapped_counts[name])
        for _ in range(missing_count):
            rows.append(
                NodeRow(
                    node_name=name,
                    process=None,
                    metrics=None,
                    mapping="PID unavailable (possibly a composed node)",
                )
            )

    pid_row_counts = Counter(
        row.process.pid for row in rows if row.process is not None
    )
    if any(count > 1 for count in pid_row_counts.values()):
        rows = [
            NodeRow(
                node_name=row.node_name,
                process=row.process,
                metrics=row.metrics,
                mapping=(
                    f"shared process ({pid_row_counts[row.process.pid]} nodes)"
                    if row.process is not None
                    and pid_row_counts[row.process.pid] > 1
                    else row.mapping
                ),
            )
            for row in rows
        ]

    return rows, matches


class ProcessSampler:
    """Keep psutil Process objects alive so cpu_percent is interval-based."""

    def __init__(self) -> None:
        self._processes: dict[int, tuple[float, psutil.Process]] = {}
        self._system_cpu_initialized = False

    def sample(
        self,
        records: Iterable[ProcessRecord],
    ) -> dict[int, ProcessMetrics]:
        metrics: dict[int, ProcessMetrics] = {}
        active_pids: set[int] = set()

        for record in records:
            active_pids.add(record.pid)
            cached = self._processes.get(record.pid)
            if cached is None or abs(cached[0] - record.create_time) > 1.0e-6:
                try:
                    process = psutil.Process(record.pid)
                    process.cpu_percent(interval=None)
                    self._processes[record.pid] = (record.create_time, process)
                except (psutil.AccessDenied, psutil.NoSuchProcess):
                    continue
                cpu_percent = 0.0
            else:
                process = cached[1]
                try:
                    cpu_percent = float(process.cpu_percent(interval=None))
                except (psutil.AccessDenied, psutil.NoSuchProcess):
                    continue

            try:
                memory_info = process.memory_info()
                metrics[record.pid] = ProcessMetrics(
                    cpu_percent=cpu_percent,
                    memory_percent=float(process.memory_percent()),
                    rss_bytes=int(memory_info.rss),
                    thread_count=int(process.num_threads()),
                    status=str(process.status()),
                )
            except (psutil.AccessDenied, psutil.NoSuchProcess):
                continue

        stale_pids = set(self._processes) - active_pids
        for pid in stale_pids:
            self._processes.pop(pid, None)

        return metrics

    def system_cpu_percent(self) -> float:
        value = float(psutil.cpu_percent(interval=None))
        if not self._system_cpu_initialized:
            self._system_cpu_initialized = True
            return 0.0
        return value


def read_optional_int(path: Optional[Path]) -> Optional[int]:
    if path is None:
        return None
    try:
        return int(path.read_text(encoding="utf-8").strip())
    except (OSError, UnicodeError, ValueError):
        return None


def core_frequency_path(index: int) -> Optional[Path]:
    cpufreq_path = (
        Path("/sys/devices/system/cpu")
        / f"cpu{index}"
        / "cpufreq"
    )
    try:
        resolved = cpufreq_path.resolve(strict=True)
    except OSError:
        return None
    for filename in ("scaling_cur_freq", "cpuinfo_cur_freq"):
        candidate = resolved / filename
        if candidate.is_file():
            return candidate
    return None


class SystemCpuSampler:
    """Sample per-core interval load and Linux cpufreq values."""

    def __init__(self) -> None:
        self.core_count = psutil.cpu_count(logical=True) or 1
        cpu_root = Path("/sys/devices/system/cpu")
        self._frequency_paths = tuple(
            core_frequency_path(index) for index in range(self.core_count)
        )
        self._online_paths = tuple(
            (
                cpu_root / f"cpu{index}" / "online"
                if (cpu_root / f"cpu{index}" / "online").exists()
                else None
            )
            for index in range(self.core_count)
        )
        self._initialized = False

    @property
    def core_indices(self) -> tuple[int, ...]:
        return tuple(range(self.core_count))

    def prime(self) -> None:
        psutil.cpu_percent(interval=None)
        psutil.cpu_percent(interval=None, percpu=True)
        self._initialized = True

    def sample(self) -> SystemCpuMetrics:
        total_percent = float(psutil.cpu_percent(interval=None))
        per_core_percent = tuple(
            float(value)
            for value in psutil.cpu_percent(interval=None, percpu=True)
        )
        if not self._initialized:
            self._initialized = True
            total_percent = 0.0
            per_core_percent = tuple(0.0 for _ in per_core_percent)

        needs_frequency_fallback = any(
            path is None for path in self._frequency_paths
        )
        fallback_frequencies = (
            psutil.cpu_freq(percpu=True) or []
            if needs_frequency_fallback
            else []
        )
        fallback_global = (
            psutil.cpu_freq(percpu=False)
            if needs_frequency_fallback
            else None
        )
        frequency_cache: dict[Path, Optional[int]] = {}
        cores: list[CpuCoreMetrics] = []
        for index in range(self.core_count):
            online_value = read_optional_int(self._online_paths[index])
            online = (
                online_value != 0
                if self._online_paths[index] is not None
                else True
            )
            frequency_hz: Optional[int] = None
            frequency_path = self._frequency_paths[index]
            if online and frequency_path is not None:
                if frequency_path not in frequency_cache:
                    frequency_khz = read_optional_int(frequency_path)
                    frequency_cache[frequency_path] = (
                        frequency_khz * 1000
                        if frequency_khz is not None
                        else None
                    )
                frequency_hz = frequency_cache[frequency_path]
            elif online:
                fallback_mhz: Optional[float] = None
                if len(fallback_frequencies) == self.core_count:
                    fallback_mhz = float(
                        fallback_frequencies[index].current
                    )
                elif fallback_global is not None:
                    fallback_mhz = float(fallback_global.current)
                if fallback_mhz is not None:
                    frequency_hz = int(round(fallback_mhz * 1_000_000.0))

            cpu_percent = (
                per_core_percent[index]
                if index < len(per_core_percent)
                else 0.0
            )
            cores.append(
                CpuCoreMetrics(
                    index=index,
                    cpu_percent=cpu_percent,
                    frequency_hz=frequency_hz,
                    online=online,
                )
            )
        return SystemCpuMetrics(
            cpu_percent=total_percent,
            cores=tuple(cores),
        )


class RosGraphReader:
    def __init__(self) -> None:
        self.node_name = f"{MONITOR_NODE_PREFIX}_{os.getpid()}"
        self.node: Node = rclpy.create_node(self.node_name, enable_rosout=False)

    def pump(self) -> None:
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def node_names(self, show_hidden: bool) -> list[str]:
        self.pump()
        names: list[str] = []
        for name, namespace in self.node.get_node_names_and_namespaces():
            full_name = normalized_node_name(name, namespace)
            if node_basename(full_name).startswith(MONITOR_NODE_PREFIX):
                continue
            if not show_hidden and node_is_hidden(full_name):
                continue
            names.append(full_name)
        return sorted(names)

    def close(self) -> None:
        self.node.destroy_node()


class NumericItem(QTableWidgetItem):
    def __init__(self, text: str, value: Optional[float] = None) -> None:
        super().__init__(text)
        self.setData(Qt.UserRole, value)

    def __lt__(self, other: QTableWidgetItem) -> bool:
        left = self.data(Qt.UserRole)
        right = other.data(Qt.UserRole)
        if left is None:
            return right is not None
        if right is None:
            return False
        try:
            return float(left) < float(right)
        except (TypeError, ValueError):
            return super().__lt__(other)


def cpu_color(cpu_percent: float) -> QColor:
    if cpu_percent >= 50.0:
        return QColor("#ffb3b3")
    if cpu_percent >= 20.0:
        return QColor("#ffe5a3")
    if cpu_percent >= 5.0:
        return QColor("#fff7cc")
    return QColor("#d9f2df")


def mib(byte_count: int) -> float:
    return float(byte_count) / (1024.0 * 1024.0)


class NodeCpuMonitorWindow(QMainWindow):
    NODE_HEADERS = [
        "ROS node",
        "PID",
        "CPU %",
        "RSS MiB",
        "Memory %",
        "Threads",
        "Process",
        "Mapping",
    ]
    PROCESS_HEADERS = [
        "Process",
        "PID",
        "CPU %",
        "RSS MiB",
        "Memory %",
        "Threads",
        "Inferred ROS node",
        "Command",
    ]

    def __init__(self, graph: RosGraphReader, refresh_ms: int) -> None:
        super().__init__()
        self.graph = graph
        self.sampler = ProcessSampler()
        self.last_node_rows: list[NodeRow] = []
        self.last_process_matches: list[ProcessNodeMatch] = []
        self.last_metrics: dict[int, ProcessMetrics] = {}

        self.setWindowTitle("CARKit ROS 2 Node CPU Monitor")
        self.resize(1180, 720)

        root = QWidget(self)
        layout = QVBoxLayout(root)

        title = QLabel("CARKit ROS 2 Node CPU Monitor")
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title.setFont(title_font)
        layout.addWidget(title)

        self.summary = QLabel("Waiting for the first CPU sample…")
        self.summary.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self.summary)

        notice = QLabel(
            "CPU is measured per OS process. A composed process cannot be split "
            "reliably among its component nodes; CPU may exceed 100% when multiple "
            "logical cores are used."
        )
        notice.setWordWrap(True)
        notice.setStyleSheet("color: #555; padding-bottom: 4px;")
        layout.addWidget(notice)

        controls = QHBoxLayout()
        controls.addWidget(QLabel("Filter:"))
        self.filter_edit = QLineEdit()
        self.filter_edit.setPlaceholderText("node, process, PID, or command")
        self.filter_edit.textChanged.connect(self.render_tables)
        controls.addWidget(self.filter_edit, 1)

        self.show_hidden = QCheckBox("Show hidden nodes")
        self.show_hidden.stateChanged.connect(self.refresh)
        controls.addWidget(self.show_hidden)

        self.show_unmapped = QCheckBox("Show unmapped nodes")
        self.show_unmapped.setChecked(True)
        self.show_unmapped.stateChanged.connect(self.render_tables)
        controls.addWidget(self.show_unmapped)

        controls.addWidget(QLabel("Refresh:"))
        self.interval_combo = QComboBox()
        for label, milliseconds in (
            ("0.5 s", 500),
            ("1 s", 1000),
            ("2 s", 2000),
            ("5 s", 5000),
        ):
            self.interval_combo.addItem(label, milliseconds)
        closest_index = min(
            range(self.interval_combo.count()),
            key=lambda index: abs(
                int(self.interval_combo.itemData(index)) - refresh_ms
            ),
        )
        self.interval_combo.setCurrentIndex(closest_index)
        self.interval_combo.currentIndexChanged.connect(self.change_interval)
        controls.addWidget(self.interval_combo)

        self.pause_button = QPushButton("Pause")
        self.pause_button.setCheckable(True)
        self.pause_button.toggled.connect(self.toggle_pause)
        controls.addWidget(self.pause_button)

        refresh_button = QPushButton("Refresh now")
        refresh_button.clicked.connect(self.refresh)
        controls.addWidget(refresh_button)
        layout.addLayout(controls)

        self.tabs = QTabWidget()
        self.node_table = self.make_table(self.NODE_HEADERS)
        self.process_table = self.make_table(self.PROCESS_HEADERS)
        self.node_table.sortItems(2, Qt.DescendingOrder)
        self.process_table.sortItems(2, Qt.DescendingOrder)
        self.tabs.addTab(self.node_table, "ROS Nodes")
        self.tabs.addTab(self.process_table, "ROS Processes")
        layout.addWidget(self.tabs, 1)

        self.detail = QPlainTextEdit()
        self.detail.setReadOnly(True)
        self.detail.setMaximumHeight(105)
        self.detail.setPlaceholderText("Select a row to see the full command line.")
        layout.addWidget(self.detail)

        self.node_table.currentCellChanged.connect(self.show_node_detail)
        self.process_table.currentCellChanged.connect(self.show_process_detail)
        self.tabs.currentChanged.connect(self.update_detail_for_current_tab)

        self.setCentralWidget(root)
        self.statusBar().showMessage("Starting…")

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(refresh_ms)

        psutil.cpu_percent(interval=None)
        self.refresh()

    @staticmethod
    def make_table(headers: Sequence[str]) -> QTableWidget:
        table = QTableWidget(0, len(headers))
        table.setHorizontalHeaderLabels(list(headers))
        table.setAlternatingRowColors(True)
        table.setEditTriggers(QTableWidget.NoEditTriggers)
        table.setSelectionBehavior(QTableWidget.SelectRows)
        table.setSelectionMode(QTableWidget.SingleSelection)
        table.setSortingEnabled(True)
        table.verticalHeader().setVisible(False)
        header = table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeToContents)
        header.setStretchLastSection(True)
        return table

    def change_interval(self) -> None:
        milliseconds = int(self.interval_combo.currentData())
        self.timer.setInterval(milliseconds)

    def toggle_pause(self, paused: bool) -> None:
        if paused:
            self.timer.stop()
            self.pause_button.setText("Resume")
            self.statusBar().showMessage("Paused")
        else:
            self.timer.start(int(self.interval_combo.currentData()))
            self.pause_button.setText("Pause")
            self.refresh()

    def refresh(self) -> None:
        try:
            graph_nodes = self.graph.node_names(self.show_hidden.isChecked())
            process_records = discover_processes()
            node_rows, process_matches = match_processes_to_nodes(
                graph_nodes,
                process_records,
            )
            candidate_records = [
                match.process for match in process_matches
            ]
            metrics = self.sampler.sample(candidate_records)

            self.last_node_rows = [
                NodeRow(
                    node_name=row.node_name,
                    process=row.process,
                    metrics=(metrics.get(row.process.pid) if row.process else None),
                    mapping=row.mapping,
                )
                for row in node_rows
            ]
            self.last_process_matches = process_matches
            self.last_metrics = metrics
            self.render_tables()
            self.update_summary()
            self.statusBar().showMessage(
                f"Updated {time.strftime('%H:%M:%S')}"
            )
        except Exception as error:  # Keep the monitor alive on transient /proc races.
            self.statusBar().showMessage(f"Refresh failed: {error}")

    def update_summary(self) -> None:
        mapped_rows = [row for row in self.last_node_rows if row.process is not None]
        unmapped_count = len(self.last_node_rows) - len(mapped_rows)
        unique_pids = {row.process.pid for row in mapped_rows if row.process}
        total_cpu = sum(
            self.last_metrics[pid].cpu_percent
            for pid in unique_pids
            if pid in self.last_metrics
        )
        total_rss = sum(
            self.last_metrics[pid].rss_bytes
            for pid in unique_pids
            if pid in self.last_metrics
        )
        system_cpu = self.sampler.system_cpu_percent()
        core_count = psutil.cpu_count(logical=True) or 1
        self.summary.setText(
            f"Nodes: <b>{len(self.last_node_rows)}</b> &nbsp; "
            f"Mapped: <b>{len(mapped_rows)}</b> &nbsp; "
            f"Unmapped: <b>{unmapped_count}</b> &nbsp; "
            f"Mapped-process CPU: <b>{total_cpu:.1f}%</b> &nbsp; "
            f"Mapped RSS: <b>{mib(total_rss):.1f} MiB</b> &nbsp; "
            f"System CPU: <b>{system_cpu:.1f}%</b> across {core_count} logical cores"
        )

    def render_tables(self) -> None:
        self.render_node_table()
        self.render_process_table()

    def filter_matches(self, fields: Iterable[object]) -> bool:
        needle = self.filter_edit.text().strip().lower()
        if not needle:
            return True
        return needle in " ".join(str(field) for field in fields).lower()

    def render_node_table(self) -> None:
        rows = []
        for row in self.last_node_rows:
            if row.process is None and not self.show_unmapped.isChecked():
                continue
            if not self.filter_matches(
                (
                    row.node_name,
                    row.process.pid if row.process else "",
                    row.process.executable_hint if row.process else "",
                    row.process.command if row.process else "",
                    row.mapping,
                )
            ):
                continue
            rows.append(row)

        sorting = self.node_table.isSortingEnabled()
        self.node_table.setSortingEnabled(False)
        self.node_table.setRowCount(len(rows))

        for index, row in enumerate(rows):
            process = row.process
            metrics = row.metrics
            command = process.command if process else "No local PID mapping available"
            details = (
                f"Node: {row.node_name}\n"
                f"Mapping: {row.mapping}\n"
                f"PID: {process.pid if process else 'unavailable'}\n"
                f"Status: {metrics.status if metrics else 'unavailable'}\n"
                f"Command: {command}"
            )

            values: list[QTableWidgetItem] = [
                QTableWidgetItem(row.node_name),
                NumericItem(
                    str(process.pid) if process else "—",
                    process.pid if process else None,
                ),
                NumericItem(
                    f"{metrics.cpu_percent:.1f}" if metrics else "—",
                    metrics.cpu_percent if metrics else None,
                ),
                NumericItem(
                    f"{mib(metrics.rss_bytes):.1f}" if metrics else "—",
                    mib(metrics.rss_bytes) if metrics else None,
                ),
                NumericItem(
                    f"{metrics.memory_percent:.2f}" if metrics else "—",
                    metrics.memory_percent if metrics else None,
                ),
                NumericItem(
                    str(metrics.thread_count) if metrics else "—",
                    metrics.thread_count if metrics else None,
                ),
                QTableWidgetItem(process.executable_hint if process else "—"),
                QTableWidgetItem(row.mapping),
            ]
            for column, item in enumerate(values):
                item.setToolTip(details)
                item.setData(DETAIL_ROLE, details)
                if process is None:
                    item.setForeground(QColor("#777"))
                self.node_table.setItem(index, column, item)

            if metrics:
                self.node_table.item(index, 2).setBackground(
                    cpu_color(metrics.cpu_percent)
                )

        self.node_table.setSortingEnabled(sorting)
        if sorting and self.node_table.horizontalHeader().sortIndicatorSection() < 0:
            self.node_table.sortItems(2, Qt.DescendingOrder)

    def render_process_table(self) -> None:
        visible_matches = []
        for match in self.last_process_matches:
            metrics = self.last_metrics.get(match.process.pid)
            if not self.filter_matches(
                (
                    match.process.executable_hint,
                    match.process.pid,
                    match.node_name or "",
                    match.process.command,
                )
            ):
                continue
            visible_matches.append((match, metrics))

        sorting = self.process_table.isSortingEnabled()
        self.process_table.setSortingEnabled(False)
        self.process_table.setRowCount(len(visible_matches))

        for index, (match, metrics) in enumerate(visible_matches):
            process = match.process
            node_name = match.node_name or "—"
            details = (
                f"Process: {process.executable_hint}\n"
                f"PID: {process.pid}\n"
                f"Inferred node: {node_name}\n"
                f"Mapping: {match.method}\n"
                f"Status: {metrics.status if metrics else 'unavailable'}\n"
                f"Command: {process.command}"
            )
            command_display = process.command
            if len(command_display) > 120:
                command_display = command_display[:117] + "…"

            values: list[QTableWidgetItem] = [
                QTableWidgetItem(process.executable_hint),
                NumericItem(str(process.pid), process.pid),
                NumericItem(
                    f"{metrics.cpu_percent:.1f}" if metrics else "—",
                    metrics.cpu_percent if metrics else None,
                ),
                NumericItem(
                    f"{mib(metrics.rss_bytes):.1f}" if metrics else "—",
                    mib(metrics.rss_bytes) if metrics else None,
                ),
                NumericItem(
                    f"{metrics.memory_percent:.2f}" if metrics else "—",
                    metrics.memory_percent if metrics else None,
                ),
                NumericItem(
                    str(metrics.thread_count) if metrics else "—",
                    metrics.thread_count if metrics else None,
                ),
                QTableWidgetItem(node_name),
                QTableWidgetItem(command_display),
            ]
            for column, item in enumerate(values):
                item.setToolTip(details)
                item.setData(DETAIL_ROLE, details)
                self.process_table.setItem(index, column, item)
            if metrics:
                self.process_table.item(index, 2).setBackground(
                    cpu_color(metrics.cpu_percent)
                )

        self.process_table.setSortingEnabled(sorting)

    def show_node_detail(self, row: int, _column: int, *_args: int) -> None:
        self.show_table_detail(self.node_table, row)

    def show_process_detail(self, row: int, _column: int, *_args: int) -> None:
        self.show_table_detail(self.process_table, row)

    def show_table_detail(self, table: QTableWidget, row: int) -> None:
        if row < 0:
            return
        item = table.item(row, 0)
        if item is not None:
            self.detail.setPlainText(str(item.data(DETAIL_ROLE) or ""))

    def update_detail_for_current_tab(self) -> None:
        table = self.node_table if self.tabs.currentIndex() == 0 else self.process_table
        self.show_table_detail(table, table.currentRow())


def default_csv_output_path() -> Path:
    repository_root = Path(__file__).resolve().parents[2]
    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    return (
        repository_root
        / "log"
        / "node_cpu_monitor"
        / f"node_cpu_metrics_{timestamp}.csv"
    )


def csv_fieldnames(
    core_indices: Sequence[int],
    top_count: int = CSV_TOP_NODE_COUNT,
) -> list[str]:
    fields = ["timestamp_iso", "elapsed_s", "sample", "cpu_total_percent"]
    for index in core_indices:
        fields.extend((f"cpu{index}_percent", f"cpu{index}_freq_hz"))
    for rank in range(1, top_count + 1):
        prefix = f"top{rank:02d}"
        fields.extend(
            (
                f"{prefix}_node",
                f"{prefix}_process",
                f"{prefix}_pid",
                f"{prefix}_cpu_percent",
            )
        )
    return fields


def ranked_processes(
    matches: Sequence[ProcessNodeMatch],
    metrics: dict[int, ProcessMetrics],
) -> list[tuple[ProcessNodeMatch, ProcessMetrics]]:
    populated = [
        (match, metrics[match.process.pid])
        for match in matches
        if match.process.pid in metrics
    ]
    populated.sort(
        key=lambda item: (
            -item[1].cpu_percent,
            item[0].node_name or item[0].process.executable_hint,
            item[0].process.pid,
        )
    )
    return populated


def csv_row(
    timestamp: datetime,
    elapsed_seconds: float,
    sample_number: int,
    system_cpu: SystemCpuMetrics,
    matches: Sequence[ProcessNodeMatch],
    metrics: dict[int, ProcessMetrics],
    top_count: int = CSV_TOP_NODE_COUNT,
) -> dict[str, object]:
    row: dict[str, object] = {
        "timestamp_iso": timestamp.isoformat(timespec="milliseconds"),
        "elapsed_s": f"{elapsed_seconds:.3f}",
        "sample": sample_number,
        "cpu_total_percent": f"{system_cpu.cpu_percent:.1f}",
    }
    for core in system_cpu.cores:
        row[f"cpu{core.index}_percent"] = (
            f"{core.cpu_percent:.1f}" if core.online else ""
        )
        row[f"cpu{core.index}_freq_hz"] = (
            core.frequency_hz
            if core.online and core.frequency_hz is not None
            else ""
        )

    ranked = ranked_processes(matches, metrics)
    for rank in range(1, top_count + 1):
        prefix = f"top{rank:02d}"
        if rank <= len(ranked):
            match, process_metrics = ranked[rank - 1]
            row[f"{prefix}_node"] = (
                match.node_name or match.process.executable_hint
            )
            row[f"{prefix}_process"] = match.process.executable_hint
            row[f"{prefix}_pid"] = match.process.pid
            row[f"{prefix}_cpu_percent"] = (
                f"{process_metrics.cpu_percent:.1f}"
            )
        else:
            row[f"{prefix}_node"] = ""
            row[f"{prefix}_process"] = ""
            row[f"{prefix}_pid"] = ""
            row[f"{prefix}_cpu_percent"] = ""
    return row


class CsvRecorder:
    def __init__(
        self,
        output_path: Path,
        core_indices: Sequence[int],
        top_count: int = CSV_TOP_NODE_COUNT,
    ) -> None:
        self.output_path = output_path
        self.top_count = top_count
        try:
            output_path.parent.mkdir(parents=True, exist_ok=True)
            self._stream: TextIO = output_path.open(
                "w",
                encoding="utf-8",
                newline="",
            )
        except OSError as error:
            raise SystemExit(
                f"Cannot open CSV output '{output_path}': {error}"
            ) from error
        self._writer = csv.DictWriter(
            self._stream,
            fieldnames=csv_fieldnames(core_indices, top_count),
        )
        self._writer.writeheader()
        self._stream.flush()

    def write(
        self,
        timestamp: datetime,
        elapsed_seconds: float,
        sample_number: int,
        system_cpu: SystemCpuMetrics,
        matches: Sequence[ProcessNodeMatch],
        metrics: dict[int, ProcessMetrics],
    ) -> None:
        self._writer.writerow(
            csv_row(
                timestamp,
                elapsed_seconds,
                sample_number,
                system_cpu,
                matches,
                metrics,
                self.top_count,
            )
        )
        self._stream.flush()

    def close(self) -> None:
        self._stream.close()


def wait_with_graph_pump(graph: RosGraphReader, seconds: float) -> None:
    """Wait without starving ROS graph discovery."""

    deadline = time.monotonic() + max(0.0, seconds)
    while time.monotonic() < deadline:
        graph.pump()
        remaining = deadline - time.monotonic()
        time.sleep(min(0.05, max(0.0, remaining)))


def sample_console_rows(
    graph: RosGraphReader,
    sampler: ProcessSampler,
    show_hidden: bool,
) -> tuple[
    list[NodeRow],
    list[ProcessNodeMatch],
    dict[int, ProcessMetrics],
]:
    records = discover_processes()
    rows, matches = match_processes_to_nodes(
        graph.node_names(show_hidden),
        records,
    )
    metrics = sampler.sample(match.process for match in matches)
    populated_rows = [
        NodeRow(
            node_name=row.node_name,
            process=row.process,
            metrics=metrics.get(row.process.pid) if row.process else None,
            mapping=row.mapping,
        )
        for row in rows
    ]
    populated_rows.sort(
        key=lambda row: row.metrics.cpu_percent if row.metrics else -1.0,
        reverse=True,
    )
    return populated_rows, matches, metrics


def print_console_snapshot(
    rows: Sequence[NodeRow],
    matches: Sequence[ProcessNodeMatch],
    metrics: dict[int, ProcessMetrics],
    system_cpu: SystemCpuMetrics,
    refresh_seconds: Optional[float] = None,
    clear_screen: bool = False,
    csv_output_path: Optional[Path] = None,
) -> None:
    if clear_screen and sys.stdout.isatty():
        print("\033[2J\033[H", end="")

    mapped_rows = [row for row in rows if row.process is not None]
    unique_mapped_pids = {
        row.process.pid for row in mapped_rows if row.process is not None
    }
    mapped_cpu = sum(
        metrics[pid].cpu_percent
        for pid in unique_mapped_pids
        if pid in metrics
    )
    mapped_rss = sum(
        metrics[pid].rss_bytes
        for pid in unique_mapped_pids
        if pid in metrics
    )
    logical_cores = len(system_cpu.cores)
    interval_text = (
        f" | refresh {refresh_seconds:.1f}s"
        if refresh_seconds is not None
        else ""
    )
    print(
        f"CARKit ROS 2 CPU Monitor | {time.strftime('%Y-%m-%d %H:%M:%S')}"
        f"{interval_text}"
    )
    print(
        f"System CPU: {system_cpu.cpu_percent:5.1f}% across "
        f"{logical_cores} logical cores | "
        f"mapped-process CPU: {mapped_cpu:5.1f}% | "
        f"mapped RSS: {mib(mapped_rss):.1f} MiB | "
        f"nodes: {len(rows)} ({len(mapped_rows)} mapped)"
    )
    if csv_output_path is not None:
        print(f"Recording CSV: {csv_output_path}")
    print("CPU% may exceed 100 for a process using multiple logical cores.")

    print()
    print(f"{'CORE':>6} {'CPU%':>7} {'FREQUENCY':>14} {'STATE':>9}")
    print("-" * 41)
    for core in system_cpu.cores:
        frequency = (
            f"{core.frequency_hz / 1_000_000.0:.1f} MHz"
            if core.frequency_hz is not None
            else "unavailable"
        )
        state = "online" if core.online else "offline"
        cpu_percent = f"{core.cpu_percent:.1f}" if core.online else "—"
        print(
            f"{('CPU' + str(core.index)):>6} "
            f"{cpu_percent:>7} {frequency:>14} {state:>9}"
        )

    process_rows = [
        (match, metrics.get(match.process.pid))
        for match in matches
    ]
    process_rows.sort(
        key=lambda item: item[1].cpu_percent if item[1] else -1.0,
        reverse=True,
    )
    print()
    print(
        f"{'ROS PROCESS':24} {'PID':>7} {'CPU%':>7} {'RSS MiB':>9} "
        f"{'THR':>5}  {'INFERRED ROS NODE':32} MAPPING"
    )
    print("-" * 112)
    for match, process_metrics in process_rows:
        cpu = f"{process_metrics.cpu_percent:.1f}" if process_metrics else "—"
        rss = (
            f"{mib(process_metrics.rss_bytes):.1f}"
            if process_metrics
            else "—"
        )
        threads = str(process_metrics.thread_count) if process_metrics else "—"
        node_name = match.node_name or "—"
        print(
            f"{match.process.executable_hint[:24]:24} "
            f"{match.process.pid:>7} {cpu:>7} {rss:>9} {threads:>5}  "
            f"{node_name[:32]:32} {match.method}"
        )

    unmapped_nodes = [row.node_name for row in rows if row.process is None]
    if unmapped_nodes:
        print()
        print(
            "Graph nodes without a unique local PID "
            f"({len(unmapped_nodes)}; often composed nodes):"
        )
        print("  " + ", ".join(unmapped_nodes))
    sys.stdout.flush()


def prime_console_sampler(
    graph: RosGraphReader,
    sampler: ProcessSampler,
    system_sampler: SystemCpuSampler,
    show_hidden: bool,
) -> None:
    wait_with_graph_pump(graph, 0.5)
    sample_console_rows(graph, sampler, show_hidden)
    system_sampler.prime()


def collect_console_snapshot(
    graph: RosGraphReader,
    refresh_seconds: float,
    show_hidden: bool,
) -> int:
    """Print one interval-based sample for SSH diagnostics."""

    sampler = ProcessSampler()
    system_sampler = SystemCpuSampler()
    prime_console_sampler(graph, sampler, system_sampler, show_hidden)
    wait_with_graph_pump(graph, max(0.2, refresh_seconds))
    rows, matches, metrics = sample_console_rows(
        graph,
        sampler,
        show_hidden,
    )
    print_console_snapshot(
        rows,
        matches,
        metrics,
        system_sampler.sample(),
    )
    return 0


def run_terminal_monitor(
    graph: RosGraphReader,
    refresh_seconds: float,
    show_hidden: bool,
    clear_screen: bool,
    csv_output_path: Optional[Path],
) -> int:
    """Continuously refresh process CPU usage in an SSH terminal."""

    sampler = ProcessSampler()
    system_sampler = SystemCpuSampler()
    recorder = (
        CsvRecorder(csv_output_path, system_sampler.core_indices)
        if csv_output_path is not None
        else None
    )
    start_time = time.monotonic()
    next_sample_time = start_time + refresh_seconds
    sample_number = 0
    try:
        print(f"Collecting the first {refresh_seconds:.1f}s CPU sample…")
        prime_console_sampler(
            graph,
            sampler,
            system_sampler,
            show_hidden,
        )
        while True:
            delay = next_sample_time - time.monotonic()
            if delay > 0:
                wait_with_graph_pump(graph, delay)
            sample_number += 1
            rows, matches, metrics = sample_console_rows(
                graph,
                sampler,
                show_hidden,
            )
            system_cpu = system_sampler.sample()
            sample_time = time.monotonic()
            timestamp = datetime.now().astimezone()
            print_console_snapshot(
                rows,
                matches,
                metrics,
                system_cpu,
                refresh_seconds=refresh_seconds,
                clear_screen=clear_screen,
                csv_output_path=csv_output_path,
            )
            if recorder is not None:
                recorder.write(
                    timestamp,
                    sample_time - start_time,
                    sample_number,
                    system_cpu,
                    matches,
                    metrics,
                )

            next_sample_time += refresh_seconds
            if next_sample_time <= time.monotonic():
                next_sample_time = time.monotonic() + refresh_seconds
    except KeyboardInterrupt:
        print("\nCPU monitor stopped.")
        if csv_output_path is not None:
            print(f"CSV saved to: {csv_output_path}")
        return 0
    except Exception:
        if rclpy.ok():
            raise
        print("\nCPU monitor stopped after ROS shutdown.")
        if csv_output_path is not None:
            print(f"CSV saved to: {csv_output_path}")
        return 0
    finally:
        if recorder is not None:
            recorder.close()


def run_self_test() -> int:
    native = (
        "/opt/ros/humble/lib/example/controller_server",
        "--ros-args",
        "-r",
        "__node:=controller",
        "-r",
        "__ns:=/car",
    )
    assert remap_value(native, "__node") == "controller"
    assert remap_value(native, "__ns") == "/car"
    assert executable_hint(native, "controller_server") == "controller_server"
    assert normalized_node_name("controller", "/car") == "/car/controller"

    python_command = (
        "/usr/bin/python3",
        "/workspaces/CARKit/install/pkg/lib/pkg/control_center_node",
        "--ros-args",
        "-r",
        "__node:=control_center_node",
    )
    record = ProcessRecord(
        pid=123,
        create_time=1.0,
        process_name="python3",
        executable_hint=executable_hint(python_command, "python3"),
        cmdline=python_command,
        explicit_node_name=remap_value(python_command, "__node"),
        explicit_namespace=None,
        ros_candidate=True,
    )
    rows, matches = match_processes_to_nodes(
        ["/control_center_node", "/unmapped_component"],
        [record],
    )
    assert matches[0].node_name == "/control_center_node"
    assert any(row.process and row.process.pid == 123 for row in rows)
    assert any(
        row.node_name == "/unmapped_component" and not row.process
        for row in rows
    )
    sample_metrics = {
        123: ProcessMetrics(
            cpu_percent=25.0,
            memory_percent=1.0,
            rss_bytes=1024,
            thread_count=2,
            status="running",
        )
    }
    sample_system = SystemCpuMetrics(
        cpu_percent=12.5,
        cores=(CpuCoreMetrics(0, 15.0, 1_728_000_000, True),),
    )
    output_row = csv_row(
        datetime.fromisoformat("2026-07-23T12:00:00-04:00"),
        1.0,
        1,
        sample_system,
        matches,
        sample_metrics,
    )
    assert output_row["cpu0_percent"] == "15.0"
    assert output_row["cpu0_freq_hz"] == 1_728_000_000
    assert output_row["top01_node"] == "/control_center_node"
    assert output_row["top01_cpu_percent"] == "25.0"
    assert len(csv_fieldnames((0,))) == 4 + 2 + 4 * CSV_TOP_NODE_COUNT
    print("node_cpu_monitor self-test passed")
    return 0


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Show CPU and memory usage for local ROS 2 node processes."
    )
    parser.add_argument(
        "--refresh-ms",
        type=int,
        default=1000,
        help="Sampling/refresh period in milliseconds (default: 1000)",
    )
    output_mode = parser.add_mutually_exclusive_group()
    output_mode.add_argument(
        "--terminal",
        action="store_true",
        help="Continuously refresh a CPU table in the terminal",
    )
    output_mode.add_argument(
        "--once",
        action="store_true",
        help="Print one sampled table instead of opening the GUI",
    )
    parser.add_argument(
        "--show-hidden",
        action="store_true",
        help="Include hidden ROS nodes in terminal output",
    )
    parser.add_argument(
        "--no-clear",
        action="store_true",
        help="Append terminal samples instead of clearing the screen",
    )
    parser.add_argument(
        "--csv",
        action="store_true",
        help=(
            "Record terminal samples to CSV; defaults to "
            "log/node_cpu_monitor/node_cpu_metrics_<timestamp>.csv"
        ),
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="CSV output path used with --terminal --csv",
    )
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="Run parser and mapping tests without joining the ROS graph",
    )
    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()
    if arguments.self_test:
        return run_self_test()

    if arguments.refresh_ms < 200:
        raise SystemExit("--refresh-ms must be at least 200")
    if arguments.csv and not arguments.terminal:
        raise SystemExit("--csv requires --terminal")
    if arguments.output is not None and not arguments.csv:
        raise SystemExit("--output requires --csv")

    csv_output_path = (
        arguments.output or default_csv_output_path()
        if arguments.csv
        else None
    )

    rclpy.init(args=[])
    graph = RosGraphReader()
    try:
        if arguments.terminal:
            return run_terminal_monitor(
                graph,
                arguments.refresh_ms / 1000.0,
                arguments.show_hidden,
                clear_screen=not arguments.no_clear,
                csv_output_path=csv_output_path,
            )

        if arguments.once:
            return collect_console_snapshot(
                graph,
                arguments.refresh_ms / 1000.0,
                arguments.show_hidden,
            )

        if GUI_IMPORT_ERROR is not None or QApplication is None:
            raise SystemExit(
                "PyQt5 is required for GUI mode. Use --terminal over SSH."
            )
        application = QApplication([sys.argv[0]])
        application.setApplicationName("CARKit Node CPU Monitor")
        window = NodeCpuMonitorWindow(graph, arguments.refresh_ms)
        window.show()
        return int(application.exec_())
    finally:
        graph.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
