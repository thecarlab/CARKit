"""Collect ROS 2 node resource usage from the carkit Docker container."""

from __future__ import annotations

import json
import re
import shutil
import subprocess
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


NODE_ARG_RE = re.compile(r"__node:=([^\s]+)")
WRAPPER_MARKERS = (
    "ros2 launch ",
    "ros2 run ",
    "/opt/ros/humble/bin/ros2 run ",
    "/opt/ros/humble/bin/ros2 launch ",
    "timeout ",
)

METRICS_SCRIPT = """
source /opt/ros/humble/setup.bash
if [ -f /workspaces/CARKit/install/setup.bash ]; then
  source /workspaces/CARKit/install/setup.bash
fi
ros2 node list 2>/dev/null || true
echo "===SECTION==="
ps -eo pid=,pcpu=,pmem=,rss=,vsz=,args= --no-headers 2>/dev/null | grep -Ev 'ps -eo|grep -Ev' || true
echo "===SECTION==="
grep -E 'MemTotal|MemAvailable' /proc/meminfo
echo "===SECTION==="
awk '{print $1,$2,$3}' /proc/loadavg
echo "===SECTION==="
nproc
"""


@dataclass
class ProcessMetrics:
    pid: int
    cpu_percent: float
    memory_percent: float
    rss_mb: float
    vsz_mb: float
    command: str
    node_name: str | None = None
    role: str = "process"


@dataclass
class Snapshot:
    timestamp: str
    container_name: str
    container_running: bool
    container_cpu_percent: float | None
    container_memory_used_mb: float | None
    container_memory_limit_mb: float | None
    container_memory_percent: float | None
    system_cpus: int
    system_mem_total_mb: float
    system_mem_available_mb: float
    system_load_avg: list[float]
    ros2_nodes: list[str] = field(default_factory=list)
    nodes: list[ProcessMetrics] = field(default_factory=list)
    launch_processes: list[ProcessMetrics] = field(default_factory=list)
    other_processes: list[ProcessMetrics] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _run(cmd: list[str], timeout: float = 15.0) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        cmd,
        capture_output=True,
        text=True,
        timeout=timeout,
        check=False,
    )


def _docker_available() -> bool:
    return shutil.which("docker") is not None


def _inside_container() -> bool:
    return Path("/.dockerenv").exists()


def _use_local_collection() -> bool:
    return _inside_container() or not _docker_available()


def _read_cgroup_memory_mb() -> tuple[float | None, float | None, float | None]:
    cgroup = Path("/sys/fs/cgroup")
    current_path = cgroup / "memory.current"
    max_path = cgroup / "memory.max"
    if not current_path.is_file():
        return None, None, None

    try:
        used_bytes = int(current_path.read_text().strip())
        max_raw = max_path.read_text().strip()
        limit_bytes = int(max_raw) if max_raw.isdigit() else None
    except (OSError, ValueError):
        return None, None, None

    used_mb = round(used_bytes / (1024 * 1024), 1)
    if limit_bytes is None:
        return used_mb, None, None

    limit_mb = round(limit_bytes / (1024 * 1024), 1)
    percent = round(used_mb / limit_mb * 100, 2) if limit_mb else None
    return used_mb, limit_mb, percent


def _run_metrics_script() -> subprocess.CompletedProcess[str]:
    return _run(["bash", "-lc", METRICS_SCRIPT])


def _parse_meminfo_kb(value: str) -> float:
    return float(value.split()[0])


def _parse_docker_memory(value: str) -> tuple[float | None, float | None]:
    if not value or "/" not in value:
        return None, None
    used_raw, limit_raw = (part.strip() for part in value.split("/", 1))

    def to_mb(raw: str) -> float | None:
        raw = raw.strip()
        for suffix, scale in (("GiB", 1024), ("MiB", 1), ("KiB", 1 / 1024), ("B", 1 / (1024 * 1024))):
            if raw.endswith(suffix):
                try:
                    return float(raw[: -len(suffix)]) * scale
                except ValueError:
                    return None
        return None

    return to_mb(used_raw), to_mb(limit_raw)


def _is_wrapper(command: str) -> bool:
    return any(marker in command for marker in WRAPPER_MARKERS)


def _basename(command: str) -> str:
    first = command.split()[0]
    return first.rsplit("/", 1)[-1]


def _extract_node_from_command(command: str) -> str | None:
    match = NODE_ARG_RE.search(command)
    if match:
        return f"/{match.group(1)}"
    return None


def _map_nodes(
    ros2_nodes: list[str],
    processes: list[ProcessMetrics],
) -> tuple[list[ProcessMetrics], list[ProcessMetrics], list[ProcessMetrics]]:
    node_set = set(ros2_nodes)
    by_node: dict[str, ProcessMetrics] = {}
    launch_processes: list[ProcessMetrics] = []
    other_processes: list[ProcessMetrics] = []

    for proc in processes:
        node_name = _extract_node_from_command(proc.command)
        if node_name:
            proc.node_name = node_name
            proc.role = "node"
            if node_name not in by_node or proc.rss_mb > by_node[node_name].rss_mb:
                by_node[node_name] = proc
            continue

        if "ros2 launch " in proc.command:
            proc.role = "launch"
            launch_processes.append(proc)
            continue

        if _is_wrapper(proc.command):
            proc.role = "wrapper"
            other_processes.append(proc)
            continue

        basename = _basename(proc.command)
        candidate = f"/{basename}"
        if candidate in node_set and candidate not in by_node:
            proc.node_name = candidate
            proc.role = "node"
            by_node[candidate] = proc
            continue

        if "ros2-daemon" in proc.command:
            proc.node_name = "/ros2_daemon"
            proc.role = "daemon"
            by_node["/ros2_daemon"] = proc
            continue

        proc.role = "other"
        other_processes.append(proc)

    mapped_nodes = list(by_node.values())

    for node_name in ros2_nodes:
        if node_name not in by_node:
            mapped_nodes.append(
                ProcessMetrics(
                    pid=0,
                    cpu_percent=0.0,
                    memory_percent=0.0,
                    rss_mb=0.0,
                    vsz_mb=0.0,
                    command="(process not found)",
                    node_name=node_name,
                    role="node",
                )
            )

    mapped_nodes.sort(key=lambda item: item.cpu_percent, reverse=True)
    launch_processes.sort(key=lambda item: item.cpu_percent, reverse=True)
    other_processes.sort(key=lambda item: item.cpu_percent, reverse=True)
    return mapped_nodes, launch_processes, other_processes


def _parse_remote_output(output: str) -> tuple[list[str], list[ProcessMetrics], dict[str, float], list[float], int]:
    sections = output.split("===SECTION===")
    if len(sections) != 5:
        raise ValueError("Unexpected collector output from container.")

    nodes_raw, processes_raw, meminfo_raw, load_raw, nproc_raw = sections

    ros2_nodes = [line.strip() for line in nodes_raw.splitlines() if line.strip().startswith("/")]

    processes: list[ProcessMetrics] = []
    for line in processes_raw.splitlines():
        line = line.strip()
        if not line:
            continue
        parts = line.split(None, 5)
        if len(parts) < 6:
            continue
        pid, cpu, mem, rss, vsz, command = parts
        processes.append(
            ProcessMetrics(
                pid=int(pid),
                cpu_percent=float(cpu),
                memory_percent=float(mem),
                rss_mb=round(int(rss) / 1024, 1),
                vsz_mb=round(int(vsz) / 1024, 1),
                command=command,
            )
        )

    meminfo: dict[str, float] = {}
    for line in meminfo_raw.splitlines():
        if ":" in line:
            key, value = line.split(":", 1)
            meminfo[key.strip()] = _parse_meminfo_kb(value.strip())

    load_avg = [float(part) for part in load_raw.strip().split()[:3]]
    cpus = int(nproc_raw.strip() or "1")
    return ros2_nodes, processes, meminfo, load_avg, cpus


def _build_snapshot(
    *,
    container_name: str,
    running: bool,
    container_cpu_percent: float | None,
    container_memory_used_mb: float | None,
    container_memory_limit_mb: float | None,
    container_memory_percent: float | None,
    ros2_nodes: list[str],
    processes: list[ProcessMetrics],
    meminfo: dict[str, float],
    load_avg: list[float],
    cpus: int,
    errors: list[str],
) -> Snapshot:
    nodes, launch_processes, other_processes = _map_nodes(ros2_nodes, processes)
    if container_cpu_percent is None and processes:
        container_cpu_percent = round(sum(proc.cpu_percent for proc in processes), 1)

    return Snapshot(
        timestamp=datetime.now(timezone.utc).isoformat(),
        container_name=container_name,
        container_running=running,
        container_cpu_percent=container_cpu_percent,
        container_memory_used_mb=container_memory_used_mb,
        container_memory_limit_mb=container_memory_limit_mb,
        container_memory_percent=container_memory_percent,
        system_cpus=cpus,
        system_mem_total_mb=round(meminfo.get("MemTotal", 0) / 1024, 1),
        system_mem_available_mb=round(meminfo.get("MemAvailable", 0) / 1024, 1),
        system_load_avg=load_avg,
        ros2_nodes=ros2_nodes,
        nodes=nodes,
        launch_processes=launch_processes,
        other_processes=other_processes,
        errors=errors,
    )


def _collect_local(container_name: str) -> Snapshot:
    errors: list[str] = []
    metrics = _run_metrics_script()
    if metrics.returncode != 0:
        errors.append(metrics.stderr.strip() or "Failed to collect local metrics.")
        return _build_snapshot(
            container_name=container_name,
            running=True,
            container_cpu_percent=None,
            container_memory_used_mb=None,
            container_memory_limit_mb=None,
            container_memory_percent=None,
            ros2_nodes=[],
            processes=[],
            meminfo={},
            load_avg=[0.0, 0.0, 0.0],
            cpus=0,
            errors=errors,
        )

    try:
        ros2_nodes, processes, meminfo, load_avg, cpus = _parse_remote_output(metrics.stdout)
    except ValueError as exc:
        errors.append(str(exc))
        ros2_nodes, processes, meminfo, load_avg, cpus = [], [], {}, [0.0, 0.0, 0.0], 0

    mem_total_mb = meminfo.get("MemTotal", 0) / 1024
    mem_available_mb = meminfo.get("MemAvailable", 0) / 1024
    cgroup_used_mb, cgroup_limit_mb, cgroup_percent = _read_cgroup_memory_mb()

    container_memory_used_mb = (
        cgroup_used_mb if cgroup_used_mb is not None else round(mem_total_mb - mem_available_mb, 1)
    )
    container_memory_limit_mb = cgroup_limit_mb if cgroup_limit_mb is not None else round(mem_total_mb, 1)
    if cgroup_percent is not None:
        container_memory_percent = cgroup_percent
    elif container_memory_limit_mb:
        container_memory_percent = round(container_memory_used_mb / container_memory_limit_mb * 100, 2)
    else:
        container_memory_percent = None

    return _build_snapshot(
        container_name=container_name,
        running=True,
        container_cpu_percent=None,
        container_memory_used_mb=container_memory_used_mb,
        container_memory_limit_mb=container_memory_limit_mb,
        container_memory_percent=container_memory_percent,
        ros2_nodes=ros2_nodes,
        processes=processes,
        meminfo=meminfo,
        load_avg=load_avg,
        cpus=cpus,
        errors=errors,
    )


def _collect_via_docker(container_name: str) -> Snapshot:
    errors: list[str] = []
    now = datetime.now(timezone.utc).isoformat()

    inspect = _run(["docker", "inspect", "-f", "{{.State.Running}}", container_name])
    running = inspect.stdout.strip() == "true"
    if inspect.returncode != 0:
        errors.append(inspect.stderr.strip() or f"Container '{container_name}' not found.")
        return Snapshot(
            timestamp=now,
            container_name=container_name,
            container_running=False,
            container_cpu_percent=None,
            container_memory_used_mb=None,
            container_memory_limit_mb=None,
            container_memory_percent=None,
            system_cpus=0,
            system_mem_total_mb=0.0,
            system_mem_available_mb=0.0,
            system_load_avg=[0.0, 0.0, 0.0],
            errors=errors,
        )

    container_cpu_percent: float | None = None
    container_memory_used_mb: float | None = None
    container_memory_limit_mb: float | None = None
    container_memory_percent: float | None = None

    stats = _run(
        [
            "docker",
            "stats",
            container_name,
            "--no-stream",
            "--format",
            "{{.CPUPerc}}|{{.MemUsage}}|{{.MemPerc}}",
        ]
    )
    if stats.returncode == 0 and stats.stdout.strip():
        cpu_raw, mem_usage_raw, mem_pct_raw = (stats.stdout.strip().split("|") + ["", "", ""])[:3]
        try:
            container_cpu_percent = float(cpu_raw.replace("%", ""))
        except ValueError:
            errors.append("Failed to parse container CPU usage.")
        container_memory_used_mb, container_memory_limit_mb = _parse_docker_memory(mem_usage_raw)
        try:
            container_memory_percent = float(mem_pct_raw.replace("%", ""))
        except ValueError:
            pass
    else:
        errors.append(stats.stderr.strip() or "Failed to read docker stats.")

    remote = _run(["docker", "exec", container_name, "bash", "-lc", METRICS_SCRIPT])
    if remote.returncode != 0:
        errors.append(remote.stderr.strip() or "Failed to collect metrics inside container.")
        return Snapshot(
            timestamp=now,
            container_name=container_name,
            container_running=running,
            container_cpu_percent=container_cpu_percent,
            container_memory_used_mb=container_memory_used_mb,
            container_memory_limit_mb=container_memory_limit_mb,
            container_memory_percent=container_memory_percent,
            system_cpus=0,
            system_mem_total_mb=0.0,
            system_mem_available_mb=0.0,
            system_load_avg=[0.0, 0.0, 0.0],
            errors=errors,
        )

    try:
        ros2_nodes, processes, meminfo, load_avg, cpus = _parse_remote_output(remote.stdout)
    except ValueError as exc:
        errors.append(str(exc))
        ros2_nodes, processes, meminfo, load_avg, cpus = [], [], {}, [0.0, 0.0, 0.0], 0

    return _build_snapshot(
        container_name=container_name,
        running=running,
        container_cpu_percent=container_cpu_percent,
        container_memory_used_mb=container_memory_used_mb,
        container_memory_limit_mb=container_memory_limit_mb,
        container_memory_percent=container_memory_percent,
        ros2_nodes=ros2_nodes,
        processes=processes,
        meminfo=meminfo,
        load_avg=load_avg,
        cpus=cpus,
        errors=errors,
    )


def collect_snapshot(container_name: str = "carkit") -> Snapshot:
    if _use_local_collection():
        return _collect_local(container_name)
    return _collect_via_docker(container_name)


def snapshot_to_json(container_name: str = "carkit") -> str:
    return json.dumps(collect_snapshot(container_name).to_dict())
