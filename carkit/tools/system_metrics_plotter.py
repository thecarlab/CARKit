#!/usr/bin/env python3
"""Interactively plot and export CARKit pipeline-monitor CSV files.

The tool is deliberately independent of ROS.  It uses the Python standard
library, NumPy, Matplotlib, and (for GUI mode only) Tkinter, all of which are
already present in the CARKit Jetson image.  It accepts system, per-process
CPU, topic-rate, and pipeline-event monitor CSV schemas.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass, replace
from datetime import datetime, tzinfo
import json
import math
from pathlib import Path
import re
import sys
from typing import Any, Optional, Sequence

import matplotlib as mpl
import matplotlib.dates as mdates
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
from matplotlib.widgets import RangeSlider
import numpy as np


SECONDS_PER_TIME_UNIT = {
    "seconds": 1.0,
    "minutes": 60.0,
    "hours": 3600.0,
}
FIGURE_WIDTHS_INCHES = {
    "single": 3.5,
    "double": 7.2,
}
PAPER_COLORS = (
    "#0072B2",
    "#D55E00",
    "#009E73",
    "#CC79A7",
    "#E69F00",
    "#56B4E9",
    "#000000",
)
PAPER_LINESTYLES = (
    "-",
    "--",
    "-.",
    ":",
    (0, (5, 1)),
    (0, (3, 1, 1, 1)),
    (0, (1, 1)),
)
GROUP_ORDER = (
    "CPU",
    "ROS Processes",
    "Topic Hz",
    "Topic Counts",
    "Topic Age",
    "Temperature",
    "VDD_IN",
    "VDD_CPU_GPU_CV",
    "VDD_SOC",
    "Other",
)
DEFAULT_METRICS = (
    "cpu_total_percent",
    "vdd_in_voltage_v",
    "vdd_in_power_w",
    "vdd_cpu_gpu_cv_power_w",
    "cpu_mean_freq_mhz",
    "temp_cpu_thermal_c",
)
IGNORED_COLUMNS = {"timestamp_iso", "timestamp", "elapsed_s", "sample"}
RECIPE_VERSION = 1
FIGURE_SUFFIXES = (".pdf", ".svg", ".png")
DEFAULT_EXPORT_DIRECTORY = Path(__file__).resolve().parents[2] / "data"
TOP_SLOT_COLUMN_RE = re.compile(
    r"top(\d+)_(node|process|pid|cpu_percent)"
)
CSV_KIND_LABELS = {
    "system": "System",
    "node_cpu": "Node CPU",
    "topic_rate": "Topic Rate",
    "pipeline_event": "Pipeline Events",
}
EVENT_REQUIRED_COLUMNS = {
    "event_id",
    "event",
    "source_topic",
    "auto_session",
    "route_session",
    "details",
}
EVENT_COLORS = (
    "#B2182B",
    "#2166AC",
    "#1B7837",
    "#8C510A",
    "#762A83",
    "#E08214",
    "#008080",
    "#4D4D4D",
)
EVENT_LABEL_LANES = 4
BEHAVIOR_STATE_COLORS = {
    "NORMAL_NAV2": "#1B7837",
    "STOP_SIGN": "#D55E00",
    "TRAFFIC_LIGHT": "#E69F00",
}


class PlotterError(RuntimeError):
    """Expected input or configuration error."""


@dataclass(frozen=True)
class MetricSpec:
    key: str
    group: str
    label_en: str
    label_zh: str
    unit: str
    scale: float = 1.0

    def label(self, language: str) -> str:
        return self.label_zh if language == "zh" else self.label_en

    def axis_label(self, language: str) -> str:
        label = self.label(language)
        return f"{label} ({self.unit})" if self.unit else label


@dataclass
class DataRun:
    path: Path
    name: str
    elapsed_seconds: np.ndarray
    timestamps: tuple[Optional[datetime], ...]
    metrics: dict[str, np.ndarray]
    specs: dict[str, MetricSpec]
    kind: str = "system"
    events: tuple[PipelineEvent, ...] = ()


@dataclass(frozen=True)
class PipelineEvent:
    event_id: str
    event_type: str
    source_topic: str
    auto_session: str
    route_session: str
    details: str


@dataclass(frozen=True)
class PlotConfig:
    time_mode: str = "elapsed"
    time_unit: str = "minutes"
    start: Optional[float] = None
    end: Optional[float] = None
    language: str = "en"
    figure_width: str = "double"
    custom_width: float = 7.2
    dpi: int = 600
    title: str = ""
    event_types: Optional[tuple[str, ...]] = None
    combine_topic_hz: bool = False


@dataclass(frozen=True)
class StatisticsRecord:
    run: str
    metric_key: str
    metric_label: str
    unit: str
    selection_start: str
    selection_end: str
    count: int
    mean: float
    std: float
    minimum: float
    median: float
    p95: float
    maximum: float


@dataclass(frozen=True)
class PlotResult:
    figure: Figure
    data_axes: tuple[Any, ...]
    warnings: tuple[str, ...]


def known_metric_spec(key: str) -> Optional[MetricSpec]:
    if key == "cpu_total_percent":
        return MetricSpec(
            key,
            "CPU",
            "CPU Utilization",
            "CPU占用率",
            "%",
        )
    if key == "cpu_mean_freq_mhz":
        return MetricSpec(
            key,
            "CPU",
            "Mean CPU Frequency",
            "CPU平均频率",
            "MHz",
        )

    utilization_match = re.fullmatch(r"cpu(\d+)_percent", key)
    if utilization_match:
        index = utilization_match.group(1)
        return MetricSpec(
            key,
            "CPU",
            f"CPU {index} Utilization",
            f"CPU {index}占用率",
            "%",
        )

    frequency_match = re.fullmatch(r"cpu(\d+)_freq_hz", key)
    if frequency_match:
        index = frequency_match.group(1)
        return MetricSpec(
            key,
            "CPU",
            f"CPU {index} Frequency",
            f"CPU {index}频率",
            "MHz",
            scale=1e-6,
        )

    temperature_labels = {
        "temp_cpu_thermal_c": ("CPU Temperature", "CPU温度"),
        "temp_cv0_thermal_c": ("CV0 Temperature", "CV0温度"),
        "temp_cv1_thermal_c": ("CV1 Temperature", "CV1温度"),
        "temp_cv2_thermal_c": ("CV2 Temperature", "CV2温度"),
        "temp_gpu_thermal_c": ("GPU Temperature", "GPU温度"),
        "temp_soc0_thermal_c": ("SoC0 Temperature", "SoC0温度"),
        "temp_soc1_thermal_c": ("SoC1 Temperature", "SoC1温度"),
        "temp_soc2_thermal_c": ("SoC2 Temperature", "SoC2温度"),
        "temp_tj_thermal_c": ("Junction Temperature", "结温"),
    }
    if key in temperature_labels:
        label_en, label_zh = temperature_labels[key]
        return MetricSpec(
            key,
            "Temperature",
            label_en,
            label_zh,
            "°C",
        )

    rail_match = re.fullmatch(
        r"(vdd_in|vdd_cpu_gpu_cv|vdd_soc)_"
        r"(voltage_v|current_a|power_w|avg_power_w)",
        key,
    )
    if rail_match:
        rail_key, measurement_key = rail_match.groups()
        rail_names = {
            "vdd_in": ("VDD_IN", "VDD_IN"),
            "vdd_cpu_gpu_cv": (
                "CPU/GPU/CV Combined Rail",
                "CPU/GPU/CV合并电源轨",
            ),
            "vdd_soc": ("VDD_SOC", "VDD_SOC"),
        }
        group_names = {
            "vdd_in": "VDD_IN",
            "vdd_cpu_gpu_cv": "VDD_CPU_GPU_CV",
            "vdd_soc": "VDD_SOC",
        }
        measurements = {
            "voltage_v": ("Voltage", "电压", "V"),
            "current_a": ("Current", "电流", "A"),
            "power_w": ("Power", "功率", "W"),
            "avg_power_w": ("Running-average Power", "运行平均功率", "W"),
        }
        rail_en, rail_zh = rail_names[rail_key]
        measure_en, measure_zh, unit = measurements[measurement_key]
        return MetricSpec(
            key,
            group_names[rail_key],
            f"{rail_en} {measure_en}",
            f"{rail_zh}{measure_zh}",
            unit,
        )
    return None


def parse_iso_timestamp(value: str, context: str) -> datetime:
    normalized = value.strip()
    if normalized.endswith("Z"):
        normalized = normalized[:-1] + "+00:00"
    try:
        return datetime.fromisoformat(normalized)
    except ValueError as error:
        raise PlotterError(
            f"{context}: invalid ISO timestamp {value!r}"
        ) from error


def parse_optional_float(value: Optional[str]) -> float:
    if value is None or value.strip() == "":
        return math.nan
    try:
        parsed = float(value)
    except ValueError:
        return math.nan
    return parsed if math.isfinite(parsed) else math.nan


def csv_kind(fieldnames: Sequence[str]) -> str:
    fields = set(fieldnames)
    if EVENT_REQUIRED_COLUMNS.issubset(fields):
        return "pipeline_event"
    if {
        "signal",
        "source_topic",
        "message_count",
        "hz",
        "total_count",
    }.issubset(fields):
        return "topic_rate"
    if any(TOP_SLOT_COLUMN_RE.fullmatch(field) for field in fieldnames):
        return "node_cpu"
    return "system"


def row_is_blank(row: dict[str, Any]) -> bool:
    for value in row.values():
        values = value if isinstance(value, list) else (value,)
        if any(item is not None and str(item).strip() for item in values):
            return False
    return True


def parse_elapsed(
    row: dict[str, Any],
    resolved: Path,
    line_number: int,
) -> float:
    raw_elapsed = row.get("elapsed_s")
    try:
        elapsed_value = float(raw_elapsed or "")
    except (TypeError, ValueError) as error:
        raise PlotterError(
            f"{resolved}:{line_number}: elapsed_s must be numeric"
        ) from error
    if not math.isfinite(elapsed_value):
        raise PlotterError(
            f"{resolved}:{line_number}: elapsed_s must be finite"
        )
    return elapsed_value


def parse_row_timestamp(
    row: dict[str, Any],
    resolved: Path,
    line_number: int,
) -> Optional[datetime]:
    raw_timestamp = str(
        row.get("timestamp_iso") or row.get("timestamp") or ""
    ).strip()
    if not raw_timestamp:
        return None
    return parse_iso_timestamp(raw_timestamp, f"{resolved}:{line_number}")


def add_mean_cpu_frequency(
    metrics: dict[str, np.ndarray],
    specs: dict[str, MetricSpec],
    sample_count: int,
) -> None:
    frequency_keys = sorted(
        key for key in metrics if re.fullmatch(r"cpu\d+_freq_hz", key)
    )
    if not frequency_keys:
        return
    stacked = np.vstack([metrics[key] for key in frequency_keys])
    finite = np.isfinite(stacked)
    counts = finite.sum(axis=0)
    sums = np.where(finite, stacked, 0.0).sum(axis=0)
    mean_frequency = np.full(sample_count, np.nan, dtype=float)
    np.divide(sums, counts, out=mean_frequency, where=counts > 0)
    derived = known_metric_spec("cpu_mean_freq_mhz")
    assert derived is not None
    metrics[derived.key] = mean_frequency
    specs[derived.key] = derived


def metric_tokens(values: Sequence[str]) -> dict[str, str]:
    """Return deterministic, collision-free metric-key tokens."""
    result: dict[str, str] = {}
    used: set[str] = set()
    for value in sorted(set(values)):
        base = re.sub(r"[^a-z0-9]+", "_", value.lower()).strip("_")
        base = base or "unnamed"
        token = base
        suffix = 2
        while token in used:
            token = f"{base}_{suffix}"
            suffix += 1
        result[value] = token
        used.add(token)
    return result


def default_run_name(resolved: Path, kind: str, name: Optional[str]) -> str:
    if name:
        return name
    return f"{resolved.stem} [{CSV_KIND_LABELS[kind]}]"


def load_wide_csv(
    resolved: Path,
    fieldnames: Sequence[str],
    rows: Sequence[tuple[int, dict[str, Any]]],
    kind: str,
    name: Optional[str],
) -> DataRun:
    ignored = set(IGNORED_COLUMNS)
    if kind == "node_cpu":
        ignored.update(
            field
            for field in fieldnames
            if TOP_SLOT_COLUMN_RE.fullmatch(field)
        )
    raw_columns: dict[str, list[float]] = {
        key: [] for key in fieldnames if key not in ignored
    }
    elapsed: list[float] = []
    timestamps: list[Optional[datetime]] = []

    for line_number, row in rows:
        elapsed_value = parse_elapsed(row, resolved, line_number)
        if elapsed and elapsed_value < elapsed[-1]:
            raise PlotterError(
                f"{resolved}:{line_number}: elapsed_s moved backwards "
                f"from {elapsed[-1]} to {elapsed_value}"
            )
        elapsed.append(elapsed_value)
        timestamps.append(parse_row_timestamp(row, resolved, line_number))
        for key in raw_columns:
            raw_columns[key].append(
                parse_optional_float(
                    None if row.get(key) is None else str(row.get(key))
                )
            )

    metrics: dict[str, np.ndarray] = {}
    specs: dict[str, MetricSpec] = {}
    for key, values in raw_columns.items():
        array = np.asarray(values, dtype=float)
        if not np.isfinite(array).any():
            continue
        spec = known_metric_spec(key)
        if spec is None:
            spec = MetricSpec(key, "Other", key, key, "")
        metrics[key] = array * spec.scale
        specs[key] = spec

    if kind == "node_cpu":
        slot_numbers = sorted(
            {
                int(match.group(1))
                for field in fieldnames
                if (match := TOP_SLOT_COLUMN_RE.fullmatch(field))
            }
        )
        process_names = sorted(
            {
                process
                for _, row in rows
                for slot in slot_numbers
                if (
                    process := str(
                        row.get(f"top{slot:02d}_process") or ""
                    ).strip()
                )
            }
        )
        tokens = metric_tokens(process_names)
        process_values = {
            process: np.full(len(rows), np.nan, dtype=float)
            for process in process_names
        }
        for row_index, (_, row) in enumerate(rows):
            totals: dict[str, float] = {}
            for slot in slot_numbers:
                process = str(
                    row.get(f"top{slot:02d}_process") or ""
                ).strip()
                cpu = parse_optional_float(
                    str(row.get(f"top{slot:02d}_cpu_percent") or "")
                )
                if process and math.isfinite(cpu):
                    totals[process] = totals.get(process, 0.0) + cpu
            for process, cpu in totals.items():
                process_values[process][row_index] = cpu
        for process, values in process_values.items():
            key = f"process_{tokens[process]}_cpu_percent"
            metrics[key] = values
            specs[key] = MetricSpec(
                key,
                "ROS Processes",
                f"{process} CPU",
                f"{process} CPU占用率",
                "%",
            )

    add_mean_cpu_frequency(metrics, specs, len(elapsed))
    if not metrics:
        raise PlotterError(f"{resolved}: CSV has no numeric metric columns")
    return DataRun(
        path=resolved,
        name=default_run_name(resolved, kind, name),
        elapsed_seconds=np.asarray(elapsed, dtype=float),
        timestamps=tuple(timestamps),
        metrics=metrics,
        specs=specs,
        kind=kind,
    )


def load_topic_rate_csv(
    resolved: Path,
    rows: Sequence[tuple[int, dict[str, Any]]],
    name: Optional[str],
) -> DataRun:
    groups: list[dict[str, Any]] = []
    groups_by_sample: dict[str, dict[str, Any]] = {}
    signal_sources: dict[str, str] = {}

    for line_number, row in rows:
        sample = str(row.get("sample") or "").strip()
        if not sample:
            raise PlotterError(
                f"{resolved}:{line_number}: topic-rate sample is missing"
            )
        elapsed_value = parse_elapsed(row, resolved, line_number)
        signal = str(row.get("signal") or "").strip()
        if not signal:
            raise PlotterError(
                f"{resolved}:{line_number}: topic-rate signal is missing"
            )
        group = groups_by_sample.get(sample)
        if group is None:
            group = {
                "sample": sample,
                "elapsed": elapsed_value,
                "timestamp": parse_row_timestamp(
                    row, resolved, line_number
                ),
                "line": line_number,
                "signals": {},
                "interval": parse_optional_float(
                    str(row.get("interval_s") or "")
                ),
            }
            groups_by_sample[sample] = group
            groups.append(group)
        elif not math.isclose(
            elapsed_value,
            float(group["elapsed"]),
            rel_tol=0.0,
            abs_tol=1e-9,
        ):
            raise PlotterError(
                f"{resolved}:{line_number}: sample {sample!r} has "
                "inconsistent elapsed_s values"
            )
        if signal in group["signals"]:
            raise PlotterError(
                f"{resolved}:{line_number}: duplicate signal {signal!r} "
                f"in sample {sample!r}"
            )
        group["signals"][signal] = {
            column: parse_optional_float(str(row.get(column) or ""))
            for column in (
                "hz",
                "message_count",
                "total_count",
                "last_message_age_s",
            )
        }
        signal_sources.setdefault(
            signal, str(row.get("source_topic") or "").strip()
        )

    elapsed = [float(group["elapsed"]) for group in groups]
    for index in range(1, len(elapsed)):
        if elapsed[index] < elapsed[index - 1]:
            line_number = groups[index]["line"]
            raise PlotterError(
                f"{resolved}:{line_number}: elapsed_s moved backwards "
                f"from {elapsed[index - 1]} to {elapsed[index]}"
            )

    signals = sorted(
        {
            signal
            for group in groups
            for signal in group["signals"]
        }
    )
    tokens = metric_tokens(signals)
    metrics: dict[str, np.ndarray] = {}
    specs: dict[str, MetricSpec] = {}
    metric_definitions = (
        ("hz", "Topic Hz", "Rate", "频率", "Hz"),
        (
            "message_count",
            "Topic Counts",
            "Interval Messages",
            "周期消息数",
            "messages",
        ),
        (
            "total_count",
            "Topic Counts",
            "Total Messages",
            "累计消息数",
            "messages",
        ),
        (
            "last_message_age_s",
            "Topic Age",
            "Last-message Age",
            "最后消息年龄",
            "s",
        ),
    )
    for signal in signals:
        source = signal_sources.get(signal, "")
        label_en = f"{signal} ({source})" if source else signal
        label_zh = label_en
        for column, group_name, suffix_en, suffix_zh, unit in metric_definitions:
            values = np.asarray(
                [
                    group["signals"].get(signal, {}).get(column, math.nan)
                    for group in groups
                ],
                dtype=float,
            )
            if not np.isfinite(values).any():
                continue
            key = f"topic_{tokens[signal]}_{column}"
            metrics[key] = values
            specs[key] = MetricSpec(
                key,
                group_name,
                f"{label_en} {suffix_en}",
                f"{label_zh}{suffix_zh}",
                unit,
            )

    intervals = np.asarray(
        [float(group["interval"]) for group in groups],
        dtype=float,
    )
    if np.isfinite(intervals).any():
        key = "topic_sample_interval_s"
        metrics[key] = intervals
        specs[key] = MetricSpec(
            key,
            "Other",
            "Topic Monitor Sample Interval",
            "Topic监控采样间隔",
            "s",
        )
    if not metrics:
        raise PlotterError(f"{resolved}: CSV has no numeric topic-rate data")
    return DataRun(
        path=resolved,
        name=default_run_name(resolved, "topic_rate", name),
        elapsed_seconds=np.asarray(elapsed, dtype=float),
        timestamps=tuple(group["timestamp"] for group in groups),
        metrics=metrics,
        specs=specs,
        kind="topic_rate",
    )


def load_pipeline_event_csv(
    resolved: Path,
    rows: Sequence[tuple[int, dict[str, Any]]],
    name: Optional[str],
) -> DataRun:
    elapsed: list[float] = []
    timestamps: list[Optional[datetime]] = []
    events: list[PipelineEvent] = []

    for line_number, row in rows:
        elapsed_value = parse_elapsed(row, resolved, line_number)
        if elapsed and elapsed_value < elapsed[-1]:
            raise PlotterError(
                f"{resolved}:{line_number}: elapsed_s moved backwards "
                f"from {elapsed[-1]} to {elapsed_value}"
            )
        event_type = str(row.get("event") or "").strip()
        if not event_type:
            raise PlotterError(
                f"{resolved}:{line_number}: pipeline event type is missing"
            )
        elapsed.append(elapsed_value)
        timestamps.append(parse_row_timestamp(row, resolved, line_number))
        events.append(
            PipelineEvent(
                event_id=str(row.get("event_id") or "").strip(),
                event_type=event_type,
                source_topic=str(row.get("source_topic") or "").strip(),
                auto_session=str(row.get("auto_session") or "").strip(),
                route_session=str(row.get("route_session") or "").strip(),
                details=str(row.get("details") or "").strip(),
            )
        )

    return DataRun(
        path=resolved,
        name=default_run_name(resolved, "pipeline_event", name),
        elapsed_seconds=np.asarray(elapsed, dtype=float),
        timestamps=tuple(timestamps),
        metrics={},
        specs={},
        kind="pipeline_event",
        events=tuple(events),
    )


def load_csv(path: Path, name: Optional[str] = None) -> DataRun:
    resolved = path.expanduser().resolve()
    if not resolved.is_file():
        raise PlotterError(f"CSV file does not exist: {path}")

    with resolved.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames:
            raise PlotterError(f"{resolved}: CSV header is missing")
        fieldnames = [field.strip() for field in reader.fieldnames if field]
        if "elapsed_s" not in fieldnames:
            raise PlotterError(f"{resolved}: required column elapsed_s is missing")
        rows = [
            (line_number, row)
            for line_number, row in enumerate(reader, start=2)
            if row and not row_is_blank(row)
        ]

    if not rows:
        raise PlotterError(f"{resolved}: CSV contains no data rows")
    kind = csv_kind(fieldnames)
    if kind == "topic_rate":
        return load_topic_rate_csv(resolved, rows, name)
    if kind == "pipeline_event":
        return load_pipeline_event_csv(resolved, rows, name)
    return load_wide_csv(resolved, fieldnames, rows, kind, name)


def union_specs(runs: Sequence[DataRun]) -> dict[str, MetricSpec]:
    specs: dict[str, MetricSpec] = {}
    for run in runs:
        specs.update(run.specs)
    return specs


def available_event_types(runs: Sequence[DataRun]) -> tuple[str, ...]:
    return tuple(
        sorted(
            {
                event.event_type
                for run in runs
                for event in run.events
            }
        )
    )


def resolved_event_types(
    runs: Sequence[DataRun],
    config: PlotConfig,
) -> tuple[str, ...]:
    available = available_event_types(runs)
    if config.event_types is None:
        return available
    selected = tuple(dict.fromkeys(config.event_types))
    unknown = sorted(set(selected) - set(available))
    if unknown:
        raise PlotterError(
            "unknown pipeline event type(s): " + ", ".join(unknown)
        )
    return selected


def pipeline_event_label(event: PipelineEvent, language: str) -> str:
    labels = {
        "initial_pose_published": ("Initial pose", "初始位姿"),
        "auto_drive_entered": ("AUTO ON", "进入自动"),
        "auto_drive_exited": ("AUTO OFF", "退出自动"),
        "route_start_requested": ("Route start", "路线开始"),
        "goal_accepted": ("Goal OK", "目标接受"),
        "planning_completed": ("Plan ready", "规划完成"),
        "navigation_failed": ("Nav failed", "导航失败"),
        "controller_output_started": ("CmdVel start", "速度指令"),
        "ackermann_output_started": ("Ack start", "转向指令"),
        "behavior_override_started": ("Override ON", "接管开始"),
        "behavior_override_ended": ("Override OFF", "接管结束"),
        "vehicle_motion_started": ("Moving", "车辆启动"),
        "vehicle_stopped": ("STOPPED", "车辆停止"),
        "route_completed": ("Route done", "路线完成"),
    }
    if event.event_type == "behavior_state_changed":
        state = pipeline_event_behavior_state(event)
        state_labels = {
            "NORMAL_NAV2": ("NAV", "状态:导航"),
            "STOP_SIGN": ("STOP SIGN", "状态:停车牌"),
            "TRAFFIC_LIGHT": ("LIGHT", "状态:红绿灯"),
        }
        label = state_labels.get(
            state,
            (
                f"State: {state}" if state else "State changed",
                f"状态:{state}" if state else "状态变化",
            ),
        )
    else:
        label = labels.get(
            event.event_type,
            (
                event.event_type.replace("_", " "),
                event.event_type.replace("_", " "),
            ),
        )
    return label[1] if language == "zh" else label[0]


def pipeline_event_behavior_state(event: PipelineEvent) -> str:
    match = re.search(r"(?:^|\s)current=([^\s]+)", event.details)
    return match.group(1) if match else ""


def pipeline_event_color_map(
    runs: Sequence[DataRun],
) -> dict[str, str]:
    return {
        event_type: EVENT_COLORS[index % len(EVENT_COLORS)]
        for index, event_type in enumerate(available_event_types(runs))
    }


def pipeline_event_color(
    event: PipelineEvent,
    color_map: dict[str, str],
) -> str:
    if event.event_type == "behavior_state_changed":
        state_color = BEHAVIOR_STATE_COLORS.get(
            pipeline_event_behavior_state(event)
        )
        if state_color is not None:
            return state_color
    return color_map[event.event_type]


def pipeline_event_label_lanes(
    entries: Sequence[tuple[float, PipelineEvent]],
    start: float,
    end: float,
    column_count: int,
) -> tuple[int, ...]:
    span = end - start
    minimum_spacing = span * (0.032 if column_count == 1 else 0.05)
    last_x = [-math.inf] * EVENT_LABEL_LANES
    lanes: list[int] = []
    for x_value, _event in entries:
        available = [
            lane
            for lane, previous_x in enumerate(last_x)
            if x_value - previous_x >= minimum_spacing
        ]
        lane = available[0] if available else min(
            range(EVENT_LABEL_LANES),
            key=last_x.__getitem__,
        )
        last_x[lane] = x_value
        lanes.append(lane)
    return tuple(lanes)


def use_run_for_metric(
    runs: Sequence[DataRun],
    run: DataRun,
    metric_key: str,
) -> bool:
    """Prefer direct system-monitor data over duplicated node-monitor data."""
    if metric_key not in run.metrics:
        return False
    if run.kind != "node_cpu":
        return True
    return not any(
        other.kind == "system" and metric_key in other.metrics
        for other in runs
    )


def metric_sort_key(spec: MetricSpec) -> tuple[int, str, str]:
    try:
        group_index = GROUP_ORDER.index(spec.group)
    except ValueError:
        group_index = len(GROUP_ORDER)
    return group_index, spec.group, spec.key


def validate_metric_keys(
    runs: Sequence[DataRun],
    metric_keys: Sequence[str],
) -> tuple[str, ...]:
    available = union_specs(runs)
    if not metric_keys:
        raise PlotterError("select at least one metric")
    missing = [key for key in metric_keys if key not in available]
    if missing:
        raise PlotterError(
            "unknown metric(s): " + ", ".join(sorted(missing))
        )
    return tuple(dict.fromkeys(metric_keys))


def default_metric_keys(runs: Sequence[DataRun]) -> tuple[str, ...]:
    available = union_specs(runs)
    selected = tuple(key for key in DEFAULT_METRICS if key in available)
    if selected:
        return selected
    ordered = sorted(available.values(), key=metric_sort_key)
    return tuple(spec.key for spec in ordered[: min(5, len(ordered))])


def time_values(
    run: DataRun,
    config: PlotConfig,
    assumed_timezone: Optional[tzinfo] = None,
) -> np.ndarray:
    if config.time_mode == "elapsed":
        divisor = SECONDS_PER_TIME_UNIT[config.time_unit]
        return run.elapsed_seconds / divisor
    values = np.full(len(run.elapsed_seconds), np.nan, dtype=float)
    for index, timestamp in enumerate(run.timestamps):
        if timestamp is not None:
            normalized = timestamp
            if normalized.tzinfo is None and assumed_timezone is not None:
                normalized = normalized.replace(tzinfo=assumed_timezone)
            values[index] = mdates.date2num(normalized)
    return values


def pipeline_event_plot_entries(
    runs: Sequence[DataRun],
    config: PlotConfig,
    timezone: Optional[tzinfo] = None,
) -> tuple[tuple[float, PipelineEvent], ...]:
    selected = set(resolved_event_types(runs, config))
    entries: list[tuple[float, PipelineEvent]] = []
    if not selected:
        return ()
    for run in runs:
        if not run.events:
            continue
        x_values = time_values(run, config, timezone)
        for x_value, event in zip(x_values, run.events):
            if math.isfinite(float(x_value)) and event.event_type in selected:
                entries.append((float(x_value), event))
    return tuple(sorted(entries, key=lambda entry: entry[0]))


def data_time_bounds(
    runs: Sequence[DataRun],
    config: PlotConfig,
    metric_keys: Optional[Sequence[str]] = None,
) -> tuple[float, float]:
    finite_values: list[np.ndarray] = []
    timezone = display_timezone(runs)
    for run in runs:
        values = time_values(run, config, timezone)
        valid = np.isfinite(values)
        if metric_keys is not None:
            metric_valid = np.zeros(len(values), dtype=bool)
            for key in metric_keys:
                metric = run.metrics.get(key)
                if metric is not None and use_run_for_metric(
                    runs,
                    run,
                    key,
                ):
                    metric_valid |= np.isfinite(metric)
            valid &= metric_valid
        finite = values[valid]
        if finite.size:
            finite_values.append(finite)
    if not finite_values:
        if config.time_mode == "timestamp":
            raise PlotterError(
                "timestamp mode requires valid timestamps for the selected metrics"
            )
        raise PlotterError("no finite time values exist for the selected metrics")
    minimum = min(float(values.min()) for values in finite_values)
    maximum = max(float(values.max()) for values in finite_values)
    if maximum <= minimum:
        maximum = minimum + 1.0
    return minimum, maximum


def resolved_selection(
    runs: Sequence[DataRun],
    config: PlotConfig,
    metric_keys: Optional[Sequence[str]] = None,
) -> tuple[float, float]:
    validate_config(config)
    data_start, data_end = data_time_bounds(runs, config, metric_keys)
    start = data_start if config.start is None else float(config.start)
    end = data_end if config.end is None else float(config.end)
    if not math.isfinite(start) or not math.isfinite(end):
        raise PlotterError("start and end must be finite")
    if end < data_start or start > data_end:
        return data_start, data_end
    start = max(start, data_start)
    end = min(end, data_end)
    if start >= end:
        raise PlotterError("start must be earlier than end")
    return start, end


def parse_boundary(value: Optional[str], config: PlotConfig) -> Optional[float]:
    if value is None or value.strip() == "":
        return None
    if config.time_mode == "elapsed":
        try:
            parsed = float(value)
        except ValueError as error:
            raise PlotterError(
                f"elapsed time boundary must be numeric: {value!r}"
            ) from error
        if not math.isfinite(parsed):
            raise PlotterError("elapsed time boundary must be finite")
        return parsed
    timestamp = parse_iso_timestamp(value, "time boundary")
    return float(mdates.date2num(timestamp))


def display_timezone(runs: Sequence[DataRun]) -> Optional[tzinfo]:
    for run in runs:
        for timestamp in run.timestamps:
            if timestamp is not None and timestamp.tzinfo is not None:
                return timestamp.tzinfo
    return None


def format_boundary(
    value: float,
    config: PlotConfig,
    timezone: Optional[tzinfo] = None,
) -> str:
    if config.time_mode == "elapsed":
        return f"{value:.3f}".rstrip("0").rstrip(".")
    timestamp = mdates.num2date(value, tz=timezone)
    return timestamp.isoformat(timespec="milliseconds")


def paper_rc(language: str) -> dict[str, Any]:
    fonts = (
        ["Noto Sans CJK SC", "DejaVu Sans"]
        if language == "zh"
        else ["DejaVu Sans", "Noto Sans CJK SC"]
    )
    return {
        "font.family": "sans-serif",
        "font.sans-serif": fonts,
        "font.size": 8.0,
        "axes.titlesize": 9.0,
        "axes.labelsize": 8.5,
        "xtick.labelsize": 7.5,
        "ytick.labelsize": 7.5,
        "legend.fontsize": 7.5,
        "lines.linewidth": 1.2,
        "axes.linewidth": 0.8,
        "grid.linewidth": 0.5,
        "grid.alpha": 0.28,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
        "svg.fonttype": "none",
        "savefig.bbox": "tight",
        "savefig.pad_inches": 0.04,
    }


def figure_width(config: PlotConfig) -> float:
    if config.figure_width == "custom":
        if config.custom_width <= 0:
            raise PlotterError("custom figure width must be greater than zero")
        return config.custom_width
    try:
        return FIGURE_WIDTHS_INCHES[config.figure_width]
    except KeyError as error:
        raise PlotterError(
            f"unsupported figure width preset: {config.figure_width}"
        ) from error


def validate_config(config: PlotConfig) -> PlotConfig:
    if config.time_mode not in {"elapsed", "timestamp"}:
        raise PlotterError(f"unsupported time mode: {config.time_mode}")
    if config.time_unit not in SECONDS_PER_TIME_UNIT:
        raise PlotterError(f"unsupported elapsed time unit: {config.time_unit}")
    if config.language not in {"en", "zh"}:
        raise PlotterError(f"unsupported language: {config.language}")
    if config.figure_width not in {*FIGURE_WIDTHS_INCHES, "custom"}:
        raise PlotterError(
            f"unsupported figure width preset: {config.figure_width}"
        )
    if config.custom_width <= 0 or not math.isfinite(config.custom_width):
        raise PlotterError("custom figure width must be a positive number")
    if config.dpi not in {300, 600}:
        raise PlotterError("PNG DPI must be 300 or 600")
    if config.event_types is not None and any(
        not isinstance(event_type, str) or not event_type
        for event_type in config.event_types
    ):
        raise PlotterError("pipeline event types must be non-empty text")
    if not isinstance(config.combine_topic_hz, bool):
        raise PlotterError("combine_topic_hz must be true or false")
    return config


def metric_plot_panels(
    metric_keys: Sequence[str],
    specs: dict[str, MetricSpec],
    combine_topic_hz: bool,
) -> tuple[tuple[str, ...], ...]:
    """Group selected Topic Hz metrics into one panel when requested."""
    keys = tuple(metric_keys)
    if not combine_topic_hz:
        return tuple((key,) for key in keys)
    topic_hz_keys = tuple(
        key for key in keys if specs[key].group == "Topic Hz"
    )
    if len(topic_hz_keys) < 2:
        return tuple((key,) for key in keys)

    panels: list[tuple[str, ...]] = []
    topic_panel_added = False
    for key in keys:
        if specs[key].group == "Topic Hz":
            if not topic_panel_added:
                panels.append(topic_hz_keys)
                topic_panel_added = True
            continue
        panels.append((key,))
    return tuple(panels)


def topic_hz_series_label(spec: MetricSpec, language: str) -> str:
    """Return the signal/topic label without the redundant rate suffix."""
    label = spec.label(language)
    suffix = "频率" if language == "zh" else " Rate"
    if label.endswith(suffix):
        return label[: -len(suffix)]
    return label


def create_plot_figure(
    runs: Sequence[DataRun],
    metric_keys: Sequence[str],
    config: PlotConfig,
) -> PlotResult:
    if not runs:
        raise PlotterError("load at least one CSV file")
    validate_config(config)
    keys = validate_metric_keys(runs, metric_keys)
    specs = union_specs(runs)
    panels = metric_plot_panels(
        keys,
        specs,
        config.combine_topic_hz,
    )
    start, end = resolved_selection(runs, config, keys)
    timezone = display_timezone(runs)
    event_entries = pipeline_event_plot_entries(runs, config, timezone)
    event_colors = pipeline_event_color_map(runs)
    metric_runs = [run for run in runs if run.metrics]
    width = figure_width(config)
    column_count = 1 if width < 6.0 or len(panels) == 1 else 2
    row_count = math.ceil(len(panels) / column_count)
    height = max(2.25, 2.05 * row_count + (0.35 if config.title else 0.1))
    warnings: list[str] = []

    with mpl.rc_context(rc=paper_rc(config.language)):
        figure = Figure(figsize=(width, height), dpi=100)
        axes_grid = figure.subplots(
            row_count,
            column_count,
            sharex=True,
            squeeze=False,
        )
        axes = list(axes_grid.flat)

        for panel_index, panel_keys in enumerate(panels):
            axis = axes[panel_index]
            combined_topic_hz = len(panel_keys) > 1 and all(
                specs[key].group == "Topic Hz" for key in panel_keys
            )
            plotted = 0
            series_index = 0
            topic_run_count = sum(
                run.kind == "topic_rate"
                and any(key in run.metrics for key in panel_keys)
                for run in metric_runs
            )
            for key in panel_keys:
                spec = specs[key]
                expected_kinds = {
                    run.kind for run in metric_runs if key in run.metrics
                }
                for run_index, run in enumerate(metric_runs):
                    values = run.metrics.get(key)
                    if values is None:
                        # Mixed monitor exports intentionally have disjoint
                        # schemas. Only warn when another run of the same CSV
                        # kind establishes that this metric should be present.
                        if run.kind in expected_kinds:
                            warnings.append(
                                f"{run.name}: metric {key} is not present"
                            )
                        continue
                    if not use_run_for_metric(runs, run, key):
                        continue
                    x_values = time_values(run, config, timezone)
                    mask = np.isfinite(x_values) & np.isfinite(values)
                    if not mask.any():
                        warnings.append(
                            f"{run.name}: metric {key} has no finite values"
                        )
                        continue
                    if combined_topic_hz:
                        label = topic_hz_series_label(
                            spec,
                            config.language,
                        )
                        if topic_run_count > 1:
                            label = f"{label} — {run.name}"
                        color_index = series_index % len(PAPER_COLORS)
                        style_index = (
                            series_index // len(PAPER_COLORS)
                        ) % len(PAPER_LINESTYLES)
                    else:
                        label = run.name
                        color_index = run_index % len(PAPER_COLORS)
                        style_index = run_index % len(PAPER_LINESTYLES)
                    line = axis.plot(
                        x_values[mask],
                        values[mask],
                        color=PAPER_COLORS[color_index],
                        linestyle=PAPER_LINESTYLES[style_index],
                        label=label,
                    )[0]
                    line.set_gid(f"metric-series:{key}")
                    plotted += 1
                    series_index += 1

            if plotted == 0:
                axis.text(
                    0.5,
                    0.5,
                    "No data" if config.language == "en" else "无数据",
                    ha="center",
                    va="center",
                    transform=axis.transAxes,
                )
            if combined_topic_hz:
                axis.set_ylabel(
                    "Topic频率 (Hz)"
                    if config.language == "zh"
                    else "Topic Rate (Hz)"
                )
                panel_label = (
                    "Topic频率对比"
                    if config.language == "zh"
                    else "Topic Hz Comparison"
                )
            else:
                spec = specs[panel_keys[0]]
                axis.set_ylabel(spec.axis_label(config.language))
                panel_label = spec.label(config.language)
            if len(panels) > 1:
                panel = chr(ord("a") + panel_index)
                axis.set_title(
                    f"({panel}) {panel_label}",
                    loc="left",
                    pad=3,
                )
            axis.grid(True, which="major", linestyle=":")
            axis.spines["top"].set_visible(False)
            axis.spines["right"].set_visible(False)
            axis.tick_params(direction="out", length=3)
            axis.set_xlim(start, end)

        for extra_axis in axes[len(panels) :]:
            extra_axis.set_visible(False)

        visible_axes = tuple(axes[: len(panels)])
        visible_events = [
            (x_value, event)
            for x_value, event in event_entries
            if start <= x_value <= end
        ]
        for x_value, event in visible_events:
            color = pipeline_event_color(event, event_colors)
            for axis in visible_axes:
                event_line = axis.axvline(
                    x_value,
                    color=color,
                    linestyle="--",
                    linewidth=1.1,
                    alpha=0.88,
                    zorder=1,
                )
                event_line.set_gid(
                    f"pipeline-event-line:{event.event_type}"
                )
        if visible_axes:
            label_axis = visible_axes[0]
            label_lanes = pipeline_event_label_lanes(
                visible_events,
                start,
                end,
                column_count,
            )
            label_y = (0.86, 0.64, 0.42, 0.20)
            for (x_value, event), lane in zip(
                visible_events,
                label_lanes,
            ):
                color = pipeline_event_color(event, event_colors)
                event_label_artist = label_axis.annotate(
                    pipeline_event_label(event, config.language),
                    xy=(
                        x_value,
                        label_y[lane],
                    ),
                    xycoords=("data", "axes fraction"),
                    xytext=(2, 0),
                    textcoords="offset points",
                    rotation=90,
                    ha="center",
                    va="center",
                    fontsize=8.2,
                    fontweight="semibold",
                    color=color,
                    bbox={
                        "boxstyle": "round,pad=0.16",
                        "facecolor": "white",
                        "edgecolor": color,
                        "linewidth": 0.55,
                        "alpha": 0.90,
                    },
                    clip_on=True,
                    annotation_clip=True,
                    zorder=3,
                )
                event_label_artist.set_gid(
                    f"pipeline-event-label:{event.event_type}"
                )
        x_label = (
            {
                "seconds": "Elapsed Time (s)",
                "minutes": "Elapsed Time (min)",
                "hours": "Elapsed Time (h)",
            }[config.time_unit]
            if config.time_mode == "elapsed"
            else ("Timestamp" if config.language == "en" else "系统时间")
        )
        for axis in visible_axes:
            axis.tick_params(labelbottom=True)
            axis.set_xlabel(
                x_label
                if config.language == "en"
                else x_label.replace("Elapsed Time", "运行时间")
            )

        if config.time_mode == "timestamp":
            timezone = display_timezone(runs)
            locator = mdates.AutoDateLocator(
                tz=timezone,
                minticks=3,
                maxticks=8,
            )
            formatter = mdates.ConciseDateFormatter(locator, tz=timezone)
            for axis in visible_axes:
                axis.xaxis.set_major_locator(locator)
                axis.xaxis.set_major_formatter(formatter)

        show_legend = len(metric_runs) > 1 or any(
            len(panel_keys) > 1 for panel_keys in panels
        )
        if show_legend:
            handles = []
            labels = []
            for axis in visible_axes:
                axis_handles, axis_labels = axis.get_legend_handles_labels()
                for handle, label in zip(axis_handles, axis_labels):
                    if label not in labels:
                        handles.append(handle)
                        labels.append(label)
            if handles:
                figure.legend(
                    handles,
                    labels,
                    loc="upper center",
                    bbox_to_anchor=(0.5, 0.995),
                    ncol=min(4, len(handles)),
                    frameon=False,
                )

        if config.title:
            figure.suptitle(config.title, fontsize=10, y=0.995)
        top = 0.90 if show_legend or config.title else 0.97
        figure.subplots_adjust(
            left=0.14 if column_count == 1 else 0.10,
            right=0.98,
            top=top,
            bottom=0.12,
            hspace=0.58,
            wspace=0.38,
        )
        figure.align_ylabels(visible_axes)

    return PlotResult(figure, visible_axes, tuple(dict.fromkeys(warnings)))


def selected_mask(
    run: DataRun,
    metric_key: str,
    config: PlotConfig,
    start: float,
    end: float,
    timezone: Optional[tzinfo] = None,
) -> np.ndarray:
    values = run.metrics[metric_key]
    x_values = time_values(run, config, timezone)
    return (
        np.isfinite(x_values)
        & np.isfinite(values)
        & (x_values >= start)
        & (x_values <= end)
    )


def compute_statistics(
    runs: Sequence[DataRun],
    metric_keys: Sequence[str],
    config: PlotConfig,
) -> tuple[StatisticsRecord, ...]:
    keys = validate_metric_keys(runs, metric_keys)
    specs = union_specs(runs)
    timezone = display_timezone(runs)
    start, end = resolved_selection(runs, config, keys)
    start_text = format_boundary(start, config, timezone)
    end_text = format_boundary(end, config, timezone)
    records: list[StatisticsRecord] = []
    for run in runs:
        for key in keys:
            if key not in run.metrics:
                continue
            if not use_run_for_metric(runs, run, key):
                continue
            mask = selected_mask(
                run,
                key,
                config,
                start,
                end,
                timezone,
            )
            values = run.metrics[key][mask]
            if values.size == 0:
                continue
            standard_deviation = (
                float(np.std(values, ddof=1))
                if values.size > 1
                else math.nan
            )
            spec = specs[key]
            records.append(
                StatisticsRecord(
                    run=run.name,
                    metric_key=key,
                    metric_label=spec.label(config.language),
                    unit=spec.unit,
                    selection_start=start_text,
                    selection_end=end_text,
                    count=int(values.size),
                    mean=float(np.mean(values)),
                    std=standard_deviation,
                    minimum=float(np.min(values)),
                    median=float(np.median(values)),
                    p95=float(np.percentile(values, 95)),
                    maximum=float(np.max(values)),
                )
            )
    return tuple(records)


def ensure_parent(path: Path) -> Path:
    resolved = path.expanduser().resolve()
    resolved.parent.mkdir(parents=True, exist_ok=True)
    return resolved


def selected_figure_suffix(selected_file_type: str) -> str:
    normalized = selected_file_type.strip().lower()
    for suffix in FIGURE_SUFFIXES:
        if suffix in normalized or suffix[1:] in normalized.split():
            return suffix
    return ".svg"


def resolve_figure_output_path(
    path: Path,
    selected_file_type: str,
) -> Path:
    suffix = path.suffix.lower()
    if suffix in FIGURE_SUFFIXES:
        return path
    if suffix:
        raise PlotterError("figure output must end in .pdf, .svg, or .png")
    return path.with_suffix(selected_figure_suffix(selected_file_type))


def save_rendered_figure(
    figure: Figure,
    config: PlotConfig,
    output_path: Path,
) -> None:
    output = ensure_parent(output_path)
    suffix = output.suffix.lower()
    if suffix not in FIGURE_SUFFIXES:
        raise PlotterError("figure output must end in .pdf, .svg, or .png")
    try:
        with mpl.rc_context(rc=paper_rc(config.language)):
            figure.savefig(
                output,
                format=suffix[1:],
                dpi=config.dpi if suffix == ".png" else None,
                facecolor="white",
            )
    except (OSError, RuntimeError, ValueError) as error:
        raise PlotterError(f"failed to export figure {output}: {error}") from error


def save_figure(
    runs: Sequence[DataRun],
    metric_keys: Sequence[str],
    config: PlotConfig,
    output_path: Path,
) -> tuple[str, ...]:
    result = create_plot_figure(runs, metric_keys, config)
    FigureCanvasAgg(result.figure)
    save_rendered_figure(result.figure, config, output_path)
    return result.warnings


def statistics_value(value: float) -> str:
    return "" if not math.isfinite(value) else f"{value:.9g}"


def write_statistics_csv(
    records: Sequence[StatisticsRecord],
    output_path: Path,
) -> None:
    output = ensure_parent(output_path)
    fields = (
        "run",
        "metric_key",
        "metric",
        "unit",
        "start",
        "end",
        "n",
        "mean",
        "std",
        "min",
        "median",
        "p95",
        "max",
    )
    with output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for record in records:
            writer.writerow(
                {
                    "run": record.run,
                    "metric_key": record.metric_key,
                    "metric": record.metric_label,
                    "unit": record.unit,
                    "start": record.selection_start,
                    "end": record.selection_end,
                    "n": record.count,
                    "mean": statistics_value(record.mean),
                    "std": statistics_value(record.std),
                    "min": statistics_value(record.minimum),
                    "median": statistics_value(record.median),
                    "p95": statistics_value(record.p95),
                    "max": statistics_value(record.maximum),
                }
            )


def latex_escape(value: str) -> str:
    replacements = {
        "\\": r"\textbackslash{}",
        "&": r"\&",
        "%": r"\%",
        "$": r"\$",
        "#": r"\#",
        "_": r"\_",
        "{": r"\{",
        "}": r"\}",
        "~": r"\textasciitilde{}",
        "^": r"\textasciicircum{}",
    }
    return "".join(replacements.get(character, character) for character in value)


def latex_number(value: float) -> str:
    return "--" if not math.isfinite(value) else f"{value:.5g}"


def write_statistics_latex(
    records: Sequence[StatisticsRecord],
    output_path: Path,
) -> None:
    output = ensure_parent(output_path)
    lines = [
        r"\begin{table*}[t]",
        r"\centering",
        r"\small",
        r"\begin{tabular}{lllrrrrrrr}",
        r"\toprule",
        (
            r"Run & Metric & Unit & $N$ & Mean & Std. & Min & Median "
            r"& P95 & Max \\"
        ),
        r"\midrule",
    ]
    for record in records:
        lines.append(
            " & ".join(
                (
                    latex_escape(record.run),
                    latex_escape(record.metric_label),
                    latex_escape(record.unit),
                    str(record.count),
                    latex_number(record.mean),
                    latex_number(record.std),
                    latex_number(record.minimum),
                    latex_number(record.median),
                    latex_number(record.p95),
                    latex_number(record.maximum),
                )
            )
            + r" \\"
        )
    lines.extend(
        (
            r"\bottomrule",
            r"\end{tabular}",
            r"\end{table*}",
            "",
        )
    )
    output.write_text("\n".join(lines), encoding="utf-8")


def recipe_payload(
    runs: Sequence[DataRun],
    metric_keys: Sequence[str],
    config: PlotConfig,
) -> dict[str, Any]:
    return {
        "version": RECIPE_VERSION,
        "files": [
            {"path": str(run.path.resolve()), "name": run.name} for run in runs
        ],
        "metrics": list(metric_keys),
        "time_mode": config.time_mode,
        "time_unit": config.time_unit,
        "start": (
            None
            if config.start is None
            else format_boundary(
                config.start,
                config,
                display_timezone(runs),
            )
        ),
        "end": (
            None
            if config.end is None
            else format_boundary(
                config.end,
                config,
                display_timezone(runs),
            )
        ),
        "language": config.language,
        "figure_width": config.figure_width,
        "custom_width": config.custom_width,
        "dpi": config.dpi,
        "title": config.title,
        "event_types": (
            None
            if config.event_types is None
            else list(config.event_types)
        ),
        "combine_topic_hz": config.combine_topic_hz,
    }


def write_recipe(
    runs: Sequence[DataRun],
    metric_keys: Sequence[str],
    config: PlotConfig,
    output_path: Path,
) -> None:
    output = ensure_parent(output_path)
    output.write_text(
        json.dumps(
            recipe_payload(runs, metric_keys, config),
            indent=2,
            ensure_ascii=False,
        )
        + "\n",
        encoding="utf-8",
    )


def load_recipe(
    recipe_path: Path,
) -> tuple[list[DataRun], tuple[str, ...], PlotConfig]:
    path = recipe_path.expanduser().resolve()
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise PlotterError(f"failed to read recipe {path}: {error}") from error
    if not isinstance(payload, dict):
        raise PlotterError(f"{path}: recipe root must be a JSON object")
    if payload.get("version") != RECIPE_VERSION:
        raise PlotterError(
            f"{path}: unsupported recipe version {payload.get('version')!r}"
        )
    file_entries = payload.get("files")
    if not isinstance(file_entries, list) or not file_entries:
        raise PlotterError(f"{path}: recipe files must be a non-empty list")

    runs = []
    for entry in file_entries:
        if (
            not isinstance(entry, dict)
            or not isinstance(entry.get("path"), str)
            or not entry["path"]
        ):
            raise PlotterError(f"{path}: invalid recipe file entry")
        name = entry.get("name")
        if name is not None and not isinstance(name, str):
            raise PlotterError(f"{path}: recipe experiment name must be text")
        runs.append(load_csv(Path(entry["path"]), name))

    raw_event_types = payload.get("event_types")
    if raw_event_types is None:
        event_types = None
    elif (
        isinstance(raw_event_types, list)
        and all(
            isinstance(event_type, str) and event_type
            for event_type in raw_event_types
        )
    ):
        event_types = tuple(raw_event_types)
    else:
        raise PlotterError(
            f"{path}: recipe event_types must be null or a text list"
        )
    raw_combine_topic_hz = payload.get("combine_topic_hz", False)
    if not isinstance(raw_combine_topic_hz, bool):
        raise PlotterError(
            f"{path}: recipe combine_topic_hz must be true or false"
        )

    try:
        base_config = validate_config(
            PlotConfig(
                time_mode=str(payload.get("time_mode", "elapsed")),
                time_unit=str(payload.get("time_unit", "minutes")),
                language=str(payload.get("language", "en")),
                figure_width=str(payload.get("figure_width", "double")),
                custom_width=float(payload.get("custom_width", 7.2)),
                dpi=int(payload.get("dpi", 600)),
                title=str(payload.get("title", "")),
                event_types=event_types,
                combine_topic_hz=raw_combine_topic_hz,
            )
        )
    except (TypeError, ValueError) as error:
        raise PlotterError(f"{path}: invalid recipe plot settings") from error
    config = replace(
        base_config,
        start=parse_boundary(
            None if payload.get("start") is None else str(payload["start"]),
            base_config,
        ),
        end=parse_boundary(
            None if payload.get("end") is None else str(payload["end"]),
            base_config,
        ),
    )
    metric_entries = payload.get("metrics")
    if (
        not isinstance(metric_entries, list)
        or not metric_entries
        or not all(isinstance(key, str) and key for key in metric_entries)
    ):
        raise PlotterError(
            f"{path}: recipe metrics must be a non-empty text list"
        )
    metrics = validate_metric_keys(runs, metric_entries)
    resolved_selection(runs, config, metrics)
    resolved_event_types(runs, config)
    return runs, metrics, config


def group_title(group: str, language: str) -> str:
    if language == "en":
        return group
    return {
        "CPU": "CPU",
        "ROS Processes": "ROS进程",
        "Topic Hz": "Topic频率",
        "Topic Counts": "Topic消息数",
        "Topic Age": "Topic消息年龄",
        "Temperature": "温度",
        "VDD_IN": "整机输入 VDD_IN",
        "VDD_CPU_GPU_CV": "CPU/GPU/CV合并电源轨",
        "VDD_SOC": "SoC电源轨",
        "Other": "其他",
    }.get(group, group)


def launch_gui(
    initial_runs: Sequence[DataRun],
    initial_metrics: Sequence[str],
    initial_config: PlotConfig,
) -> int:
    try:
        import tkinter as tk
        from tkinter import filedialog, messagebox, ttk
        from matplotlib.backends.backend_tkagg import (
            FigureCanvasTkAgg,
            NavigationToolbar2Tk,
        )
    except ImportError as error:
        raise PlotterError(
            "GUI mode requires Tkinter and Matplotlib TkAgg; "
            "use --no-gui for headless export"
        ) from error

    class PlotterWindow:
        def __init__(self) -> None:
            self.root = tk.Tk()
            self.root.title("CARKit Pipeline Metrics Plotter")
            self.root.geometry("1420x850")
            self.runs = list(initial_runs)
            self.current_metrics = tuple(initial_metrics)
            self.current_config = initial_config
            self.metric_item_keys: dict[str, str] = {}
            self.event_item_types: dict[str, str] = {}
            self.figure_canvas: Optional[Any] = None
            self.toolbar: Optional[Any] = None
            self.slider: Optional[RangeSlider] = None
            self.plot_result: Optional[PlotResult] = None
            self.slider_callback_blocked = False

            self.build_layout()
            self.rebuild_run_tree()
            self.rebuild_metric_tree(self.current_metrics)
            self.rebuild_event_tree(self.current_config.event_types)
            if self.runs and self.current_metrics:
                start, end = resolved_selection(
                    self.runs,
                    self.current_config,
                    self.current_metrics or None,
                )
                self.current_config = replace(
                    self.current_config,
                    start=start,
                    end=end,
                )
                timezone = display_timezone(self.runs)
                self.start_var.set(
                    format_boundary(start, self.current_config, timezone)
                )
                self.end_var.set(
                    format_boundary(end, self.current_config, timezone)
                )
                self.refresh_plot()
            elif self.runs:
                self.status_var.set(
                    "Pipeline event CSV loaded. Add a metrics CSV to draw "
                    "event markers."
                )
            else:
                self.status_var.set("Add one or more CSV files to begin.")

        def build_layout(self) -> None:
            main = ttk.Panedwindow(self.root, orient=tk.HORIZONTAL)
            main.pack(fill=tk.BOTH, expand=True)
            controls_container = ttk.Frame(main)
            fixed_action_row = ttk.Frame(
                controls_container,
                padding=(8, 8, 8, 4),
            )
            fixed_action_row.pack(side=tk.TOP, fill=tk.X)
            ttk.Button(
                fixed_action_row,
                text="Redraw",
                command=self.refresh_plot,
            ).pack(side=tk.LEFT, fill=tk.X, expand=True)
            ttk.Button(
                fixed_action_row,
                text="Full range",
                command=self.reset_and_refresh,
            ).pack(side=tk.LEFT, padx=(4, 0))

            self.controls_canvas = tk.Canvas(
                controls_container,
                width=350,
                borderwidth=0,
                highlightthickness=0,
            )
            controls_scroll = ttk.Scrollbar(
                controls_container,
                orient=tk.VERTICAL,
                command=self.controls_canvas.yview,
            )
            frame_background = (
                ttk.Style().lookup("TFrame", "background")
                or self.root.cget("background")
            )
            self.controls_canvas.configure(
                yscrollcommand=controls_scroll.set,
                background=frame_background,
            )
            self.controls_canvas.pack(
                side=tk.LEFT,
                fill=tk.BOTH,
                expand=True,
            )
            controls_scroll.pack(side=tk.RIGHT, fill=tk.Y)

            controls = ttk.Frame(self.controls_canvas, padding=8)
            controls_window = self.controls_canvas.create_window(
                (0, 0),
                window=controls,
                anchor=tk.NW,
            )

            def update_controls_scroll_region(_event: Any = None) -> None:
                bounds = self.controls_canvas.bbox("all")
                if bounds is not None:
                    self.controls_canvas.configure(scrollregion=bounds)

            def resize_controls(event: Any) -> None:
                self.controls_canvas.itemconfigure(
                    controls_window,
                    width=event.width,
                )

            controls.bind("<Configure>", update_controls_scroll_region)
            self.controls_canvas.bind("<Configure>", resize_controls)

            def pointer_is_over_controls(widget: Any) -> bool:
                while widget is not None:
                    if widget in {
                        controls,
                        self.controls_canvas,
                        controls_container,
                    }:
                        return True
                    widget = getattr(widget, "master", None)
                return False

            def scroll_controls(event: Any) -> Optional[str]:
                if not pointer_is_over_controls(event.widget):
                    return None
                if getattr(event, "num", None) == 4:
                    steps = -1
                elif getattr(event, "num", None) == 5:
                    steps = 1
                else:
                    delta = int(getattr(event, "delta", 0))
                    if delta == 0:
                        return None
                    steps = -max(1, abs(delta) // 120)
                    if delta < 0:
                        steps = -steps
                self.controls_canvas.yview_scroll(steps, "units")
                return "break"

            self.root.bind_all("<MouseWheel>", scroll_controls, add="+")
            self.root.bind_all("<Button-4>", scroll_controls, add="+")
            self.root.bind_all("<Button-5>", scroll_controls, add="+")

            display = ttk.Frame(main, padding=(4, 8, 8, 8))
            main.add(controls_container, weight=0)
            main.add(display, weight=1)

            ttk.Label(
                controls,
                text="CARKit CSV Plotter",
                font=("TkDefaultFont", 14, "bold"),
            ).pack(anchor=tk.W, pady=(0, 8))

            files_frame = ttk.LabelFrame(controls, text="CSV experiments")
            files_frame.pack(fill=tk.X, pady=4)
            self.run_tree = ttk.Treeview(
                files_frame,
                columns=("name", "path"),
                show="headings",
                height=5,
                selectmode="browse",
            )
            self.run_tree.heading("name", text="Name")
            self.run_tree.heading("path", text="File")
            self.run_tree.column("name", width=110, stretch=False)
            self.run_tree.column("path", width=205)
            self.run_tree.pack(fill=tk.X, padx=4, pady=4)
            self.run_tree.bind("<<TreeviewSelect>>", self.load_selected_run_name)

            file_buttons = ttk.Frame(files_frame)
            file_buttons.pack(fill=tk.X, padx=4, pady=(0, 4))
            ttk.Button(
                file_buttons,
                text="Add CSV…",
                command=self.add_csv_files,
            ).pack(side=tk.LEFT)
            ttk.Button(
                file_buttons,
                text="Remove",
                command=self.remove_selected_run,
            ).pack(side=tk.LEFT, padx=4)

            rename_row = ttk.Frame(files_frame)
            rename_row.pack(fill=tk.X, padx=4, pady=(0, 4))
            self.run_name_var = tk.StringVar()
            ttk.Entry(
                rename_row,
                textvariable=self.run_name_var,
                width=25,
            ).pack(side=tk.LEFT, fill=tk.X, expand=True)
            ttk.Button(
                rename_row,
                text="Rename",
                command=self.rename_selected_run,
            ).pack(side=tk.LEFT, padx=(4, 0))

            metrics_frame = ttk.LabelFrame(controls, text="Metrics")
            metrics_frame.pack(fill=tk.BOTH, expand=True, pady=4)
            self.metric_tree = ttk.Treeview(
                metrics_frame,
                show="tree",
                height=15,
                selectmode="extended",
            )
            metric_scroll = ttk.Scrollbar(
                metrics_frame,
                orient=tk.VERTICAL,
                command=self.metric_tree.yview,
            )
            self.metric_tree.configure(yscrollcommand=metric_scroll.set)
            self.metric_tree.pack(
                side=tk.LEFT,
                fill=tk.BOTH,
                expand=True,
                padx=(4, 0),
                pady=4,
            )
            metric_scroll.pack(side=tk.RIGHT, fill=tk.Y, padx=(0, 4), pady=4)
            self.metric_tree.bind(
                "<Double-1>",
                self.select_group_children,
            )

            metric_buttons = ttk.Frame(controls)
            metric_buttons.pack(fill=tk.X, pady=(0, 4))
            ttk.Button(
                metric_buttons,
                text="Recommended",
                command=self.select_recommended_metrics,
            ).pack(side=tk.LEFT)
            ttk.Button(
                metric_buttons,
                text="Clear",
                command=lambda: self.metric_tree.selection_set(()),
            ).pack(side=tk.LEFT, padx=4)
            self.combine_topic_hz_var = tk.BooleanVar(
                value=self.current_config.combine_topic_hz
            )
            ttk.Checkbutton(
                controls,
                text="Combine selected Topic Hz",
                variable=self.combine_topic_hz_var,
                command=self.combine_topic_hz_changed,
            ).pack(anchor=tk.W, pady=(0, 4))

            events_frame = ttk.LabelFrame(
                controls,
                text="Pipeline event markers",
            )
            events_frame.pack(fill=tk.BOTH, expand=True, pady=4)
            self.event_tree = ttk.Treeview(
                events_frame,
                show="tree",
                height=7,
                selectmode="extended",
            )
            event_scroll = ttk.Scrollbar(
                events_frame,
                orient=tk.VERTICAL,
                command=self.event_tree.yview,
            )
            self.event_tree.configure(yscrollcommand=event_scroll.set)
            self.event_tree.pack(
                side=tk.LEFT,
                fill=tk.BOTH,
                expand=True,
                padx=(4, 0),
                pady=4,
            )
            event_scroll.pack(
                side=tk.RIGHT,
                fill=tk.Y,
                padx=(0, 4),
                pady=4,
            )

            event_buttons = ttk.Frame(controls)
            event_buttons.pack(fill=tk.X, pady=(0, 4))
            ttk.Button(
                event_buttons,
                text="All events",
                command=self.select_all_event_types,
            ).pack(side=tk.LEFT)
            ttk.Button(
                event_buttons,
                text="Clear",
                command=lambda: self.event_tree.selection_set(()),
            ).pack(side=tk.LEFT, padx=4)

            time_frame = ttk.LabelFrame(controls, text="Time selection")
            time_frame.pack(fill=tk.X, pady=4)
            time_options = ttk.Frame(time_frame)
            time_options.pack(fill=tk.X, padx=4, pady=4)
            self.time_mode_var = tk.StringVar(value=self.current_config.time_mode)
            self.time_unit_var = tk.StringVar(value=self.current_config.time_unit)
            ttk.Combobox(
                time_options,
                textvariable=self.time_mode_var,
                values=("elapsed", "timestamp"),
                state="readonly",
                width=10,
            ).pack(side=tk.LEFT)
            self.time_unit_combo = ttk.Combobox(
                time_options,
                textvariable=self.time_unit_var,
                values=("seconds", "minutes", "hours"),
                state="readonly",
                width=9,
            )
            self.time_unit_combo.pack(side=tk.LEFT, padx=4)
            self.time_mode_var.trace_add("write", self.time_mode_changed)
            self.time_unit_var.trace_add("write", self.time_unit_changed)
            self.time_unit_combo.configure(
                state=(
                    "readonly"
                    if self.current_config.time_mode == "elapsed"
                    else "disabled"
                )
            )

            range_row = ttk.Frame(time_frame)
            range_row.pack(fill=tk.X, padx=4, pady=(0, 4))
            self.start_var = tk.StringVar()
            self.end_var = tk.StringVar()
            ttk.Label(range_row, text="Start").grid(row=0, column=0, sticky=tk.W)
            ttk.Entry(
                range_row,
                textvariable=self.start_var,
                width=18,
            ).grid(row=1, column=0, padx=(0, 4))
            ttk.Label(range_row, text="End").grid(row=0, column=1, sticky=tk.W)
            ttk.Entry(
                range_row,
                textvariable=self.end_var,
                width=18,
            ).grid(row=1, column=1)

            # Export styling remains available through command-line arguments
            # and saved recipes, but does not occupy space in the GUI.
            self.language_var = tk.StringVar(value=self.current_config.language)
            self.width_var = tk.StringVar(value=self.current_config.figure_width)
            self.custom_width_var = tk.StringVar(
                value=f"{self.current_config.custom_width:g}"
            )
            self.dpi_var = tk.StringVar(value=str(self.current_config.dpi))
            self.title_var = tk.StringVar(value=self.current_config.title)

            export_row = ttk.Frame(controls)
            export_row.pack(fill=tk.X, pady=5)
            ttk.Button(
                export_row,
                text="Export Figure…",
                command=self.export_figure,
            ).pack(side=tk.LEFT)
            ttk.Button(
                export_row,
                text="Export Statistics…",
                command=self.export_statistics,
            ).pack(side=tk.LEFT, padx=4)

            self.status_var = tk.StringVar()
            ttk.Label(
                controls,
                textvariable=self.status_var,
                wraplength=330,
                foreground="#555555",
            ).pack(fill=tk.X, pady=(4, 0))

            self.notebook = ttk.Notebook(display)
            self.notebook.pack(fill=tk.BOTH, expand=True)
            self.plot_tab = ttk.Frame(self.notebook)
            self.statistics_tab = ttk.Frame(self.notebook)
            self.notebook.add(self.plot_tab, text="Plot")
            self.notebook.add(self.statistics_tab, text="Statistics")

            stat_columns = (
                "run",
                "metric",
                "unit",
                "n",
                "mean",
                "std",
                "min",
                "median",
                "p95",
                "max",
            )
            self.statistics_tree = ttk.Treeview(
                self.statistics_tab,
                columns=stat_columns,
                show="headings",
            )
            for column in stat_columns:
                self.statistics_tree.heading(
                    column,
                    text=column.upper() if column == "n" else column.title(),
                )
                self.statistics_tree.column(
                    column,
                    width=115 if column in {"run", "metric"} else 80,
                    anchor=tk.E if column not in {"run", "metric", "unit"} else tk.W,
                )
            stat_y_scroll = ttk.Scrollbar(
                self.statistics_tab,
                orient=tk.VERTICAL,
                command=self.statistics_tree.yview,
            )
            stat_x_scroll = ttk.Scrollbar(
                self.statistics_tab,
                orient=tk.HORIZONTAL,
                command=self.statistics_tree.xview,
            )
            self.statistics_tree.configure(
                yscrollcommand=stat_y_scroll.set,
                xscrollcommand=stat_x_scroll.set,
            )
            self.statistics_tree.grid(row=0, column=0, sticky="nsew")
            stat_y_scroll.grid(row=0, column=1, sticky="ns")
            stat_x_scroll.grid(row=1, column=0, sticky="ew")
            self.statistics_tab.rowconfigure(0, weight=1)
            self.statistics_tab.columnconfigure(0, weight=1)

        def run(self) -> int:
            self.root.mainloop()
            return 0

        def selected_metric_keys(self) -> tuple[str, ...]:
            keys = []
            for item_id in self.metric_tree.selection():
                key = self.metric_item_keys.get(item_id)
                if key is not None:
                    keys.append(key)
            return tuple(keys)

        def selected_event_types(self) -> tuple[str, ...]:
            event_types = []
            for item_id in self.event_tree.selection():
                event_type = self.event_item_types.get(item_id)
                if event_type is not None:
                    event_types.append(event_type)
            return tuple(event_types)

        def select_all_event_types(self) -> None:
            self.event_tree.selection_set(tuple(self.event_item_types))

        def rebuild_run_tree(self) -> None:
            self.run_tree.delete(*self.run_tree.get_children())
            for index, run in enumerate(self.runs):
                self.run_tree.insert(
                    "",
                    tk.END,
                    iid=f"run:{index}",
                    values=(run.name, str(run.path)),
                )

        def rebuild_metric_tree(
            self,
            selected_keys: Sequence[str] = (),
        ) -> None:
            self.metric_tree.delete(*self.metric_tree.get_children())
            self.metric_item_keys.clear()
            specs = union_specs(self.runs)
            language = self.language_var.get() if hasattr(self, "language_var") else "en"
            selected = set(selected_keys)
            selection_ids = []
            for group in GROUP_ORDER:
                group_specs = sorted(
                    (spec for spec in specs.values() if spec.group == group),
                    key=lambda spec: spec.label(language),
                )
                if not group_specs:
                    continue
                group_id = f"group:{group}"
                self.metric_tree.insert(
                    "",
                    tk.END,
                    iid=group_id,
                    text=group_title(group, language),
                    open=group in {
                        "CPU",
                        "Topic Hz",
                        "VDD_IN",
                        "VDD_CPU_GPU_CV",
                    },
                )
                for spec in group_specs:
                    item_id = f"metric:{spec.key}"
                    label = spec.label(language)
                    if spec.unit:
                        label += f" [{spec.unit}]"
                    self.metric_tree.insert(
                        group_id,
                        tk.END,
                        iid=item_id,
                        text=label,
                    )
                    self.metric_item_keys[item_id] = spec.key
                    if spec.key in selected:
                        selection_ids.append(item_id)
            self.metric_tree.selection_set(selection_ids)

        def rebuild_event_tree(
            self,
            selected_event_types: Optional[Sequence[str]] = None,
        ) -> None:
            self.event_tree.delete(*self.event_tree.get_children())
            self.event_item_types.clear()
            available = available_event_types(self.runs)
            selected = (
                set(available)
                if selected_event_types is None
                else set(selected_event_types)
            )
            selection_ids = []
            for index, event_type in enumerate(available):
                item_id = f"event:{index}"
                self.event_tree.insert(
                    "",
                    tk.END,
                    iid=item_id,
                    text=event_type,
                )
                self.event_item_types[item_id] = event_type
                if event_type in selected:
                    selection_ids.append(item_id)
            self.event_tree.selection_set(selection_ids)

        def select_group_children(self, event: Any) -> None:
            item_id = self.metric_tree.identify_row(event.y)
            if not item_id.startswith("group:"):
                return
            children = self.metric_tree.get_children(item_id)
            current = set(self.metric_tree.selection())
            if all(child in current for child in children):
                for child in children:
                    current.discard(child)
            else:
                current.update(children)
            self.metric_tree.selection_set(tuple(current))

        def select_recommended_metrics(self) -> None:
            keys = default_metric_keys(self.runs) if self.runs else ()
            ids = [
                item_id
                for item_id, key in self.metric_item_keys.items()
                if key in keys
            ]
            self.metric_tree.selection_set(ids)

        def add_csv_files(self) -> None:
            paths = filedialog.askopenfilenames(
                title="Select CARKit monitor CSV files",
                filetypes=(("CSV files", "*.csv"), ("All files", "*")),
            )
            if not paths:
                return
            try:
                previous_metrics = self.selected_metric_keys()
                previous_events = self.selected_event_types()
                previous_available_events = available_event_types(self.runs)
                all_events_selected = (
                    set(previous_events) == set(previous_available_events)
                )
                existing = {run.path for run in self.runs}
                for raw_path in paths:
                    run = load_csv(Path(raw_path))
                    if run.path not in existing:
                        self.runs.append(run)
                        existing.add(run.path)
                self.rebuild_run_tree()
                self.rebuild_metric_tree(
                    previous_metrics or default_metric_keys(self.runs)
                )
                self.rebuild_event_tree(
                    None if all_events_selected else previous_events
                )
                if self.selected_metric_keys():
                    self.reset_time_range()
                    self.refresh_plot()
                else:
                    self.status_var.set(
                        "Pipeline event CSV loaded. Add a metrics CSV to "
                        "draw event markers."
                    )
            except (PlotterError, OSError, UnicodeError, csv.Error) as error:
                messagebox.showerror("CSV error", str(error))

        def selected_run_index(self) -> Optional[int]:
            selection = self.run_tree.selection()
            if not selection:
                return None
            return int(selection[0].split(":", 1)[1])

        def load_selected_run_name(self, _event: Any = None) -> None:
            index = self.selected_run_index()
            if index is not None:
                self.run_name_var.set(self.runs[index].name)

        def remove_selected_run(self) -> None:
            index = self.selected_run_index()
            if index is None:
                return
            selected = self.selected_metric_keys()
            selected_events = self.selected_event_types()
            all_events_selected = (
                set(selected_events) == set(available_event_types(self.runs))
            )
            del self.runs[index]
            self.rebuild_run_tree()
            self.rebuild_metric_tree(selected)
            self.rebuild_event_tree(
                None if all_events_selected else selected_events
            )
            if self.runs and self.selected_metric_keys():
                self.reset_time_range()
                self.refresh_plot()
            elif self.runs:
                self.destroy_plot()
                self.clear_statistics()
                self.status_var.set(
                    "Add or select at least one metrics CSV to draw."
                )
            else:
                self.destroy_plot()
                self.clear_statistics()
                self.status_var.set("Add one or more CSV files to begin.")

        def rename_selected_run(self) -> None:
            index = self.selected_run_index()
            name = self.run_name_var.get().strip()
            if index is None or not name:
                return
            self.runs[index].name = name
            self.rebuild_run_tree()
            self.run_tree.selection_set(f"run:{index}")
            self.refresh_plot()

        def time_mode_changed(self, *_args: Any) -> None:
            mode = self.time_mode_var.get()
            self.time_unit_combo.configure(
                state=(
                    "readonly"
                    if mode == "elapsed"
                    else "disabled"
                )
            )
            if self.runs:
                try:
                    # Boundaries from elapsed and timestamp modes are not
                    # interchangeable.  Clear them before computing the new
                    # mode's full valid-data range.
                    self.start_var.set("")
                    self.end_var.set("")
                    self.current_config = replace(
                        self.current_config,
                        time_mode=mode,
                        start=None,
                        end=None,
                    )
                    self.reset_time_range()
                    self.refresh_plot()
                except PlotterError as error:
                    messagebox.showerror("Time error", str(error))

        def time_unit_changed(self, *_args: Any) -> None:
            if self.runs and self.time_mode_var.get() == "elapsed":
                self.reset_time_range()
                self.refresh_plot()

        def language_changed(self, *_args: Any) -> None:
            selected = self.selected_metric_keys()
            self.rebuild_metric_tree(selected)

        def combine_topic_hz_changed(self) -> None:
            if self.runs and self.selected_metric_keys():
                self.refresh_plot()

        def config_from_controls(self) -> PlotConfig:
            try:
                custom_width = float(self.custom_width_var.get())
                dpi = int(self.dpi_var.get())
            except ValueError as error:
                raise PlotterError("figure width and DPI must be numeric") from error
            base = PlotConfig(
                time_mode=self.time_mode_var.get(),
                time_unit=self.time_unit_var.get(),
                language=self.language_var.get(),
                figure_width=self.width_var.get(),
                custom_width=custom_width,
                dpi=dpi,
                title=self.title_var.get().strip(),
                event_types=self.selected_event_types(),
                combine_topic_hz=self.combine_topic_hz_var.get(),
            )
            return replace(
                base,
                start=parse_boundary(self.start_var.get(), base),
                end=parse_boundary(self.end_var.get(), base),
            )

        def reset_time_range(self) -> None:
            base = PlotConfig(
                time_mode=self.time_mode_var.get(),
                time_unit=self.time_unit_var.get(),
                language=self.language_var.get(),
            )
            metrics = self.selected_metric_keys()
            start, end = data_time_bounds(
                self.runs,
                base,
                metrics or None,
            )
            timezone = display_timezone(self.runs)
            self.start_var.set(format_boundary(start, base, timezone))
            self.end_var.set(format_boundary(end, base, timezone))

        def reset_and_refresh(self) -> None:
            try:
                self.reset_time_range()
                self.refresh_plot()
            except PlotterError as error:
                messagebox.showerror("Plot error", str(error))

        def destroy_plot(self) -> None:
            if self.toolbar is not None:
                self.toolbar.destroy()
                self.toolbar = None
            if self.figure_canvas is not None:
                self.figure_canvas.get_tk_widget().destroy()
                self.figure_canvas = None
            self.plot_result = None
            self.slider = None

        def refresh_plot(self) -> None:
            if not self.runs:
                return
            try:
                metrics = self.selected_metric_keys()
                if not metrics:
                    raise PlotterError("select at least one metric")
                config = self.config_from_controls()
                start, end = resolved_selection(
                    self.runs,
                    config,
                    metrics,
                )
                config = replace(config, start=start, end=end)
                timezone = display_timezone(self.runs)
                self.start_var.set(
                    format_boundary(start, config, timezone)
                )
                self.end_var.set(
                    format_boundary(end, config, timezone)
                )
                result = create_plot_figure(self.runs, metrics, config)
                full_start, full_end = data_time_bounds(
                    self.runs,
                    config,
                    metrics,
                )

                result.figure.subplots_adjust(bottom=0.17)
                slider_axis = result.figure.add_axes((0.18, 0.035, 0.64, 0.025))
                slider = RangeSlider(
                    slider_axis,
                    "Range",
                    full_start,
                    full_end,
                    valinit=(start, end),
                )
                slider.valtext.set_visible(False)
                slider.on_changed(self.slider_changed)

                self.destroy_plot()
                self.current_metrics = metrics
                self.current_config = config
                self.plot_result = result
                self.slider = slider
                self.figure_canvas = FigureCanvasTkAgg(
                    result.figure,
                    master=self.plot_tab,
                )
                self.figure_canvas.draw()
                self.figure_canvas.get_tk_widget().pack(
                    side=tk.TOP,
                    fill=tk.BOTH,
                    expand=True,
                )
                self.figure_canvas.mpl_connect(
                    "scroll_event",
                    self.scroll_zoom,
                )

                owner = self

                class RecipeToolbar(NavigationToolbar2Tk):
                    def save_figure(toolbar_self, *args: Any) -> None:
                        del args
                        owner.export_figure()

                self.toolbar = RecipeToolbar(
                    self.figure_canvas,
                    self.plot_tab,
                    pack_toolbar=False,
                )
                self.toolbar.update()
                self.toolbar.pack(side=tk.BOTTOM, fill=tk.X)
                self.update_statistics()
                warning_text = "; ".join(result.warnings)
                self.status_var.set(
                    warning_text
                    or (
                        f"{len(self.runs)} run(s), {len(metrics)} metric(s), "
                        f"{format_boundary(start, config, display_timezone(self.runs))} "
                        f"to {format_boundary(end, config, display_timezone(self.runs))}"
                    )
                )
            except (PlotterError, ValueError) as error:
                messagebox.showerror("Plot error", str(error))

        def slider_changed(self, values: tuple[float, float]) -> None:
            if self.slider_callback_blocked or self.plot_result is None:
                return
            start, end = map(float, values)
            self.current_config = replace(
                self.current_config,
                start=start,
                end=end,
            )
            timezone = display_timezone(self.runs)
            self.start_var.set(
                format_boundary(start, self.current_config, timezone)
            )
            self.end_var.set(
                format_boundary(end, self.current_config, timezone)
            )
            for axis in self.plot_result.data_axes:
                axis.set_xlim(start, end)
            if self.figure_canvas is not None:
                self.figure_canvas.draw_idle()
            self.update_statistics()

        def scroll_zoom(self, event: Any) -> None:
            if (
                self.plot_result is None
                or event.inaxes not in self.plot_result.data_axes
                or event.xdata is None
            ):
                return
            left, right = event.inaxes.get_xlim()
            factor = 1 / 1.25 if event.button == "up" else 1.25
            center = float(event.xdata)
            new_left = center - (center - left) * factor
            new_right = center + (right - center) * factor
            full_start, full_end = data_time_bounds(
                self.runs,
                self.current_config,
                self.current_metrics,
            )
            new_left = max(new_left, full_start)
            new_right = min(new_right, full_end)
            if new_left >= new_right:
                return
            for axis in self.plot_result.data_axes:
                axis.set_xlim(new_left, new_right)
            if self.figure_canvas is not None:
                self.figure_canvas.draw_idle()

        def clear_statistics(self) -> None:
            self.statistics_tree.delete(*self.statistics_tree.get_children())

        def update_statistics(self) -> None:
            self.clear_statistics()
            if not self.runs or not self.current_metrics:
                return
            try:
                records = compute_statistics(
                    self.runs,
                    self.current_metrics,
                    self.current_config,
                )
            except PlotterError:
                return
            for index, record in enumerate(records):
                self.statistics_tree.insert(
                    "",
                    tk.END,
                    iid=f"stat:{index}",
                    values=(
                        record.run,
                        record.metric_label,
                        record.unit,
                        record.count,
                        latex_number(record.mean),
                        latex_number(record.std),
                        latex_number(record.minimum),
                        latex_number(record.median),
                        latex_number(record.p95),
                        latex_number(record.maximum),
                    ),
                )

        def export_figure(self) -> None:
            if (
                not self.runs
                or self.plot_result is None
                or not self.current_metrics
            ):
                return
            DEFAULT_EXPORT_DIRECTORY.mkdir(parents=True, exist_ok=True)
            selected_file_type = tk.StringVar(value="SVG vector")
            path = filedialog.asksaveasfilename(
                title="Export publication figure",
                defaultextension="",
                initialdir=str(DEFAULT_EXPORT_DIRECTORY),
                filetypes=(
                    ("SVG vector", "*.svg"),
                    ("PDF vector", "*.pdf"),
                    ("PNG image", "*.png"),
                ),
                typevariable=selected_file_type,
            )
            if not path:
                return
            try:
                output = resolve_figure_output_path(
                    Path(path),
                    selected_file_type.get(),
                )
                try:
                    export_dpi = int(self.dpi_var.get())
                except ValueError as error:
                    raise PlotterError("DPI must be numeric") from error
                config = replace(self.current_config, dpi=export_dpi)
                validate_config(config)
                if self.plot_result.data_axes:
                    start, end = self.plot_result.data_axes[0].get_xlim()
                    config = replace(
                        config,
                        start=float(start),
                        end=float(end),
                    )

                # Save the exact Figure already rendered in the GUI. This
                # preserves its current zoom, event markers, layout, and
                # any interactive view changes instead of rebuilding it.
                save_rendered_figure(
                    self.plot_result.figure,
                    config,
                    output,
                )
                metrics = self.current_metrics
                warnings = self.plot_result.warnings
                recipe_path = output.with_suffix(".json")
                write_recipe(
                    self.runs,
                    metrics,
                    config,
                    recipe_path,
                )
                suffix = (
                    "\nWarnings: " + "; ".join(warnings) if warnings else ""
                )
                messagebox.showinfo(
                    "Export complete",
                    f"Figure: {output}\nRecipe: {recipe_path}{suffix}",
                )
            except (PlotterError, OSError) as error:
                messagebox.showerror("Export error", str(error))

        def export_statistics(self) -> None:
            if not self.runs:
                return
            DEFAULT_EXPORT_DIRECTORY.mkdir(parents=True, exist_ok=True)
            path = filedialog.asksaveasfilename(
                title="Export statistics",
                defaultextension=".csv",
                initialdir=str(DEFAULT_EXPORT_DIRECTORY),
                filetypes=(("CSV table", "*.csv"),),
            )
            if not path:
                return
            try:
                metrics = self.selected_metric_keys()
                config = self.config_from_controls()
                start, end = resolved_selection(
                    self.runs,
                    config,
                    metrics,
                )
                config = replace(config, start=start, end=end)
                records = compute_statistics(self.runs, metrics, config)
                csv_path = Path(path)
                tex_path = csv_path.with_suffix(".tex")
                write_statistics_csv(records, csv_path)
                write_statistics_latex(records, tex_path)
                messagebox.showinfo(
                    "Export complete",
                    f"CSV: {csv_path}\nLaTeX: {tex_path}",
                )
            except (PlotterError, OSError) as error:
                messagebox.showerror("Export error", str(error))

    try:
        return PlotterWindow().run()
    except tk.TclError as error:
        raise PlotterError(
            "cannot start the Tk GUI because no working display is available; "
            "use --no-gui for headless export"
        ) from error


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Plot CARKit system, node-CPU, and topic-rate monitor CSV files, "
            "overlay pipeline events, and export publication-quality figures "
            "and statistics."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("files", nargs="*", type=Path, help="input CSV files")
    parser.add_argument("--recipe", type=Path, help="load a saved JSON recipe")
    parser.add_argument(
        "--no-gui",
        action="store_true",
        help="export without opening the Tk GUI",
    )
    parser.add_argument(
        "--metrics",
        nargs="+",
        help="metric column keys to plot",
    )
    parser.add_argument(
        "--combine-topic-hz",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="draw all selected Topic Hz metrics on one axis",
    )
    event_group = parser.add_mutually_exclusive_group()
    event_group.add_argument(
        "--event-types",
        nargs="+",
        help="pipeline event types to mark; all types are marked by default",
    )
    event_group.add_argument(
        "--no-events",
        action="store_true",
        help="do not draw pipeline event markers",
    )
    parser.add_argument(
        "--time-mode",
        choices=("elapsed", "timestamp"),
        default=None,
    )
    parser.add_argument(
        "--time-unit",
        choices=tuple(SECONDS_PER_TIME_UNIT),
        default=None,
        help="unit for elapsed --start/--end values and x-axis",
    )
    parser.add_argument("--start", help="numeric elapsed time or ISO timestamp")
    parser.add_argument("--end", help="numeric elapsed time or ISO timestamp")
    parser.add_argument(
        "--language",
        choices=("en", "zh"),
        default=None,
    )
    parser.add_argument(
        "--figure-width",
        choices=("single", "double", "custom"),
        default=None,
    )
    parser.add_argument("--custom-width", type=float, default=None)
    parser.add_argument("--dpi", type=int, choices=(300, 600), default=None)
    parser.add_argument("--title", default=None)
    parser.add_argument("--output", type=Path, help="PDF, SVG, or PNG figure")
    parser.add_argument("--stats-output", type=Path, help="statistics CSV")
    parser.add_argument("--latex-output", type=Path, help="statistics LaTeX")
    return parser


def config_from_arguments(args: argparse.Namespace) -> PlotConfig:
    base = validate_config(
        PlotConfig(
            time_mode=args.time_mode or "elapsed",
            time_unit=args.time_unit or "minutes",
            language=args.language or "en",
            figure_width=args.figure_width or "double",
            custom_width=(
                args.custom_width if args.custom_width is not None else 7.2
            ),
            dpi=args.dpi or 600,
            title=args.title or "",
            combine_topic_hz=bool(args.combine_topic_hz),
            event_types=(
                ()
                if args.no_events
                else (
                    tuple(args.event_types)
                    if args.event_types is not None
                    else None
                )
            ),
        )
    )
    return replace(
        base,
        start=parse_boundary(args.start, base),
        end=parse_boundary(args.end, base),
    )


def apply_recipe_output_overrides(
    config: PlotConfig,
    args: argparse.Namespace,
) -> PlotConfig:
    return replace(
        config,
        dpi=args.dpi if args.dpi is not None else config.dpi,
        title=args.title if args.title is not None else config.title,
        combine_topic_hz=(
            args.combine_topic_hz
            if args.combine_topic_hz is not None
            else config.combine_topic_hz
        ),
        event_types=(
            ()
            if args.no_events
            else (
                tuple(args.event_types)
                if args.event_types is not None
                else config.event_types
            )
        ),
    )


def main(arguments: Optional[Sequence[str]] = None) -> int:
    parser = build_argument_parser()
    args = parser.parse_args(arguments)
    try:
        if args.recipe is not None:
            if args.files or args.metrics or args.start or args.end:
                raise PlotterError(
                    "--recipe cannot be combined with input files, --metrics, "
                    "--start, or --end"
                )
            runs, metrics, config = load_recipe(args.recipe)
            config = apply_recipe_output_overrides(config, args)
        else:
            runs = [load_csv(path) for path in args.files]
            if not runs:
                if args.no_gui:
                    raise PlotterError(
                        "headless mode requires input CSV files or --recipe"
                    )
                metrics = ()
            else:
                requested_metrics = args.metrics or default_metric_keys(runs)
                metrics = (
                    validate_metric_keys(runs, requested_metrics)
                    if requested_metrics
                    else ()
                )
            config = config_from_arguments(args)

        if runs:
            resolved_event_types(runs, config)
        if not args.no_gui:
            return launch_gui(runs, metrics, config)

        output = args.output
        if (
            output is None
            and args.recipe is not None
            and not (args.stats_output or args.latex_output)
        ):
            output = args.recipe.with_suffix(".pdf")
        if not (output or args.stats_output or args.latex_output):
            raise PlotterError(
                "--no-gui requires --output, --stats-output, or --latex-output"
            )
        resolved_selection(runs, config, metrics)
        if output:
            warnings = save_figure(runs, metrics, config, output)
            print(f"Wrote figure: {output.resolve()}")
            for warning in warnings:
                print(f"Warning: {warning}", file=sys.stderr)
        records: Optional[tuple[StatisticsRecord, ...]] = None
        if args.stats_output or args.latex_output:
            records = compute_statistics(runs, metrics, config)
        if args.stats_output:
            assert records is not None
            write_statistics_csv(records, args.stats_output)
            print(f"Wrote statistics CSV: {args.stats_output.resolve()}")
        if args.latex_output:
            assert records is not None
            write_statistics_latex(records, args.latex_output)
            print(f"Wrote statistics LaTeX: {args.latex_output.resolve()}")
        return 0
    except (PlotterError, OSError, UnicodeError, csv.Error, ValueError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
