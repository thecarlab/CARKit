#!/usr/bin/env python3
"""Display or record Jetson CPU, thermal, and INA3221 power metrics.

This monitor deliberately reads Linux procfs/sysfs directly.  It does not
require ROS, a desktop session, jtop, or parsing the human-oriented tegrastats
output.  All INA3221 files are read-only; this tool never changes power limits.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import re
import sys
import time
from typing import Optional, Sequence, TextIO


THERMAL_NAMES = (
    "cpu-thermal",
    "cv0-thermal",
    "cv1-thermal",
    "cv2-thermal",
    "gpu-thermal",
    "soc0-thermal",
    "soc1-thermal",
    "soc2-thermal",
    "tj-thermal",
)
POWER_RAIL_NAMES = (
    "VDD_IN",
    "VDD_CPU_GPU_CV",
    "VDD_SOC",
)
MIN_INTERVAL_SECONDS = 0.2


@dataclass(frozen=True)
class SystemPaths:
    proc_stat: Path = Path("/proc/stat")
    cpu_root: Path = Path("/sys/devices/system/cpu")
    thermal_root: Path = Path("/sys/class/thermal")
    hwmon_root: Path = Path("/sys/class/hwmon")


@dataclass(frozen=True)
class CpuTimes:
    total: int
    idle: int


@dataclass(frozen=True)
class CpuCore:
    index: int
    online_path: Optional[Path]
    frequency_path: Optional[Path]


@dataclass(frozen=True)
class CpuReading:
    index: int
    online: bool
    frequency_hz: Optional[int]


@dataclass(frozen=True)
class TemperatureSensor:
    name: str
    temperature_path: Optional[Path]


@dataclass(frozen=True)
class TemperatureReading:
    value_c: Optional[float]
    status: str


@dataclass(frozen=True)
class PowerRail:
    name: str
    voltage_path: Optional[Path]
    current_path: Optional[Path]


@dataclass(frozen=True)
class PowerReading:
    voltage_v: Optional[float]
    current_a: Optional[float]
    power_w: Optional[float]
    average_power_w: Optional[float]


@dataclass(frozen=True)
class MonitorSample:
    timestamp: datetime
    elapsed_seconds: float
    sample_number: int
    cpu_percent: Optional[float]
    cpus: tuple[CpuReading, ...]
    temperatures: dict[str, TemperatureReading]
    power: dict[str, PowerReading]


class RunningAverage:
    def __init__(self) -> None:
        self.count = 0
        self.value: Optional[float] = None

    def add(self, sample: Optional[float]) -> Optional[float]:
        if sample is None:
            return self.value
        self.count += 1
        if self.value is None:
            self.value = sample
        else:
            self.value += (sample - self.value) / self.count
        return self.value


def read_text(path: Path) -> Optional[str]:
    try:
        raw = path.read_bytes()
        # Some Jetson sysfs attributes return no buffer while their sensor is
        # offline instead of returning b"" or raising a conventional OSError.
        if raw is None:
            return None
        return raw.decode("utf-8").strip()
    except (OSError, UnicodeError):
        return None


def read_int(path: Optional[Path]) -> Optional[int]:
    if path is None:
        return None
    value = read_text(path)
    if value is None or value == "":
        return None
    try:
        return int(value)
    except ValueError:
        return None


def read_cpu_times(proc_stat: Path) -> Optional[CpuTimes]:
    text = read_text(proc_stat)
    if text is None:
        return None
    for line in text.splitlines():
        fields = line.split()
        if not fields or fields[0] != "cpu":
            continue
        try:
            values = [int(value) for value in fields[1:]]
        except ValueError:
            return None
        if len(values) < 4:
            return None
        idle = values[3] + (values[4] if len(values) > 4 else 0)
        return CpuTimes(total=sum(values), idle=idle)
    return None


def cpu_percent(
    previous: Optional[CpuTimes],
    current: Optional[CpuTimes],
) -> Optional[float]:
    if previous is None or current is None:
        return None
    total_delta = current.total - previous.total
    idle_delta = current.idle - previous.idle
    if total_delta <= 0:
        return None
    busy_fraction = 1.0 - (idle_delta / total_delta)
    return min(100.0, max(0.0, busy_fraction * 100.0))


def cpu_index(path: Path) -> Optional[int]:
    match = re.fullmatch(r"cpu(\d+)", path.name)
    return int(match.group(1)) if match else None


def frequency_file(cpufreq_path: Path) -> Optional[Path]:
    try:
        resolved = cpufreq_path.resolve(strict=True)
    except OSError:
        return None
    for filename in ("scaling_cur_freq", "cpuinfo_cur_freq"):
        candidate = resolved / filename
        if candidate.is_file():
            return candidate
    return None


def discover_cpus(cpu_root: Path) -> tuple[CpuCore, ...]:
    cores: list[CpuCore] = []
    try:
        candidates = list(cpu_root.glob("cpu[0-9]*"))
    except OSError:
        candidates = []
    for path in candidates:
        index = cpu_index(path)
        if index is None or not path.is_dir():
            continue
        online = path / "online"
        cores.append(
            CpuCore(
                index=index,
                online_path=online if online.exists() else None,
                frequency_path=frequency_file(path / "cpufreq"),
            )
        )
    return tuple(sorted(cores, key=lambda core: core.index))


def read_cpu_frequencies(cores: Sequence[CpuCore]) -> tuple[CpuReading, ...]:
    # CPU cores in one cpufreq policy share one file. Read it once per sample so
    # all members receive a coherent value instead of values from different
    # instants while the governor changes frequency.
    frequency_cache: dict[Path, Optional[int]] = {}
    readings: list[CpuReading] = []
    for core in cores:
        online_value = read_int(core.online_path)
        online = online_value != 0 if core.online_path is not None else True
        frequency_hz: Optional[int] = None
        if online and core.frequency_path is not None:
            path = core.frequency_path
            if path not in frequency_cache:
                frequency_khz = read_int(path)
                frequency_cache[path] = (
                    frequency_khz * 1000 if frequency_khz is not None else None
                )
            frequency_hz = frequency_cache[path]
        readings.append(CpuReading(core.index, online, frequency_hz))
    return tuple(readings)


def discover_thermal_sensors(thermal_root: Path) -> tuple[TemperatureSensor, ...]:
    discovered: dict[str, Path] = {}
    try:
        zones = list(thermal_root.glob("thermal_zone*"))
    except OSError:
        zones = []
    for zone in zones:
        name = read_text(zone / "type")
        if name in THERMAL_NAMES:
            discovered[name] = zone / "temp"
    return tuple(
        TemperatureSensor(name, discovered.get(name)) for name in THERMAL_NAMES
    )


def read_temperature(sensor: TemperatureSensor) -> TemperatureReading:
    if sensor.temperature_path is None:
        return TemperatureReading(None, "unavailable")
    try:
        raw_bytes = sensor.temperature_path.read_bytes()
    except BlockingIOError:
        return TemperatureReading(None, "offline")
    except OSError:
        return TemperatureReading(None, "unavailable")
    if raw_bytes is None or raw_bytes.strip() == b"":
        return TemperatureReading(None, "offline")
    try:
        raw = raw_bytes.decode("utf-8").strip()
    except UnicodeError:
        return TemperatureReading(None, "unavailable")
    try:
        return TemperatureReading(float(raw) / 1000.0, "ok")
    except ValueError:
        return TemperatureReading(None, "unavailable")


def discover_power_rails(hwmon_root: Path) -> tuple[PowerRail, ...]:
    discovered: dict[str, PowerRail] = {}
    try:
        hwmons = list(hwmon_root.glob("hwmon*"))
    except OSError:
        hwmons = []
    for hwmon in hwmons:
        if read_text(hwmon / "name") != "ina3221":
            continue
        for channel in range(1, 4):
            label = read_text(hwmon / f"in{channel}_label")
            if label not in POWER_RAIL_NAMES:
                continue
            voltage_path = hwmon / f"in{channel}_input"
            current_path = hwmon / f"curr{channel}_input"
            discovered[label] = PowerRail(
                name=label,
                voltage_path=voltage_path if voltage_path.exists() else None,
                current_path=current_path if current_path.exists() else None,
            )
    return tuple(
        discovered.get(name, PowerRail(name, None, None))
        for name in POWER_RAIL_NAMES
    )


def read_power_rail(
    rail: PowerRail,
    average: RunningAverage,
) -> PowerReading:
    voltage_mv = read_int(rail.voltage_path)
    current_ma = read_int(rail.current_path)
    voltage_v = voltage_mv / 1000.0 if voltage_mv is not None else None
    current_a = current_ma / 1000.0 if current_ma is not None else None
    power_w = (
        voltage_v * current_a
        if voltage_v is not None and current_a is not None
        else None
    )
    return PowerReading(
        voltage_v=voltage_v,
        current_a=current_a,
        power_w=power_w,
        average_power_w=average.add(power_w),
    )


def csv_name(name: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")


def csv_fieldnames(cpu_indices: Sequence[int]) -> list[str]:
    fields = ["timestamp_iso", "elapsed_s", "sample", "cpu_total_percent"]
    fields.extend(f"cpu{index}_freq_hz" for index in cpu_indices)
    fields.extend(f"temp_{csv_name(name)}_c" for name in THERMAL_NAMES)
    for rail in POWER_RAIL_NAMES:
        prefix = csv_name(rail)
        fields.extend(
            (
                f"{prefix}_voltage_v",
                f"{prefix}_current_a",
                f"{prefix}_power_w",
                f"{prefix}_avg_power_w",
            )
        )
    return fields


def formatted(value: Optional[float], digits: int) -> str:
    return "" if value is None else f"{value:.{digits}f}"


def sample_to_csv_row(sample: MonitorSample) -> dict[str, object]:
    row: dict[str, object] = {
        "timestamp_iso": sample.timestamp.isoformat(timespec="milliseconds"),
        "elapsed_s": f"{sample.elapsed_seconds:.3f}",
        "sample": sample.sample_number,
        "cpu_total_percent": formatted(sample.cpu_percent, 1),
    }
    for cpu in sample.cpus:
        row[f"cpu{cpu.index}_freq_hz"] = (
            cpu.frequency_hz if cpu.online and cpu.frequency_hz is not None else ""
        )
    for name in THERMAL_NAMES:
        reading = sample.temperatures[name]
        row[f"temp_{csv_name(name)}_c"] = formatted(reading.value_c, 3)
    for rail in POWER_RAIL_NAMES:
        reading = sample.power[rail]
        prefix = csv_name(rail)
        row[f"{prefix}_voltage_v"] = formatted(reading.voltage_v, 3)
        row[f"{prefix}_current_a"] = formatted(reading.current_a, 3)
        row[f"{prefix}_power_w"] = formatted(reading.power_w, 3)
        row[f"{prefix}_avg_power_w"] = formatted(reading.average_power_w, 3)
    return row


def display_number(value: Optional[float], suffix: str, digits: int = 2) -> str:
    return "unavailable" if value is None else f"{value:.{digits}f} {suffix}"


def print_sample(
    sample: MonitorSample,
    interval_seconds: float,
    clear_screen: bool,
    output_path: Optional[Path],
) -> None:
    if clear_screen and sys.stdout.isatty():
        print("\033[2J\033[H", end="")
    print(
        "Jetson System Monitor | "
        f"{sample.timestamp.isoformat(timespec='seconds')} | "
        f"sample {sample.sample_number} | interval {interval_seconds:.1f}s"
    )
    if output_path is not None:
        print(f"Recording CSV: {output_path}")
    print(
        "Total CPU usage (previous interval): "
        + display_number(sample.cpu_percent, "%", 1)
    )

    print("\nCPU frequencies")
    print(f"{'CPU':>5}  {'STATE':8}  {'FREQUENCY':>13}")
    for cpu in sample.cpus:
        if not cpu.online:
            state = "offline"
            frequency = "offline"
        else:
            state = "online"
            frequency = (
                f"{cpu.frequency_hz / 1_000_000.0:.1f} MHz"
                if cpu.frequency_hz is not None
                else "unavailable"
            )
        print(f"{('CPU' + str(cpu.index)):>5}  {state:8}  {frequency:>13}")
    if not sample.cpus:
        print("  unavailable")

    print("\nTemperatures")
    print(f"{'SENSOR':20}  {'TEMPERATURE':>13}")
    for name in THERMAL_NAMES:
        reading = sample.temperatures[name]
        value = (
            f"{reading.value_c:.1f} C"
            if reading.value_c is not None
            else reading.status
        )
        print(f"{name:20}  {value:>13}")

    print("\nPower rails (VDD_IN is total module input; rails are not additive)")
    print(
        f"{'RAIL':20} {'VOLTAGE':>10} {'CURRENT':>10} "
        f"{'INST':>10} {'RUN AVG':>10}"
    )
    for name in POWER_RAIL_NAMES:
        reading = sample.power[name]
        voltage = display_number(reading.voltage_v, "V", 3)
        current = display_number(reading.current_a, "A", 3)
        power = display_number(reading.power_w, "W", 3)
        average = display_number(reading.average_power_w, "W", 3)
        print(
            f"{name:20} {voltage:>10} {current:>10} "
            f"{power:>10} {average:>10}"
        )
    sys.stdout.flush()


class JetsonMonitor:
    def __init__(self, paths: SystemPaths = SystemPaths()) -> None:
        self.paths = paths
        self.cpus = discover_cpus(paths.cpu_root)
        self.thermal_sensors = discover_thermal_sensors(paths.thermal_root)
        self.power_rails = discover_power_rails(paths.hwmon_root)
        self.power_averages = {
            name: RunningAverage() for name in POWER_RAIL_NAMES
        }

    @property
    def cpu_indices(self) -> tuple[int, ...]:
        return tuple(cpu.index for cpu in self.cpus)

    def collect(
        self,
        previous_cpu_times: Optional[CpuTimes],
        start_time: float,
        sample_number: int,
    ) -> tuple[MonitorSample, Optional[CpuTimes]]:
        current_cpu_times = read_cpu_times(self.paths.proc_stat)
        temperatures = {
            sensor.name: read_temperature(sensor)
            for sensor in self.thermal_sensors
        }
        power = {
            rail.name: read_power_rail(
                rail,
                self.power_averages[rail.name],
            )
            for rail in self.power_rails
        }
        now = time.monotonic()
        sample = MonitorSample(
            timestamp=datetime.now().astimezone(),
            elapsed_seconds=now - start_time,
            sample_number=sample_number,
            cpu_percent=cpu_percent(previous_cpu_times, current_cpu_times),
            cpus=read_cpu_frequencies(self.cpus),
            temperatures=temperatures,
            power=power,
        )
        return sample, current_cpu_times or previous_cpu_times


def default_output_path() -> Path:
    repository_root = Path(__file__).resolve().parents[2]
    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    filename = f"system_metrics_{timestamp}.csv"
    return repository_root / "log" / "system_monitor" / filename


def run_monitor(
    mode: str,
    interval_seconds: float,
    no_clear: bool,
    output_path: Optional[Path],
) -> int:
    monitor = JetsonMonitor()
    csv_file: Optional[TextIO] = None
    writer: Optional[csv.DictWriter] = None

    if mode == "mode2":
        output_path = output_path or default_output_path()
        try:
            output_path.parent.mkdir(parents=True, exist_ok=True)
            csv_file = output_path.open("w", encoding="utf-8", newline="")
        except OSError as error:
            message = f"Cannot open CSV output '{output_path}': {error}"
            raise SystemExit(message) from error
        writer = csv.DictWriter(
            csv_file,
            fieldnames=csv_fieldnames(monitor.cpu_indices),
        )
        writer.writeheader()
        csv_file.flush()

    start_time = time.monotonic()
    previous_cpu_times = read_cpu_times(monitor.paths.proc_stat)
    next_sample_time = start_time + interval_seconds
    sample_number = 0
    try:
        while True:
            delay = next_sample_time - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            sample_number += 1
            sample, previous_cpu_times = monitor.collect(
                previous_cpu_times,
                start_time,
                sample_number,
            )
            print_sample(
                sample,
                interval_seconds,
                clear_screen=not no_clear,
                output_path=output_path if mode == "mode2" else None,
            )
            if writer is not None and csv_file is not None:
                writer.writerow(sample_to_csv_row(sample))
                csv_file.flush()

            next_sample_time += interval_seconds
            if next_sample_time <= time.monotonic():
                next_sample_time = time.monotonic() + interval_seconds
    except KeyboardInterrupt:
        print("\nJetson system monitor stopped.")
        if output_path is not None:
            print(f"CSV saved to: {output_path}")
        return 0
    finally:
        if csv_file is not None:
            csv_file.close()


def parse_arguments(arguments: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Display or record Jetson CPU, thermal, and power metrics."
    )
    parser.add_argument(
        "mode",
        choices=("mode1", "mode2"),
        help="mode1 displays metrics; mode2 displays and records CSV",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help="Sampling interval in seconds (default: 1.0; minimum: 0.2)",
    )
    parser.add_argument(
        "--no-clear",
        action="store_true",
        help="Append samples instead of refreshing the terminal in place",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Mode 2 CSV path (default: log/system_monitor/<timestamp>.csv)",
    )
    parsed = parser.parse_args(arguments)
    if parsed.interval < MIN_INTERVAL_SECONDS:
        parser.error(f"--interval must be at least {MIN_INTERVAL_SECONDS:.1f} seconds")
    if parsed.mode == "mode1" and parsed.output is not None:
        parser.error("--output is only valid in mode2")
    return parsed


def main(arguments: Optional[Sequence[str]] = None) -> int:
    parsed = parse_arguments(arguments)
    return run_monitor(
        mode=parsed.mode,
        interval_seconds=parsed.interval,
        no_clear=parsed.no_clear,
        output_path=parsed.output,
    )


if __name__ == "__main__":
    raise SystemExit(main())
