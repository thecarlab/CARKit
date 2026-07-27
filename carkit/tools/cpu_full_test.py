#!/usr/bin/env python3
"""Keep every CPU available to this process busy with a repeatable workload.

This program is a load generator, not a clock controller.  The kernel and the
Jetson firmware remain responsible for DVFS, power limits, and thermal
throttling.  Run jetson_system_monitor.py separately to record those effects.
"""

from __future__ import annotations

import argparse
import ctypes
import gc
import multiprocessing as mp
import os
from pathlib import Path
import signal
import sys
import time
from typing import Optional, Sequence


BATCH_ITERATIONS = 50_000
COUNTER_PADDING = 8
MASK_64 = (1 << 64) - 1
DEFAULT_DURATION_SECONDS = 300.0
DEFAULT_STATUS_INTERVAL_SECONDS = 2.0
DEFAULT_MAX_TEMPERATURE_C = 90.0


def available_cpus() -> tuple[int, ...]:
    """Return CPUs this process may use, respecting Docker/cgroup affinity."""
    try:
        return tuple(sorted(os.sched_getaffinity(0)))
    except AttributeError:
        count = os.cpu_count() or 1
        return tuple(range(count))


def discover_cpu_temperature_paths() -> tuple[tuple[str, Path], ...]:
    """Find Linux thermal zones whose names identify CPU/package sensors."""
    sensors: list[tuple[str, Path]] = []
    for zone in sorted(Path("/sys/class/thermal").glob("thermal_zone*")):
        try:
            name = (zone / "type").read_text(encoding="utf-8").strip()
        except OSError:
            continue
        normalized = name.lower()
        if "cpu" in normalized or "package" in normalized:
            sensors.append((name, zone / "temp"))
    return tuple(sensors)


def read_hottest_cpu_temperature(
    sensors: Sequence[tuple[str, Path]],
) -> tuple[Optional[float], Optional[str]]:
    """Read the hottest available CPU temperature in degrees Celsius."""
    readings: list[tuple[float, str]] = []
    for name, path in sensors:
        try:
            raw = float(path.read_text(encoding="utf-8").strip())
        except (OSError, ValueError):
            continue
        temperature_c = raw / 1000.0 if abs(raw) >= 1000.0 else raw
        readings.append((temperature_c, name))
    if not readings:
        return None, None
    return max(readings, key=lambda reading: reading[0])


def worker(
    cpu_id: int,
    worker_index: int,
    stop_event: mp.synchronize.Event,
    counters: object,
    pin_cpu: bool,
) -> None:
    """Run an integer-mixing loop until the parent asks this worker to stop."""
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    gc.disable()

    if pin_cpu:
        try:
            os.sched_setaffinity(0, {cpu_id})
        except (AttributeError, OSError) as error:
            print(
                f"Warning: worker {worker_index} could not bind to CPU "
                f"{cpu_id}: {error}",
                file=sys.stderr,
                flush=True,
            )

    # Each worker uses a different seed. Python cannot optimise this loop away,
    # and separate processes avoid the GIL limiting the load to one CPU core.
    value = ((worker_index + 1) * 0x9E3779B97F4A7C15) & MASK_64
    completed_batches = 0
    counter_slot = worker_index * COUNTER_PADDING
    while not stop_event.is_set():
        for _ in range(BATCH_ITERATIONS):
            value = (value * 6364136223846793005 + 1442695040888963407) & MASK_64
            value ^= value >> 23
        completed_batches += 1
        counters[counter_slot] = completed_batches


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return parsed


def nonnegative_float(value: str) -> float:
    parsed = float(value)
    if parsed < 0.0:
        raise argparse.ArgumentTypeError("must be zero or greater")
    return parsed


def positive_int(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return parsed


def parse_arguments(arguments: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Saturate all CPUs available to the process with one pinned "
            "worker process per CPU."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--duration",
        type=nonnegative_float,
        default=DEFAULT_DURATION_SECONDS,
        metavar="SECONDS",
        help="run time; use 0 to continue until Ctrl+C",
    )
    parser.add_argument(
        "--workers",
        type=positive_int,
        metavar="COUNT",
        help="number of CPU workers; default is every available CPU",
    )
    parser.add_argument(
        "--status-interval",
        type=positive_float,
        default=DEFAULT_STATUS_INTERVAL_SECONDS,
        metavar="SECONDS",
        help="console status interval",
    )
    parser.add_argument(
        "--max-temp",
        type=nonnegative_float,
        default=DEFAULT_MAX_TEMPERATURE_C,
        metavar="CELSIUS",
        help="stop at this CPU temperature; use 0 to disable",
    )
    parser.add_argument(
        "--no-affinity",
        action="store_true",
        help="do not pin each worker to a distinct CPU",
    )
    parsed = parser.parse_args(arguments)

    cpu_count = len(available_cpus())
    if parsed.workers is not None and parsed.workers > cpu_count:
        parser.error(
            f"--workers cannot exceed the {cpu_count} CPUs available to "
            "this container"
        )
    return parsed


def run(arguments: argparse.Namespace) -> int:
    cpus = available_cpus()
    worker_count = arguments.workers or len(cpus)
    selected_cpus = cpus[:worker_count]
    thermal_sensors = discover_cpu_temperature_paths()

    context = mp.get_context("fork")
    stop_event = context.Event()
    counters = context.RawArray(
        ctypes.c_ulonglong,
        worker_count * COUNTER_PADDING,
    )
    processes = [
        context.Process(
            target=worker,
            args=(
                cpu_id,
                index,
                stop_event,
                counters,
                not arguments.no_affinity,
            ),
            name=f"cpu-load-{cpu_id}",
        )
        for index, cpu_id in enumerate(selected_cpus)
    ]

    duration_description = (
        "until Ctrl+C"
        if arguments.duration == 0.0
        else f"{arguments.duration:.1f} seconds"
    )
    print(f"Available CPUs: {','.join(str(cpu) for cpu in cpus)}")
    print(
        f"Starting {worker_count} workers for {duration_description}; "
        f"CPU affinity is {'off' if arguments.no_affinity else 'on'}."
    )
    if arguments.max_temp == 0.0:
        print("Software temperature limit: disabled")
    elif thermal_sensors:
        names = ", ".join(name for name, _ in thermal_sensors)
        print(
            f"Software temperature limit: {arguments.max_temp:.1f} C "
            f"(sensors: {names})"
        )
    else:
        print(
            "Warning: no CPU thermal sensor was found; the requested software "
            "temperature limit cannot be enforced.",
            file=sys.stderr,
        )
    print(
        "This generates load only; current frequency is controlled by the "
        "kernel/Jetson firmware."
    )

    for process in processes:
        process.start()

    start_time = time.monotonic()
    previous_status_time = start_time
    previous_iterations = 0
    exit_code = 0
    stop_reason = "requested"

    try:
        while True:
            now = time.monotonic()
            elapsed = now - start_time
            if arguments.duration > 0.0 and elapsed >= arguments.duration:
                stop_reason = "duration reached"
                break

            dead_workers = [
                process for process in processes if not process.is_alive()
            ]
            if dead_workers:
                names = ", ".join(process.name for process in dead_workers)
                print(f"Error: workers exited unexpectedly: {names}", file=sys.stderr)
                stop_reason = "worker failure"
                exit_code = 1
                break

            wait_time = arguments.status_interval
            if arguments.duration > 0.0:
                wait_time = min(wait_time, arguments.duration - elapsed)
            if stop_event.wait(max(0.0, wait_time)):
                stop_reason = "stop event"
                break

            now = time.monotonic()
            elapsed = now - start_time
            total_batches = sum(
                counters[index * COUNTER_PADDING]
                for index in range(worker_count)
            )
            total_iterations = total_batches * BATCH_ITERATIONS
            status_elapsed = now - previous_status_time
            iterations_per_second = (
                (total_iterations - previous_iterations) / status_elapsed
                if status_elapsed > 0.0
                else 0.0
            )
            temperature_c, temperature_name = read_hottest_cpu_temperature(
                thermal_sensors
            )
            temperature_text = (
                f"{temperature_c:.1f} C ({temperature_name})"
                if temperature_c is not None
                else "unavailable"
            )
            alive_count = sum(process.is_alive() for process in processes)
            print(
                f"elapsed={elapsed:8.1f}s  workers={alive_count}/{worker_count}  "
                f"rate={iterations_per_second / 1_000_000:9.2f} Miter/s  "
                f"temperature={temperature_text}",
                flush=True,
            )
            previous_status_time = now
            previous_iterations = total_iterations

            if (
                arguments.max_temp > 0.0
                and temperature_c is not None
                and temperature_c >= arguments.max_temp
            ):
                print(
                    f"Temperature limit reached: {temperature_c:.1f} C >= "
                    f"{arguments.max_temp:.1f} C",
                    file=sys.stderr,
                )
                stop_reason = "temperature limit reached"
                exit_code = 2
                break
    except KeyboardInterrupt:
        stop_reason = "Ctrl+C"
    finally:
        stop_event.set()
        for process in processes:
            process.join(timeout=3.0)
        for process in processes:
            if process.is_alive():
                process.terminate()
                process.join(timeout=1.0)

    print(f"CPU load stopped: {stop_reason}.")
    return exit_code


def main(arguments: Optional[Sequence[str]] = None) -> int:
    return run(parse_arguments(arguments))


if __name__ == "__main__":
    raise SystemExit(main())
