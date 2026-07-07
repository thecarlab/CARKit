#!/usr/bin/env python3
"""Burn CPU to test whether scheduler load affects camera / perception rates."""

from __future__ import annotations

import argparse
import multiprocessing as mp
import os
import re
import signal
import sys
import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from multiprocessing.sharedctypes import Synchronized


_PERIOD_SEC = 0.10

_stop_main = False
_workers: list[mp.Process] = []
_worker_stops: list[mp.synchronize.Event] = []
_load_percent = 100.0
_load_shared: Synchronized | None = None


def _init_load_shared(initial_load: float) -> None:
    global _load_percent, _load_shared
    _load_percent = max(0.0, min(100.0, initial_load))
    _load_shared = mp.Value("d", _load_percent)


def _set_load_percent(load_percent: float) -> None:
    global _load_percent
    _load_percent = max(0.0, min(100.0, load_percent))
    if _load_shared is not None:
        _load_shared.value = _load_percent


def _busy_until(stop_event: mp.synchronize.Event, deadline: float) -> bool:
    """Spin until deadline. Return True if stop_event was set."""
    while not stop_event.is_set():
        if time.perf_counter() >= deadline:
            return False
    return True


def _cpu_worker(load_shared: Synchronized, stop_event: mp.synchronize.Event) -> None:
    while not stop_event.is_set():
        load_percent = max(0.0, min(100.0, load_shared.value))

        if load_percent <= 0.0:
            if stop_event.wait(_PERIOD_SEC):
                return
            continue

        if load_percent >= 99.9:
            if _busy_until(stop_event, time.perf_counter() + _PERIOD_SEC):
                return
            continue

        busy_sec = _PERIOD_SEC * load_percent / 100.0
        idle_sec = _PERIOD_SEC - busy_sec
        if _busy_until(stop_event, time.perf_counter() + busy_sec):
            return
        if idle_sec > 0.0 and stop_event.wait(idle_sec):
            return


def _reap_dead_workers() -> None:
    alive_workers: list[mp.Process] = []
    alive_stops: list[mp.synchronize.Event] = []
    for process, stop_event in zip(_workers, _worker_stops):
        if process.is_alive():
            alive_workers.append(process)
            alive_stops.append(stop_event)
            continue
        process.join(timeout=0.1)
    if len(alive_workers) != len(_workers):
        _workers[:] = alive_workers
        _worker_stops[:] = alive_stops


def _stop_worker(index: int) -> None:
    if index < 0 or index >= len(_workers):
        return

    process = _workers[index]
    _worker_stops[index].set()
    process.join(timeout=2.0)
    if process.is_alive():
        process.kill()
        process.join(timeout=1.0)

    _workers.pop(index)
    _worker_stops.pop(index)


def _stop_all_workers() -> None:
    while _workers:
        _stop_worker(len(_workers) - 1)


def _start_worker() -> None:
    if _load_shared is None:
        raise RuntimeError("load shared value is not initialized")

    stop_event = mp.Event()
    process = mp.Process(
        target=_cpu_worker,
        args=(_load_shared, stop_event),
        daemon=True,
    )
    process.start()
    _workers.append(process)
    _worker_stops.append(stop_event)


def active_cores() -> int:
    _reap_dead_workers()
    return len(_workers)


def set_target_cores(count: int, load_percent: float | None = None) -> None:
    max_cores = os.cpu_count() or 1
    count = max(0, min(int(count), max_cores))

    if load_percent is not None:
        _set_load_percent(load_percent)

    while active_cores() > count:
        _stop_worker(active_cores() - 1)

    while active_cores() < count:
        _start_worker()


def adjust_cores(delta: int) -> None:
    set_target_cores(active_cores() + int(delta))


def print_status() -> None:
    max_cores = os.cpu_count() or 1
    cores = active_cores()
    approx_cpu = cores * _load_percent
    print(
        f"burning {cores} / {max_cores} worker(s) "
        f"at {_load_percent:.0f}% each "
        f"(~{approx_cpu:.0f}% CPU, "
        f"~{approx_cpu / max_cores:.0f}% of all cores)",
        flush=True,
    )


def _handle_stop(_signum: int, _frame) -> None:
    global _stop_main
    _stop_main = True
    _stop_all_workers()
    print("stopping workers...", flush=True)


def _parse_command(line: str) -> bool:
    text = line.strip().lower()
    if not text:
        return True
    if text in {"q", "quit", "exit"}:
        return False
    if text in {"s", "status"}:
        print_status()
        return True
    if text in {"off", "stop", "clear"}:
        set_target_cores(0)
        print_status()
        return True

    load_match = re.fullmatch(r"load\s*(\d+(?:\.\d+)?)", text)
    if load_match:
        if active_cores() == 0:
            print("no workers running; use set <cores> [load] first", flush=True)
            return True
        _set_load_percent(float(load_match.group(1)))
        print(f"live load -> {_load_percent:.0f}% on {active_cores()} worker(s)", flush=True)
        print_status()
        return True

    set_match = re.fullmatch(r"set\s*(\d+)\s*(\d+(?:\.\d+)?)?", text)
    if set_match:
        cores = int(set_match.group(1))
        load = float(set_match.group(2)) if set_match.group(2) is not None else None
        set_target_cores(cores, load)
        print_status()
        return True

    delta_match = re.fullmatch(r"([+-])\s*(\d+)", text)
    if delta_match:
        sign = 1 if delta_match.group(1) == "+" else -1
        adjust_cores(sign * int(delta_match.group(2)))
        print_status()
        return True

    print(
        "commands: +<n>  -<n>  set <cores> [load]  load <percent>  "
        "off  status  quit",
        file=sys.stderr,
    )
    return True


def run_interactive() -> None:
    print("cpu_occupier interactive mode", flush=True)
    print(
        "examples: set 4 60  load 20  +1  off  status  quit",
        flush=True,
    )
    print_status()
    while not _stop_main:
        try:
            line = input("cpu> ")
        except EOFError:
            break
        if not _parse_command(line):
            break


def run_hold(cores: int, load_percent: float, interval_sec: float) -> None:
    set_target_cores(cores, load_percent)
    print_status()
    print(
        "hold mode: use Ctrl+C to stop (typed commands are ignored; "
        "re-run with --interactive to use load/set/off)",
        flush=True,
    )
    while not _stop_main:
        time.sleep(interval_sec)
        if not _stop_main:
            print_status()


def run_ramp(
    target_cores: int,
    step_cores: int,
    load_percent: float,
    interval_sec: float,
) -> None:
    print(
        f"ramping to {target_cores} core(s) at {load_percent:.0f}% "
        f"in steps of {step_cores} every {interval_sec:.1f}s",
        flush=True,
    )
    while not _stop_main and active_cores() < target_cores:
        adjust_cores(step_cores)
        if active_cores() > target_cores:
            set_target_cores(target_cores, load_percent)
        print_status()
        time.sleep(interval_sec)
    if not _stop_main:
        set_target_cores(target_cores, load_percent)
        print("ramp complete; press Ctrl+C to stop workers and exit", flush=True)
    while not _stop_main:
        time.sleep(interval_sec)
        if not _stop_main:
            print_status()


def build_parser() -> argparse.ArgumentParser:
    max_cores = os.cpu_count() or 1
    parser = argparse.ArgumentParser(
        description=(
            "Spawn CPU-burn worker processes to test whether scheduler load "
            "affects RealSense image rate or perception latency."
        ),
    )
    parser.add_argument(
        "--cores",
        type=int,
        default=0,
        help=f"Number of worker processes to run (1-{max_cores}).",
    )
    parser.add_argument(
        "--load-percent",
        type=float,
        default=100.0,
        help="Per-worker CPU duty cycle 0-100 (default: 100).",
    )
    parser.add_argument(
        "--ramp-to-cores",
        type=int,
        default=None,
        help="Gradually add workers until this many cores are busy.",
    )
    parser.add_argument(
        "--step-cores",
        type=int,
        default=1,
        help="Workers added per ramp step (default: 1).",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=5.0,
        help="Seconds between status prints / ramp steps (default: 5).",
    )
    parser.add_argument(
        "-i",
        "--interactive",
        action="store_true",
        help="Adjust active workers from stdin (+1, set 4 80, load 50, off).",
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    _init_load_shared(args.load_percent)

    signal.signal(signal.SIGINT, _handle_stop)
    signal.signal(signal.SIGTERM, _handle_stop)

    try:
        if args.interactive:
            run_interactive()
        elif args.ramp_to_cores is not None:
            run_ramp(
                args.ramp_to_cores,
                max(1, args.step_cores),
                args.load_percent,
                args.interval,
            )
        elif args.cores > 0:
            run_hold(args.cores, args.load_percent, args.interval)
        else:
            parser.print_help()
            return 1
    finally:
        stopped = active_cores()
        _stop_all_workers()
        if stopped:
            print(f"stopped {stopped} worker(s)", flush=True)

    return 0


if __name__ == "__main__":
    mp.freeze_support()
    raise SystemExit(main())
