#!/usr/bin/env python3
"""Hold process RSS to simulate memory pressure on the Jetson / in Docker."""

from __future__ import annotations

import argparse
import signal
import sys
import time


_PAGE_BYTES = 4096
_chunks: list[bytearray] = []
_stop = False


def _touch_pages(chunk: bytearray) -> None:
    for offset in range(0, len(chunk), _PAGE_BYTES):
        chunk[offset] = 1


def _allocate_mb(size_mb: int) -> None:
    if size_mb <= 0:
        return
    chunk = bytearray(size_mb * 1024 * 1024)
    _touch_pages(chunk)
    _chunks.append(chunk)


def _release_mb(size_mb: int) -> None:
    if size_mb <= 0:
        return
    remaining = size_mb * 1024 * 1024
    while remaining > 0 and _chunks:
        last = _chunks[-1]
        if len(last) <= remaining:
            remaining -= len(last)
            _chunks.pop()
        else:
            keep = len(last) - remaining
            _chunks[-1] = last[:keep]
            remaining = 0


def held_mb() -> float:
    return sum(len(chunk) for chunk in _chunks) / (1024 * 1024)


def set_target_mb(target_mb: float) -> None:
    target_mb = max(0.0, target_mb)
    current = held_mb()
    if target_mb > current:
        _allocate_mb(int(target_mb - current))
    elif target_mb < current:
        _release_mb(int(current - target_mb))


def adjust_mb(delta_mb: float) -> None:
    if delta_mb >= 0:
        _allocate_mb(int(delta_mb))
    else:
        _release_mb(int(-delta_mb))


def print_status() -> None:
    print(f"holding {held_mb():.1f} MiB in {len(_chunks)} chunk(s)", flush=True)


def _handle_stop(_signum: int, _frame) -> None:
    global _stop
    _stop = True


def _parse_command(line: str) -> bool:
    text = line.strip().lower()
    if not text:
        return True
    if text in {"q", "quit", "exit"}:
        return False
    if text in {"s", "status"}:
        print_status()
        return True
    if text.startswith("set "):
        set_target_mb(float(text[4:].strip()))
        print_status()
        return True
    if text[0] in {"+", "-"}:
        adjust_mb(float(text))
        print_status()
        return True
    print(
        "commands: +<mb>  -<mb>  set <mb>  status  quit",
        file=sys.stderr,
    )
    return True


def run_interactive() -> None:
    print("memory_occupier interactive mode", flush=True)
    print("examples: +512  -256  set 2048  status  quit", flush=True)
    print_status()
    while not _stop:
        try:
            line = input("mem> ")
        except EOFError:
            break
        if not _parse_command(line):
            break


def run_hold(target_mb: float, interval_sec: float) -> None:
    set_target_mb(target_mb)
    print_status()
    print("press Ctrl+C to release and exit", flush=True)
    while not _stop:
        time.sleep(interval_sec)
        print_status()


def run_ramp(
    target_mb: float,
    step_mb: float,
    interval_sec: float,
) -> None:
    print(
        f"ramping to {target_mb:.1f} MiB in {step_mb:.1f} MiB steps "
        f"every {interval_sec:.1f}s",
        flush=True,
    )
    while not _stop and held_mb() < target_mb:
        adjust_mb(step_mb)
        print_status()
        time.sleep(interval_sec)
    print("ramp complete; press Ctrl+C to release and exit", flush=True)
    while not _stop:
        time.sleep(interval_sec)
        print_status()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Allocate and hold resident memory to test system behavior under "
            "memory pressure. Uses bytearray pages so RSS increases for real."
        ),
    )
    parser.add_argument(
        "--hold-mb",
        type=float,
        default=0.0,
        help="Allocate this many MiB immediately and hold until interrupted.",
    )
    parser.add_argument(
        "--ramp-to-mb",
        type=float,
        default=None,
        help="Gradually allocate memory until this many MiB are held.",
    )
    parser.add_argument(
        "--step-mb",
        type=float,
        default=256.0,
        help="MiB added per ramp step (default: 256).",
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
        help="Adjust held memory from stdin (+512, -256, set 2048).",
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    signal.signal(signal.SIGINT, _handle_stop)
    signal.signal(signal.SIGTERM, _handle_stop)

    if args.interactive:
        run_interactive()
    elif args.ramp_to_mb is not None:
        run_ramp(args.ramp_to_mb, args.step_mb, args.interval)
    elif args.hold_mb > 0:
        run_hold(args.hold_mb, args.interval)
    else:
        parser.print_help()
        return 1

    released = held_mb()
    _chunks.clear()
    print(f"released {released:.1f} MiB", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
