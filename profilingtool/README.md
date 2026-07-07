# ROS 2 Node Profiler

Live web dashboard for CPU and memory usage of ROS 2 nodes running inside the `carkit` Docker container.

## Quick start

Works from the **Jetson host** or **inside the carkit container** (auto-detected).

From the host:

```bash
cd profilingtool
./run.sh
```

From inside the container:

```bash
cd /workspaces/CARKit/profilingtool
python3 server.py --host 0.0.0.0 --port 8765
```

Open `http://localhost:8765` in a browser. The page polls the backend every 2 seconds and updates node metrics automatically.

## What it reports

- **Container CPU / memory** from `docker stats`
- **Per ROS 2 node** CPU %, memory %, RSS, and PID
- **Launch supervisors** (`ros2 launch ...`) shown separately
- **Host memory and load** from `/proc` inside the container

Node-to-process mapping uses ROS 2 `--ros-args -r __node:=...` markers and executable names from `ros2 node list`.

## Options

Environment variables:

```bash
PROFILER_HOST=0.0.0.0 PROFILER_PORT=8765 PROFILER_CONTAINER=carkit ./run.sh
```

Or pass CLI flags directly:

```bash
python3 server.py --host 0.0.0.0 --port 8765 --container carkit
```

## Requirements

- Python 3 (stdlib only; no pip packages)
- ROS 2 nodes launched inside the container

When run on the host, the profiler uses `docker exec` and `docker stats`.
When run inside the container, it reads ROS 2 and `/proc` data directly (no Docker CLI required).

## Memory pressure test

Use `memory_occupier.py` to hold resident RAM while the rest of the stack runs.
This helps verify whether low `free` / swap use is causing camera or perception slowdown.

Inside the container:

```bash
# Hold 2 GiB until Ctrl+C
python3 memory_occupier.py --hold-mb 2048

# Ramp up by 256 MiB every 5 seconds until 4 GiB is held
python3 memory_occupier.py --ramp-to-mb 4096 --step-mb 256 --interval 5

# Adjust manually while watching ros2 topic hz / camera topics
python3 memory_occupier.py --interactive
```

Interactive commands: `+512`, `-256`, `set 3000`, `status`, `quit`.

## CPU load test

Use `cpu_occupier.py` to add controlled CPU load while the camera stack runs.

```bash
# Saturate 4 cores until Ctrl+C
python3 cpu_occupier.py --cores 4

# Ramp from 1 core to all cores, one core every 5 seconds
python3 cpu_occupier.py --ramp-to-cores 6 --step-cores 1 --interval 5

# Half-load on 2 cores
python3 cpu_occupier.py --cores 2 --load-percent 50

# Manual control
python3 cpu_occupier.py --interactive
```

Interactive commands: `+1`, `-1`, `set 4`, `set 0`, `off`, `load 50`, `status`, `quit`.

**Stopping workers**

- Interactive mode: `quit`, `off`, or `set 0` stops workers; `quit` also exits.
- Hold / ramp mode: **Ctrl+C** only (typed commands are ignored).
- Stuck workers from an old run: `pkill -f cpu_occupier.py`

Pair with:

```bash
ros2 topic hz /camera/camera/color/image_raw
```
