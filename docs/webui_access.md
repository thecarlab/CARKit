# WebUI access and operator control

CARKit requires access-key sign-in by default before the HTTP API or ROS WebSocket can be used.
The default WebUI and bridge bindings are `127.0.0.1` on ports 8080 and 9090.
Starting the WebUI creates `.carkit/access.key` with mode 0600 if it does not
already exist. Read that file on the vehicle computer and enter the key in the
sign-in form. Do not commit it or include it in logs or screenshots. An optional
`CARKIT_ACCESS_KEY` environment variable (at least 32 characters) replaces the file.

The browser receives an HttpOnly, SameSite=Strict session cookie valid for 12 hours.
Server restart invalidates browser sessions and rotates the separate bridge token.
The bridge accepts no subscriptions or commands without that token. WebUI-launched
ROS processes inherit it; it is not placed in ROS parameters or command-line arguments.
Direct command-line bridge launches must explicitly set `CARKIT_CONTROL_TOKEN` to
an unpredictable value of at least 32 characters. Without it, the bridge fails closed.

## Remote access

Use SSH forwarding to retain the default loopback bindings and encrypt both ports:

```bash
ssh -L 8080:127.0.0.1:8080 -L 9090:127.0.0.1:9090 USER@VEHICLE
```

Then open `http://127.0.0.1:8080` on the client. Keep the local and remote WebUI ports
the same for the default origin checks.

For an explicitly network-accessible deployment, set `CARKIT_WEBUI_HOST=0.0.0.0`
before `docker/run_jetson.sh`, or pass `--host 0.0.0.0` to the Python server.
The WebUI passes that binding to the bridge it launches. Authentication does not
encrypt plain HTTP/WS; use a trusted isolated network or configure HTTPS/WSS reverse
proxies for the WebUI and bridge. The browser uses WSS on port 9090 when the page is
HTTPS. HTTP mutations reject foreign origins. WebSocket upgrades accept the matching
WebUI host/port, or origins explicitly listed with `--allowed-origin` on the server.

To disable the browser access-key prompt for a deployment, set
`CARKIT_REQUIRE_ACCESS_KEY=0` in the environment of `carkit.service` and restart
the service. The Docker launcher passes this setting through. Browsers then
receive a bridge session automatically; single-operator ownership and origin
checks still apply. Anyone who can reach the dashboard can access its APIs.
Set the value back to `1` and restart to require the existing key again.

## Driving and observing

After sign-in, browsers start as observers. Choose **Take control** to become the
single browser operator. Only that browser can drive, request a control-mode change,
reset emergency stop, set an initial pose, or send a navigation goal. An observer
can always request emergency stop. Every command is checked by the bridge, not just
by disabled UI controls. The browser shows ownership and receives acceptance or
rejection responses.

**Release control**, disconnect, or three seconds without an operator heartbeat
releases browser ownership and sends zero manual input. Autonomous driving remains
controlled by the ROS arbiter and does not require a focused browser. A new browser
must explicitly claim control; reconnect never automatically claims it.

The persistent safety bar is available on Overview, Code, Compile, Terminal, and
while configuration is open. **Ctrl + Shift + Space** requests emergency stop from
any page, including an editor or terminal. Ordinary Space remains an emergency-stop
shortcut on the focused keyboard Overview. Reset always returns to manual, clears
buffered manual commands, and requires a new autonomy request to resume autonomy.

Optional joystick input follows confirmed arbiter state. After emergency stop,
mode changes, startup, or stale mode feedback, return both driving axes to neutral
before driving. The explicitly standalone diagnostic relay remains manual-only.

## Status and geometry

Commanded speed and steering show the arbiter's output, not measured motion.
Chassis health requires odometry or hardware telemetry. Sensor, planning, and
perception health expire after three seconds without a distinct update. Repeated
source timestamps do not renew health. A static map is shown as available without
claiming the planner is live. Stale camera frames are labelled stale.

The bridge publishes the effective hardware footprint, including its base-frame
centre offset, to the browser. The map outline and legend use that configuration;
the browser does not guess a vehicle size before receiving it.

## Guided steering calibration

The **Calibration** button sits immediately left of **Configure**. Red means the
current hardware settings need verification; green means a complete six-stage run
passed at the tested speed. Changing the relevant hardware configuration invalidates
the previous pass. The method switches automatically with the running chassis (or the saved selection
when stopped). Both `f1tenth` and `vesc_rplidar` use the guided VESC command sequence
with RealSense IMU and LiDAR. OSRacer has a separate, not-yet-configured method;
its panel hides the VESC sequence and cannot start or apply VESC calibration.

Select Human control, take browser control, release the driving controls, and open
Calibration. Confirm the vehicle is stationary on a flat surface in an attended,
clear test area. The sequence is forward straight, backward straight, maximum left,
half maximum left, maximum right, and half maximum right. Turning stages drive
forward. The commanded speed is 1.00 m/s. Choose the named single-check button
to run one maneuver and its return, then stop with E-stop latched. Reset E-stop
and confirm the clear area again to start the next named maneuver. Alternatively,
**Run all 6 checks** runs every unfinished check automatically, returning and stopping
between maneuvers, then latches E-stop at the end. With saved progress this button
shows **Run remaining N checks** and skips measurements already complete. Both
buttons require the attended-area checkbox. Only explicitly choosing Run all
authorizes automatic advancement; single-check mode remains the default.
Keep 6.5 m ahead and behind and 3.5 m on each side clear, measured from the
starting footprint. Measurements stop once at least 20 usable steady samples span
at least 1 second of actual sampled motion. Forward/backward may extend to 6 m or
12 seconds; turns may extend beyond 1 m, up to 3 m or 12 seconds. Each is followed
by a full stop, a reverse retrace of the measured path (bounded to 20 seconds),
and another stop before control is released. Return position and heading
are estimated from RealSense gyro and VESC speed and recorded for diagnostics.
Position and heading error do not abort the return or calibration. This is dead reckoning,
not an externally verified physical position; wheel slip can affect accuracy. Maximum steering refers to the
configured limit, not an untested mechanical endpoint.

The worker checks IMU, odometry, VESC state, servo commands, transforms, and LiDAR.
It uses a stationary gyro baseline and steady-motion samples to estimate equivalent
steering angle from yaw rate, speed, and configured wheelbase. The recommendation
fits servo center and steering gain, with the steering limit bounded by existing
servo travel. It does not independently calibrate wheelbase or motor-speed scale.
Green requires straight error no greater than 0.5 degrees and turning RMS error no
greater than 1.5 degrees. These are low-speed checks, not high-speed validation.

The dedicated arbiter mode excludes manual and autonomous commands during the run.
Missing onboard command heartbeat, stale sensor data, an obstacle in the projected
swept footprint, or explicit cancellation stops the sequence. Browser focus, tab
visibility, page closure, and browser heartbeat loss do not cancel the on-vehicle run. Completion and
failure leave E-stop latched. Stay with the vehicle; use **Stop calibration** or E-stop to cancel. Closing the
calibration dialog or browser does not cancel the run. Reset E-stop before another run.

Finished passes are checkpointed after their return and stop. A maneuver is marked
measurement-complete only when enough usable data and its stationary gyro baseline
are saved. A retry skips those measurements and retains usable partial samples for
checks that need more data. Time gaps between passes or sessions do not count as
steady motion. Changing hardware settings or commanded speed invalidates reuse.
Each session measures its own stationary gyro bias. Samples without their original
baseline are not reused. A completed sequence is repeated only when **Start a new
sequence** is checked. Saved motion alone never implies a verified steering fit.

Results, samples, logs, and the prior hardware profile are saved under
`.carkit/calibration/`. **Apply & restart** requires the explicit checkbox, saves the
recommendation to the hardware profile, and restarts the current stack. The status
stays red until another complete run verifies the changed settings. Both the updated
WebUI server and rebuilt control-center node must be restarted to activate this feature.

Calibration LiDAR checks sweep the vehicle footprint 20 cm along the commanded
forward/reverse arc, with a 20 cm side margin. This is an obstacle trigger distance,
not a guarantee of physical braking distance.
