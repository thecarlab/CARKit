# CARKit desktop integration

`install_gnome_resources.sh` installs the GNOME Shell 46 resource indicator
for the current desktop user. The indicator polls the existing container API
at `http://127.0.0.1:8080/api/status` every five seconds; it does not run a
second ROS node or monitoring daemon on the host.

The compact top-bar label shows Jetson-wide CPU and RAM use, CPU temperature,
chassis battery voltage, and estimated battery percentage. CPU follows the
aggregate core scale: one fully occupied core is 100%, so the six-core Jetson
display has a 600% maximum. Its menu translates that value into busy cores and
shows the active course/chassis session and a shortcut to the WebUI. Warning color starts
at 85% of total CPU capacity, 75°C, or 20% remaining battery.

Install it from the repository root on the Jetson desktop:

```bash
./desktop/install_gnome_resources.sh
```

The extension is intentionally host-side because GNOME Shell itself runs on
the host. All measured data still comes from the CARKit WebUI in the container.
