# CARLab ADA Fleet Monitor

This project is a ChatGPT Site, separate from the CARKit WebUI. It keeps a public,
filterable list of ADA vehicle WebUI addresses. A host-side service reports the
vehicle's routed IP at startup and every minute. When offline, it tries every five
minutes and stops after 20 consecutive failures.

The monitor also supports a secret named `CARKIT_REPORTER_TOKEN_ADA2` (and
equivalent names for other ADA IDs). That token can check in only its named
vehicle; the shared `CARKIT_REPORTER_TOKEN` continues to work for the fleet.
Provision the vehicle secret in Sites, deploy the updated monitor, and give
the vehicle's installer the same value. Store it locally under the reporter's
existing `CARKIT_REPORTER_TOKEN` key. Do not rotate the shared secret merely
to provision another car. Sites does not return stored secret values.

Run `node test/check_in.mjs` to test shared and vehicle-specific authentication.

The public dashboard is read-only. `POST /api/check-in` requires the shared
`CARKIT_REPORTER_TOKEN` Sites secret. Vehicle state is persisted in the `DB` D1
binding configured by `.openai/hosting.json`. A vehicle is shown online for 2
minutes after its last successful check-in.

## Vehicle reporter

The installer expects a vehicle ID and the deployed Site base URL. It reads the
secret token from standard input so the token does not appear in shell history:

```bash
sudo ./vehicle/install-reporter.sh ADA5 https://your-site.example
```

Inspect the reporter with:

```bash
systemctl status carkit-webmonitor.service
journalctl -u carkit-webmonitor.service -f
```

The WebUI and reporter run as system services without a display or desktop login.
At startup, routing and DNS get a two-minute grace period before normal upload
retries begin; container readiness and clock synchronization are always required.

CARKit setup now enables the host NTP provider, installing `systemd-timesyncd`
when none is available. Each CARKit service start re-enables synchronization;
the container shares the host clock and uses the host timezone by default.
The reporter starts with `carkit.service` and follows its stop/restart lifecycle.
Before every upload it verifies that the named `carkit` container is running,
the local WebUI landing page responds, and `timedatectl` confirms the clock is
synchronized. These readiness checks repeat every five seconds without consuming
upload retries. If time cannot synchronize, the WebUI can still run, but reporting
waits; inspect `timedatectl status` and the host NTP service in that case.
