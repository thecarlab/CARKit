# CARLab ADA Fleet Monitor

This project is a ChatGPT Site, separate from the CARKit WebUI. It keeps a public,
filterable list of ADA vehicle WebUI addresses. A host-side service reports the
vehicle's routed IP at startup and hourly, retrying every minute while offline.

The public dashboard is read-only. `POST /api/check-in` requires the shared
`CARKIT_REPORTER_TOKEN` Sites secret. Vehicle state is persisted in the `DB` D1
binding configured by `.openai/hosting.json`. A vehicle is shown online for 90
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
