# Docker demo

This folder starts the full Yamcs/Grafana quickstart.

- Yamcs on <http://localhost:8090>
- Python simulator sending drone telemetry at 1 Hz
- Python image generator publishing drone camera frames to Yamcs at 2 FPS
- Grafana on <http://localhost:3000>
- configured GitHub release of the `jaops-yamcs-app` Grafana plugin
- local Grafana provisioning from `docker/grafana/provisioning`

Run:

```sh
docker compose -f compose.yaml up --build
```

Default Grafana login:

```text
admin / admin
```

Override the plugin version:

```sh
GRAFANA_YAMCS_PLUGIN_VERSION=1.1.1 docker compose -f compose.yaml up --build
```
