# Docker demo

This folder starts the full Yamcs/Grafana quickstart:

- Yamcs on <http://localhost:8090>
- Python simulator sending drone telemetry at 1 Hz
- Grafana on <http://localhost:3000>
- latest published `jaops-yamcs-app` Grafana plugin

Run:

```sh
docker compose -f compose.yaml up --build
```

Default Grafana login:

```text
admin / admin
```
