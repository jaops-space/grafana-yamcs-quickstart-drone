# Grafana Yamcs Quickstart Drone

This repository is a Yamcs + Grafana demo built around one coherent simulated
drone system.

It is based on the Yamcs quickstart structure, but it is intentionally separate
from `yamcs-quickstart` so the canonical Yamcs example stays unchanged.

The demo uses:

- one Yamcs instance: `demo`
- one XTCE space system: `/drone`
- one Python simulator sending deterministic CCSDS telemetry
- one Python image generator publishing Yamcs bucket image URLs
- one Grafana stack with the current GitHub release of `jaops-yamcs-app`

## Quick start

Start Yamcs, the simulator, the image generator, and Grafana:

```sh
cd docker
docker compose -f compose.yaml up --build
```

Open:

- Yamcs: <http://localhost:8090>
- Grafana: <http://localhost:3000>

Grafana login:

```text
admin / admin
```

Grafana is provisioned by this quickstart repo with one demo setup:

- the latest configured GitHub release of `jaops-yamcs-app`
- a `Yamcs Demo` datasource pointing to `demo/realtime`
- the drone dashboard under `Yamcs Grafana Demo`

The plugin version is controlled from Docker Compose. Override it with:

```sh
export GRAFANA_YAMCS_PLUGIN_VERSION=1.1.1
```

The image generator publishes camera frames at 2 FPS by default. Override it with:

```sh
export IMAGE_INTERVAL=0.25
```

## Demo model

The `/drone` MDB demonstrates:

- calibrated battery, current, temperature, cell-voltage, and motor values
- raw ADC variants for calibrated parameters
- warning and critical alarms with persistence
- enum alarms for flight mode, motor status, and power faults
- booleans, enumerations, fixed strings, aggregates, arrays, arrays of
  aggregates, aggregates containing arrays, and a two-dimensional array
- derived battery power, remaining energy, and flight-readiness parameters
- local camera image parameters backed by the Yamcs `images` bucket
- command arguments, valid ranges, transmission constraints, significance, and
  completion verification

The telemetry follows a repeating mission profile: standby, takeoff, survey,
return-to-home, landing, and charging. A motor-3 bearing fault is included near
the end of each survey phase to exercise warning/critical states and failsafe
logic.

The image generator renders a synthetic forward camera view over a perspective
ground grid, using the same drone profile, and publishes these local parameters:

- `/drone/CameraImageNumber`
- `/drone/CameraImageStorageUrl`
- `/drone/CameraImageUrl`
- `/drone/CameraView`

## Commands

The simulator applies supported commands to subsequent telemetry so Yamcs can
verify the result.

Included commands:

- `/drone/Arm`
- `/drone/Disarm`
- `/drone/SetFlightMode`
- `/drone/SetTargetAltitude`
- `/drone/MotorTest`
- `/drone/ConfigureGeofence`
- `/drone/EmergencyLand`

## Development note

After changing the deterministic flight model, regenerate `testdata.ccsds`:

```sh
docker compose -f docker/compose.yaml run --rm simulator python3 generate_testdata.py
```
