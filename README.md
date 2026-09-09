# Yamcs Grafana Quickstart

This repository is a Yamcs + Grafana demo built around one coherent simulated
drone system.

It is based on the Yamcs quickstart structure, but it is intentionally separate
from `yamcs-quickstart` so the canonical Yamcs example stays unchanged.

The demo uses:

- one Yamcs instance: `demo`
- one XTCE space system: `/drone`
- one Python simulator sending deterministic CCSDS telemetry
- one Grafana stack with the latest published `jaops-yamcs-app` plugin

## Quick start with Docker

Start Yamcs, the simulator, and Grafana:

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

Grafana is provisioned with:

- the latest published `jaops-yamcs-app` plugin
- a `Yamcs Demo` datasource pointing to `demo/realtime`
- a small starter dashboard under `Yamcs Grafana Quickstart`

## Native run

Prerequisites:

- Java 17+
- Python 3.10+

Compile:

```sh
./mvnw compile
```

Start Yamcs:

```sh
./mvnw yamcs:run
```

In another terminal, start telemetry:

```sh
python3 simulator.py
```

Yamcs listens on <http://localhost:8090>. The simulator sends telemetry at 1 Hz
over UDP port `10015` and receives commands on UDP port `10025`.

## Demo model

The `/drone` MDB demonstrates:

- calibrated battery, current, temperature, cell-voltage, and motor values
- raw ADC variants for calibrated parameters
- warning and critical alarms with persistence
- enum alarms for flight mode, motor status, and power faults
- booleans, enumerations, fixed strings, aggregates, arrays, arrays of
  aggregates, aggregates containing arrays, and a two-dimensional array
- derived battery power, remaining energy, and flight-readiness parameters
- command arguments, valid ranges, transmission constraints, significance, and
  completion verification

The telemetry follows a repeating mission profile: standby, takeoff, survey,
return-to-home, landing, and charging. A motor-3 bearing fault is included near
the end of each survey phase to exercise warning/critical states and failsafe
logic.

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

## Regenerate telemetry

After changing the deterministic flight model, regenerate `testdata.ccsds`:

```sh
python3 generate_testdata.py
```

## Package Yamcs

For a distributable Yamcs application:

```sh
./mvnw package
```
