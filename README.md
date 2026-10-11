# SITL Altitude PID

PID altitude control for an ArduPilot SITL copter over MAVLink.

The vehicle arms, switches to `GUIDED`, and then flies a full profile under custom
PID control — climbing, holding, flying forward, descending and landing — while a
live plot tracks the setpoint against the actual altitude.

## Demo

Screen recording of the mission running against local SITL: [docs/demo.mp4](docs/demo.mp4).

<!-- For an inline player, drag & drop docs/demo.mp4 into this section on github.com. -->


## Mission profile

```
GUIDED → ARM → short takeoff → climb to 15 m, hold 5 s
       → fly 10 m forward (holding 15 m)
       → descend to 10 m, hold 5 s
       → LAND
```

Altitude is controlled purely by **thrust** via `SET_ATTITUDE_TARGET` (thrust-only),
and forward flight adds a **pitch** tilt to the same command. No position/velocity
setpoints are used — just a PID around hover thrust.

## Architecture

The code is split into single-responsibility layers so new missions reuse the same
control loops:

| Module | Responsibility |
| --- | --- |
| `pid_controller.py` | Generic PID with output clamping + anti-windup |
| `copter.py` | MAVLink wrapper (connect, arm, modes, telemetry, send thrust/attitude) |
| `thrust_controller.py` | Altitude hold via thrust (owns the altitude PID) |
| `forward_controller.py` | Forward flight via pitch; reuses `ThrustController` for altitude |
| `flight_recorder.py` | Live matplotlib plot of setpoint vs. altitude |
| `config.py` | Shared, mission-agnostic configuration (aggregated in `VehicleConfig`) |
| `mission.py` | The flight scenario (`AltitudeMission` + its profile) |
| `main.py` | Composition root: builds everything and runs the mission |

## Requirements

- Python 3.10+
- [ArduPilot SITL](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html) (`sim_vehicle.py`, ArduCopter)
- Python packages in `requirements.txt` (`pymavlink`, `matplotlib`)

## Setup

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Running

**1. Start SITL.** The `--streamrate=-1` flag is important: without it MAVProxy
resets the telemetry rate to ~4 Hz every few seconds, which starves the control
loop and causes overshoot.

```bash
sim_vehicle.py -v ArduCopter --console --map --mavproxy-args="--streamrate=-1"
```

**2. Run the mission.**

```bash
python main.py
```

Mission parameters (target altitudes, hold time, forward distance) and all PID
gains live in `config.py` and `mission.py` — edit them there.

## How the control works

- **Altitude** is a PID whose output is a *thrust offset around hover*:
  `thrust = hover_thrust + PID(altitude_error)`, clamped to `[0, 1]`.
  Requires `GUID_OPTIONS=8` so ArduPilot interprets the thrust field as real thrust
  (the script sets this automatically on connect).
- **Asymmetric thrust limits:** climb may add more thrust, while descent is kept
  gentle — a thrust of 0 would be free fall (motors off), which makes the vehicle
  dip below the target. The descent floor is kept above zero.
- **Anti-windup:** the integral term stops accumulating while the output is
  saturated, preventing overshoot after long climbs.
- **Fresh telemetry:** the MAVLink buffer is drained on every read so control
  always acts on the latest altitude, not a stale queued message.
- **Forward flight** adds a pitch PID (distance error → tilt angle) on top of the
  reused altitude thrust, both sent in one `SET_ATTITUDE_TARGET`.

PID gains and limits live in `config.py` (`ThrustControllerConfig`, `ForwardFlightConfig`).
