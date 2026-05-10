# Simulation Installation and Runtime Guide

This document is focused on simulation runtime usage only.

Provisioning and bootstrap steps live in the root guide: [INSTALL.md](../INSTALL.md).

## Scope of This Document

This simulation guide covers:
- launching the simulation stack with [run.sh](run.sh)
- stopping runtime safely with [stop.sh](stop.sh)
- when to use [cleanup.sh](cleanup.sh) for reset/rebuild
- running flight tests with [test-flight.py](test-flight.py)

This simulation guide does not cover:
- AWS instance provisioning
- first-time host bootstrap (`setup.sh`) details

## 1. Runtime Prerequisites

Before using this guide, ensure root provisioning is complete:
- [INSTALL.md](../INSTALL.md) has been followed
- `$HOME/dedalus/venv` exists
- DCV session `dedalus-sim` is available

Quick checks:

```bash
cd ~/dedalus/simulation
ls "$HOME/dedalus/venv/bin/activate"
dcv describe-session dedalus-sim --json
```

## 2. Launch the Simulation

From the simulation directory:

```bash
cd ~/dedalus/simulation
./run.sh AirSimNH
```

Behavior summary:
- resolves DCV display metadata
- launches a `tmux` session named `dedalus-sim`
- starts `iox-roudi`
- starts Unreal/AirSim and waits for TCP 4560
- starts PX4 SITL in a dedicated tmux window

Useful runtime outputs:
- main simulation log under `simulation/logs/`
- PX4 log under `simulation/logs/`

## 3. Stop vs Cleanup

Use the correct script for intent:

1. Runtime stop only:

```bash
cd ~/dedalus/simulation
./stop.sh
```

`stop.sh` is intentionally light. It stops running processes and tmux session, and does not remove build/cache state.

2. Reset or rebuild-oriented cleanup:

```bash
cd ~/dedalus/simulation
./cleanup.sh --soft --yes
```

Use `cleanup.sh` only when you explicitly want reset semantics.

## 4. Operating in tmux

Attach to the runtime session:

```bash
tmux attach -t dedalus-sim
```

Typical checks:

```bash
# verify TCP endpoint from AirSim
ss -ltnp | grep 4560

# list tmux sessions/windows
tmux ls
```

## 5. Flight Test Execution

Activate venv first:

```bash
cd ~/dedalus/simulation
source "$HOME/dedalus/venv/bin/activate"
```

Run default flight:

```bash
python test-flight.py
```

Run explicit PX4 hybrid mode:

```bash
python test-flight.py --control px4
```

Run custom trajectory:

```bash
python test-flight.py --control px4 --trajectory trajectories/circle_figure8.json
```

Run with explicit safe climb height:

```bash
python test-flight.py --control px4 --safe-height 10.0
```

## 6. Control Modes Quick Reference

```text
--control auto     prefer PX4 shell path, fallback chain enabled
--control px4      PX4 shell arm/takeoff/land + MAVLink trajectory body
--control mavlink  MAVLink command path (experimental)
--control airsim   AirSim-only fallback path
```

## 7. Troubleshooting

1. AirSim TCP not up:

```bash
ss -ltnp | grep 4560
```

If missing, restart runtime with [stop.sh](stop.sh) then [run.sh](run.sh).

2. PX4 shell checks:

```bash
tmux attach -t dedalus-sim
```

In PX4 window:

```bash
commander status
mavlink status
```

3. DCV session issues:

```bash
systemctl --user restart dcv-session.service
dcv list-sessions
```

## 8. Related Docs

- Root provisioning: [INSTALL.md](../INSTALL.md)
- Simulation overview: [README.md](README.md)
- Flight harness details: [test-flight.py](test-flight.py)
