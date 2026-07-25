# Simulation Installation and Runtime Guide

This document is focused on simulation runtime usage only.

Provisioning and bootstrap steps live in the root guide: [INSTALL.md](../../INSTALL.md).

## Scope of This Document

This simulation guide covers:
- launching the simulation stack with [run.sh](run.sh)
- stopping runtime safely with [stop.sh](stop.sh)
- when to use [cleanup.sh](../../cleanup.sh) for reset/rebuild
- running flight tests with [test-flight.py](scripts/test-flight.py)
- running a full Dedalus mission with `run_mission.sh` and viewing it live with `dedalus_viewer`

This simulation guide does not cover:
- AWS instance provisioning
- first-time host bootstrap (`setup.sh`) details

## 1. Runtime Prerequisites

Before using this guide, ensure root provisioning is complete:
- [INSTALL.md](../../INSTALL.md) has been followed
- `$HOME/dedalus/venv` exists
- DCV session `dedalus-sim` is available

Quick checks:

```bash
cd ~/dedalus/simulation/airsim
ls "$HOME/dedalus/venv/bin/activate"
dcv describe-session dedalus-sim --json
```

## 2. Launch the Simulation

From the simulation directory:

```bash
cd ~/dedalus/simulation/airsim
./run.sh AirSimNH
```

Behavior summary:
- resolves DCV display metadata
- launches a `tmux` session named `dedalus-sim`
- starts `iox-roudi`
- starts Unreal/AirSim and waits for TCP 4560
- starts PX4 SITL in a dedicated tmux window

Useful runtime outputs:
- main simulation log under `simulation/airsim/logs/`
- PX4 log under `simulation/airsim/logs/`

## 3. Stop vs Cleanup

Use the correct script for intent:

1. Runtime stop only:

```bash
cd ~/dedalus/simulation/airsim
./stop.sh
```

`stop.sh` is intentionally light. It stops running processes and tmux session, and does not remove build/cache state.

2. Reset or rebuild-oriented cleanup:

```bash
cd ~/dedalus
./cleanup.sh --soft --yes
```

`cleanup.sh` is a repo-root script, not a simulation-directory script like `run.sh`/`stop.sh` — it is the root rebuild/reset helper, not the normal simulation stop command. Use it only when you explicitly want reset semantics.

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
cd ~/dedalus/simulation/airsim
source "$HOME/dedalus/venv/bin/activate"
```

Run default flight:

```bash
python scripts/test-flight.py
```

Run explicit PX4 hybrid mode:

```bash
python scripts/test-flight.py --control px4
```

Run custom trajectory:

```bash
python scripts/test-flight.py --control px4 --trajectory ../../../config/behaviors/trajectories/circle_figure8.json
```

Run with explicit safe climb height:

```bash
python scripts/test-flight.py --control px4 --safe-height 10.0
```

## 6. Running a Full Dedalus Mission (run_mission.sh + Viewer)

`run_mission.sh` starts the Dedalus mission loop (obstacle mapping + behavior execution)
plus companion camera-bridge/overlay/validation tools in a dedicated tmux session. It
assumes `run.sh` (Section 2) has already started AirSim/PX4 — it does not start them
itself. This is a separate, higher-level workflow than `test-flight.py` (Section 5), which
only drives raw velocity trajectories without the mission loop or obstacle map.

1. Set a site ID once per physical/simulated location — this geo-anchors the persistent
   L2 obstacle map so it accumulates across missions instead of starting empty each time:

```bash
export DEDALUS_SITE_ID=airsim_47.641N_122.140W
```

2. Launch the mission (mission-id, output dir, and L2 DB path all auto-derive from
   `DEDALUS_SITE_ID` and the config filename when `--output-dir` is omitted):

```bash
cd ~/dedalus/simulation/airsim
./run_mission.sh \
  --config ../../config/runs/airsim_circle_airsim_gt.yaml \
  --runtime-event-http-port 8080 \
  --runtime-event-static-root build-staging \
  --pipeline-timing \
  --tail
```

   Output lands in `out/<mission-slug>/<timestamp>/`. The persistent L2 obstacle map
   accumulates in `maps/$DEDALUS_SITE_ID/l2_map.db`.

3. In another terminal, start the viewer (same `DEDALUS_SITE_ID` picks up the L2 map):

```bash
cd ~/dedalus/simulation/airsim
DEDALUS_SITE_ID=airsim_47.641N_122.140W \
../../build-staging/apps/dedalus_viewer \
  --host 127.0.0.1 --port 47770 \
  --http-port 8090 --static-root ../../build-staging
```

4. From your local machine, forward the viewer's HTTP port over SSH and open it in a browser:

```bash
ssh -L 8090:127.0.0.1:8090 <ec2-host>
open http://127.0.0.1:8090/
```

## 7. Control Modes Quick Reference

```text
--control auto     prefer PX4 shell path, fallback chain enabled
--control px4      PX4 shell arm/takeoff/land + MAVLink trajectory body
--control mavlink  MAVLink command path (experimental)
--control airsim   AirSim-only fallback path
```

## 8. Troubleshooting

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

## 9. Related Docs

- Root provisioning: [INSTALL.md](../../INSTALL.md)
- Simulation overview: [README.md](README.md)
- Flight harness details: [test-flight.py](scripts/test-flight.py)
