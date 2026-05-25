# 🎮 The Virtual Proving Ground (Simulation Environment)

![Sim Engine](https://img.shields.io/badge/Engine-Colosseum_(AirSim)-blue)
![Flight Controller](https://img.shields.io/badge/FCU-PX4_SITL-orange)
![Visualization](https://img.shields.io/badge/Display-NICE_DCV-green)
![Hardware](https://img.shields.io/badge/GPU-NVIDIA_L4-76B900)

<img width="1446" height="833" alt="Screenshot 2026-05-07 at 1 55 20 PM" src="https://github.com/user-attachments/assets/83ebe7e1-0355-4400-9ac6-f7514c291f7c" />

## Overview
Welcome to the Project Dedalus **Virtual Proving Ground**. 

Due to the extreme hardware risks of testing high-speed tactical flight software, this directory houses the integration layer between our C++ flight stack and our virtual physics environment. This environment strictly enforces the core project philosophy: **Train Heavy, Simulate Accurately, Fly Light.**

Before any Dedalus C++ binary is flashed to the physical Jetson Orin airframe, it must successfully execute its flight behavior here without crashing.

## Core Components
1. **Colosseum (Unreal Engine):** An actively maintained fork of Microsoft AirSim. It provides high-fidelity physics, spawns dynamic targets (moving drones/cars), and renders virtual camera buffers.
2. **PX4 SITL (Software In The Loop):** The exact same flight controller firmware that runs on the physical Pixhawk, compiled to run locally.
3. **NICE DCV ("The Windshield"):** Our high-performance remote visualization bridge. It enables GPU-accelerated X11 rendering over the network, allowing low-latency interaction with the simulation from a remote workstation (e.g., MacBook Pro).

## Hardware & Infrastructure Requirements
Simulating high-speed vision pipelines is computationally heavy.
* **Local Development:** NVIDIA RTX 3060 (Minimum) / RTX 4070 (Recommended), 32GB RAM.
* **Cloud Development (Recommended):** AWS **`g6.2xlarge`** (NVIDIA L4 GPU).
    * **IMDSv2 Requirement:** Instance metadata hop limit must be set to **2** for NICE DCV licensing and IAM S3 access.
    * **Security Group:** Inbound **TCP/UDP 8443** must be open for your local IP.

## 🛠 Setup & Initialization
The environment uses a "Ghost Boot" strategy. Once configured, the visual desktop is persistent and auto-starts on boot.

1. **Provision the Environment:**
   Run the master setup script from the repository root to install dependencies, fetch Colosseum binaries, and configure the XDCV virtual display manager:
   ```bash
   ./setup.sh
   ```
2. **Set User Password:**
   NICE DCV requires a system password for the session owner:
   ```bash
   sudo passwd $(whoami)
   ```
3. **Connect the Client:**
   Use the **NICE DCV Client** on your local machine to connect to `<Instance_IP>:8443`.

## 🚀 Running the Simulation
The `run.sh` script is configured to dynamically bridge your terminal session to the virtual "Windshield."

### Launching via SSH or DCV Terminal:
```bash
cd ~/dedalus/simulation/airsim
./run.sh AirSimNH
```
*The script automatically probes the DCV metadata to export the correct `DISPLAY` and `XAUTHORITY` variables, and grants X11 permissions via `xhost`.*

### Stopping the simulation runtime

Use the AirSim target stop script for normal runtime shutdown:

```bash
cd ~/dedalus/simulation/airsim
./stop.sh
```

`stop.sh` stops the tmux session and runtime processes without removing build artifacts or generated third-party dependencies.

Do not use the root `cleanup.sh` as the normal stop command. Use `cleanup.sh` only when intentionally resetting/rebuilding generated state.

---

## 🕹 Interactive Flight Controls (Mac / DCV)
Once the AirSim window is visible in your DCV client, use the following mapping. 

> **Note:** You must click inside the AirSim window to give it focus. If the drone is unresponsive, ensure it is armed by running `commander arm` in the PX4 terminal/tmux pane.

| Action | Mac Keyboard Key |
| :--- | :--- |
| **Toggle Manual Control** | **`Delete`** (Standard Backspace) |
| **Throttle Up / Down** | **`W`** / **`S`** |
| **Yaw Left / Right** | **`A`** / **`D`** |
| **Pitch / Roll** | **Arrow Keys** (`↑`, `↓`, `←`, `→`) |
| **Toggle Follow Camera** | **`F`** |
| **Cycle Sensor Views** | **`1`, `2`, `3`** (Depth, Seg, IR) |
| **F-Key Commands** | **`fn` + `F1-F12`** |

---

## 🤖 Autonomous Flight Testing (`test-flight.py`)

The **`test-flight.py`** harness enables automated flight sequences with multi-mode control fallback and JSON-based trajectory playback. It handles arm/takeoff, trajectory execution, landing, and disarm with graceful error recovery.

### Control Mode Architecture

```
Command: python scripts/test-flight.py --control <MODE>

MODE=auto (default)
├─ Tries: PX4 shell (arm/takeoff/land) + AirSim velocity injection
├─ If PX4 shell unavailable:
│  └─ Tries: MAVLink with climb verification
│     └─ If MAVLink takeoff ACK but no altitude gain:
│        └─ Fails with diagnostic (detects false ACKs)
│           └─ If --force-mavlink-arm: Ignores false ACK and continues
└─ Final fallback: Pure AirSim RPC arm/takeoff/land

MODE=px4
└─ Direct: PX4 shell → arm → takeoff → play velocity trajectory → land → disarm

MODE=mavlink  
└─ Direct: MAVLink → arm → takeoff (with climb check) → play velocity trajectory → land → disarm

MODE=airsim
└─ Direct: Pure RPC → arm → takeoff → play trajectory → land → disarm
```

### Trajectory System

**JSON Format:**
Trajectories are collections of multi-second segments with instantaneous velocity generators. The system streams velocity commands at a configurable rate (default 10 Hz) to AirSim or the flight controller.

**Segment Types:**

| Type | Fields | Behavior |
|------|--------|----------|
| `hold` | `duration_s`, `vx_mps`, `vy_mps`, `vz_mps` | Fixed velocity (typically 0,0,0 for hovering) |
| `circle_velocity` | `duration_s`, `center_x`, `center_y`, `radius`, `altitude`, `clockwise` | Circular orbit around offset center |
| `figure8_velocity` | `duration_s`, `center_x`, `center_y`, `size`, `altitude` | Horizontal figure-eight pattern |
| `velocity_keyframes` | `duration_s`, `keyframes` (list of `[t, vx, vy, vz]`) | Piecewise linear velocity interpolation |

**Example Trajectory:**
```json
{
  "name": "combat_weave",
  "description": "Figure-8 evasion pattern with altitude change.",
  "rate_hz": 10,
  "segments": [
    {
      "type": "hold",
      "label": "pre_flight_hover",
      "duration_s": 3,
      "vx_mps": 0.0,
      "vy_mps": 0.0,
      "vz_mps": 0.0
    },
    {
      "type": "figure8_velocity",
      "label": "evasion_weave",
      "duration_s": 15,
      "center_x": 0.0,
      "center_y": 0.0,
      "size": 20.0,
      "altitude": 30.0
    },
    {
      "type": "circle_velocity",
      "label": "high_orbit",
      "duration_s": 10,
      "center_x": 5.0,
      "center_y": -5.0,
      "radius": 15.0,
      "altitude": 40.0,
      "clockwise": false
    }
  ]
}
```

**Default Behavior:**
If no trajectory is specified (`--trajectory` omitted), test-flight.py uses a hardcoded 10s hover sequence (takeoff → hold → land).

### CLI Reference

```bash
python scripts/test-flight.py [OPTIONS]
```

| Option | Default | Description |
|--------|---------|-------------|
| `--vehicle` | `PX4` | AirSim vehicle name |
| `--control` | `auto` | Control mode: `auto`, `px4`, `mavlink`, `airsim` |
| `--trajectory` | None | Path to JSON trajectory file. If omitted, uses default hover. Pre-flight validates file. |
| `--timeout` | `30` | Total flight timeout in seconds |
| `--skip-arm` | False | Skip arm phase (for testing trajectory playback only) |
| `--mavlink-endpoint` | `127.0.0.1:14550` (repeatable) | MAVLink UDP endpoints to try (can specify multiple) |
| `--px4-tmux-target` | `dedalus-sim:px4` | tmux pane for PX4 shell commands (format: `session:pane`) |
| `--force-mavlink-arm` | False | Ignore false ACK errors during MAVLink takeoff |

### Usage Examples

**1. Simple Test (Default Hover)**
```bash
python scripts/test-flight.py
# Runs: arm → 10s hover → land via PX4 shell (auto mode)
```

**2. Custom Trajectory with PX4 Shell**
```bash
python scripts/test-flight.py --control px4 --trajectory ../../config/behaviors/trajectories/circle_figure8.json
# Runs: arm → execute orbit/figure-8 → land via PX4 shell
```

**3. MAVLink Testing with Climb Verification**
```bash
python scripts/test-flight.py --control mavlink --mavlink-endpoint 127.0.0.1:14550
# Runs: arm via MAVLink (checks altitude gain) → 10s hover on quad → land
# Fails if takeoff returns false ACK or no altitude gain
```

**4. Rapid Iteration (Skip Arm Phase)**
```bash
python scripts/test-flight.py --skip-arm --trajectory my_payload.json
# Assumes vehicle already armed; tests only trajectory playback and landing
```

**5. Multi-Endpoint Fallback**
```bash
python scripts/test-flight.py --control mavlink \
  --mavlink-endpoint 127.0.0.1:14550 \
  --mavlink-endpoint 127.0.0.1:14540 \
  --mavlink-endpoint 127.0.0.1:14600
# Tries each MAVLink endpoint in order until one responds
```

**6. AirSim-Only Control (for debugging)**
```bash
python scripts/test-flight.py --control airsim --trajectory minimal.json
# Tests pure AirSim RPC arm/takeoff (not recommended for real payloads)
```

### Error Handling

**MAVLink Climb Verification:**
If MAVLink returns an ACK for takeoff but the aircraft does not gain altitude within 2 seconds, test-flight.py fails with diagnostic output:
```
MAVLink takeoff ACK received, but no altitude gain detected. Possible false ACK.
```
Use `--force-mavlink-arm` to bypass this check (at your own risk).

**PX4 Shell Arm Failures:**
The auto mode fallback chain logs each attempt. If PX4 shell is unavailable, MAVLink is tried next.

**File Validation:**
Trajectory files are validated at startup:
* Pre-flight check: File must exist, be valid JSON, and contain a `segments` array
* Errors are reported at argparse stage (before any flight commands execute)

---

## Directory Structure
```text
.
├── setup.sh                         # Root provisioner
├── cleanup.sh                       # Root reset/rebuild cleanup helper
├── third_party/
│   ├── PX4-Autopilot/               # Generated PX4 upstream checkout
│   ├── iceoryx_build/               # Generated iceoryx checkout/build state
│   └── colosseum_environments/      # Downloaded Colosseum/AirSim environments
├── config/behaviors/trajectories/   # JSON trajectory files
└── simulation/airsim/
    ├── run.sh                       # AirSim/PX4 SITL launcher
    ├── stop.sh                      # Normal AirSim/PX4 SITL runtime stop helper
    ├── settings.json                # AirSim vehicle config
    ├── scripts/                     # AirSim RPC and diagnostic scripts
    ├── validation/                  # AirSim-specific validation helpers
    ├── INSTALL.md                   # EC2 provisioning guide
    └── README.md                    # This document
```

## Maintenance & Persistence
* **Linger Mode:** The user session is set to "linger," meaning the DCV session starts automatically on boot.
* **Service Control:** To reset the visual display without rebooting, run:
  ```bash
  systemctl --user restart dcv-session.service
  ```

## Cleanup versus stop

Use `simulation/airsim/stop.sh` for normal runtime shutdown. It stops the tmux session and simulator/PX4 runtime processes without removing build artifacts.

Use root `cleanup.sh` only when you intentionally want reset/rebuild cleanup. It is not the normal runtime stop path.
