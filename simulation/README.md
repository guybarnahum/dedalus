# 🎮 The Virtual Proving Ground (Simulation Environment)

<img width="1446" height="833" alt="Screenshot 2026-05-07 at 1 55 20 PM" src="https://github.com/user-attachments/assets/83ebe7e1-0355-4400-9ac6-f7514c291f7c" />

![Sim Engine](https://img.shields.io/badge/Engine-Colosseum_(AirSim)-blue)
![Flight Controller](https://img.shields.io/badge/FCU-PX4_SITL-orange)
![Visualization](https://img.shields.io/badge/Display-NICE_DCV-green)
![Hardware](https://img.shields.io/badge/GPU-NVIDIA_L4-76B900)

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
   Run the master setup script to install dependencies, fetch Colosseum binaries, and configure the XDCV virtual display manager:
   ```bash
   ./simulation/setup.sh
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
cd ~/dedalus/simulation
./run.sh AirSimNH
```
*The script automatically probes the DCV metadata to export the correct `DISPLAY` and `XAUTHORITY` variables, and grants X11 permissions via `xhost`.*

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

## Directory Structure
```text
simulation/
├── colosseum_environments/  # Compiled Unreal Engine binaries (Blocks, AirSimNH)
├── setup.sh                 # Master provisioner (NICE DCV, XDCV, systemd)
├── run.sh                   # Visual-aware simulation launcher
├── scenarios/               # JSON files defining CI/CD test gates
└── README.md                # This document
```

## Maintenance & Persistence
* **Linger Mode:** The user session is set to "linger," meaning the DCV session starts automatically on boot.
* **Service Control:** To reset the visual display without rebooting, run:
  ```bash
  systemctl --user restart dcv-session.service
  ```
* **Session Safety:** Avoid clicking "Log Out" inside the DCV desktop. Simply close the client app to keep the simulation running in the background.
