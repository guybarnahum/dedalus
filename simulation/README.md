# 🎮 The Virtual Proving Ground (Simulation Environment)

![Sim Engine](https://img.shields.io/badge/Engine-Colosseum_(AirSim)-blue)
![Flight Controller](https://img.shields.io/badge/FCU-PX4_SITL-orange)
![Mode](https://img.shields.io/badge/Mode-Headless_%2F_Interactive-brightgreen)

## Overview
Welcome to the Project Dedalus **Virtual Proving Ground**. 

Due to the extreme hardware risks of testing high-speed tactical flight software, this directory houses the integration layer between our C++ flight stack and our virtual physics environment. This environment strictly enforces the core project philosophy: **Train Heavy, Simulate Accurately, Fly Light.**

Before any Dedalus C++ binary is flashed to the physical Jetson Orin airframe, it must successfully execute its flight behavior here without crashing.

## Core Components
This simulation environment fuses three distinct software stacks:
1. **Colosseum (Unreal Engine):** An actively maintained fork of Microsoft AirSim. It provides high-fidelity physics, spawns dynamic targets (moving drones/cars), and renders virtual camera buffers.
    * *Note on Optics:* Colosseum is configured to render **raw, distorted fisheye images** to match our physical MIPI global shutter lenses, as our pipeline strictly forbids real-time dewarping.
2. **PX4 SITL (Software In The Loop):** The exact same flight controller firmware that runs on the physical Pixhawk, compiled to run locally on your dev machine. It receives MAVLink vectors from Dedalus and translates them into virtual motor mixing.
3. **The IPC Bridge:** A translation layer that pipes Colosseum's virtual camera buffers directly into **Eclipse iceoryx** shared memory, tricking the Dedalus Perception Node into thinking it is connected to a physical MIPI camera.

## Hardware Requirements
Simulating high-speed vision pipelines is computationally heavy. Attempting to run this without a dedicated GPU will result in dropped frames, PID loop desyncs, and catastrophic virtual crashes.

* **Local Development:** NVIDIA RTX 3060 (Minimum) / RTX 4070 (Recommended), 32GB RAM. Native Ubuntu 22.04 or Windows 11 WSL2.
* **Cloud Development (EC2 Workstation):** AWS `g5.2xlarge` using NICE DCV or Parsec for remote display.

## Directory Structure
```text
simulation/
├── colosseum_environments/  # Compiled Unreal Engine binaries (e.g., CityBlock, DesertTest)
├── scenarios/               # JSON files defining CI/CD test gates
│   ├── test_01_hover.json
│   ├── test_02_intercept.json
│   └── test_03_repulse.json
├── sitl_runner.sh           # Master script to boot PX4 + Sim + IPC Bridge
└── README.md                # This document
```

## Running the Simulation (Interactive Developer Mode)
*(Note: Full installation scripts are located in `infrastructure/`)*

To test your local C++ changes interactively:
1. **Boot the Physics Engine:** Launch the Colosseum Unreal Engine binary in windowed mode.
2. **Boot the Flight Controller:** Run the PX4 SITL daemon (`make px4_sitl none_iris`).
3. **Execute Dedalus:** Spin up your local L4T cross-compilation Docker container and execute the flight stack. 
   ```bash
   ./simulation/sitl_runner.sh --interactive
   ```
4. Watch the Colosseum window to visually verify that your target bounding boxes project correctly into 3D space and that the drone generates safe potential fields.

## CI/CD Pipeline Gating (Headless Mode)
This directory acts as the automated gatekeeper for the `main` branch. 

When a Pull Request is opened, GitHub Actions spins up an ephemeral AWS `g5` runner. It executes `sitl_runner.sh --headless`, loading specific JSON configurations from the `scenarios/` folder. 

If your code causes the drone to violate minimum safe distance parameters (e.g., a collision in `test_03_repulse.json`), the PR is automatically blocked.