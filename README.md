# 🚁 Project Dedalus: Vision-Centric Autonomous Flight Stack

![C++20](https://img.shields.io/badge/C++-20-blue.svg)
![NVIDIA Jetson](https://img.shields.io/badge/Platform-Jetson_Orin-76B900.svg)
![IPC](https://img.shields.io/badge/IPC-Eclipse_iceoryx-purple.svg)
![Simulation](https://img.shields.io/badge/Sim-Colosseum_%2B_PX4_SITL-orange.svg)

## Overview
Project Dedalus is a high-performance, vision-only autonomous drone flight stack designed for high-speed tactical interception and dynamic tracking. 

Built specifically for **NVIDIA Jetson Orin** edge-compute modules, the system acts as an aggressive, lightweight autonomous agent. It achieves tactical autonomy—including target tracking, persistent 3D world-modeling, and reactive collision avoidance—entirely through passive optical sensors, bypassing the SWaP (Size, Weight, and Power) penalties of LiDAR or Radar.

## Core Engineering Principles
* **Physics First:** High-speed flight dictates hardware. The stack mandates **MIPI Global Shutter** cameras. Ethernet cameras and rolling shutters are fundamentally incompatible with the flight control loop.
* **Compute Efficiency:** Real-time fisheye dewarping is bypassed. YOLO11 models are trained directly on raw distorted images and compiled to **INT8 TensorRT** engines to guarantee a strict 30+ FPS inference rate.
* **Zero-Copy Architecture:** Inter-Process Communication (IPC) is handled by **Eclipse iceoryx**, providing lock-free, zero-copy POSIX shared memory, completely avoiding the latency and serialization overhead of ROS2.

## Key Features
* **Monocular 3D Perception:** Fuses a single camera feed with Visual-Inertial Odometry (VIO) and YOLO geometric priors to extract 3D Cartesian coordinates from 2D images.
* **Faction-Aware Tracking:** Custom YOLO11 detector categorizes dynamic entities as Friend, Neutral, or Foe.
* **Persistent World Model:** Extrapolates trajectories of dynamic targets via Extended Kalman Filters (EKF), maintaining lock during severe occlusions.
* **Behavior Tree Intelligence:** Evaluates complex geometric triggers defined in `behaviors.yaml` to dynamically switch flight goals (Attract, Repulse, Circle, Intercept).

## The Three-Tier Architecture (Sim-First Development)
To guarantee stability and prevent hardware loss, Dedalus enforces a strict separation of environments:

1. **Training Environment (Cloud):** Python/PyTorch pipelines spin up ephemeral AWS EC2 `g5` Spot Instances to train models on raw fisheye data and export ONNX files.
2. **Simulation Environment (CI/CD):** Code executes inside **Colosseum** (AirSim fork) and **PX4 SITL**. Pull requests are automatically gated by headless simulated flight tests.
3. **Edge Environment (Physical Drone):** A pure C++20 runtime operating bare-metal via `.deb` packages and A/B partition flashing on the Jetson Orin.

## Simulation & Flight Testing

The Virtual Proving Ground (`simulation/` directory) provides a complete high-fidelity environment for testing autonomous flight logic before risking physical hardware.

### Quick Start (AWS EC2)
1. **Provision:** Follow [simulation/INSTALL.md](simulation/INSTALL.md) to bootstrap a `g6.2xlarge` GPU instance.
2. **Launch:** Connect via NICE DCV and run:
   ```bash
   cd ~/dedalus/simulation && ./run.sh AirSimNH
   ```
3. **Test Flight:** Execute autonomous flight sequences with:
   ```bash
   python test-flight.py --control px4 --trajectory trajectories/circle_figure8.json
   ```

### Trajectory-Based Flight Testing

The **`test-flight.py`** harness provides multi-mode autonomous flight control with JSON-based trajectory playback:

**Control Modes:**
* **`auto` (default):** Attempts PX4 shell (most reliable) → MAVLink (with climb verification) → AirSim RPC (fallback)
* **`px4`:** Direct shell arm/takeoff/land with AirSim velocity injection for payload execution
* **`mavlink`:** MAVLink protocol with enhanced error detection (detects false ACKs if no altitude gain)
* **`airsim`:** Pure AirSim RPC (least reliable, included for comparison)

**Trajectory Format (JSON):**
Trajectories define multi-segment flight sequences with rate-controlled playback (default 10 Hz):

```json
{
  "name": "example_trajectory",
  "rate_hz": 10,
  "segments": [
    {
      "type": "hold",
      "label": "hover",
      "duration_s": 5,
      "vx_mps": 0.0,
      "vy_mps": 0.0,
      "vz_mps": 0.0
    },
    {
      "type": "circle_velocity",
      "label": "orbit_right",
      "duration_s": 10,
      "center_x": 0.0,
      "center_y": 0.0,
      "radius": 10.0,
      "altitude": 25.0,
      "clockwise": true
    }
  ]
}
```

**Supported Segment Types:**
* `hold` — Maintain position (zero velocity)
* `circle_velocity` — Circular orbit with center offset
* `figure8_velocity` — Figure-eight pattern
* `velocity_keyframes` — Arbitrary waypoint list with linear interpolation

**Interactive Testing:**
```bash
# Basic flight test with default 10s hover trajectory
python test-flight.py

# Custom trajectory with explicit control mode
python test-flight.py --control px4 --trajectory my_trajectory.json

# MAVLink with climb verification enabled
python test-flight.py --control mavlink --mavlink-endpoint 127.0.0.1:14550

# Real-time diagnostics (verbose PX4 shell output)
python test-flight.py --control px4 --px4-tmux-target dedalus-sim:px4
```

See [simulation/README.md](simulation/README.md) for complete CLI reference and [simulation/INSTALL.md](simulation/INSTALL.md) for environment setup.

---

## Getting Started
*(Documentation on bootstrapping the local Colosseum simulation environment and Jetson L4T cross-compilation Docker containers is forthcoming.)*