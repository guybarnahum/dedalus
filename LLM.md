# Project Context & State: Project Dedalus (LLM.md)
> **Purpose:** This document is intended to be ingested by Large Language Models to instantly load the architectural context, constraints, and current state of the project.

## 1. System Identity & Core Constraints
* **Project Name:** Dedalus
* **Target Hardware:** NVIDIA Jetson Orin NX 16GB or AGX Orin (Optimizing for SWaP; nano is insufficient for 30+ FPS).
* **Primary Paradigm:** Vision-only (passive sensors), GPS-denied capable, high-speed tactical interceptor.
* **Language Constraints:** Strictly C++20 for Edge Runtime. Python/Shell strictly limited to MLOps/Infrastructure.
* **AI Ecosystem:** NVIDIA JetPack SDK (CUDA, TensorRT, VPI).

## 2. Hardware Realities (Strict Mandates)
* **Camera Sensor:** Must be a Global Shutter sensor (e.g., Sony IMX series). Rolling shutters are strictly forbidden due to high-speed jello distortion.
* **Camera Interface:** Direct MIPI CSI-2 or GMSL2. Ethernet IP cameras are forbidden due to network stack latency.
* **Performance Baseline:** The flight control loop MUST operate at 30+ FPS. 
* **Pre-processing:** Real-time fisheye dewarping is forbidden to save compute. YOLO models must be trained/fine-tuned on raw, distorted fisheye images.

## 3. Architecture Domains (The C++ Node System)
* **Perception Node (GPU/DLA):** VIO (Visual-Inertial Odometry) for ego-motion and static sparse clouds.
* **Detection Node (GPU/DLA):** YOLO11 compiled to INT8 TensorRT, featuring a Faction-classification head (Friend/Neutral/Foe).
* **Depth Estimation Node:** Far-field 2D-to-3D raycasting (direction) and Near-field Geometric Priors (pinhole model using known object scales).
* **World Model Node (CPU):** OctoMap (static voxel grid) combined with a Dynamic Agent List.
* **Tracking Node (CPU):** Object persistence managed via ByteTrack. Temporal extrapolation via Extended Kalman Filters (EKF) using Kinematic models (CV/CA) with confidence decay.
* **Behavior & Planning Node (CPU):** Config-driven (`behaviors.yaml`) triggers feeding into Behavior Trees. Potential Field path planner translates intents into 3D velocity vectors and Yaw rate.
* **Safety & Control Node (CPU):** Strict Command Multiplexer. Priority: 0 (Kill Switch) > 1 (Human RC Override) > 2 (AI Velocity Vector). Outputs via MAVLink.
* **IPC Transport:** Eclipse iceoryx. Provides true zero-copy, lock-free POSIX shared memory pub/sub without the overhead of ROS2.

## 4. Environment Segregation (The "Airgaps")
* **ENV_EDGE (The Drone):** Hybrid deployment. Runs bare-metal `.deb` packages managed by `systemd` via A/B RootFS partition flashing.
* **ENV_TRAIN (The Cloud):** Ephemeral AWS `g5.2xlarge` Spot Instances triggered via GitHub Actions + Terraform. Trains YOLO models, exports to ONNX.
* **ENV_SIM (The Proving Ground):** Colosseum (AirSim fork) + PX4 SITL. Executes the exact Edge C++ binaries via cross-compilation Docker containers against simulated MAVLink and shared-memory virtual camera buffers.

## 5. Implementation & Simulation Plan (Phased Rollout)
1. **Phase 1 (Virtual Proving Ground):** Establish headless Colosseum game environment and MAVLink bridge to virtual PX4.
2. **Phase 2 (Perception Binding):** Connect `camera_driver` to virtual shared-memory buffers, integrate YOLO11, and validate 2D-to-3D projection against game-engine ground truth.
3. **Phase 3 (Intelligence):** Activate World Model EKFs and Behavior Trees to autonomously track virtual dynamic targets.
4. **Phase 4 (CI/CD Integration):** Deploy automated GitHub Actions (`sitl_gpu_sim.yml`) enforcing PR gating via simulated flight scenarios on AWS GPU runners.

## 6. Repository Structure
```text
dedalus/
├── .github/workflows/   (CI/CD: standard build, aws-gpu-provision, sitl-sim)
├── config/              (behaviors.yaml, camera_intrinsics.yaml)
├── infrastructure/      (AWS Terraform, L4T Cross-compilation Dockerfiles)
├── models/              (TensorRT engines, faction mapping JSON)
├── scripts/             (MLOps, YOLO training pipelines)
├── simulation/          (Headless Colosseum binaries, PX4 SITL boot scripts)
└── src/                 (C++20 source tree)
    ├── sensors/         (MIPI drivers, FCU MAVLink ingestion)
    ├── perception/      (TensorRT inference, Tracker, Geometric Depth)
    ├── world_model/     (OctoMap, EKF Trajectory Predictors)
    ├── behavior/        (Behavior Trees, Potential Fields)
    ├── safety/          (Multiplexer, Manual Override)
    └── ipc/             (Eclipse iceoryx configuration and structs)
