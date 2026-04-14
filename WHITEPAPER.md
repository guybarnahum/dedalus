# Architecture White Paper: Project Dedalus

**Document Type:** Technical Strategy & Architecture Review
**Target Audience:** Engineering Leadership, Principal Architects
**Subject:** Vision-Centric Autonomous Flight Stack for High-Speed Edge-Compute UAVs

---

## 1. Executive Summary
This document outlines the architectural foundation for Project Dedalus, a high-performance, vision-only autonomous drone flight stack. Designed for high-speed dynamic tracking and interception, the system operates entirely on edge compute (NVIDIA Jetson Orin NX/AGX). 

By eschewing active sensors (LiDAR/Radar) in favor of passive optics, we optimize for strict Size, Weight, and Power (SWaP) constraints. The architecture is modular, relies on zero-copy memory transport, and is designed with rigorous mission-assurance safeguards to ensure safe operation in complex, GPS-denied environments.

---

## 2. Hardware Realities & Physics Constraints
High-speed autonomous flight dictates strict hardware requirements. Software cannot compensate for fundamental sensor physics.

* **Sensor Typology:** The system mandates a **Global Shutter** sensor with a direct **MIPI CSI-2 or GMSL2** interface. Ethernet IP cameras and rolling shutters are strictly forbidden due to network stack latency and high-speed motion distortion (jello effect).
* **Compute & Frame Rate:** A 5 FPS inference rate is a critical failure state (at 30 m/s, a target moves 6 meters between frames). The system is mandated to operate at **30+ FPS**. This requires Jetson Orin NX (16GB) or AGX Orin compute tiers; Nano variants and desktop-class GPUs (RTX 3060) are rejected due to SWaP violations.
* **Pipeline Efficiency:** Real-time fisheye dewarping wastes critical TOPS. The pipeline bypasses geometric correction by training the neural network directly on raw, distorted fisheye images.

---

## 3. Technology Strategy & Language Selection
To meet strict latency budgets, the software stack is bifurcated between runtime execution and offline infrastructure.

* **The Core Runtime (C++20):** The production flight stack is written in C++20. It provides native, zero-copy access to NVIDIA’s JetPack APIs (CUDA, TensorRT) and ensures deterministic execution without garbage collection pauses.
* **The ML Layer (Python/Shell):** Python is strictly isolated from the flight path, utilized exclusively for the MLOps pipeline: dataset curation, YOLO training, and generating TensorRT `.engine` execution files.
* **Inter-Process Communication (Eclipse iceoryx):** ROS2 is rejected due to serialization overhead. IPC is handled by Eclipse iceoryx, an automotive-grade middleware providing true zero-copy, lock-free POSIX shared memory for passing 4K video frames between C++ nodes with $O(1)$ latency.

---

## 4. Core System Architecture
The software architecture follows a high-frequency node paradigm.

* **Perception Domain:** Ingests raw MIPI video. Utilizes an INT8-quantized YOLO11 network optimized via TensorRT for base object detection and "Faction" assignment (Friend/Foe). Employs Visual-Inertial Odometry (VIO) and geometric pinhole camera models to project 2D bounding boxes into 3D Cartesian space.
* **World Model Domain:** Acts as the single source of truth. Employs a localized voxel grid for static geometry and assigns Extended Kalman Filters (EKF) to dynamic agents. EKFs seamlessly transition to trajectory extrapolation using kinematic models when targets are occluded.
* **Behavior Domain:** Evaluates the World Model against a human-readable `behaviors.yaml` configuration. Behavior Trees (BTs) select active flight goals, and a Potential Fields motion planner outputs the resultant 3D velocity vector and Yaw rate.
* **Control Domain:** A strict Command Multiplexer enforces priority control: Hardware Kill Switch > Human RC Override > AI Motion Planner. Target vectors are translated into MAVLink commands for the flight controller (PX4).

---

## 5. The Three-Tiered Environment Strategy
To prevent hardware loss, Dedalus enforces the robotics philosophy: **Train Heavy, Simulate Accurately, Fly Light.**

* **The Edge Runtime (Physical Drone):** Deterministic execution on the Jetson Orin. Deployment is handled via a hybrid approach: Docker (L4T Base) for development parity, and bare-metal `.deb` packages managed by `systemd` with A/B RootFS partition flashing for safe production OTA updates.
* **The Training Environment (ML Factory):** High-throughput data ingestion utilizing ephemeral AWS EC2 `g5.2xlarge` Spot Instances provisioned dynamically via GitHub Actions.
* **The Simulation Environment (CI/CD):** Hardware-free integration testing. Runs the *exact same* compiled C++ Edge binaries and TensorRT engines inside **Colosseum** (Unreal Engine physics) and **PX4 SITL**, providing automated PR gating before any code reaches the physical airframe.

