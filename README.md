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

## Runtime Dataflow

The current runtime is organized around typed publishers and subscribers. Control-critical consumers subscribe in-process; external tools subscribe to the optional runtime event stream. Artifact files remain evidence/debug outputs, not runtime IPC.

```text
AirSim / PX4
  -> AirSimFrameSource + FrameHintEgoProvider
  -> CoreStackRunner
       -> PerceptionPipeline
       -> optional GhostTargetProvider::frame_at(...)
            -> GhostDetectionsPublisher
            -> PerceptionPipelineOutput.observations
       -> InMemoryWorldModel
       -> WorldSnapshotPublisher
            -> LatestWorldSnapshotSubscriber -> MissionRuntime -> FlightCommandSink -> PX4 / AirSim
            -> ArtifactSnapshotWriter        -> snapshot_XXXX.json / snapshot_manifest.txt
            -> RuntimeEventStreamServer      -> TCP JSONL stream

RuntimeEventStreamServer
  -> ghost_detections events for planned simulation/debug markers
  -> world_snapshot events for AG/EGO world-model markers
  -> simulation/airsim/scripts/airsim-world-overlay.py and other external subscribers
```

The AirSim overlay is now a subscriber/renderer only: it consumes `ghost_detections` and `world_snapshot` records from the runtime stream and renders PLAN / PLAN* / AG / EGO markers. It does not evaluate ghost scenarios locally and does not poll snapshot artifacts in normal operation.

See [docs/runtime_dataflow.md](docs/runtime_dataflow.md) for the full source → publisher → server → subscriber → sink diagrams.

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
   cd ~/dedalus/simulation/airsim && ./run.sh AirSimNH
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

### Using AirSim Providers with the Core Stack

The core stack uses config-driven provider composition. Provider configs live under `config/` and are loaded by the C++ debug apps:

```bash
cmake -S . -B build-validation \
  -DDEDALUS_BUILD_APPS=ON \
  -DDEDALUS_BUILD_TESTS=ON
cmake --build build-validation -j$(nproc)
```

The CI-safe synthetic provider path is:

```bash
./build-validation/apps/dedalus_core_stack \
  --config config/core_stack_ci.yaml
```

The CI-safe recorded-frame path is:

```bash
./build-validation/apps/dedalus_core_stack \
  --config config/core_stack_recorded_ci.yaml
```

That recorded config reads the tiny fixture manifest in `tests/fixtures/recorded_frames/` through `RecordedFrameSource`. It does not require AirSim, OpenCV, GStreamer, FFmpeg, or media codecs.

#### Export AirSim Frames into the Recorded-Frame Provider

The current practical AirSim integration path is a Python bridge that exports AirSim camera frames into the same recorded-frame manifest format consumed by the C++ core stack.

Start the simulation first:

```bash
cd ~/dedalus/simulation
./run.sh
```

In another terminal, export one or more AirSim frames:

```bash
cd ~/dedalus
source ~/dedalus/venv/bin/activate
python simulation/airsim/scripts/export-airsim-frames.py \
  --host 127.0.0.1 \
  --rpc-port 41451 \
  --vehicle-name PX4 \
  --camera-name front_center \
  --count 5 \
  --output out/airsim_frames
```

The exporter writes:

```text
out/airsim_frames/*.ppm
out/airsim_frames/manifest.txt
out/airsim_frames/metadata.json
out/airsim_frames/core_stack_airsim_recorded.yaml
```

Then run the C++ core stack on those exported AirSim frames:

```bash
./build-validation/apps/dedalus_core_stack \
  --config out/airsim_frames/core_stack_airsim_recorded.yaml
```

This executes:

```text
RecordedFrameSource
  -> NoTelemetryEgoProvider
  -> ScriptedDetector
  -> SimpleCentroidTracker
  -> AppearanceOnlyIdentityResolver
  -> FlatGroundProjector
  -> InMemoryWorldModel
  -> WorldSnapshot JSON
```

This path is the preferred AirSim workflow right now because it exercises real simulation imagery while keeping the C++ core stack dependency-free and reproducible.

#### Direct AirSim Provider Names

The following provider names are already reserved and registered for direct AirSim integration:

```yaml
frame_source: airsim
ego_provider: airsim
detector: airsim_ground_truth
projector: airsim_depth
```

The example config is:

```bash
config/core_stack_airsim_example.yaml
```

It includes:

```yaml
frame_source: airsim
ego_provider: airsim
detector: airsim_ground_truth
tracker: simple_centroid
identity_resolver: appearance_only
projector: airsim_depth
world_model: in_memory
fallback_map_frame_id: map_airsim_0001

airsim_host: 127.0.0.1
airsim_rpc_port: 41451
airsim_vehicle_name: PX4
airsim_camera_name: front_center
```

In the current dependency-free build, these direct AirSim providers are explicit stubs. Running this config will fail with a clear integration-provider unavailable error. That is intentional: it protects CI and unit tests from accidentally depending on AirSim/PX4/media libraries while preserving the exact provider names and config shape for the future direct backend.

#### AirSim Bridge Latency Profiler

With AirSim running, profile the binary bridge across both ingestion modes in one command:

```bash
./scripts/profile-airsim-bridge-latency.sh \
  --frames 300 \
  --width 1280 --height 720 \
  --skip-build --skip-ctest
```

This runs two passes back to back:
- **`separate_ego`** — `stream_binary` RGB bridge + `ego_provider: airsim` (separate RPC per frame)
- **`frame_ego`** — `stream_binary_ego` RGB+ego co-stream + `ego_provider: frame_hint` (one RPC per frame)

Key options:

| Option | Description |
|---|---|
| `--mode MODE` | `separate-ego`, `frame-ego`, or `both` (default) |
| `--capacity` | Disable bridge pacing; measures raw capture throughput (default) |
| `--paced` | Use `--rate-hz FPS`; tests mission-like pacing |
| `--frames N` | Frames per pass (default: 300) |
| `--fps FPS` | FPS reference / paced rate (default: 5) |
| `--bridge-timing` | Write bridge-internal timing JSONL (default on) |
| `--no-bridge-timing` | Skip bridge-internal timing |
| `--skip-build` | Skip cmake build |
| `--skip-ctest` | Skip CTest |

The script prints p95/p99 latency with colour-coded capacity ratings and a full stage breakdown:

```text
GREEN   p95 ≤ 33.3 ms   ~30 FPS capable
YELLOW  p95 ≤ 66.7 ms   ~15 FPS capable
RED     p95  > 66.7 ms  below 15 FPS
```

When bridge-internal timing is enabled (the default), a second breakdown table shows how each frame's time is spent inside the Python bridge (`sim_get_images_ms`, `ego_sample_ms`, `rgb_convert_ms`, `stdout_write_ms`, `sleep_ms`). This is produced by `scripts/_summarize-bridge-timing.py` from a `bridge_timing.jsonl` written by the bridge script.

Current validated 640x360 `frame_ego` capacity baseline, after async one-frame prefetch and measured/accounted profile accounting:

```text
actual_resolution_counts: 640x360:600
bridge/main wait p95:     30.220 ms  (GREEN, ~33.1 FPS capacity)
measured runner p95:      31.129 ms  (~32.1 FPS capacity)
accounted runner p95:     30.343 ms
accounting delta abs p95:  0.989 ms

background_fetch_detail:
  read_header p95:        30.519 ms
  read_payload p95:        0.856 ms

bridge-internal timing:
  sim_get_images_ms p95:  29.933 ms
  stdout_write_ms p95:     0.870 ms
```

This confirms that 640x360 `frame_ego` is inside the 30 FPS p95 budget. The remaining dominant cost is AirSim `simGetImages` / render readback / RPC, not C++ parsing, frame construction, or payload transfer. `frame_source.detail.*` timings are background-fetch attribution only and are excluded from `accounted_total_us` to avoid double-counting overlapped work.

All output lands in `out/airsim_bridge_latency_<timestamp>/`.

### Behavior / Flight Pipeline Direction

The next integration goal is to close the loop from perception and world-model state into flight behavior:

```text
FrameSource
  -> PerceptionPipeline
  -> WorldModel
  -> BehaviorPipeline
  -> FlightCommandSink
  -> AirSim / PX4 SITL velocity control
```

The behavior pipeline should consume `WorldSnapshot` or a future `EffectiveWorldView` and emit bounded kinematic intents, not direct motor or attitude commands. PX4 remains responsible for stabilization, estimator fusion, arming, motor control, and low-level failsafes.

Initial placeholder behavior should mirror `simulation/airsim/scripts/test-flight.py`: it reads the existing trajectory JSON format and emits velocity vectors at a configured rate while ignoring the world model. That gives a complete perception -> world-model -> behavior -> flight-control path before higher-level autonomy is ready.

Proposed initial config shape:

```yaml
behavior_pipeline: trajectory
behavior_trajectory_path: config/behaviors/trajectories/circle_figure8.json
behavior_rate_hz: 10

flight_command_sink: airsim_velocity
flight_control_mode: px4
flight_safe_height_m: 8
```

Implementation plan and safety notes are captured in [docs/behavior_flight_pipeline_plan.md](docs/behavior_flight_pipeline_plan.md).

---

## CI/CD

Three GitHub Actions workflows gate the build and smoke-validate the `WorldSnapshot` JSON contract on every relevant event.

### Workflows

| Workflow | File | Trigger | Build type | Purpose |
|---|---|---|---|---|
| **CI** | `ci.yml` | PR targeting `staging` or `main`; manual dispatch | Debug | Gate PRs — must pass before merge |
| **Staging** | `staging.yml` | Push to `staging` or `main`; manual dispatch | Debug | Pre-release gate for branch integrations |
| **Production** | `production.yml` | Push of `v*.*.*` tag; manual dispatch | Release (`-O2`) | Validate release candidates |

**Trigger rationale:**
- CI is PR-only — every contribution is validated before merge, not just after.
- Staging fires on both the `staging` branch (normal integration path) **and** `main` (direct-to-production shortcut) so `main` is never dark after a merge.
- Production is tag-triggered, not merge-triggered — a human must explicitly cut a `v*.*.*` tag. A push to `main` alone does not promote to production.

### Smoke Validation Contract

Every workflow builds `dedalus_core_stack` and asserts the emitted `WorldSnapshot` JSON satisfies:

| Check | Assertion |
|---|---|
| Valid JSON | `python3 -m json.tool` exits 0 |
| `active_map_frame_id` | field present and equals `"map_local_0001"` |
| Person agent | `"class": "person"` present in agents array |
| Car container | `"type": "car"` present in containers array |
| `map_frames` populated | `"map_frame_id": "map_local_0001"` present (not silently empty) |

Results are written to `$GITHUB_STEP_SUMMARY` as a markdown table visible in the GitHub Actions UI.

### Cutting a Release

```bash
# After merging to main and staging workflow passes:
git tag -a v0.1.0 -m "Milestone 1A: world snapshot spine"
git push origin v0.1.0
# → triggers Production workflow
```

### Actions Versions

All workflows use `actions/checkout@v6` and `actions/upload-artifact@v6` (Node.js 24-native, no deprecation warnings).

---

## Getting Started
*(Documentation on bootstrapping the local Colosseum simulation environment and Jetson L4T cross-compilation Docker containers is forthcoming.)*

## Useful commands

```bash
titan@Titans-MacBook-Pro dedalus % git rev-parse --short HEAD
5fedb2f
```
