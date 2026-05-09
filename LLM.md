# Project Dedalus - LLM Context & Operational State

> **Purpose:** This file is the project handoff document for LLMs. It should let an LLM understand the architecture, repo conventions, current simulation state, known bugs, and safe modification patterns without rediscovering the debugging history.

---

## 1. Project Identity

**Project Name:** Dedalus

Dedalus is a virtual proving ground and edge-autonomy stack for drone behavior, perception, world modeling, and control experiments.

The current repo combines:

- C++20 autonomy/runtime code under `src/`
- configuration under `config/`
- simulation orchestration under `simulation/`
- infrastructure helpers under `infrastructure/`
- project strategy/architecture notes in `WHITEPAPER.md` and this file

The immediate working focus is the simulation environment:

```text
Colosseum / AirSim fork  <->  PX4 SITL  <->  Python flight test client
```

The simulation is being run on:

```text
Ubuntu 22.04
AWS g6 instance
NVIDIA L4 GPU
NICE DCV virtual desktop
Colosseum / AirSim fork
PX4 SITL
AirSim Python API 1.8.1
```

---

## 2. Repository Structure

Current expected top-level structure:

```text
dedalus/
├── CMakeLists.txt
├── config/
│   ├── behaviors.yaml
│   └── camera_intrinsics.yaml
├── infrastructure/
│   ├── aws/
│   │   └── main.tf
│   └── docker/
│       └── Dockerfile.l4t_cross
├── LLM.md
├── models/
├── README.md
├── scripts/
├── simulation/
│   ├── cleanup.sh
│   ├── INSTALL.md
│   ├── README.md
│   ├── run.sh
│   ├── scenarios/
│   ├── settings.json
│   ├── setup.sh
│   ├── test-flight.py
│   └── trajectories/
│       └── circle_figure8.json
├── src/
│   ├── behavior/
│   ├── CMakeLists.txt
│   ├── ipc/
│   ├── perception/
│   ├── safety/
│   ├── sensors/
│   └── world_model/
└── WHITEPAPER.md
```

---

## 3. Architectural Direction

Dedalus is intended to support a modular drone autonomy stack.

### Edge Runtime

The future edge runtime should be C++20-first.

Major runtime domains:

- `sensors/`: camera, IMU, MAVLink/FCU ingestion
- `perception/`: detection, tracking, pose/features, TensorRT inference
- `world_model/`: local state, dynamic agents, trajectories, confidence
- `behavior/`: policy, intent, behavior trees, mission logic
- `safety/`: command mux, kill switch, manual override, bounded outputs
- `ipc/`: low-latency data exchange; current intended IPC is Eclipse iceoryx

### Control Philosophy

The autonomy stack should produce bounded kinematic intents:

```text
velocity vector + yaw/yaw-rate intent
```

The flight controller remains responsible for stabilization, estimator fusion, arming state, motor control, failsafes, and low-level flight safety.

---

## 4. Simulation Stack

The current simulation environment uses:

```text
Colosseum / AirSim fork
PX4 SITL
AirSim Python API
NICE DCV for rendering
tmux for process orchestration
```

The important files are:

```text
simulation/setup.sh       installs/builds dependencies
simulation/cleanup.sh     resets runtime/build state
simulation/run.sh         launches DCV-bound simulation stack
simulation/settings.json  AirSim/Colosseum vehicle configuration
simulation/test-flight.py flight test and trajectory runner
```

The intended workflow is:

```bash
cd ~/dedalus/simulation
./setup.sh --yes
./cleanup.sh --soft --yes
./run.sh
```

Then in another terminal:

```bash
cd ~/dedalus/simulation
source ~/dedalus/venv/bin/activate
python test-flight.py
```

---

## 5. Setup / Cleanup / Run Responsibilities

### `setup.sh`

`setup.sh` should be the only script responsible for installing or verifying dependencies.

It should:

- verify GPU / NVIDIA runtime
- configure NICE DCV
- build/install Eclipse iceoryx
- clone/stage PX4
- create and update `~/dedalus/venv`
- install Python dependencies:

```text
airsim
numpy
msgpack-rpc-python
pymavlink
pyserial
kconfiglib
```

Important PX4 dependency note:

```text
kconfiglib is required because PX4 imports menuconfig/kconfiglib during build.
```

Do **not** run:

```bash
make px4_sitl none_iris
```

inside `setup.sh` as a "build verification" step. That target starts PX4 and will block waiting for the simulator TCP server on port `4560`.

Use build-only targets in setup, for example:

```bash
make px4_sitl
```

### `cleanup.sh`

`cleanup.sh` should reset runtime state before restarting simulation.

Use:

```bash
./cleanup.sh --soft --yes
```

for normal restarts.

Use:

```bash
./cleanup.sh --px4 --yes
```

when PX4 build/source state must be forced to rebuild.

Use:

```bash
./cleanup.sh --hard --yes
```

only for deeper reset.

Cleanup should kill stale:

```text
AirSimNH / Blocks / Linux-Shipping
px4
iox-roudi
dedalus-sim tmux session
stale iceoryx shared memory
```

### `run.sh`

`run.sh` should own process orchestration.

Current working order:

```text
1. Resolve DCV DISPLAY / XAUTHORITY
2. Spawn tmux session if not already inside tmux
3. Start iox-roudi
4. Copy simulation/settings.json to ~/Documents/AirSim/settings.json
5. Verify Python/PX4 dependencies
6. Launch Colosseum / Unreal / AirSim first
7. Wait for AirSim TCP server on 127.0.0.1:4560
8. Launch PX4 SITL in tmux window named px4
9. Keep Unreal process in foreground of main tmux window
```

The critical point is:

```text
PX4 waits for simulator TCP 4560, so AirSim/Colosseum must be started first.
```

Expected healthy AirSim log:

```text
Waiting for TCP connection on port 4560, local IP 127.0.0.1
Connected to SITL over TCP.
Connecting to PX4 Control UDP port 14600, local IP 127.0.0.1, remote IP 127.0.0.1 ...
received first heartbeat
Ground control connected over UDP.
```

Expected healthy PX4 log:

```text
INFO  [simulator_mavlink] using TCP on remote host localhost port 4560
INFO  [simulator_mavlink] Simulator connected on TCP port 4560.
INFO  [tone_alarm] home set
INFO  [commander] Ready for takeoff!
```

---

## 6. AirSim / PX4 Settings

The working AirSim PX4 mode is TCP simulator link, not UDP.

`simulation/settings.json` should use:

```json
{
    "SettingsVersion": 1.2,
    "SimMode": "Multirotor",
    "LogMessagesVisible": true,
    "ShowDashboard": true,
    "ViewMode": "FlyWithMe",
    "OriginGeopoint": {
        "Latitude": 47.641468,
        "Longitude": -122.140165,
        "Altitude": 121
    },
    "ClockSpeed": 1.0,
    "Vehicles": {
        "PX4": {
            "VehicleType": "PX4Multirotor",
            "UseSerial": false,
            "UseTcp": true,
            "LockStep": true,
            "TcpPort": 4560,
            "ControlIp": "127.0.0.1",
            "ControlPort": 14600,
            "Parameters": {
                "COM_ARM_WO_GPS": 1,
                "NAV_RCL_ACT": 0,
                "NAV_DLL_ACT": 0,
                "COM_OBL_ACT": 1
            }
        }
    }
}
```

Historical note:

```text
UDP mode created partial/invalid states for this environment.
PX4 was waiting for simulator TCP 4560 while AirSim was configured for UDP.
That caused HIL missing warnings and arming/control failures.
```

---

## 7. Known AirSim / Colosseum PX4 Limitation

The AirSim/Colosseum RPC call:

```python
client.armDisarm(True, vehicle_name="PX4")
```

currently fails with:

```text
rpclib: function 'armDisarm' (called with 2 arg(s)) threw an exception.
The exception is not derived from std::exception.
No further information available.
```

This is isolated to the AirSim/Colosseum PX4 arm/disarm RPC bridge.

Evidence:

```text
client.enableApiControl(True) works
AirSim GPS sensor is valid
PX4 TCP simulator link works
PX4 receives heartbeat
PX4 reports Ready for takeoff
pxh> commander arm spins props
pymavlink MAV_CMD_COMPONENT_ARM_DISARM returns MAV_RESULT_ACCEPTED
```

Therefore:

```text
Do not depend on AirSim armDisarm() for PX4 in this environment.
```

Confirmed working path:

```text
PX4 shell commander arm
PX4 shell commander takeoff
AirSim API velocity-vector trajectory playback
PX4 shell commander land
PX4 shell commander disarm
```

The Python flight test currently exposes this via:

```bash
python test-flight.py --control px4
```

and the default:

```bash
python test-flight.py
```

should use:

```text
--control auto
```

where `auto` currently prefers the confirmed PX4 shell path.

---

## 8. PX4 Shell Access

PX4 runs in a tmux window named:

```text
dedalus-sim:px4
```

Attach:

```bash
tmux attach -t dedalus-sim
```

Select window:

```text
Ctrl-b w
```

The PX4 shell prompt is:

```text
pxh>
```

Important terminal note:

```text
PX4 shell input may not visibly echo in tmux/DCV, but commands still execute after pressing Enter.
```

Alternative from another SSH terminal:

```bash
tmux send-keys -t dedalus-sim:px4 'commander status' C-m
tmux send-keys -t dedalus-sim:px4 'mavlink status' C-m
tmux send-keys -t dedalus-sim:px4 'commander arm' C-m
tmux send-keys -t dedalus-sim:px4 'commander takeoff' C-m
tmux send-keys -t dedalus-sim:px4 'commander land' C-m
```

Useful PX4 shell commands:

```bash
commander status
mavlink status
listener sensor_gps
listener vehicle_gps_position
listener vehicle_local_position
commander arm
commander takeoff
commander land
commander disarm
```

---

## 9. `test-flight.py` Current Behavior

`simulation/test-flight.py` is a flight-test harness.

It should:

- connect to AirSim
- detect vehicle from `listVehicles()`
- verify AirSim vehicle state
- verify AirSim GPS sensor
- enable API control
- optionally measure timestamp-derived API FPS
- run a selected control strategy
- play velocity-vector trajectories from JSON when using the PX4 shell path

Control modes:

```bash
python test-flight.py --control auto
python test-flight.py --control px4
python test-flight.py --control mavlink
python test-flight.py --control airsim
```

Current status:

```text
--control auto     confirmed working; prefers PX4 shell path
--control px4      confirmed working; arm/takeoff/land via pxh>, trajectory via AirSim velocity commands
--control mavlink  experimental; arms successfully, NAV_TAKEOFF ACKs, but may not produce physical climb
--control airsim   expected to fail due to broken AirSim armDisarm() RPC
```

MAVLink status:

```text
pymavlink can receive heartbeat
pymavlink arm command gets MAV_RESULT_ACCEPTED
NAV_TAKEOFF can get MAV_RESULT_ACCEPTED
but no real climb may occur
```

So `--control mavlink` should verify real local-z movement and report failure if takeoff is ACKed but no climb is observed.

---

## 10. Trajectory System

The preferred mission-body interface is a JSON trajectory file that defines velocity-vector commands.

Default intended trajectory file:

```text
simulation/trajectories/circle_figure8.json
```

Run:

```bash
python test-flight.py --control px4 --trajectory trajectories/circle_figure8.json
```

The pattern is:

```text
PX4 shell: commander arm
PX4 shell: commander takeoff
AirSim: stream velocity vectors from JSON trajectory
PX4 shell: commander land
PX4 shell: commander disarm
```

Supported trajectory segment types:

```text
hold
circle_velocity
figure8_velocity
velocity_keyframes
```

Example JSON:

```json
{
    "name": "circle_then_figure8",
    "description": "Large circle followed by figure eight using NED velocity vectors.",
    "rate_hz": 10,
    "segments": [
        {
            "type": "circle_velocity",
            "label": "large clockwise circle",
            "duration_s": 36,
            "speed_mps": 3.0,
            "radius_m": 18,
            "direction": "cw",
            "vz_mps": 0.0
        },
        {
            "type": "hold",
            "label": "brief center hold",
            "duration_s": 4,
            "vx_mps": 0.0,
            "vy_mps": 0.0,
            "vz_mps": 0.0
        },
        {
            "type": "figure8_velocity",
            "label": "large figure eight",
            "duration_s": 48,
            "speed_mps": 3.0,
            "scale_m": 18,
            "vz_mps": 0.0
        },
        {
            "type": "velocity_keyframes",
            "label": "soft exit and settle",
            "keyframes": [
                { "t": 0, "vx_mps": 1.5, "vy_mps": 0.0, "vz_mps": 0.0 },
                { "t": 3, "vx_mps": 0.8, "vy_mps": 0.0, "vz_mps": 0.0 },
                { "t": 6, "vx_mps": 0.0, "vy_mps": 0.0, "vz_mps": 0.0 }
            ]
        }
    ]
}
```

The trajectory player should print live status on one terminal line where appropriate, for example:

```text
t=  9.9/ 10.0s v=(+0.00,+0.00,+0.00) m/s
```

and avoid spamming repeated lines for MAVLink climb verification.

---

## 11. Known Debugging Milestones

These are important facts future LLMs should not rediscover from scratch:

1. SimpleFlight works with AirSim RPC:

     ```text
     enableApiControl works
     armDisarm works
     props spin
     ```

2. PX4 with wrong settings can produce a partial AirSim vehicle shell:

     ```text
     listVehicles() returns ['PX4']
     getMultirotorState() returns timestamp
     but HIL/GPS/control path is broken
     ```

3. The real PX4 connection fix was:

     ```text
     UseTcp=true
     TcpPort=4560
     start AirSim first
     wait for TCP 4560
     then start PX4
     ```

4. The AirSim `armDisarm()` crash persisted even after PX4 was healthy.

5. Manual PX4 shell control is currently the only confirmed complete arm/takeoff/land path.

6. MAVLink raw command path can arm but may not cause takeoff motion.

7. AirSim velocity-vector commands are used for trajectory playback after PX4 shell takeoff.

---

## 12. Common Commands

Start clean:

```bash
cd ~/dedalus/simulation
./cleanup.sh --soft --yes
./run.sh
```

Force PX4 rebuild:

```bash
cd ~/dedalus/simulation
./cleanup.sh --px4 --yes
./setup.sh --yes
./run.sh
```

Run default flight:

```bash
cd ~/dedalus/simulation
source ~/dedalus/venv/bin/activate
python test-flight.py
```

Run explicit PX4 shell control:

```bash
python test-flight.py --control px4
```

Run circle + figure-eight:

```bash
python test-flight.py --control px4 --trajectory trajectories/circle_figure8.json
```

Check sockets:

```bash
ss -ltnp | grep 4560
ss -tanp | grep 4560
ss -lunp | grep -E '145|146|4560'
```

Attach PX4 shell:

```bash
tmux attach -t dedalus-sim
```

Send PX4 shell commands from SSH:

```bash
tmux send-keys -t dedalus-sim:px4 'commander status' C-m
```

---

## 13. Style / Contribution Guidance for LLMs

When modifying this repo:

- Prefer targeted patches over large rewrites.
- Keep all setup state inside `setup.sh`.
- Keep all reset behavior inside `cleanup.sh`.
- Keep orchestration inside `run.sh`.
- Avoid one-off shell instructions unless also captured in scripts.
- Do not assume AirSim PX4 `armDisarm()` works.
- Do not silently report flight success unless motion is actually observed.
- Preserve `--control auto` as a safe default.
- Keep trajectory behavior editable through JSON rather than hardcoding mission paths in Python.

When debugging:

- First check whether PX4 and AirSim are connected:

    ```text
    AirSim log: Connected to SITL over TCP.
    PX4 log: Simulator connected on TCP port 4560.
    PX4 log: Ready for takeoff!
    ```

- If AirSim says:

    ```text
    not receiving any messages from HIL
    ```

    then PX4 is not connected to the simulator.

- If PX4 says:

    ```text
    Waiting for simulator to accept connection on TCP port 4560
    ```

    then AirSim did not open TCP `4560`, or PX4 started too early.

- If `client.armDisarm()` throws:

    ```text
    rpclib ... exception is not derived from std::exception
    ```

    do not keep debugging PX4 health. Use `--control px4` and treat it as the known Colosseum/AirSim RPC bridge bug.