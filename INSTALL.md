# 🚀 Project Dedalus Installation Guide

This guide covers two main paths: **Simulation** (AWS EC2 virtual proving ground) and **Edge** (NVIDIA Jetson Orin hardware deployment). Most developers start with Simulation.

---

## 🖥️ Simulation Environment (Virtual Proving Ground)

### Quick Summary
Project Dedalus simulation runs on **AWS EC2 GPU instances** with Unreal AirSim physics, PX4 flight controller, and automated flight testing. It's the entry point for all development and testing.

### Setup Path

**Step 1: Provision AWS Instance**
Follow the complete provisioning guide: [simulation/INSTALL.md](simulation/INSTALL.md)

Key requirements:
* Instance: `g6.2xlarge` (NVIDIA L4 GPU, 32GB RAM)
* AMI: Ubuntu 22.04 LTS
* Metadata: IMDSv2 hop limit = 2
* Security Group: SSH (22), DCV (8443 TCP/UDP)

**Step 2: Connect & Bootstrap**
```bash
# SSH into the instance
ssh -i "your-key.pem" guy@<YOUR_EC2_IP>

# Clone repository
git clone https://github.com/guybarnahum/dedalus.git
cd dedalus

# Run master provisioner (30-40 min)
./simulation/setup.sh
```

**Step 3: Set Credentials**
```bash
# NICE DCV requires a system password
sudo passwd $(whoami)
```

**Step 4: Connect NICE DCV Client**
* Download [NICE DCV Client](https://download.nice-dcv.com/)
* Connect to `<YOUR_EC2_IP>:8443`
* Login with your Ubuntu username and password

**Step 5: Launch Simulation**
```bash
cd ~/dedalus/simulation
./run.sh AirSimNH
```
Unreal Engine loads (2-3 min). You'll see a virtual environment with a simulated quadcopter.

---

## 🤖 Flight Testing (test-flight.py)

Once the simulation environment is running, autonomous flight tests are executed via **test-flight.py** with trajectory-based control.

### Basic Test (Default Hover)
```bash
cd ~/dedalus/simulation
python test-flight.py
```
**Sequence:** Arm → Hover 10s → Land (via PX4 shell)

### Test with Custom Trajectory
```bash
python test-flight.py --control px4 --trajectory trajectories/circle_figure8.json
```
**Sequence:** Arm → Circle orbit + figure-8 pattern → Land

### Test with MAVLink + Climb Verification
```bash
python test-flight.py --control mavlink --mavlink-endpoint 127.0.0.1:14550
```
**Sequence:** Arm via MAVLink (verifies altitude gain) → Hover 10s → Land

---

## 📋 Trajectory System

Trajectories are **JSON files** that define multi-second autonomous flight sequences.

### Example: Orbital Flight
```json
{
  "name": "orbit_mission",
  "rate_hz": 10,
  "segments": [
    {
      "type": "hold",
      "label": "hover_pre",
      "duration_s": 3,
      "vx_mps": 0.0,
      "vy_mps": 0.0,
      "vz_mps": 0.0
    },
    {
      "type": "circle_velocity",
      "label": "orbit_right",
      "duration_s": 20,
      "center_x": 0.0,
      "center_y": 0.0,
      "radius": 15.0,
      "altitude": 30.0,
      "clockwise": true
    },
    {
      "type": "hold",
      "label": "hover_post",
      "duration_s": 2,
      "vx_mps": 0.0,
      "vy_mps": 0.0,
      "vz_mps": 0.0
    }
  ]
}
```

### Segment Types
| Type | Key Fields | Example Use |
|------|-----------|-------------|
| `hold` | `duration_s`, `vx/vy/vz_mps` | Hover, pause between maneuvers |
| `circle_velocity` | `duration_s`, `center_x/y`, `radius`, `altitude`, `clockwise` | Orbital reconnaissance |
| `figure8_velocity` | `duration_s`, `center_x/y`, `size`, `altitude` | Evasive weaving patterns |
| `velocity_keyframes` | `duration_s`, `keyframes` (list of `[t, vx, vy, vz]`) | Complex custom patterns |

### Create & Test Custom Trajectory
```bash
# 1. Create your trajectory file
cat > trajectories/my_mission.json << 'EOF'
{
  "name": "my_mission",
  "rate_hz": 10,
  "segments": [
    {
      "type": "hold",
      "duration_s": 5,
      "vx_mps": 0.0,
      "vy_mps": 0.0,
      "vz_mps": 0.0
    }
  ]
}
EOF

# 2. Test it
python test-flight.py --control px4 --trajectory trajectories/my_mission.json
```

The trajectory is **pre-flight validated** at startup—file must exist, be valid JSON, and contain a `segments` array.

---

## 🎮 Control Modes (Multi-Layer Fallback)

```
--control px4        → Use PX4 shell (most reliable)
--control mavlink    → Use MAVLink protocol (with climb verification)
--control airsim     → Use AirSim RPC only (fallback, least reliable)
--control auto       → Try PX4 shell, fall back to MAVLink, then AirSim
```

**Default:** `--control auto` tries PX4 shell first. If unavailable, cascades to MAVLink with climb detection (fails if takeoff ACK but no altitude gain), then AirSim RPC.

---

## 📚 Detailed Documentation

* **Full Simulation Setup:** [simulation/INSTALL.md](simulation/INSTALL.md) — AWS provisioning, DCV client, environment bootstrap
* **Flight Testing Reference:** [simulation/README.md](simulation/README.md) — test-flight.py CLI, trajectory format, control modes, debugging
* **Project Overview:** [README.md](README.md) — Architecture, algorithms, three-tier development model

---

## 🛠️ Troubleshooting

**DCV Client Won't Connect**
```bash
# Restart the user-level DCV session
systemctl --user restart dcv-session.service

# Verify it's active
dcv list-sessions
```

**PX4 Shell Arm Fails**
```bash
# Connect to the PX4 tmux pane
tmux attach-session -t dedalus-sim
# Type: commander arm
# Check for errors (e.g., "Need to be in position mode")
```

**AirSim TCP Port Not Listening**
```bash
# Verify TCP 4560 is open
netstat -tuln | grep 4560

# If not, check settings.json
cat simulation/settings.json | grep -i tcpport
```

**Trajectory Validation Error**
```bash
# Pre-flight checks run at startup
# Error messages indicate: missing file, invalid JSON, or no "segments" key
# Validate JSON: python -c "import json; json.load(open('trajectories/my.json'))"
```

---

## ⚡ Advanced Usage

### Multi-Endpoint MAVLink Fallback
```bash
python test-flight.py --control mavlink \
  --mavlink-endpoint 127.0.0.1:14550 \
  --mavlink-endpoint 127.0.0.1:14540 \
  --mavlink-endpoint 127.0.0.1:14600
```

### Skip Arm Phase (Trajectory Playback Only)
```bash
python test-flight.py --skip-arm --trajectory payload.json
```
Useful for rapid iteration when vehicle is already armed.

### Ignore MAVLink Climb Verification
```bash
python test-flight.py --control mavlink --force-mavlink-arm
```
Bypasses the false-ACK check; use cautiously.

### Custom PX4 Shell Target
```bash
python test-flight.py --control px4 --px4-tmux-target custom-session:px4-pane
```
If your PX4 tmux pane has a different name.

---

## 📦 Edge Deployment (Jetson Orin)

*Cross-compilation and L4T deployment documentation is forthcoming.*

For now, the C++20 runtime is compiled via Docker container on the host machine:
```bash
docker build -f infrastructure/docker/Dockerfile.l4t_cross -t dedalus:l4t-cross .
docker run --rm -v $(pwd):/workspace dedalus:l4t-cross make -C src
```

---

## 📝 Project Structure
```
dedalus/
├── src/                     # C++20 flight runtime
│   ├── behavior/            # State machines & decision logic
│   ├── perception/          # Vision pipeline & 3D reconstruction
│   ├── safety/              # Collision avoidance & watchdogs
│   └── sensors/             # Camera & IMU interfaces
├── simulation/              # Virtual proving ground (AirSim + PX4)
│   ├── test-flight.py       # Autonomous flight test harness
│   ├── trajectories/        # JSON flight sequences
│   ├── setup.sh             # EC2 environment bootstrap
│   ├── run.sh               # Simulation launcher
│   └── INSTALL.md           # Detailed AWS setup guide
├── config/                  # YAML behavior definitions
├── infrastructure/          # Terraform, Docker
└── README.md                # Project overview
```

---

## 🔗 Quick Links

* **AWS EC2 Setup:** [simulation/INSTALL.md](simulation/INSTALL.md)
* **Flight Testing CLI:** [simulation/README.md](simulation/README.md)
* **Project Architecture:** [README.md](README.md)
* **Behavior Configuration:** [config/behaviors.yaml](config/behaviors.yaml)
* **NICE DCV Client:** https://download.nice-dcv.com/

---

## 💡 Next Steps

1. **Provision** an AWS instance following [simulation/INSTALL.md](simulation/INSTALL.md)
2. **Launch** the simulation: `./run.sh AirSimNH`
3. **Test** a basic flight: `python test-flight.py`
4. **Iterate** with custom trajectories and control modes
5. **Deploy** proven logic to physical Jetson Orin

Happy flying! 🚁
