# ☁️ EC2 Simulation Setup Guide

This guide details how to provision and bootstrap an AWS EC2 GPU instance to run the Project Dedalus Virtual Proving Ground.

## Phase 1: AWS Provisioning Requirements

To ensure the `setup.sh` script passes its hardware and licensing gates, you must provision an instance that meets these exact specifications.

### 1. Instance Type
* **Recommended:** `g6.2xlarge` (NVIDIA L4 GPU, 32GB RAM). This is the current project standard for high-fidelity tactical flight simulation.
* **Alternative:** `g5.2xlarge` (NVIDIA A10G).

### 2. AMI (Amazon Machine Image)
* **Standard:** **Ubuntu Server 22.04 LTS (HVM)**
* *Note:* While Deep Learning AMIs are available, our `setup.sh` is designed to provision a clean Ubuntu 22.04 environment with the specific NICE DCV/XDCV versions required for our L4-optimized "Windshield" architecture.

### 3. Metadata and Permissions (Critical)
NICE DCV validates its license via the AWS Instance Metadata Service (IMDS). If this is misconfigured, hardware acceleration will be disabled.
* **IMDSv2:** Required.
* **Hop Limit:** Must be set to **2** (to allow the bridge from the XDCV container to the metadata endpoint).
* **IAM Role:** Attach a role with `S3:GetObject` permissions for the `dedalus-sim-assets-colosseum` bucket.

### 4. Networking (Security Group)
* **SSH (TCP 22):** Restrict to your IP.
* **DCV (TCP & UDP 8443):** Required for the remote visual stream. Both protocols must be open for optimal performance.

---

## Phase 2: Bootstrapping the Environment

Connect via SSH and execute the automated setup. The script handles NVIDIA drivers, X11 configuration, and the DCV session manager.

```bash
# 1. Connect to the instance
ssh -i "your-key.pem" guy@<YOUR_EC2_IP>

# 2. Clone the repository
git clone [https://github.com/guybarnahum/dedalus.git](https://github.com/guybarnahum/dedalus.git)
cd dedalus

# 3. Execute the setup
# This script installs the Windshield (XDCV), builds iceoryx/PX4, and pulls S3 assets.
./simulation/setup.sh
```

### Post-Setup: User Credentials
NICE DCV requires a system password for the remote handshake:
```bash
sudo passwd $(whoami)
```

---

## Phase 3: Connecting the "Windshield"

We use **NICE DCV** to stream the GPU frame buffer. Unlike standard VNC, DCV allows Unreal Engine to render directly on the L4 hardware.

1. **Verify the Session:**
   The setup script creates a persistent session named `dedalus-sim`. Verify it is active:
   ```bash
   dcv list-sessions
   ```
2. **Launch the Client:**
   * Download the [NICE DCV Client](https://download.nice-dcv.com/) for your local OS.
   * Enter `<YOUR_EC2_IP>:8443`.
   * Log in using your Ubuntu username and the password you set above.

---

## Phase 4: Launching the Simulation

The simulation environment is "Visual-Aware." It must be launched into the DCV display context.

### Option A: From the DCV Desktop
Open the terminal emulator inside the DCV window and run:
```bash
cd ~/dedalus/simulation
./run.sh AirSimNH
```

### Option B: From SSH (Headless Handoff)
If you prefer launching via SSH, `run.sh` will dynamically probe the DCV session for the correct `DISPLAY` and `XAUTHORITY` variables:
```bash
# The script handles the X11 bridge automatically
cd ~/dedalus/simulation && ./run.sh AirSimNH
```

---

## Phase 5: Autonomous Flight Testing

Once the simulation environment is running, use **`test-flight.py`** to execute autonomous flight sequences with trajectory playback.

### Running Flight Tests

1. **Basic Test (Default 10-Second Hover):**
   ```bash
   cd ~/dedalus/simulation
   python test-flight.py
   ```
   This executes: arm → hover for 10s → land via PX4 shell.

2. **Custom Trajectory (Orbit & Figure-Eight):**
   ```bash
   python test-flight.py --control px4 --trajectory trajectories/circle_figure8.json
   ```
   Streams velocity commands during the autonomous sequence.

3. **MAVLink Control with Climb Verification:**
   ```bash
   python test-flight.py --control mavlink --mavlink-endpoint 127.0.0.1:14550
   ```
   Uses MAVLink protocol; detects false ACKs if aircraft doesn't gain altitude.

4. **AirSim-Only Control (for debugging):**
   ```bash
   python test-flight.py --control airsim --trajectory my_trajectory.json
   ```

### Trajectory Format

Create custom trajectories in `simulation/trajectories/` as JSON files with segment definitions:

```json
{
  "name": "custom_mission",
  "rate_hz": 10,
  "segments": [
    {
      "type": "hold",
      "duration_s": 3,
      "vx_mps": 0.0,
      "vy_mps": 0.0,
      "vz_mps": 0.0
    },
    {
      "type": "figure8_velocity",
      "duration_s": 15,
      "center_x": 0.0,
      "center_y": 0.0,
      "size": 20.0,
      "altitude": 30.0
    },
    {
      "type": "hold",
      "duration_s": 2,
      "vx_mps": 0.0,
      "vy_mps": 0.0,
      "vz_mps": 0.0
    }
  ]
}
```

**Segment Types:**

| Type | Key Fields | Purpose |
|------|-----------|---------|
| `hold` | `duration_s`, `vx/vy/vz_mps` | Fixed velocity (0,0,0 for hover) |
| `circle_velocity` | `duration_s`, `center_x/y`, `radius`, `altitude`, `clockwise` | Orbital flight |
| `figure8_velocity` | `duration_s`, `center_x/y`, `size`, `altitude` | Figure-eight pattern |
| `velocity_keyframes` | `duration_s`, `keyframes` | Custom velocity waypoints |

### Debugging Tips

**Check PX4 Shell Status:**
```bash
# In a separate tmux pane or DCV terminal
tmux list-sessions
tmux attach-session -t dedalus-sim
# Navigate to the px4 pane and check for errors
```

**Verify AirSim Connection:**
```bash
# Confirm TCP port 4560 is listening
netstat -tuln | grep 4560
```

**Enable Verbose Output:**
Pass `-v` or inspect PX4 terminal directly for arm/takeoff diagnostics.

**Reload Test Flight Script:**
If you modify `test-flight.py`, no restart is required—just re-run:
```bash
python test-flight.py --control px4 --trajectory new_trajectory.json
```

For detailed CLI reference and control mode architecture, see [README.md](README.md).

---

## Phase 6: Lifecycle & Persistence


* **Linger:** `setup.sh` enables `loginctl enable-linger`. The DCV session will start automatically on boot.
* **Stopping:** Always **Stop** the instance via the AWS Console when finished to avoid L4 hourly compute charges.
* **Recovery:** If the DCV terminal fails to open or the session disappears, restart the user-level service:
  ```bash
  systemctl --user restart dcv-session.service
  ```