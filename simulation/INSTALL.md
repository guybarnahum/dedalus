# ☁️ EC2 Simulation Setup Guide

This guide details how to provision and bootstrap an AWS EC2 GPU instance to run the Project Dedalus Virtual Proving Ground.

## Phase 1: AWS Provisioning Requirements

To ensure the `setup.sh` script passes its hardware gatekeeper, you must provision an instance that meets these exact specifications.

### 1. Instance Type
* **Recommended:** `g5.2xlarge` (1x NVIDIA A10G, 8 vCPU, 32GB RAM). Provides a locked 60 FPS in Colosseum.
* **Budget/Minimum:** `g4dn.2xlarge` (1x NVIDIA T4, 8 vCPU, 32GB RAM).

### 2. AMI (Amazon Machine Image)
**Do not use bare Ubuntu.** You must use an AMI with NVIDIA drivers pre-installed to prevent kernel mismatches.
* Search for and select: **AWS Deep Learning Base AMI (Ubuntu 22.04)**
* *Why:* This guarantees `nvidia-smi` works out of the box, fulfilling the primary requirement of our setup script.

### 3. Storage (EBS)
* **Volume:** `200 GB gp3`
* *Why:* The Colosseum binaries, PX4 build cache, Docker cross-compilation images, and standard system overhead will easily consume 100GB. 

### 4. Security Group (Firewall)
* **Port 22 (TCP):** SSH access (Restrict to your IP).
* **Port 8443 (TCP/UDP):** AWS NICE DCV remote desktop stream (Restrict to your IP).

---

## Phase 2: Bootstrapping the Environment

Once the instance is running, connect via SSH and execute the automated setup.

```bash
# 1. Connect to the instance
ssh -i "your-key.pem" ubuntu@<YOUR_EC2_IP>

# 2. Clone the repository
git clone [https://github.com/guybarnahum/dedalus.git](https://github.com/guybarnahum/dedalus.git)
cd dedalus/simulation

# 3. Make scripts executable
chmod +x setup.sh cleanup.sh

# 4. Execute the setup
./setup.sh
```
*Note: The script will prompt for your `sudo` password upfront, then run autonomously for 10-20 minutes depending on AWS network speeds.*

---

## Phase 3: Connecting the "Windshield" (Remote Desktop)

Because SSH cannot forward a 30+ FPS Unreal Engine simulation, we use AWS NICE DCV to stream the XFCE4 desktop directly from the GPU frame buffer.

1. **Install the DCV Server (EC2):**
   *(If not already included in your specific Deep Learning AMI)*
   ```bash
   sudo apt-get install -y nice-dcv-server
   sudo systemctl enable dcvserver
   sudo systemctl start dcvserver
   ```
2. **Create a Session (EC2):**
   ```bash
   dcv create-session dedalus-sim
   ```
3. **Connect Locally (Your Laptop):**
   * Download the [NICE DCV Client](https://download.nice-dcv.com/) for your local OS (Mac/Windows/Linux).
   * Open the client, enter your `<YOUR_EC2_IP>:8443`.
   * Log in using your `ubuntu` user credentials. 

You should now see the XFCE4 Linux desktop. You can open a terminal here to launch Colosseum.

---

## Phase 4: Lifecycle Management

Do not leave `g5` instances running idly; they are expensive. 

* **To reset a broken state:** `cd simulation && ./cleanup.sh --soft`
* **To wipe heavy frameworks:** `cd simulation && ./cleanup.sh --hard`
* **When finishing work:** Always stop the instance via the AWS Console to halt the hourly compute charges (you will still pay a nominal fee for the EBS storage).