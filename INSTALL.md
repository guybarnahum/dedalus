# Project Dedalus Installation Guide (Root)

This guide is intentionally focused on repository-level setup and provisioning.

For simulation runtime operations (starting/stopping the sim, run modes, flight testing), use [simulation/INSTALL.md](simulation/INSTALL.md).

## Scope of This Document

This root install guide covers:
- host and cloud prerequisites
- repository checkout
- environment bootstrap via [simulation/setup.sh](simulation/setup.sh)
- verification that installation completed

This root install guide does not cover:
- day-to-day simulation runtime operations
- trajectory execution workflow
- `run.sh` / `stop.sh` / `cleanup.sh` usage details

## 1. Prerequisites

Required baseline:
- Ubuntu 22.04
- NVIDIA GPU-capable machine (local RTX class or AWS g5/g6)
- outbound internet access for package install and assets
- shell tools: `git`, `tmux`, `curl`, `python3`

AWS recommendations:
- instance: `g6.2xlarge` (preferred) or `g5.2xlarge`
- AMI: Ubuntu 22.04 LTS
- security group: SSH 22, NICE DCV 8443 TCP/UDP
- metadata: IMDSv2 enabled, hop limit 2

## 2. Clone the Repository

```bash
git clone https://github.com/guybarnahum/dedalus.git
cd dedalus
```

## 3. Run Bootstrap Provisioning

Use the simulation bootstrap script to install dependencies and prepare the environment:

```bash
./simulation/setup.sh --yes
```

What this step prepares at a high level:
- NICE DCV session integration
- Python virtual environment under `$HOME/dedalus/venv`
- required Python packages for PX4 and test tooling
- simulation dependency tree and build prerequisites

## 4. Set Login Password (DCV)

If you plan to access the desktop through NICE DCV:

```bash
sudo passwd "$(whoami)"
```

## 5. Install Validation

Validate provisioning without starting runtime workflows:

```bash
# Verify venv exists
ls "$HOME/dedalus/venv/bin/activate"

# Verify DCV session metadata is available
dcv describe-session dedalus-sim --json

# Verify simulation scripts exist
ls simulation/setup.sh simulation/run.sh simulation/stop.sh simulation/cleanup.sh
```

## 6. Next Step

Proceed to [simulation/INSTALL.md](simulation/INSTALL.md) for runtime usage:
- starting the simulator
- stopping the simulator (`stop.sh`)
- reset/rebuild semantics (`cleanup.sh`)
- running `test-flight.py`

## 7. Non-Simulation Build Notes

For cross-compiling runtime code paths:

```bash
docker build -f infrastructure/docker/Dockerfile.l4t_cross -t dedalus:l4t-cross .
docker run --rm -v "$(pwd)":/workspace dedalus:l4t-cross make -C src
```

Related docs:
- [README.md](README.md)
- [WHITEPAPER.md](WHITEPAPER.md)
- [simulation/README.md](simulation/README.md)
