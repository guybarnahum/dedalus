# Third-Party Dependencies

Generated or downloaded external dependencies are staged here and are intentionally not committed.

```text
third_party/PX4-Autopilot/
  PX4 upstream source checkout used for PX4 SITL.

third_party/iceoryx_build/
  Eclipse iceoryx checkout/build state used by the simulation IPC setup.

third_party/colosseum_environments/
  Downloaded Colosseum/AirSim Unreal environment packages.
```

Dedalus-owned scripts and configuration live outside this directory:

```text
simulation/airsim/
  AirSim target launch config, scripts, and validation.

tools/px4/
  Dedalus PX4/MAVLink bridge utilities.
```

To provision all dependencies from scratch:

```bash
./setup.sh --yes
```
