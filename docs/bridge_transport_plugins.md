# Bridge Transport Plugins

The simulation/provider path separates provider semantics from transport mechanics.

## Provider vs Transport

Providers describe what data is being requested:

```text
FrameSource provider      -> image frames
EgoStateProvider provider -> pose and velocity telemetry
```

Transports describe how bridge data moves into the C++ process:

```text
pipe          -> implemented
shared_memory -> placeholder, explicit not implemented
```

This keeps provider contracts stable while allowing the transport to evolve from simple pipes to shared-memory rings later.

## Config

Current configs default to pipe transport:

```yaml
bridge_transport: pipe
```

Future shared-memory shape:

```yaml
bridge_transport: shared_memory
```

Source connection and vehicle/camera fields are intentionally generic:

```yaml
source_host: 127.0.0.1
source_rpc_port: 41451
vehicle_name: PX4
vehicle_camera_name: front_center
bridge_mode: stream_jsonl
bridge_command: python3 simulation/airsim-stream-frames.py --count 0 --rate-hz 5
ego_bridge_command: python3 simulation/airsim-capture-ego.py
```

Only source-neutral bridge and source keys are accepted by the config loader.

The shared-memory transport exists as a class and config option but intentionally throws `shared_memory bridge transport is not implemented yet` today.

## Implemented Transport: PipeBridgeTransport

`PipeBridgeTransport` supports two access patterns:

```text
request_once(command)        -> run command, read all stdout, close process
read_stream_line(command)    -> start command once, then read one line per call
```

The current frame bridge modes use it as follows:

```text
one_shot_ppm
  -> request_once(simulation/airsim-capture-frame.py)
  -> P6 PPM stdout

stream_jsonl
  -> read_stream_line(simulation/airsim-stream-frames.py)
  -> JSONL frame records with base64 PPM payloads
```

The current ego provider uses `request_once()` with:

```text
simulation/airsim-capture-ego.py
```

## Future Transport: SharedMemoryBridgeTransport

The target shared-memory design should use a ring buffer:

```text
producer bridge process
  -> shared memory segment
      slot N: header + image/telemetry payload
      sequence counter
      valid/complete flag

C++ provider
  -> reads latest complete slot
  -> validates sequence and payload length
  -> returns FramePacket or EgoState
```

Recommended slot metadata:

```text
magic
version
stream_id
sequence
timestamp_ns
width
height
channels
payload_size
payload_type
```

Keep this transport optional and out of default CI until the protocol is stable.

## Rule

Do not put AirSim, OpenCV, GStreamer, camera SDK, or shared-memory implementation assumptions directly into the core provider contracts. Add them behind transport plugins or integration-only providers.
