# Dedalus — Component API Reference

This document maps the runtime architecture, every major component interface, and the data-flow between them. All types live in namespace `dedalus` unless noted.

---

## 1. Runtime Architecture Overview

```
┌─────────────────────────────────────────────────────────┐
│                   dedalus_mission_loop                   │
│                  (main application binary)               │
│                                                          │
│  CoreStackProviderConfig ──► ProviderRegistry            │
│      (YAML via config_loader)     │                      │
│                                   ▼                      │
│                          CoreStackProviders              │
│                                   │                      │
│  CoreStackRunnerConfig ──► CoreStackRunner               │
│                                   │  run_once() →        │
│                    ┌──────────────┼──────────────────┐   │
│                    ▼              ▼                   ▼   │
│             FrameSource    EgoStateProvider    GhostTargetProvider │
│                    │              │                   │   │
│                    └──────────────┘                   │   │
│                          FramePacket + EgoState        │   │
│                                   │                   │   │
│                            PerceptionPipeline          │   │
│                         (Detector→Stabilizer→          │   │
│                          Tracker→Identity→Projector)   │   │
│                                   │                   │   │
│                      PerceptionPipelineOutput          │   │
│                         + GhostDetectionsFrame         │   │
│                                   │                   │   │
│                          InMemoryWorldModel            │   │
│                                   │                   │   │
│                           WorldSnapshot                │   │
│                                   │                   │   │
│          ┌──────────────┬─────────┴──────────┐        │   │
│          ▼              ▼                    ▼        │   │
│  WorldSnapshot   LatestWorldSnapshot  ArtifactSnapshot│   │
│  Publisher       (mission hand-off)   Writer          │   │
│          │                                            │   │
│          ▼                                            │   │
│  RuntimeEventStream                                   │   │
│  Server (TCP JSONL)                                   │   │
│                                                       │   │
│  MissionRuntime (async thread @ tick_hz)              │   │
│      │                                                │   │
│      ├── MissionController::tick()                    │   │
│      │       (ObjectBehaviorMissionController)        │   │
│      ├── FlightCommandSink::send()                    │   │
│      └── CameraPointingSink::send()                   │   │
│                                                       │   │
│  MissionEventPublisher                                │   │
└─────────────────────────────────────────────────────────┘
```

---

## 2. Core Primitive Types (`include/dedalus/core/types.hpp`)

| Type | Description |
|---|---|
| `TimePoint { Nanoseconds timestamp_ns }` | Monotonic nanosecond timestamp |
| `Vec2 { x, y }` | 2-D vector (doubles) |
| `Vec3 { x, y, z }` | 3-D vector (doubles) |
| `Pose3 { position: Vec3, rotation_rpy: Vec3 }` | Position + roll/pitch/yaw in local frame |
| `Rect2 { x, y, width, height }` | 2-D bounding rectangle (pixels) |
| `Bounds3 { min, max: Vec3 }` | 3-D axis-aligned bounding box |
| `Covariance6` | 6×6 covariance matrix (array) |
| `MapFrameId / AgentId / TrackId / ...` | Typed string IDs (prevent mixing) |
| `ClassLabel` | Enum: Unknown, Person, Vehicle, Animal, … |
| `FactionLabel` | Enum: Unknown, Friendly, Hostile, Neutral |
| `LightingMode / WeatherMode / SeasonMode / SensorMode` | Enum appearance condition axes |

---

## 3. Sensor Layer

### 3.1 `FrameSource` (`include/dedalus/sensors/frame_source.hpp`)

The entry-point for all image data.

```cpp
class FrameSource {
    virtual std::optional<FramePacket> next_frame() = 0;
    virtual void request_stop() {}          // cooperative shutdown signal
};
```

**Key data: `FramePacket`**

| Field | Type | Notes |
|---|---|---|
| `frame_id` | `FrameId` | Unique frame identifier |
| `timestamp` | `TimePoint` | Frame capture time |
| `camera_id` | `CameraId` | Source camera |
| `image` | `ImageView { width, height, channels, bytes }` | Raw pixels |
| `intrinsics` | `CameraIntrinsics { fx, fy, cx, cy, k1, k2 }` | Lens model |
| `camera_T_world` | `optional<Pose3>` | Camera pose in world frame |
| `camera_T_body` | `optional<Pose3>` | Camera pose relative to body |
| `ego_hint` | `optional<EgoState>` | Embedded telemetry hint from source |
| `appearance_condition` | `optional<AppearanceCondition>` | Lighting/weather hint |
| `source_timings` | `vector<FrameSourceTiming>` | Per-step latency breakdown |

**Concrete implementations**

| Class | Source | Notes |
|---|---|---|
| `AirSimFrameSource` | AirSim via Python bridge | Supports one-shot PPM, stream JSONL, stream binary |
| `RecordedFrameSource` | Recorded manifest | Replay from disk |
| `SyntheticFrameSource` | In-process | Single synthetic frame for unit tests |
| `SyntheticMissionFrameSource` | In-process | Continuous synthetic frames, stoppable |
| `AsyncPrefetchFrameSource` | Wrapper | One-frame lookahead for I/O-bound sources |

### 3.2 `EgoStateProvider` (`include/dedalus/sensors/ego_state_provider.hpp`)

```cpp
class EgoStateProvider {
    virtual EgoStateEstimate estimate(const FramePacket& frame) = 0;
};

struct EgoStateEstimate {
    optional<EgoState> ego;
    bool telemetry_available;
    float confidence;
};
```

**`EgoState` fields:** `timestamp`, `local_T_body: Pose3`, `velocity_local: Vec3`, `angular_velocity_body: Vec3`, `height_m`, `height_valid`, `armed`, `armed_valid`, `flight_status: EgoFlightStatus`, `home_T_body: optional<Pose3>`, `confidence`.

| Class | Source |
|---|---|
| `FrameHintEgoProvider` | Reads `ego_hint` embedded in `FramePacket` |
| `AirSimEgoStateProvider` | AirSim RPC via Python bridge |
| `NoTelemetryEgoProvider` | Returns empty estimate (video-only mode) |

---

## 4. Perception Pipeline (`include/dedalus/perception/perception_pipeline.hpp`)

Sequential stages — each stage depends on the output of the previous.

```
FramePacket + EgoState
  → Detector          → vector<Detection2D>
  → CameraStabilizer  → StabilizedFrame
  → Tracker           → vector<Track2D>
  → IdentityResolver  → vector<IdentityHypothesis>
  → Projector3D       → vector<Observation3D>
```

**Output: `PerceptionPipelineOutput`**

```cpp
struct PerceptionPipelineOutput {
    vector<Detection2D>         detections;
    StabilizedFrame             stabilized_frame;
    vector<Track2D>             tracks;
    vector<IdentityHypothesis>  identities;
    vector<Observation3D>       observations;
};
```

**Stage interfaces**

| Interface | Input | Output |
|---|---|---|
| `Detector` | `FramePacket` | `vector<Detection2D>` |
| `CameraStabilizer` | `FramePacket + detections` | `StabilizedFrame { frame, detections, dx_px, dy_px, rotation_rad, confidence }` |
| `Tracker` | `vector<Detection2D>` | `vector<Track2D>` |
| `IdentityResolver` | `vector<Track2D>` | `vector<IdentityHypothesis>` |
| `Projector3D` | `tracks + FramePacket + EgoState` | `vector<Observation3D>` |

**`Observation3D`** carries `position_local: Vec3`, `velocity_local: Vec3`, `size_m: Vec3`, `map_frame_id`, `class_label`, `confidence`, `source_track_id`.

**Concrete implementations** (selected)

| Stage | Impl | Notes |
|---|---|---|
| Detector | `ScriptedDetector` | Scripted/fixture detections |
| CameraStabilizer | `NullCameraStabilizer` | Pass-through |
| Tracker | `SimpleCentroidTracker` | Centroid IoU matching |
| IdentityResolver | `AppearanceOnlyIdentityResolver` | Appearance hash matching |
| Projector3D | `FlatGroundProjector` | Flat-ground homography |
| Projector3D | `AirSimDepthProjector` | AirSim depth image RPC |

---

## 5. Ghost Target Provider (`include/dedalus/perception/ghost_targets.hpp`)

Injects synthetic targets (scripted scenarios or AirSim object poses) into the same `Observation3D` pipeline path as real detections.

```cpp
class GhostTargetProvider {
    GhostTargetProvider(GhostScenario scenario);
    GhostTargetProvider(AirSimGhostObjectSourceConfig config);

    GhostDetectionsFrame frame_at(TimePoint, MapFrameId, TimePoint scenario_start) const;
    vector<Observation3D>  observations_at(...) const;
    PerceptionPipelineOutput output_at(...) const;
};
```

**`AirSimGhostObjectSourceConfig`** — queries AirSim object poses via `bridge_command` (Python script), converts them to `GhostDetectionState` entries.

**Object binding types**

| Type | Description |
|---|---|
| `AirSimGhostObjectBinding` | Explicit: `source_track_id` ↔ `airsim_object_name` |
| `AirSimGhostObjectPatternBinding` | Pattern: prefix + regex pattern, up to `max_matches` objects |

**Events published:** `GhostDetectionsFrame` via `GhostDetectionsPublisher` (a `EventPublisher<GhostDetectionsFrame>`).

---

## 6. World Model (`include/dedalus/world_model/`)

### 6.1 `InMemoryWorldModel`

```cpp
class InMemoryWorldModel {
    void update_ego(const EgoState& ego);
    void update_appearance(const AppearanceCondition&);
    void ingest(const PerceptionPipelineOutput&);

    WorldSnapshot snapshot() const;
    EffectiveWorldView effective_view() const;   // actual + uncertain_regions
};
```

### 6.2 `WorldSnapshot` — central autonomy state

Key top-level fields:

| Field | Type | Notes |
|---|---|---|
| `timestamp` | `TimePoint` | Last update time |
| `active_map_frame_id` | `MapFrameId` | Active reference frame |
| `ego` | `EgoState` | Vehicle pose, velocity, arm state |
| `flight_control` | `FlightControlState` | Arm FSM state, last request times |
| `appearance_condition` | `AppearanceCondition` | Lighting/weather/season/sensor |
| `agents` | `vector<AgentState>` | Tracked targets |
| `containers` | `vector<ContainerState>` | Containment regions |
| `containment_events` | `vector<ContainmentEvent>` | Enter/exit events |
| `exclusion_zones` | `vector<ExclusionZone>` | No-fly cones |
| `map_frames` | `vector<MapFrame>` | Known coordinate frames |
| `static_structures` | `vector<StaticStructure>` | Mapped obstacles |
| `landmarks` | `vector<Landmark>` | Reference landmarks |
| `flight_corridors` | `vector<FlightCorridor>` | Cleared flight paths |
| `uncertain_regions` | `vector<UncertainRegion>` | Areas of limited observability |
| `ego_occupancy` | `EgoOccupancyState` | Local occupancy grid |

**Serialization:** `to_json(const WorldSnapshot&)` → pretty JSON; `to_compact_json(string)` → wire-format.

### 6.3 `WorldSnapshotPublisher` / Subscriber

```cpp
using WorldSnapshotPublisher = EventPublisher<WorldSnapshot>;

class WorldSnapshotSubscriber : public EventSubscriber<WorldSnapshot> {
    virtual void on_snapshot(const WorldSnapshot&) = 0;
};
```

**Concrete subscribers**

| Class | Action |
|---|---|
| `LatestWorldSnapshotSubscriber` | Forwards to `LatestWorldSnapshot` handoff |
| `ArtifactSnapshotWriter` | Async background writer to `out/` directory |
| `RuntimeEventStreamServer` | Enqueues for TCP JSONL stream |

---

## 7. Pub/Sub Bus (`include/dedalus/runtime/pubsub.hpp`)

Generic thread-safe, weak-ptr-cleaned event bus.

```cpp
template <typename EventT>
class EventPublisher {
    void subscribe(shared_ptr<EventSubscriber<EventT>> subscriber);
    void publish(const EventT& event);              // broadcasts; cleans expired
    size_t subscriber_count() const;
};

template <typename EventT>
class EventSubscriber {
    virtual void on_event(const EventT& event) = 0;
};
```

**Instantiated for:** `WorldSnapshot`, `GhostDetectionsFrame`, `MissionEvent`.

---

## 8. Core Stack Runner (`include/dedalus/runtime/core_stack_runner.hpp`)

Orchestrates one perception+world-model cycle per call.

```cpp
class CoreStackRunner {
    CoreStackRunner(CoreStackProviders providers, CoreStackRunnerConfig config = {});

    bool run_once();               // false = frame source exhausted / EOF
    WorldSnapshot snapshot() const;
};

struct CoreStackRunnerConfig {
    unique_ptr<PipelineProfiler>          timing_writer;
    shared_ptr<WorldSnapshotPublisher>    snapshot_publisher;
    shared_ptr<GhostDetectionsPublisher>  ghost_detections_publisher;
};
```

**`run_once()` data flow:**

1. `frame_source->next_frame()` → `FramePacket`
2. `ego_provider->estimate(frame)` → `EgoState`
3. `ghost_targets->frame_at(...)` → `GhostDetectionsFrame` (if enabled)
4. `pipeline.process(frame, ego)` → `PerceptionPipelineOutput` (merged with ghost observations)
5. `world_model->update_ego(ego); world_model->ingest(output)`
6. `snapshot_publisher->publish(snapshot)`
7. `ghost_detections_publisher->publish(frame)` (if enabled)
8. `frame_annotator->annotate({frame, perception, snapshot})`

---

## 9. Provider Registry (`include/dedalus/runtime/provider_registry.hpp`)

Factory layer — resolves string names from `CoreStackProviderConfig` to concrete implementations.

```cpp
class ProviderRegistry {
    CoreStackProviders create(const CoreStackProviderConfig& config) const;

    // Query available implementations:
    vector<string> frame_sources() const;
    vector<string> ego_providers() const;
    vector<string> detectors() const;
    vector<string> camera_stabilizers() const;
    vector<string> trackers() const;
    vector<string> identity_resolvers() const;
    vector<string> projectors() const;
    vector<string> world_models() const;
    vector<string> frame_annotators() const;
};
```

**Selected `frame_source` names:** `"synthetic"`, `"synthetic_mission"`, `"airsim"`, `"recorded_frames"`.  
`"airsim"` and `"recorded_frames"` are automatically wrapped in `AsyncPrefetchFrameSource`.

---

## 10. Configuration (`include/dedalus/runtime/config_loader.hpp`)

```cpp
CoreStackProviderConfig load_core_stack_config(const string& path);
```

Parses YAML. Flat key `mission_options.*` entries are applied via `apply_mission_option()` which maps each key to its corresponding typed field on `MissionOptions`. Unrecognized keys produce a stderr warning.

**`CoreStackProviderConfig` key fields (selected)**

| Field | Default | Notes |
|---|---|---|
| `frame_source` | `"synthetic"` | See ProviderRegistry |
| `ego_provider` | `"frame_hint"` | See ProviderRegistry |
| `detector` | `"scripted"` | See ProviderRegistry |
| `tracker` | `"simple_centroid"` | See ProviderRegistry |
| `ghost_targets_enabled` | `false` | |
| `ghost_targets_source` | `"trajectory_scenario"` | `"trajectory_scenario"` or `"airsim_existing_objects"` |
| `bridge_command` | `"python3 simulation/airsim/scripts/airsim-capture-frame.py"` | Validated against shell-safe allowlist |
| `mission_controller` | `"disabled"` | `"disabled"` or `"object_behavior"` |
| `flight_command_sink` | `"disabled"` | `"disabled"`, `"airsim"`, `"px4_bridge"`, `"px4_mavlink"` |
| `mission_tick_hz` | `10.0` | |
| `mission_options` | `MissionOptions{}` | See §11 |

---

## 11. Mission Options (`include/dedalus/behavior/mission_controller.hpp`)

Key-value string map loaded from YAML `mission_options.*` keys. Accessed via `get_or(key, fallback)`.

**Selected keys**

| Key | Consumer | Notes |
|---|---|---|
| `behavior_spec_path` | `ObjectBehaviorMissionController` | Path to behavior YAML |
| `flight_safe_height_m` | Mission controller + `px4_mavlink` sink | Takeoff/transit altitude |
| `flight_takeoff_height_m` | Mission controller | Takeoff target altitude |
| `flight_max_velocity_mps` | All flight sinks | Velocity cap |
| `flight_command_sink` | `dedalus_mission_loop` | Sink selection |
| `flight_px4_command_bridge` | `px4_bridge` sink | Command to launch `px4-command-bridge.py` |
| `flight_velocity_command_bridge` | `airsim` sink | Command to launch `airsim-send-velocity.py` |
| `flight_mavlink_command_endpoints` | `px4_mavlink` sink | MAVLink endpoint string |
| `object_behavior_*` | `ObjectBehaviorMissionController` | Follow/circle/yaw/camera tuning |

---

## 12. Mission Runtime (`include/dedalus/behavior/mission_runtime.hpp`)

Runs `MissionController::tick()` on a dedicated thread at a fixed rate with deadline scheduling.

```cpp
class MissionRuntime {
    MissionRuntime(
        MissionRuntimeConfig config,
        shared_ptr<LatestWorldSnapshot> snapshots,
        unique_ptr<MissionController> controller,
        unique_ptr<FlightCommandSink> sink,
        shared_ptr<MissionEventPublisher> mission_event_publisher = nullptr,
        unique_ptr<CameraPointingSink> camera_pointing_sink = nullptr);

    void start();
    void stop();
    void request_finish();
    bool tick_once();               // single-step (test/CI use)

    bool running() const;
    bool finish_requested() const;
    bool terminal_settled() const;
    size_t tick_count() const;
    MissionLifecycleState last_state() const;
};

struct MissionRuntimeConfig {
    double tick_hz{10.0};
    int verbosity{0};
    string event_log_path{};
};
```

**Thread model:** `loop()` runs on `thread_`; `tick_once()` and state accessors are called from outside. `running_`, `finish_requested_`, `terminal_settled_` are `atomic<bool>`. `event_log_` is guarded by `event_log_mutex_`.

**`MissionEvent`**

```cpp
struct MissionEvent {
    TimePoint timestamp;
    string json;            // compact JSON with "event" key
};
using MissionEventPublisher = EventPublisher<MissionEvent>;
```

---

## 13. Mission Controller (`include/dedalus/behavior/mission_controller.hpp`)

```cpp
class MissionController {
    virtual MissionTickOutput tick(const MissionTickInput& input) = 0;
};

struct MissionTickInput {
    TimePoint now;
    WorldSnapshot snapshot;
    optional<FlightCommandResult> last_command_result;
    optional<CameraPointingResult> last_camera_pointing_result;
    bool finish_requested{false};
};

struct MissionTickOutput {
    MissionLifecycleState state{MissionLifecycleState::Idle};
    optional<VelocityCommand> command;
    optional<CameraPointingCommand> camera_pointing;
    string status;
    vector<string> events;      // compact JSON field strings, wrapped by MissionRuntime
};
```

**`MissionLifecycleState`** enum: `Idle → Prepare → Takeoff → ExecuteMission → GoHome → Land → Complete / Aborted`.

**Concrete controllers**

| Class | Description |
|---|---|
| `ObjectBehaviorMissionController` | Full behavior spec interpreter (approach, circle, sequence) |
| `TrajectoryMissionController` | Waypoint trajectory follower |

---

## 14. `ObjectBehaviorMissionController` (`include/dedalus/behavior/object_behavior_mission_controller.hpp`)

Loads a `BehaviorMissionSpec` (YAML) and executes behavior phases against a `TargetSelector`-chosen agent.

**Key config fields (`ObjectBehaviorMissionConfig`)**

| Field | Default | Notes |
|---|---|---|
| `behavior_spec` | — | Parsed `BehaviorMissionSpec` |
| `takeoff_height_m` | `8.0` | |
| `yaw_mode` | `Trajectory` | `Trajectory / Target / Hold / None` |
| `altitude_policy` | `TargetRelative` | `TargetRelative / SafeHeightFloor` |
| `follow_min_standoff_m` | `8.0` | Approach completion radius |
| `follow_arrival_kp` | `0.35` | Closing velocity proportional gain |
| `camera_pointing_*_mode` | `"neutral" / "home" / "landing_area"` | Per-lifecycle-state camera mode |
| `zero_target_velocity` | `false` | Zero velocity-matching for static objects |

**Control law (circle/orbit):**

```
desired_velocity = target_velocity
                 + tangent_velocity_at_current_radial_angle
                 + radial_correction_velocity
                 + altitude_correction_velocity
```

---

## 15. Target Selector (`include/dedalus/behavior/target_selector.hpp`)

```cpp
class TargetSelector {
    TargetSelection select(
        const WorldSnapshot& snapshot,
        const TargetSelectorSpec& spec,
        const optional<TargetSelection>& previous = nullopt) const;
};

struct TargetSelectorSpec {
    string class_label;
    string track_id;
    string agent_id;
    double confidence_min{0.5};
    TargetSelectionPolicy policy;   // HighestConfidence / Nearest / PersistentTrack
    double reacquire_timeout_s{5.0};
};

struct TargetSelection {
    bool selected;
    TargetSelectionStatus status;   // Selected/Reacquiring/Lost/NoCandidates/InvalidSpec
    AgentId agent_id;
    TrackId source_track_id;
    Vec3 position_local;
    Vec3 velocity_local;
    double target_age_s;
    string reason;
};
```

---

## 16. Behavior Spec (`include/dedalus/behavior/behavior_spec.hpp`)

YAML-loaded mission specification.

```cpp
struct BehaviorMissionSpec {
    TargetSelectorSpec target;
    BehaviorSpec behavior;
    CompletionSpec completion;
    FallbackSpec fallback;
};

struct BehaviorSpec {
    BehaviorType type;          // Hold/Follow/Approach/Circle/GoHome/Land/Sequence/…
    double orbit_radius_m;
    int orbit_count;
    double duration_s;
    double stop_distance_m;
    CircleDirection direction;
    CompletionAction on_complete;
    vector<BehaviorSpec> steps; // non-empty when type == Sequence
    string yaw_mode;            // per-step override
    string camera_pointing_mode;// per-step override
    // …
};
```

---

## 17. Flight Command Sinks (`include/dedalus/behavior/flight_command_sinks.hpp`)

```cpp
class FlightCommandSink {
    virtual FlightCommandResult send(const VelocityCommand& command) = 0;
};

struct VelocityCommand {
    FlightCommandKind kind;     // Velocity / Arm / Takeoff / Land / Disarm
    TimePoint timestamp;
    Vec3 velocity_local_mps;
    double yaw_rate_radps;
    double yaw_rad;
    bool yaw_valid;
    string yaw_source;
};

struct FlightCommandResult {
    FlightCommandKind kind;
    bool success;
    string status;
};
```

**Concrete sinks**

| Class | Transport | Notes |
|---|---|---|
| `AirSimVelocityCommandSink` | `BridgeTransport` (pipe) | Calls `airsim-send-velocity.py` |
| `Px4BridgeCommandSink` | fork/exec persistent process | Spawns `px4-command-bridge.py`; JSON protocol |
| `Px4MavlinkCommandSink` | Native C++ MAVLink | Direct pymavlink-free MAVLink sockets |
| `NullFlightCommandSink` | — | No-op |

---

## 18. Camera Pointing Sink

```cpp
class CameraPointingSink {
    virtual CameraPointingResult send(const CameraPointingCommand& command) = 0;
};

struct CameraPointingCommand {
    TimePoint timestamp;
    vector<string> cameras;
    string mode;            // "target" / "home" / "landing_area" / "neutral"
    string source_track_id;
    double pitch_rad;
    double yaw_rad;
};
```

| Class | Notes |
|---|---|
| `NullCameraPointingSink` | Default; no-op |
| `MavlinkGimbalPointingSink` | Native C++ MAVLink Gimbal Manager commands |

AirSim camera pointing is handled as a side-channel via `mission_event camera_pointing_intent` consumed by `airsim-camera-pointing-bridge.py`.

---

## 19. Bridge Transport (`include/dedalus/simulation/bridge_transport.hpp`)

Adapter for launching Python helper scripts from C++.

```cpp
class BridgeTransport {
    virtual string request_once(const string& command) = 0;               // one-shot popen
    virtual optional<string> read_stream_line(const string& command) = 0; // persistent stream
    virtual optional<vector<uint8_t>> read_stream_byte_vector(
        const string& command, size_t byte_count) = 0;
    virtual void close_stream() = 0;
};
```

**Validation:** `command` strings are validated against a shell-safe character allowlist `[a-zA-Z0-9 /._-:]` before being passed to `popen()`.

| Class | Notes |
|---|---|
| `PipeBridgeTransport` | `popen()` / persistent `FILE*` pipe |
| `SharedMemoryBridgeTransport` | Stub — not yet implemented |

---

## 20. Runtime Event Stream Server (`include/dedalus/runtime/world_snapshot_stream_server.hpp`)

TCP server; each connected client receives a newline-delimited JSON stream.

```cpp
class RuntimeEventStreamServer
    : public WorldSnapshotSubscriber
    , public GhostDetectionsSubscriber
    , public MissionEventSubscriber {

    RuntimeEventStreamServer(RuntimeEventStreamServerConfig config);
    void start();
    void stop();

    uint16_t port() const;
    RuntimeEventStreamServerStats stats() const;
};

struct RuntimeEventStreamServerConfig {
    string bind_host{"127.0.0.1"};
    uint16_t port{0};       // 0 = OS-assigned
    int listen_backlog{8};
};
```

**Stream line types**

| `"type"` value | Wraps |
|---|---|
| `"world_snapshot"` | `to_compact_json(to_json(WorldSnapshot))` |
| `"ghost_detections"` | `to_compact_json(to_json(GhostDetectionsFrame))` |
| `"mission_event"` | `to_compact_json(event.json)` |

---

## 21. JSON Utilities (`include/dedalus/core/json_utils.hpp`)

Inline header-only utilities for hand-built JSON.

```cpp
string json_escape(const string& value);           // backslash-escape special chars
string q(const string& value);                     // json_escape + surrounding quotes
string to_compact_json(const string& pretty_json); // strip insignificant whitespace

class JsonFields {
    JsonFields& kv(const char* key, const string& val);
    JsonFields& kv(const char* key, double val);
    JsonFields& kv(const char* key, int val);
    JsonFields& kv(const char* key, size_t val);
    JsonFields& kv(const char* key, bool val);
    string str() const;     // ",\"key\":value,..." fragment
};
```

---

## 22. LatestWorldSnapshot (`include/dedalus/behavior/latest_world_snapshot.hpp`)

Thread-safe handoff between the perception thread (writer) and mission thread (reader).

```cpp
class LatestWorldSnapshot {
    void publish(WorldSnapshot snapshot);           // called by CoreStackRunner
    void mark_command_dispatched(FlightCommandKind, TimePoint, string status);
    void mark_command_failed(FlightCommandKind, TimePoint, string status);
    shared_ptr<const WorldSnapshot> latest() const;
};
```

`publish()` carries forward the previous `flight_control` state and applies ego arm confirmation heuristics. Commands dispatched from `MissionRuntime` write-back into this object so the controller sees up-to-date `FlightControlState` on its next tick.

---

## 23. Binary Applications

| Binary | Entry | Role |
|---|---|---|
| `dedalus_core_stack` | `apps/dedalus_core_stack.cpp` | Headless perception loop; prints final `WorldSnapshot` JSON |
| `dedalus_mission_loop` | `apps/dedalus_mission_loop.cpp` | Full mission pipeline: perception + world model + mission runtime |
| `dedalus_dump_world` | `apps/dedalus_dump_world.cpp` | Single-frame world model dump (human-readable) |
| `dedalus_replay_recording` | `apps/dedalus_replay_recording.cpp` | Replay a recorded frame manifest |
| `dedalus_ghost_scenario_eval` | `apps/dedalus_ghost_scenario_eval.cpp` | Offline ghost scenario evaluation |

---

## 24. Thread Model Summary

| Thread | Owner | Data produced | Data consumed |
|---|---|---|---|
| Main / perception | `CoreStackRunner::run_once()` (sync loop) | `WorldSnapshot`, `GhostDetectionsFrame` | `FramePacket`, `EgoState` |
| Writer (async) | `ArtifactSnapshotWriter::writer_loop()` | JSON files on disk | `WorldSnapshot` via queue |
| Mission | `MissionRuntime::loop()` | `VelocityCommand`, `CameraPointingCommand`, `MissionEvent` | `WorldSnapshot` via `LatestWorldSnapshot` |
| TCP accept | `RuntimeEventStreamServer::accept_loop()` | client fd list | listen socket |
| TCP writer | `RuntimeEventStreamServer::writer_loop()` | bytes on wire | JSONL send_queue |

**Cross-thread boundaries:**

- `LatestWorldSnapshot` — mutex-protected `shared_ptr<const WorldSnapshot>`
- `EventPublisher<T>` — mutex-protected weak-ptr subscriber list; dispatches inline on caller's thread
- `ArtifactSnapshotWriter` — mutex + condition_variable queue (depth-capped at 64)
- `RuntimeEventStreamServer` send queue — mutex + condition_variable

---

## 25. Architectural Issues (Work-in-Progress)

The issues below are ordered roughly by impact and grouped by concern. All are expected follow-up work, not blockers for the current CI baseline.

### Data-Typing and Schema

**1. `MissionOptions` values are untyped strings parsed ad-hoc** ✅ **FIXED**  
Every consumer called `get_or(key, fallback)` and then `std::stod` / `std::stoi` inline. There was no central schema, no type validation at load time, and no IDE-visible contract for what keys exist and what their ranges are. A typo in a YAML value silently fell back or threw at runtime.  
*Resolution:* `MissionOptions` is now a typed struct with 90+ named fields and in-struct defaults. Parsing happens once in `config_loader.cpp::apply_mission_option()` with range validation. All consumers (`object_behavior_mission_controller.cpp`, `trajectory_mission_controller.cpp`, `dedalus_mission_loop.cpp`) use direct struct member access. The `values` map and `get_or()`/`known_keys()` methods have been removed.

**2. `MissionEvent::json` is an untyped raw JSON string** ✅ **FIXED**  
`MissionTickOutput::events` is a `vector<string>` where each element is a pre-serialized compact JSON fragment. There is no typed `MissionEvent` variant enum or schema — the only contract is the informal `"event"` key. Consumers must parse JSON to branch on event type.  
*Resolution:* `ControllerEventKind` enum and `ControllerEvent{kind, json_fields}` struct added to `mission_controller.hpp`. `MissionTickOutput::events` is now `vector<ControllerEvent>`. The `"event"` key is extracted from each payload string and typed as an enum value; `MissionRuntime` serializes it by prepending `"event":q(to_string(event.kind))` at write time. JSON output is byte-identical to before. Consumers can branch on `event.kind` without parsing JSON.

**3. `AgentState` identity fields are not strongly typed** ✅ **FIXED**  
`AgentState::identity_id` and `AgentState::class_label` are carried as raw strings in some code paths. Mixing `ClassLabel` enum and string in different layers creates silent mismatch risk.  
*Resolution:* `TargetSelectorSpec::class_label`, `AirSimGhostObjectBinding::class_label`, and `AirSimGhostObjectPatternBinding::class_label` changed from `std::string` to `ClassLabel`. Added `inline class_label_from_string()` to `include/dedalus/core/types.hpp` (includes "vehicle" and "wire"/"unknown_obstacle" aliases), called at parse time in `behavior_spec.cpp`, `config_loader.cpp`. Removed duplicate `class_label_from_string` from `ghost_scenario.hpp`/`.cpp` and `parse_class_label` from `target_selector.cpp`.

---

### State Ownership and Coupling

**4. `LatestWorldSnapshot` is a write-back accumulator, not a pure handoff** ✅ **FIXED**  
`publish()` carries forward the previous snapshot's `flight_control` state and applies arm-confirmation heuristics. Separately, `MissionRuntime` calls `mark_command_dispatched()` / `mark_command_failed()` to write back into it. This means a single class plays two roles: publisher handoff and stateful flight-control FSM accumulator. Bugs in either path are hard to isolate.  
*Resolution:* `LatestWorldSnapshot` is now a pure SPSC snapshot holder (`publish` + `latest` only). Flight control state tracking — arm-state transitions and ego confirmation — is extracted into `FlightControlStateTracker` (`include/dedalus/behavior/flight_control_state_tracker.hpp`), owned and mutated exclusively on the `MissionRuntime` tick thread. `tick_once()` calls `flight_control_tracker_.apply_to_snapshot()` before handing the snapshot to the controller; `dispatch_command()` calls `on_command_dispatched/failed` directly instead of writing back through the snapshot.

**5. `EffectiveWorldView::memory` and `EffectiveWorldView::conflicts` are permanently empty stubs** ✅ **FIXED**  
`InMemoryWorldModel::effective_view()` returns an `EffectiveWorldView` whose `memory` and `conflicts` fields are never populated. Any code that branches on them silently sees empty collections. This is known and intentional for future WorldView expansion — but the fields should either be documented as stubs or gated behind a feature flag.  
*Resolution:* Both fields are now annotated with inline `// stub:` comments in `effective_world_view.hpp`. The redundant `view.memory.confidence = 0.0F` assignment (already the zero-initialized default) was removed from `effective_view()` and replaced with an explicit comment pointing at the header annotations. No behavioral change; callers that iterate these collections are unaffected.

**6. `CoreStackRunner` wires subscribers manually in the application binary** ✅ **FIXED**  
Each binary (e.g., `dedalus_core_stack.cpp`, `dedalus_mission_loop.cpp`) manually constructs `LatestWorldSnapshotSubscriber`, `ArtifactSnapshotWriter`, and `RuntimeEventStreamServer`, then subscribes them. There is no declarative subscriber registration in the config — adding a new subscriber requires editing every binary.  
*Resolution:* Added `snapshot_subscribers` (`std::vector<std::shared_ptr<WorldSnapshotSubscriber>>`) to `CoreStackRunnerConfig`. `CoreStackRunner` accepts subscribers at construction, subscribes them to `snapshot_publisher_`, and retains their `shared_ptr`s (the publisher holds weak refs so the runner must keep them alive). If `snapshot_subscribers` is non-empty and no publisher was provided, the runner creates one. `dedalus_mission_loop.cpp` now passes `{latest_snapshot_subscriber, artifact_snapshot_writer}` declaratively in the config; the conditional `RuntimeEventStreamServer` subscription (guarded by a port flag) remains a single explicit call. To add a new always-on subscriber, no binary edits to subscribe logic are needed — it goes in the config initializer list.

---

### Interface and Transport

**7. `BridgeTransport` mixes one-shot and streaming modes in one interface** ✅ **FIXED**  
`request_once()`, `read_stream_line()`, and `read_stream_byte_vector()` are all on the same abstract class. Concrete callers (`AirSimFrameSource`, `AirSimEgoStateProvider`, `AirSimDepthProjector`) each use only one or two of these modes. The implicit contract for "which mode is active" after `read_stream_line()` is never set vs. one-shot calls is not documented or enforced.  
*Resolution:* Introduced `OneShotTransport` (`request_once` only) and `StreamTransport` (`read_stream_line`, `read_stream_bytes`, `read_stream_byte_vector`, `close_stream` only) as separate abstract classes. `BridgeTransport` inherits both and serves as the combined interface for callers that genuinely need both modes. `PipeBridgeTransport` and `SharedMemoryBridgeTransport` are unchanged (still implement `BridgeTransport`). `AirSimEgoStateProvider` and `AirSimVelocityCommandSink` now hold `unique_ptr<OneShotTransport>` — their constructors accept the narrower type. `AirSimFrameSource` retains `unique_ptr<BridgeTransport>` since it selects one-shot or streaming at runtime. `FakeTransport` in the velocity-sink test is simplified to only implement `OneShotTransport`, dropping the five no-op streaming stubs.

**8. `SharedMemoryBridgeTransport` is an unimplemented stub** ✅ **FIXED**  
All five virtual methods throw or are undefined. The class is registered as a valid transport choice in config. Any attempt to use it will crash at runtime with no useful error message.  
*Resolution:* The class is retained as a planned stub (annotated in the header). The error is promoted to construction time: both `make_transport()` factories (`airsim_providers.cpp`, `ghost_targets.cpp`) now throw `"shared_memory bridge transport is not yet implemented; use bridge_transport: pipe"` immediately when `bridge_transport: shared_memory` is read from config, rather than silently constructing the class and crashing on the first method call.

**9. `AirSimVelocityCommandSink` detects success by `"OK"` substring match in stdout** ✅ **FIXED**  
The Python bridge script writes a success indicator to stdout and the C++ sink checks `response.find("OK") != npos`. This is a fragile text protocol — any debug print from the script that contains "OK" would be a false positive.  
*Suggested fix:* Adopt the same structured JSON response convention used by `Px4BridgeCommandSink` (which parses a `"status"` field).

*Resolution:* `airsim-send-velocity.py` now imports `json` and emits `json.dumps({"ok": True, ...})` for every success path (arm, takeoff, disarm, velocity). `AirSimVelocityCommandSink::send()` uses a private `json_ok_value()` helper (parsing `"ok":true`) matching the pattern in `Px4BridgeCommandSink`, replacing the `output.find("OK")` substring check. The unit-test `FakeTransport` responses were updated from plain-text `"OK sent\n"` / `"ERROR nope\n"` to `{"ok":true,...}` / `{"ok":false,"error":"nope"}`. 34/34 tests pass.

**10. `Px4MavlinkCommandSink` calls `std::system()` for tmux send-keys** ✅ **FIXED**  
Even with the `validate_tmux_target()` allowlist guard, `std::system()` spawns a full shell. Any future loosening of the allowlist reintroduces shell injection. A `fork`+`exec` (or `posix_spawn`) approach with a pre-split `argv` array would eliminate the shell entirely.

*Resolution:* `run_px4_shell()` in `px4_mavlink_command_sink.cpp` now uses `fork()`+`execvp()` with a fixed `argv` array: `{"tmux", "send-keys", "-t", target, command, "C-m", nullptr}`. No shell is invoked; the target and command are passed as separate `execvp` arguments so injection is structurally impossible regardless of their content. The `shell_quote()` helper and the `rendered` command string are removed. `<sys/wait.h>` added for `waitpid`. 34/34 tests pass.

---

### Pipeline Design

**11. `PerceptionPipeline` holds non-owning references to all five stages** ✅ **FIXED**  
Stages (`Detector`, `Tracker`, etc.) are passed by reference and owned externally (by `CoreStackProviders`). The lifetime coupling is implicit — if any stage is destroyed while the pipeline is live, the result is undefined behavior. There are no assertions or lifetime annotations.  
*Suggested fix:* `PerceptionPipeline` should hold `shared_ptr` to each stage, or stages should be owned exclusively inside the pipeline.

*Resolution:* `PerceptionPipeline` constructor and private members changed from raw references (`T&`) to `std::shared_ptr<T>` for all five stages. The five pipeline-stage members in `CoreStackProviders` changed from `unique_ptr<T>` to `shared_ptr<T>` (the `resolve()` factory still returns `unique_ptr`, which implicitly converts to `shared_ptr` on assignment). `core_stack_runner.cpp` passes the `shared_ptr` members directly. Both test files updated to construct stages via `make_shared`. 34/34 tests pass.

**12. `GhostTargetProvider` has two structurally different backends with no common config base** ✅ **FIXED**  
Construction from `GhostScenario` and from `AirSimGhostObjectSourceConfig` results in a pimpl that internally branches on which variant is active. The two backends have different config schemas, different initialization paths, and share only the output type. This makes it hard to add a third backend cleanly.  
*Suggested fix:* Define a `GhostTargetBackend` abstract interface and use a factory keyed by the same `ghost_targets_source` string used in `CoreStackProviderConfig`.

*Resolution:* `ghost_targets.cpp` now defines an abstract `GhostTargetBackend` class with a single pure-virtual `detections_at(timestamp, scenario_elapsed_s)` method. `TrajectoryScenarioBackend` and `AirSimObjectsBackend` are concrete implementations in the anonymous namespace. `GhostTargetProvider::Impl` reduces to a single `unique_ptr<GhostTargetBackend>` member, eliminating the `SourceType` enum and the branching `if` in `frame_at()`. Frame assembly (timestamp, map_frame_id, observations) remains in `GhostTargetProvider::frame_at()`. The public API and header are unchanged. 34/34 tests pass.

**13. No per-detection depth information flows from detector to projector** ✅ **FIXED**  
`Detector` produces only 2-D `Detection2D` (bounding boxes). The `Projector3D` stage must independently acquire depth (via AirSim RPC or flat-ground assumption) because depth is not part of `Detection2D`. This means every non-flat-ground projector makes its own bridge call, bypassing any depth data the detector might already have access to.

*Resolution:* Added `std::optional<double> depth_m` to both `Detection2D` and `Track2D` (non-breaking; defaults to empty). `SimpleCentroidTracker` copies `depth_m` from each source detection to the corresponding track. `FlatGroundProjector` uses `track.depth_m.value_or(kFlatGroundDepthM)` instead of the hardcoded 18.0 m constant, so detectors that populate depth (e.g. a future `AirSimGroundTruthDetector` implementation) automatically benefit without a separate RPC call. Existing behaviour is fully preserved when depth is absent. 34/34 tests pass.

---

### Reliability and Observability

**14. Exceptions in `MissionRuntime::loop()` are silently swallowed**  ✅ **FIXED**  
If `tick_once()` throws, the default `while (running_.load())` loop terminates silently. No error is propagated to the main thread, no `MissionEvent` is emitted, and external observers see `running()` flip to false without a reason.  
*Suggested fix:* Catch exceptions in the loop body, emit a terminal `MissionEvent` with the error, and surface the exception via `std::exception_ptr` accessible from the main thread.  
*Resolution:* `loop()` now wraps `tick_once()` in a try/catch. On any thrown exception the exception is stored via `std::current_exception()` into `loop_exception_`, a `runtime_error` `MissionEvent` is emitted with the error text, `running_` is set to false, and the loop exits cleanly. `MissionRuntime::rethrow_if_exception()` (new public method) lets the main thread retrieve and rethrow the stored exception after `stop()` returns.

**15. `PipelineProfiler` has no mechanism to flush or export timing data at shutdown**  ✅ **FIXED**  
Timing rows are written incrementally; if the process is killed (e.g., Ctrl-C in CI) before normal shutdown, the trailing rows may be lost. There is no signal handler or RAII flush.  
*Resolution:* Added an explicit destructor to `PipelineProfiler` that calls `end_frame()` (wrapped in try/catch to avoid throwing from a destructor) whenever a frame was started but never explicitly ended. This guarantees that any in-progress frame is committed to disk on normal object destruction. A new test case in `test_pipeline_profiler` verifies the behaviour: `begin_frame()` + `record_stage()` without `end_frame()` — destructor must flush the frame and the expected tokens must appear in the output. Also fixed a pre-existing regression in `validate_bridge_command`: validation was being called on the fully-assembled command (which includes code-appended single-quoted arguments from `shell_quote()`), causing `airsim_provider_boundary` to fail. The validator (`validate_bridge_base_command`) is now a named function in the `dedalus` namespace, called on the config-supplied base command only inside each `build_*_command` helper; the transport-layer methods no longer re-validate.

**16. TCP clients that fall behind are silently disconnected; no backpressure signaling**  ✅ **FIXED**  
`RuntimeEventStreamServer` drops the write queue when full (or on write error) and disconnects the client. There is no EAGAIN-based flow-control, no per-client queue, and no backpressure signal to the perception thread.  
*Resolution:* Three coordinated changes: (1) `RuntimeEventStreamServerConfig` gains `max_send_queue_depth` (default 256) and `max_client_pending_depth` (default 16). (2) The shared `send_queue_` is now bounded — `enqueue_line()` (new private helper, called by all three event handlers) drops the oldest message and increments `dropped_messages_` when the queue is full, preventing unbounded memory growth under slow consumers. (3) `publish_json_line()` now drains each client's per-client back-buffer (`client_pending_`, an `unordered_map<int, deque<string>>` owned by the writer thread) before sending the new line; on EAGAIN the new line is queued in the per-client buffer rather than immediately dropping the client; the client is only dropped when its buffer overflows `max_client_pending_depth`. `RuntimeEventStreamServerStats` gains `dropped_messages`. A new `bounded_queue_drops_oldest_on_overflow` test case verifies the drop counter.

**17. `ArtifactSnapshotWriter` queue depth is hardcoded at 64**  ✅ **FIXED**  
`kMaxQueueDepth = 64` is a compile-time constant with no config override. Under slow disk or high frame-rate conditions the writer silently drops frames once the queue is full. There is no dropped-frame counter exposed via the public API.  
*Resolution:* `kMaxQueueDepth` removed and replaced with `ArtifactSnapshotWriterConfig::max_queue_depth` (default 64). The constructor now takes `ArtifactSnapshotWriterConfig` and uses `config_.max_queue_depth` as the queue cap. `dropped_frames_` (atomic int) is incremented each time a frame is evicted from the full queue, and exposed via `dropped_frames()` on the public API. A new `artifact_writer_dropped_frames_counter` test verifies the counter becomes positive after a burst that exceeds the configured depth.

---

### Configuration and Testing

**18. Config validation happens only at construction time, not at load time**  
`load_core_stack_config()` parses YAML into `CoreStackProviderConfig` but does not validate that the named providers (`frame_source`, `detector`, etc.) exist in the registry, or that required `mission_options` keys are present. Invalid names produce runtime errors deep inside `ProviderRegistry::create()`.  
*Suggested fix:* Add a `validate(CoreStackProviderConfig, const ProviderRegistry&)` free function callable from test fixtures and from `load_core_stack_config()`.

**19. No integration test exercises `MissionRuntime` with a real `FlightCommandSink`**  
The CTest suite uses `NullFlightCommandSink` in all mission tests. There are no tests that verify the JSON protocol between `Px4BridgeCommandSink` and `px4-command-bridge.py`, or that `AirSimVelocityCommandSink` parses the response correctly.

**20. `dedalus_ghost_scenario_eval` binary has no CI coverage**  
The ghost scenario evaluator binary is built but not exercised by any CTest entry. Ghost scenario correctness is validated only indirectly through mission smoke tests.
