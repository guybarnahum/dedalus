# Dedalus Current Handoff

You are continuing work on the Dedalus repo.

Repository:
  guybarnahum/dedalus

Current commit:
  run `git log --oneline -1` and use the current local SHA.

First read:
  LLM.md

Then read:
  docs/runtime_dataflow.md
  docs/mission_local_obstacle_mapping_architecture.md
  docs/dynamic_mission_flight_path_map_architecture.md
  docs/mission_map_data_representations_and_retention.md
  docs/persistent_obstacle_memory_plan.md
  docs/obstacle_memory_plugin_architecture.md
  docs/airsim_depth_obstacle_detector_validation.md
  docs/sensing_coverage_architecture.md
  docs/visual_obstacle_detection_transition_plan.md
  docs/classless_geometric_occupancy_and_avoidance_plan.md
  docs/reflexive_obstacle_avoidance_architecture.md
  docs/geometric_volume_detection_and_spatial_mapping_plan.md

Historical context, only if needed:
  LLM.back.md

Active development state:
  R3 live mission-local obstacle viewer is operator-validated.
  S4A/S4B mission-derived foundational traversability artifact and viewer are operator-validated.
  Normal AirSim `run_mission.sh --merge-obstacle-map` now emits:
    - out/<mission>/mission_traversability_map_full.json
    - out/<mission>/mission_traversability_map_full.json.meta.json
    by deriving traversability artifact defaults from the existing obstacle-map env path.
  5Q-5U obstacle-memory default path is compact-delta-first and SQLite-backed.
  4.3A-D obstacle diagnostics hardening is complete:
    - MissionLocalObstacleMap owns provider-neutral same-update compaction/fusion.
    - AirSim detector-side coalescing logic/flags are removed and should not be reintroduced.
    - LocalFlightMap exposes mission-local projection and inflated exclusion diagnostics.
    - WorldSnapshot serializes those counters.
    - The existing mission-local obstacle viewer displays those diagnostics.
  4.3F viewer polish is operator-validated:
    - mission/live obstacle evidence uses stable height/topo coloring,
    - evidence draws below operator vectors,
    - short white vector is drone velocity from recent ego trajectory,
    - long white vector is published sensing/camera origin + forward direction,
    - Center / 45 / Side / Top view buttons are statically wired to animated presets,
    - Center also animates translation back to the recomputed scene center.
  4.3G adds a generated-viewer HTML validator.
  Mission evidence retention gate + dry-run manifest is complete and validated (14-test suite).
  Unified viewer sidecar is complete and validated:
    - apps/dedalus_viewer.cpp: TCP→SSE relay with --static-root / --static-default-file static file serving.
    - tools/visualization/mission_unified_viewer.py: pure SSE-driven SPA (no embedded data).
      Renders all 5 event types: world_snapshot, mission_obstacle_map_delta,
      traversability_map_snapshot (exterior voxel face rendering — boundary of union surface),
      ghost_detections (labeled spheres + velocity arrows), mission_event (scrolling log panel).
    - tools/validation/validate-mission-unified-viewer.py: 50+ contract checks, 0 violations.
  The current work is still diagnostics / visualization / persistence plumbing.
  Do not add planner blocking, replanning, or command-sink obstacle avoidance semantics unless explicitly requested.

Representation boundary to preserve:
  Read docs/mission_map_data_representations_and_retention.md before changing retention, map storage, or viewer/data plumbing.
  Keep these four representations distinct:
    1. detector raw evidence (`ObstacleEvidence`) — temporary live input for emergency/reflexive avoidance and map update,
    2. ego-relative reflexive avoidance map (`LocalFlightMapSnapshot`) — bounded current-hazard working set,
    3. mission/site foundational planning map — mission-derived traversability update now, future persistent site-frame planning memory,
    4. visualization/debug/replay representation — streamable/exportable/operator-facing, not the runtime/control storage format.
  Raw evidence can be forgotten only after:
    - active flight/emergency window is clear,
    - mission-local evidence has been compacted,
    - mission-derived traversability output has been persisted,
    - either replayable mission deltas are retained or persistent site-map merge has succeeded idempotently,
    - and a retention manifest records the decision.
  Do not implement deletion first; start with a dry-run retention manifest.

Current validated architecture:
  AirSim DepthPlanar / obstacle evidence
    -> CoreStackRunner
    -> MissionLocalObstacleMap
    -> mission-local obstacle map deltas
    -> RuntimeEventStreamServer
         -> raw TCP JSONL on --world-snapshot-stream-port (default 7788)
         -> optional HTTP/SSE/static on --runtime-event-http-port
              /healthz
              /events
              /mission_local_obstacle_viewer.html, if generated into static root
    -> browser mission-local obstacle viewer
    -> MissionMapAssimilator / MissionLocalTraversabilityMap at finalization
    -> mission_traversability_map_full.json
    -> mission_traversability_map_viewer.html, if generated into static root

  Sidecar unified viewer path (offline / replay / parallel-to-mission):
    RuntimeEventStreamServer TCP JSONL (port 7788)
    -> dedalus_viewer (apps/dedalus_viewer.cpp)
         --host / --port: upstream TCP source
         --http-port: browser-facing HTTP/SSE port (default 8090)
         --replay-dir: replay from artifact directory (auto-switches live↔replay)
         --static-root: directory to serve static HTML from
         --static-default-file: default file for GET / (default: mission_unified_viewer.html)
         Endpoints: /events (SSE), /viewer_status (JSON), /healthz (JSON), GET /<path> (static)
    -> mission_unified_viewer.html (generated by tools/visualization/mission_unified_viewer.py)
         Pure SPA: no embedded data; all state from /events SSE stream.
         Renders: obstacle map, traversability surface (exterior voxel faces), ghost detections,
                  trajectory, sensing volumes, mission event log.
         Polls /viewer_status every 3 s for source label (live / replay / disconnected).

Validated R3 / 4.3F viewer behavior:
  - Runtime serves /healthz, /events, and generated mission_local_obstacle_viewer.html from one HTTP/SSE/static port when the generated HTML exists in --runtime-event-static-root.
  - Viewer receives world_snapshot events and the World snapshots counter increments.
  - Viewer extracts ego pose from snapshot.ego.position_local arrays.
  - Ego updates counter increments and drone marker moves live.
  - AirSim anti-clockwise orbit renders anti-clockwise in the viewer after the viewer-side Y handedness fix.
  - Live trajectory samples age yellow: 0-2 seconds bright, 2-10 seconds fade, older samples dim.
  - Live obstacle delta samples render with the same stable height/topo color semantics as mission cells.
  - Mission/live obstacle evidence draws below operator vectors.
  - Short white vector is drone velocity from recent ego trajectory.
  - Long white vector is published sensing/camera origin + forward direction.
  - Left panel has Center, 45 degree, Side, and Top view buttons.
  - View buttons are statically wired to animated presets.
  - Center also translates back to the recomputed scene center.
  - Generated HTML must be regenerated after changes to tools/visualization/mission_local_obstacle_viewer.py.
  - Validate generated HTML with tools/validation/validate-mission-local-obstacle-viewer.py.

Validated S4 foundational traversability behavior:
  - Normal AirSim mission command with --merge-obstacle-map emits mission_traversability_map_full.json and .meta.json.
  - Example validated artifact:
      schema: dedalus.mission_local_traversability_map.v1
      site_id: validate_r3b1
      site_frame_id: airsim_world
      mission_id: validate_r3b1_mission
      cell_count: 40402
      occupied_cell_count: 16881
      low_clearance_cell_count: 35166
      overhead_risk_cell_count: 23287
  - tools/visualization/mission_traversability_map_viewer.py generates mission_traversability_map_viewer.html.
  - tools/validation/validate-mission-traversability-map-viewer.py validates the generated HTML.
  - Viewer display may cap cells (for example 4096) even when the artifact contains more cells; that is display decimation, not data loss.
  - `mission_traversability_map_viewer.html` can poll a large artifact with --artifact-url; for offline/final viewing, consider generating without polling.

Most recent operator observations:
  - S4 artifact exists and validated from normal wrapper path.
  - mission_traversability_map_viewer.html works in browser from static root.
  - mission_local_obstacle_viewer.html is not auto-generated by run_mission.sh; static server returns not found unless it has been generated into the output/static root.
  - /healthz may feel slow if a browser tab is polling the 41MB traversability JSON artifact.
  - dedalus_viewer static file serving compiles and smoke-tested (GET /, path traversal 404, healthz still works).
  - mission_unified_viewer.html validated: 0 violations across 50+ contract checks.

Immediate next tasks:
  1. Operator-validate mission_unified_viewer.html in a live or replay session:
       python3 tools/visualization/mission_unified_viewer.py --output out/validate_r3b1/mission_unified_viewer.html
       ./build/apps/dedalus_viewer --replay-dir out/validate_r3b1 --http-port 8090 --static-root out/validate_r3b1
       open http://localhost:8090/
     Verify: trav surface renders, obstacle dots appear, trajectory animates, status bar shows replay.
  2. Clarify the durable planner-facing site map: the current MissionLocalTraversabilityMap is a
     mission-derived foundational artifact, not yet the complete cross-mission site planning map.
  3. Keep runtime preload diagnostics-only until separately validated.
  4. Do not add planner/control coupling yet.

Runtime commands used for validated viewer workflow:
  Generate mission-local obstacle viewer HTML:
    FIRST_SNAPSHOT="$(find out/validate_r3b1 -maxdepth 1 -type f -name 'snapshot_*.json' | sort | head -1)"
    python3 tools/visualization/mission_local_obstacle_viewer.py \
      "$FIRST_SNAPSHOT" \
      --history-glob 'out/validate_r3b1/snapshot_*.json' \
      --max-cells 2048 \
      --output out/validate_r3b1/mission_local_obstacle_viewer.html

  Generate traversability viewer HTML:
    python3 tools/visualization/mission_traversability_map_viewer.py \
      out/validate_r3b1/mission_traversability_map_full.json \
      --artifact-url mission_traversability_map_full.json \
      --output out/validate_r3b1/mission_traversability_map_viewer.html

    python3 tools/validation/validate-mission-traversability-map-viewer.py \
      out/validate_r3b1/mission_traversability_map_viewer.html

  Generate and validate unified viewer HTML:
    python3 tools/visualization/mission_unified_viewer.py \
      --output out/validate_r3b1/mission_unified_viewer.html

    python3 tools/validation/validate-mission-unified-viewer.py \
      out/validate_r3b1/mission_unified_viewer.html

  Start dedalus_viewer sidecar (replay mode, serves unified viewer):
    ./build/apps/dedalus_viewer \
      --replay-dir out/validate_r3b1 \
      --http-port 8090 \
      --static-root out/validate_r3b1 \
      --static-default-file mission_unified_viewer.html

  Start dedalus_viewer sidecar (live mode, alongside running mission):
    ./build/apps/dedalus_viewer \
      --host 127.0.0.1 \
      --port 7788 \
      --http-port 8090 \
      --static-root out/validate_r3b1

  Start AirSim mission with runtime HTTP/SSE/static viewer:
    DEDALUS_AIRSIM_ENABLE_DEPTH_OBSTACLES=1 \
    DEDALUS_AIRSIM_DEPTH_OBSTACLE_MAX_POINTS=4096 \
    DEDALUS_AIRSIM_DEPTH_OBSTACLE_STRIDE=8 \
    DEDALUS_AIRSIM_DEPTH_OBSTACLE_MAX_RANGE_M=30 \
    DEDALUS_AIRSIM_DEPTH_OBSTACLE_MIN_RANGE_M=0.5 \
    DEDALUS_AIRSIM_DEPTH_OBSTACLE_CONFIDENCE=0.8 \
    simulation/airsim/run_mission.sh \
      --output-dir out/validate_r3b1 \
      --merge-obstacle-map \
      --obstacle-map-site-id validate_r3b1 \
      --obstacle-map-site-frame-id airsim_world \
      --obstacle-map-mission-id validate_r3b1_mission \
      --runtime-event-http-port 8080 \
      --runtime-event-static-root out/validate_r3b1

  Local browser through SSH port forward:
    ssh -L 8080:127.0.0.1:8080 <ec2-host>   # legacy per-viewer HTML
    http://127.0.0.1:8080/mission_local_obstacle_viewer.html
    http://127.0.0.1:8080/mission_traversability_map_viewer.html

    ssh -L 8090:127.0.0.1:8090 <ec2-host>   # unified sidecar viewer
    http://127.0.0.1:8090/                   # mission_unified_viewer.html (default)

  Run evidence retention dry-run:
    python3 tools/mission/mission-evidence-retention.py \
      --output-dir out/validate_r3b1 \
      --maps-dir maps \
      --keep-every-n 100 \
      --json

Patch output and safety policy:
  - Before code changes, inspect current repo files that define call path, data flow, flags, schemas, tests, and scripts.
  - Do not guess file structure, option names, parser blocks, test layout, function signatures, enum values, generated artifact paths, or runtime wiring.
  - Patch scripts should print concise OK:/ERROR: lines only.
  - Do not append routine grep/diff/code-section dumps after patches.
  - Do not call sys.exit(), raise SystemExit, shell exit, or intentionally terminate the user's shell/session from generated patch snippets.
  - Do not assume out/, generated artifacts, runtime logs, snapshots, or validation directories exist inside patch logic unless explicitly provided for that patch.
  - If anchors do not match, print ERROR and do not write partial changes.
  - Use prose summaries and build/test/runtime commands for validation.

Do not:
  - Do not use YOLO/DETR/classifier outputs as a prerequisite for obstacle avoidance.
  - Do not derive visual obstacle coverage from vehicle yaw alone.
  - Do not judge detector output against global AirSim GT objects outside configured sensing volume.
  - Do not use AirSim object-GT as the 4.x obstacle detector.
  - Do not add named-object class filters to obstacle detection.
  - Do not couple obstacle persistence, map-building policy, or sensing coverage to a flight command sink.
  - Do not add avoidance/replanning/control behavior from persistent memory until explicitly scoped and validated.
  - Do not name files, validators, scripts, or symbols after planning labels or temporary session shorthand.
