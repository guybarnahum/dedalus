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
  5Q-5U obstacle-memory default path is compact-delta-first and SQLite-backed.
  The current work is still diagnostics / visualization / persistence plumbing.
  Do not add planner blocking, replanning, or command-sink obstacle avoidance semantics unless explicitly requested.

Current validated architecture:
  AirSim DepthPlanar / obstacle evidence
    -> CoreStackRunner
    -> MissionLocalObstacleMap
    -> mission-local obstacle map deltas
    -> RuntimeEventStreamServer
         -> raw TCP JSONL on --world-snapshot-stream-port
         -> optional HTTP/SSE/static on --runtime-event-http-port
              /healthz
              /events
              /mission_local_obstacle_viewer.html
    -> browser mission-local obstacle viewer

Validated R3 viewer behavior:
  - Runtime serves /healthz, /events, and generated mission_local_obstacle_viewer.html from one HTTP/SSE/static port.
  - Viewer receives world_snapshot events and the World snapshots counter increments.
  - Viewer extracts ego pose from snapshot.ego.position_local arrays.
  - Ego updates counter increments and drone marker moves live.
  - AirSim anti-clockwise orbit renders anti-clockwise in the viewer after the viewer-side Y handedness fix.
  - Live trajectory samples age yellow: 0-2 seconds bright, 2-10 seconds fade, older samples dim.
  - Live obstacle delta samples age red through a separate/coalesced visual event layer: 0-2 seconds bright, 2-10 seconds fade, older samples dim.
  - Base obstacle map cells remain dim so they do not mask live red aging.
  - Left panel has Center, 45 degree, Side, and Top view buttons.
  - Generated HTML must be regenerated after changes to tools/visualization/mission_local_obstacle_viewer.py.

Most recent operator observations:
  - World snapshots increments: YES.
  - Ego updates increments: YES.
  - Drone marker moves from snapshot.ego.position_local: YES.
  - AirSim anti-clockwise orbit renders anti-clockwise in viewer: YES.
  - Red obstacle cell aging now works after event-layer/coalescing fix.
  - Yellow live track aging works.
  - View preset buttons work.
  - Remaining concern: viewer can appear seconds behind AirSim near landing/shutdown. Treat as browser/SSE processing backlog until measured.

Immediate next tasks:
  1. Preserve current validated R3 viewer behavior.
  2. If viewer lag remains visible, add client-side event coalescing:
       - store latest pending world_snapshot event instead of processing every old snapshot immediately
       - accumulate/bound pending mission_obstacle_map_delta cells
       - process pending events once per animation frame
       - keep counters diagnostic and honest
  3. Add or update lightweight viewer tests if practical, especially for:
       - array ego pose extraction
       - 0-2s bright hold / 2-10s fade / >10s dim aging
       - Y handedness projection
       - view preset handlers
       - scheduled/coalesced draw behavior
  4. Continue persistent obstacle memory work only after preserving diagnostics:
       - site-map scoring / age diagnostics
       - diagnostics-only runtime preload
       - no planner/control coupling yet

Runtime commands used for validated viewer workflow:
  Generate viewer HTML:
    FIRST_SNAPSHOT="$(find out/validate_r3b1 -maxdepth 1 -type f -name 'snapshot_*.json' | sort | head -1)"
    python3 tools/visualization/mission_local_obstacle_viewer.py \
      "$FIRST_SNAPSHOT" \
      --history-glob 'out/validate_r3b1/snapshot_*.json' \
      --max-cells 2048 \
      --output out/validate_r3b1/mission_local_obstacle_viewer.html

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
    ssh -L 8080:127.0.0.1:8080 <ec2-host>
    http://127.0.0.1:8080/mission_local_obstacle_viewer.html

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
