find_package(SQLite3 REQUIRED)

add_library(dedalus_core
    geometry/pose_transform.cpp
    avoidance/local_flight_map.cpp
    avoidance/local_flight_map_ingest.cpp
    avoidance/local_flight_map_mission.cpp
    avoidance/trajectory_safety_evaluator.cpp
    avoidance/mission_local_obstacle_map.cpp
    avoidance/mission_local_traversability_map.cpp
    avoidance/mission_local_planning_map.cpp
    avoidance/mission_local_planning_map_sqlite.cpp
    avoidance/mission_local_planning_map_spatial.cpp
    avoidance/mission_local_planning_map_query.cpp
    avoidance/local_esdf_map.cpp
    avoidance/local_esdf_map_incremental.cpp
    avoidance/compute_esdf.cpp
    avoidance/mission_local_planning_map_publisher.cpp
    avoidance/mission_map_assimilator.cpp
    avoidance/mission_obstacle_map_artifact_writer.cpp
    avoidance/mission_obstacle_map_artifact_writer_env.cpp
    avoidance/mission_traversability_map_artifact_writer.cpp
    avoidance/mission_traversability_map_artifact_writer_env.cpp
    avoidance/mission_obstacle_map_delta_writer.cpp
    avoidance/mission_local_traversability_map_publisher.cpp
    behavior/behavior_osd.cpp
    behavior/behavior_spec.cpp
    behavior/behavior_spec_strings.cpp
    behavior/follow_geometry_policy.cpp
    behavior/follow_geometry_policy_circle.cpp
    behavior/follow_geometry_policy_follow.cpp
    behavior/object_behavior_config.cpp
    behavior/mission_runtime.cpp
    behavior/mission_runtime_dispatch.cpp
    behavior/object_behavior_mission_controller.cpp
    behavior/object_behavior_mission_controller_sequence.cpp
    behavior/object_behavior_mission_controller_camera.cpp
    behavior/object_behavior_mission_controller_tick.cpp
    behavior/object_behavior_mission_controller_tick_execute.cpp
    behavior/px4_bridge_command_sink.cpp
    behavior/mavlink_gimbal_pointing_sink.cpp
    behavior/px4_mavlink_command_sink.cpp
    behavior/target_selector.cpp
    behavior/trajectory_mission_controller.cpp
    behavior/trajectory_mission_controller_tick.cpp
    behavior/velocity_trajectory.cpp
    behavior/velocity_trajectory_parse.cpp
    sensors/ego_state_provider.cpp
    sensors/async_prefetch_frame_source.cpp
    sensors/recorded_frame_source.cpp
    sensors/replay_frame_source.cpp
    sensors/synthetic_frame_source.cpp
    sensors/synthetic_mission_frame_source.cpp
    sensing/sensing_coverage.cpp
    sensing/airsim_depth_obstacle_detector.cpp
    sensing/airsim_depth_obstacle_detector_impl.cpp
    perception/ghost_targets.cpp
    perception/perception_pipeline.cpp
    runtime/camera_pointing_state_store.cpp
    runtime/config_loader.cpp
    runtime/core_stack_runner.cpp
    runtime/core_stack_runner_run.cpp
    runtime/pipeline_profiler.cpp
    runtime/provider_registry.cpp
    runtime/world_snapshot_stream_server.cpp
    runtime/world_snapshot_stream_server_messages.cpp
    runtime/world_snapshot_stream_server_publish.cpp
    simulation/airsim_providers.cpp
    simulation/airsim_providers_binary_stream.cpp
    simulation/airsim_sidecar_parser.cpp
    simulation/bridge_transport.cpp
    simulation/ghost_scenario.cpp
    simulation/ghost_scenario_parse.cpp
    visualization/frame_annotator.cpp
    visualization/frame_annotator_annotate.cpp
    visualization/frame_annotator_ppm_sequence.cpp
    visualization/obstacle_evidence_visualization.cpp
    visualization/world_overlay_sidecar.cpp
    visualization/world_to_image_projector.cpp
    world_model/in_memory_world_model.cpp
    world_model/obstacle_products.cpp
    world_model/rough_flight_map_builder.cpp
    world_model/tactical_obstacle_mapper.cpp
    world_model/world_snapshot.cpp
    world_model/world_snapshot_subscribers.cpp
)

target_include_directories(dedalus_core
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

target_compile_features(dedalus_core PUBLIC cxx_std_20)

target_link_libraries(dedalus_core
    PRIVATE
        SQLite3::SQLite3
)

target_compile_options(dedalus_core
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)

# These files are tight numeric kernels; compile at -O2 even in Debug builds
# so perf tests stay green.
set_source_files_properties(
    avoidance/compute_esdf.cpp
    avoidance/local_esdf_map_incremental.cpp
    PROPERTIES COMPILE_OPTIONS "-O2"
)
