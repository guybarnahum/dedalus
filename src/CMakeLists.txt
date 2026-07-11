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
    avoidance/local_esdf_map_publisher.cpp
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
    sensors/visual_ego_state_provider.cpp
    sensors/async_prefetch_frame_source.cpp
    sensors/recorded_frame_source.cpp
    sensors/replay_frame_source.cpp
    sensors/synthetic_frame_source.cpp
    sensors/synthetic_mission_frame_source.cpp
    sensing/sensing_coverage.cpp
    sensing/airsim_emulation_depth_obstacle_detector.cpp
    sensing/depth_debug_annotator.cpp
    sensing/depth_projection_kernel.cpp
    sensing/visual_depth_obstacle_detector.cpp
    perception/ghost_targets.cpp
    perception/perception_pipeline.cpp
    runtime/camera_pointing_state_store.cpp
    runtime/config_loader.cpp
    runtime/perch_candidate_evaluator.cpp
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
        sqlite3
)

target_compile_options(dedalus_core
    PRIVATE
        -Wall
        -Wextra
        # Scope -Wpedantic to C++ only.  nvcc forwards it to GCC *after* applying
        # -Xcompiler= overrides, so -Xcompiler=-Wno-pedantic cannot cancel it.
        # The warnings come from cudafe's generated line directives in CUDA system
        # headers — GCC extension form "# N" vs standard "#line N" — and there is
        # no nvcc flag to suppress them without hiding real issues in our code.
        $<$<COMPILE_LANGUAGE:CXX>:-Wpedantic>
)

# Optional: ONNX Runtime depth engine
# Enable with -DDEDALUS_ENABLE_ONNX_DEPTH=ON when ONNX Runtime is installed.
if(DEDALUS_ENABLE_ONNX_DEPTH)
    find_package(onnxruntime REQUIRED)
    target_sources(dedalus_core PRIVATE sensing/onnx_depth_engine.cpp)
    target_link_libraries(dedalus_core PRIVATE onnxruntime::onnxruntime)
    target_compile_definitions(dedalus_core PUBLIC DEDALUS_ONNX_DEPTH_ENABLED)
endif()

# CUDA depth kernels
if(DEDALUS_CUDA)
    target_sources(dedalus_core PRIVATE sensing/cuda_depth_kernels.cu)
    set_source_files_properties(sensing/cuda_depth_kernels.cu
        PROPERTIES CUDA_SEPARABLE_COMPILATION OFF)
    target_compile_definitions(dedalus_core PUBLIC DEDALUS_CUDA_ENABLED)
    target_link_libraries(dedalus_core PRIVATE CUDA::cudart)
    set_source_files_properties(sensing/cuda_depth_kernels.cu
        PROPERTIES COMPILE_OPTIONS "-O2")
endif()

# TensorRT inference engine (requires DEDALUS_CUDA)
if(DEDALUS_TENSORRT)
    if(NOT DEDALUS_CUDA)
        message(FATAL_ERROR "DEDALUS_TENSORRT requires DEDALUS_CUDA=ON")
    endif()
    target_sources(dedalus_core PRIVATE sensing/tensorrt_depth_engine.cpp)
    target_compile_definitions(dedalus_core PUBLIC DEDALUS_TENSORRT_ENABLED)
    set(DEDALUS_TENSORRT_ROOT "" CACHE PATH
        "TensorRT install root — set if TRT is not on the system include/lib path")
    if(DEDALUS_TENSORRT_ROOT)
        target_include_directories(dedalus_core PRIVATE ${DEDALUS_TENSORRT_ROOT}/include)
        target_link_directories(dedalus_core PRIVATE ${DEDALUS_TENSORRT_ROOT}/lib)
    endif()
    target_link_libraries(dedalus_core PRIVATE nvinfer nvinfer_plugin nvonnxparser)
endif()

# These files are tight numeric kernels; compile at -O2 even in Debug builds
# so perf tests stay green.
set_source_files_properties(
    avoidance/compute_esdf.cpp
    avoidance/local_esdf_map_incremental.cpp
    PROPERTIES COMPILE_OPTIONS "-O2"
)
