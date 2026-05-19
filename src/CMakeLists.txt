add_library(dedalus_core
    behavior/behavior_spec.cpp
    behavior/mission_runtime.cpp
    behavior/object_behavior_mission_controller.cpp
    behavior/px4_bridge_command_sink.cpp
    behavior/px4_mavlink_command_sink.cpp
    behavior/target_selector.cpp
    behavior/trajectory_mission_controller.cpp
    behavior/velocity_trajectory.cpp
    sensors/ego_state_provider.cpp
    sensors/recorded_frame_source.cpp
    sensors/replay_frame_source.cpp
    sensors/synthetic_frame_source.cpp
    sensors/synthetic_mission_frame_source.cpp
    perception/ghost_targets.cpp
    perception/perception_pipeline.cpp
    runtime/config_loader.cpp
    runtime/core_stack_runner.cpp
    runtime/pipeline_profiler.cpp
    runtime/provider_registry.cpp
    simulation/airsim_providers.cpp
    simulation/bridge_transport.cpp
    visualization/frame_annotator.cpp
    visualization/world_overlay_sidecar.cpp
    visualization/world_to_image_projector.cpp
    world_model/in_memory_world_model.cpp
    world_model/rough_flight_map_builder.cpp
    world_model/tactical_obstacle_mapper.cpp
    world_model/world_snapshot.cpp
)

target_include_directories(dedalus_core
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

target_compile_features(dedalus_core PUBLIC cxx_std_20)

target_compile_options(dedalus_core
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)
