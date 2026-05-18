add_executable(test_world_snapshot_json
    unit/test_world_snapshot_json.cpp
)

target_link_libraries(test_world_snapshot_json
    PRIVATE
        dedalus_core
)

target_compile_features(test_world_snapshot_json PRIVATE cxx_std_20)

add_test(NAME "world_snapshot_json (contracts)" COMMAND test_world_snapshot_json)

add_executable(test_perception_world_model_flow
    unit/test_perception_world_model_flow.cpp
)

target_link_libraries(test_perception_world_model_flow
    PRIVATE
        dedalus_core
)

target_compile_features(test_perception_world_model_flow PRIVATE cxx_std_20)

add_test(NAME "perception_world_model_flow (unit/synthetic)" COMMAND test_perception_world_model_flow)

add_executable(test_video_only_world_model_flow
    unit/test_video_only_world_model_flow.cpp
)

target_link_libraries(test_video_only_world_model_flow
    PRIVATE
        dedalus_core
)

target_compile_features(test_video_only_world_model_flow PRIVATE cxx_std_20)

add_test(NAME "video_only_world_model_flow (unit/synthetic)" COMMAND test_video_only_world_model_flow)

add_executable(test_provider_composition
    unit/test_provider_composition.cpp
)

target_link_libraries(test_provider_composition
    PRIVATE
        dedalus_core
)

target_compile_features(test_provider_composition PRIVATE cxx_std_20)

add_test(NAME "provider_composition (contracts)" COMMAND test_provider_composition)

add_executable(test_core_stack_config_loader
    unit/test_core_stack_config_loader.cpp
)

target_link_libraries(test_core_stack_config_loader
    PRIVATE
        dedalus_core
)

target_compile_features(test_core_stack_config_loader PRIVATE cxx_std_20)

add_test(NAME "core_stack_config_loader (contracts)" COMMAND test_core_stack_config_loader)
set_tests_properties("core_stack_config_loader (contracts)"
    PROPERTIES
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

add_executable(test_recorded_frame_world_model_flow
    unit/test_recorded_frame_world_model_flow.cpp
)

target_link_libraries(test_recorded_frame_world_model_flow
    PRIVATE
        dedalus_core
)

target_compile_features(test_recorded_frame_world_model_flow PRIVATE cxx_std_20)

add_test(NAME "recorded_frame_world_model_flow (unit/synthetic)" COMMAND test_recorded_frame_world_model_flow)
set_tests_properties("recorded_frame_world_model_flow (unit/synthetic)"
    PROPERTIES
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

add_executable(test_airsim_provider_boundary
    unit/test_airsim_provider_boundary.cpp
)

target_link_libraries(test_airsim_provider_boundary
    PRIVATE
        dedalus_core
)

target_compile_features(test_airsim_provider_boundary PRIVATE cxx_std_20)

add_test(NAME "airsim_provider_boundary (contracts/airsim)" COMMAND test_airsim_provider_boundary)
set_tests_properties("airsim_provider_boundary (contracts/airsim)"
    PROPERTIES
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

add_executable(test_ppm_frame_annotation_sink
    unit/test_ppm_frame_annotation_sink.cpp
)

target_link_libraries(test_ppm_frame_annotation_sink
    PRIVATE
        dedalus_core
)

target_compile_features(test_ppm_frame_annotation_sink PRIVATE cxx_std_20)

add_test(NAME "ppm_frame_annotation_sink (contracts)" COMMAND test_ppm_frame_annotation_sink)

add_executable(test_pipeline_profiler
    unit/test_pipeline_profiler.cpp
)

target_link_libraries(test_pipeline_profiler
    PRIVATE
        dedalus_core
)

target_compile_features(test_pipeline_profiler PRIVATE cxx_std_20)

add_test(NAME "pipeline_profiler (unit/synthetic)" COMMAND test_pipeline_profiler)

add_executable(test_trajectory_mission_controller
    unit/test_trajectory_mission_controller.cpp
)

target_link_libraries(test_trajectory_mission_controller
    PRIVATE
        dedalus_core
)

target_compile_features(test_trajectory_mission_controller PRIVATE cxx_std_20)

add_test(NAME "trajectory_mission_controller (unit/synthetic)" COMMAND test_trajectory_mission_controller)

add_executable(test_airsim_velocity_command_sink
    unit/test_airsim_velocity_command_sink.cpp
)

target_link_libraries(test_airsim_velocity_command_sink
    PRIVATE
        dedalus_core
)

target_compile_features(test_airsim_velocity_command_sink PRIVATE cxx_std_20)

add_test(NAME "airsim_velocity_command_sink (contracts)" COMMAND test_airsim_velocity_command_sink)

add_executable(test_px4_mavlink_command_sink
    unit/test_px4_mavlink_command_sink.cpp
)

target_link_libraries(test_px4_mavlink_command_sink
    PRIVATE
        dedalus_core
)

target_compile_features(test_px4_mavlink_command_sink PRIVATE cxx_std_20)

add_test(NAME "px4_mavlink_command_sink (contracts)" COMMAND test_px4_mavlink_command_sink)

add_executable(test_mission_runtime
    unit/test_mission_runtime.cpp
)

target_link_libraries(test_mission_runtime
    PRIVATE
        dedalus_core
)

target_compile_features(test_mission_runtime PRIVATE cxx_std_20)

add_test(NAME "mission_runtime (unit/synthetic)" COMMAND test_mission_runtime)

add_executable(test_latest_world_snapshot
    unit/test_latest_world_snapshot.cpp
)

target_link_libraries(test_latest_world_snapshot
    PRIVATE
        dedalus_core
)

target_compile_features(test_latest_world_snapshot PRIVATE cxx_std_20)

add_test(NAME "latest_world_snapshot (unit/synthetic)" COMMAND test_latest_world_snapshot)

add_executable(test_behavior_spec
    unit/test_behavior_spec.cpp
)

target_link_libraries(test_behavior_spec
    PRIVATE
        dedalus_core
)

target_compile_features(test_behavior_spec PRIVATE cxx_std_20)

add_test(NAME "behavior_spec (contracts)" COMMAND test_behavior_spec)

add_executable(test_target_selector
    unit/test_target_selector.cpp
)

target_link_libraries(test_target_selector
    PRIVATE
        dedalus_core
)

target_compile_features(test_target_selector PRIVATE cxx_std_20)

add_test(NAME "target_selector (contracts)" COMMAND test_target_selector)

# Validation layers:
#   contracts  — data formats, config parsing, protocol/boundary contracts
#   unit       — deterministic native unit tests, often using synthetic inputs
#   synthetic  — tests that exercise synthetic/recorded pipeline data or mission artifacts
#   scenario   — archive-grade mission scenario/campaign harness tests
#   airsim     — AirSim-facing boundaries that do not require a live simulator in CTest
#
# The primary layer is included in the CTest display name so default output looks like:
#   Start  1: world_snapshot_json (contracts)
# Labels are still used for layer filtering with `ctest -L <layer>`.
# No CTest test is labeled airsim_live today. Real live AirSim/PX4 validation remains an
# explicit operator-run smoke/campaign command outside normal CTest.
set_tests_properties(
    "world_snapshot_json (contracts)"
    "behavior_spec (contracts)"
    "target_selector (contracts)"
    "core_stack_config_loader (contracts)"
    "provider_composition (contracts)"
    "ppm_frame_annotation_sink (contracts)"
    "airsim_velocity_command_sink (contracts)"
    "px4_mavlink_command_sink (contracts)"
    PROPERTIES
        LABELS "contracts;unit"
)

set_tests_properties(
    "perception_world_model_flow (unit/synthetic)"
    "video_only_world_model_flow (unit/synthetic)"
    "recorded_frame_world_model_flow (unit/synthetic)"
    "pipeline_profiler (unit/synthetic)"
    "trajectory_mission_controller (unit/synthetic)"
    "mission_runtime (unit/synthetic)"
    "latest_world_snapshot (unit/synthetic)"
    PROPERTIES
        LABELS "unit;synthetic"
)

set_tests_properties(
    "airsim_provider_boundary (contracts/airsim)"
    PROPERTIES
        LABELS "contracts;unit;airsim"
)

find_package(Python3 COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
    add_test(
        NAME "replay_recording_smoke (synthetic)"
        COMMAND ${Python3_EXECUTABLE}
            ${PROJECT_SOURCE_DIR}/tests/integration/test_replay_recording_smoke.py
            ${PROJECT_SOURCE_DIR}
            ${PROJECT_BINARY_DIR}
    )
    add_test(
        NAME "replay_artifact_validator (synthetic)"
        COMMAND ${Python3_EXECUTABLE}
            ${PROJECT_SOURCE_DIR}/tests/integration/test_replay_artifact_validator.py
            ${PROJECT_SOURCE_DIR}
            ${PROJECT_BINARY_DIR}
    )
    add_test(
        NAME "ppm_sequence_mp4_export (synthetic)"
        COMMAND ${Python3_EXECUTABLE}
            ${PROJECT_SOURCE_DIR}/tests/integration/test_ppm_sequence_mp4_export.py
            ${PROJECT_SOURCE_DIR}
            ${PROJECT_BINARY_DIR}
    )
    add_test(
        NAME "replay_mission_smoke (synthetic)"
        COMMAND ${Python3_EXECUTABLE}
            ${PROJECT_SOURCE_DIR}/tests/integration/test_replay_mission_smoke.py
            ${PROJECT_SOURCE_DIR}
            ${PROJECT_BINARY_DIR}
    )
    add_test(
        NAME "mission_artifact_validator (synthetic)"
        COMMAND ${Python3_EXECUTABLE}
            ${PROJECT_SOURCE_DIR}/tests/integration/test_mission_artifact_validator.py
            ${PROJECT_SOURCE_DIR}
    )
    add_test(
        NAME "mission_scenario_runner (scenario)"
        COMMAND ${Python3_EXECUTABLE}
            ${PROJECT_SOURCE_DIR}/tests/integration/test_mission_scenario_runner.py
            ${PROJECT_SOURCE_DIR}
            ${PROJECT_BINARY_DIR}
    )
    add_test(
        NAME "mission_campaign_runner (scenario)"
        COMMAND ${Python3_EXECUTABLE}
            ${PROJECT_SOURCE_DIR}/tests/integration/test_mission_campaign_runner.py
            ${PROJECT_SOURCE_DIR}
            ${PROJECT_BINARY_DIR}
    )
    add_test(
        NAME "mission_abort_scenario (scenario)"
        COMMAND ${Python3_EXECUTABLE}
            ${PROJECT_SOURCE_DIR}/tests/integration/test_mission_abort_scenario.py
            ${PROJECT_SOURCE_DIR}
            ${PROJECT_BINARY_DIR}
    )

    set_tests_properties(
        "replay_recording_smoke (synthetic)"
        "replay_artifact_validator (synthetic)"
        "ppm_sequence_mp4_export (synthetic)"
        "replay_mission_smoke (synthetic)"
        "mission_artifact_validator (synthetic)"
        PROPERTIES
            LABELS "synthetic;integration"
    )

    set_tests_properties(
        "mission_scenario_runner (scenario)"
        "mission_campaign_runner (scenario)"
        "mission_abort_scenario (scenario)"
        PROPERTIES
            LABELS "scenario;synthetic;integration"
    )
endif()
