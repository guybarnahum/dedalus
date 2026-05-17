add_executable(test_world_snapshot_json
    unit/test_world_snapshot_json.cpp
)

target_link_libraries(test_world_snapshot_json
    PRIVATE
        dedalus_core
)

target_compile_features(test_world_snapshot_json PRIVATE cxx_std_20)

add_test(NAME world_snapshot_json COMMAND test_world_snapshot_json)

add_executable(test_perception_world_model_flow
    unit/test_perception_world_model_flow.cpp
)

target_link_libraries(test_perception_world_model_flow
    PRIVATE
        dedalus_core
)

target_compile_features(test_perception_world_model_flow PRIVATE cxx_std_20)

add_test(NAME perception_world_model_flow COMMAND test_perception_world_model_flow)

add_executable(test_video_only_world_model_flow
    unit/test_video_only_world_model_flow.cpp
)

target_link_libraries(test_video_only_world_model_flow
    PRIVATE
        dedalus_core
)

target_compile_features(test_video_only_world_model_flow PRIVATE cxx_std_20)

add_test(NAME video_only_world_model_flow COMMAND test_video_only_world_model_flow)

add_executable(test_provider_composition
    unit/test_provider_composition.cpp
)

target_link_libraries(test_provider_composition
    PRIVATE
        dedalus_core
)

target_compile_features(test_provider_composition PRIVATE cxx_std_20)

add_test(NAME provider_composition COMMAND test_provider_composition)

add_executable(test_core_stack_config_loader
    unit/test_core_stack_config_loader.cpp
)

target_link_libraries(test_core_stack_config_loader
    PRIVATE
        dedalus_core
)

target_compile_features(test_core_stack_config_loader PRIVATE cxx_std_20)

add_test(NAME core_stack_config_loader COMMAND test_core_stack_config_loader)
set_tests_properties(core_stack_config_loader
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

add_test(NAME recorded_frame_world_model_flow COMMAND test_recorded_frame_world_model_flow)
set_tests_properties(recorded_frame_world_model_flow
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

add_test(NAME airsim_provider_boundary COMMAND test_airsim_provider_boundary)
set_tests_properties(airsim_provider_boundary
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

add_test(NAME ppm_frame_annotation_sink COMMAND test_ppm_frame_annotation_sink)

add_executable(test_pipeline_profiler
    unit/test_pipeline_profiler.cpp
)

target_link_libraries(test_pipeline_profiler
    PRIVATE
        dedalus_core
)

target_compile_features(test_pipeline_profiler PRIVATE cxx_std_20)

add_test(NAME pipeline_profiler COMMAND test_pipeline_profiler)

add_executable(test_trajectory_mission_controller
    unit/test_trajectory_mission_controller.cpp
)

target_link_libraries(test_trajectory_mission_controller
    PRIVATE
        dedalus_core
)

target_compile_features(test_trajectory_mission_controller PRIVATE cxx_std_20)

add_test(NAME trajectory_mission_controller COMMAND test_trajectory_mission_controller)

add_executable(test_airsim_velocity_command_sink
    unit/test_airsim_velocity_command_sink.cpp
)

target_link_libraries(test_airsim_velocity_command_sink
    PRIVATE
        dedalus_core
)

target_compile_features(test_airsim_velocity_command_sink PRIVATE cxx_std_20)

add_test(NAME airsim_velocity_command_sink COMMAND test_airsim_velocity_command_sink)

add_executable(test_px4_mavlink_command_sink
    unit/test_px4_mavlink_command_sink.cpp
)

target_link_libraries(test_px4_mavlink_command_sink
    PRIVATE
        dedalus_core
)

target_compile_features(test_px4_mavlink_command_sink PRIVATE cxx_std_20)

add_test(NAME px4_mavlink_command_sink COMMAND test_px4_mavlink_command_sink)

add_executable(test_mission_runtime
    unit/test_mission_runtime.cpp
)

target_link_libraries(test_mission_runtime
    PRIVATE
        dedalus_core
)

target_compile_features(test_mission_runtime PRIVATE cxx_std_20)

add_test(NAME mission_runtime COMMAND test_mission_runtime)

add_executable(test_latest_world_snapshot
    unit/test_latest_world_snapshot.cpp
)

target_link_libraries(test_latest_world_snapshot
    PRIVATE
        dedalus_core
)

target_compile_features(test_latest_world_snapshot PRIVATE cxx_std_20)

add_test(NAME latest_world_snapshot COMMAND test_latest_world_snapshot)

find_package(Python3 COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
    add_test(
        NAME replay_recording_smoke
        COMMAND ${Python3_EXECUTABLE}
            ${PROJECT_SOURCE_DIR}/tests/integration/test_replay_recording_smoke.py
            ${PROJECT_SOURCE_DIR}
            ${PROJECT_BINARY_DIR}
    )
    add_test(
        NAME replay_artifact_validator
        COMMAND ${Python3_EXECUTABLE}
            ${PROJECT_SOURCE_DIR}/tests/integration/test_replay_artifact_validator.py
            ${PROJECT_SOURCE_DIR}
            ${PROJECT_BINARY_DIR}
    )
    add_test(
        NAME ppm_sequence_mp4_export
        COMMAND ${Python3_EXECUTABLE}
            ${PROJECT_SOURCE_DIR}/tests/integration/test_ppm_sequence_mp4_export.py
            ${PROJECT_SOURCE_DIR}
            ${PROJECT_BINARY_DIR}
    )
    add_test(
        NAME replay_mission_smoke
        COMMAND ${Python3_EXECUTABLE}
            ${PROJECT_SOURCE_DIR}/tests/integration/test_replay_mission_smoke.py
            ${PROJECT_SOURCE_DIR}
            ${PROJECT_BINARY_DIR}
    )
    add_test(
        NAME mission_artifact_validator
        COMMAND ${Python3_EXECUTABLE}
            ${PROJECT_SOURCE_DIR}/tests/integration/test_mission_artifact_validator.py
            ${PROJECT_SOURCE_DIR}
    )
    add_test(
        NAME mission_scenario_runner
        COMMAND ${Python3_EXECUTABLE}
            ${PROJECT_SOURCE_DIR}/tests/integration/test_mission_scenario_runner.py
            ${PROJECT_SOURCE_DIR}
            ${PROJECT_BINARY_DIR}
    )
endif()