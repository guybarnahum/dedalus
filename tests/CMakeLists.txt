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

find_package(Python3 COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
    add_test(
        NAME replay_recording_smoke
        COMMAND ${Python3_EXECUTABLE}
            ${PROJECT_SOURCE_DIR}/tests/integration/test_replay_recording_smoke.py
            ${PROJECT_SOURCE_DIR}
            ${PROJECT_BINARY_DIR}
    )
endif()
