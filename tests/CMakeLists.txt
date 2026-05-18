function(dedalus_layer_labels OUTPUT_VAR LAYER)
    set(labels "${LAYER}")
    foreach(extra_label IN LISTS ARGN)
        list(APPEND labels "tag:${extra_label}")
    endforeach()
    set(${OUTPUT_VAR} "${labels}" PARENT_SCOPE)
endfunction()

function(dedalus_add_native_test)
    set(options)
    set(one_value_args NAME LAYER)
    set(multi_value_args SOURCES LABELS)
    cmake_parse_arguments(DEDALUS_TEST "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

    if(NOT DEDALUS_TEST_NAME)
        message(FATAL_ERROR "dedalus_add_native_test requires NAME")
    endif()
    if(NOT DEDALUS_TEST_LAYER)
        message(FATAL_ERROR "dedalus_add_native_test(${DEDALUS_TEST_NAME}) requires LAYER")
    endif()
    if(NOT DEDALUS_TEST_SOURCES)
        message(FATAL_ERROR "dedalus_add_native_test(${DEDALUS_TEST_NAME}) requires SOURCES")
    endif()

    set(target_name "test_${DEDALUS_TEST_NAME}")
    set(display_name "${DEDALUS_TEST_NAME} (${DEDALUS_TEST_LAYER})")

    add_executable(${target_name}
        ${DEDALUS_TEST_SOURCES}
    )

    target_link_libraries(${target_name}
        PRIVATE
            dedalus_core
    )

    target_compile_features(${target_name} PRIVATE cxx_std_20)

    add_test(NAME "${display_name}" COMMAND ${target_name})

    dedalus_layer_labels(test_labels "${DEDALUS_TEST_LAYER}" ${DEDALUS_TEST_LABELS})
    set_tests_properties("${display_name}"
        PROPERTIES
            LABELS "${test_labels}"
            DEDALUS_LAYER "${DEDALUS_TEST_LAYER}"
    )
endfunction()

function(dedalus_add_python_test)
    set(options)
    set(one_value_args NAME LAYER)
    set(multi_value_args COMMAND_ARGS LABELS)
    cmake_parse_arguments(DEDALUS_TEST "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

    if(NOT DEDALUS_TEST_NAME)
        message(FATAL_ERROR "dedalus_add_python_test requires NAME")
    endif()
    if(NOT DEDALUS_TEST_LAYER)
        message(FATAL_ERROR "dedalus_add_python_test(${DEDALUS_TEST_NAME}) requires LAYER")
    endif()
    if(NOT DEDALUS_TEST_COMMAND_ARGS)
        message(FATAL_ERROR "dedalus_add_python_test(${DEDALUS_TEST_NAME}) requires COMMAND_ARGS")
    endif()

    set(display_name "${DEDALUS_TEST_NAME} (${DEDALUS_TEST_LAYER})")
    add_test(
        NAME "${display_name}"
        COMMAND ${Python3_EXECUTABLE} ${DEDALUS_TEST_COMMAND_ARGS}
    )

    dedalus_layer_labels(test_labels "${DEDALUS_TEST_LAYER}" ${DEDALUS_TEST_LABELS})
    set_tests_properties("${display_name}"
        PROPERTIES
            LABELS "${test_labels}"
            DEDALUS_LAYER "${DEDALUS_TEST_LAYER}"
    )
endfunction()

# Validation layers execute in declaration order under plain `ctest`:
#   contracts -> unit -> synthetic -> scenario
#
# `LAYER` is the primary validation layer and is used for:
#   - CTest display name, e.g. `world_snapshot_json (contracts)`
#   - the primary CTest label, so `ctest -L contracts` runs only that layer
#   - the DEDALUS_LAYER property for structured inspection tooling
#
# Additional LABELS are emitted as `tag:<name>` so they do not pollute primary layer filters.
# Example: a scenario test can have LABELS synthetic integration, which becomes:
#   scenario;tag:synthetic;tag:integration
#
# No CTest test is labeled airsim_live today. Real live AirSim/PX4 validation remains an
# explicit operator-run smoke/campaign command outside normal CTest.

# Layer 1: contracts / protocols / boundaries.
dedalus_add_native_test(
    NAME world_snapshot_json
    LAYER contracts
    LABELS unit
    SOURCES unit/test_world_snapshot_json.cpp
)

dedalus_add_native_test(
    NAME behavior_spec
    LAYER contracts
    LABELS unit
    SOURCES unit/test_behavior_spec.cpp
)

dedalus_add_native_test(
    NAME target_selector
    LAYER contracts
    LABELS unit
    SOURCES unit/test_target_selector.cpp
)

dedalus_add_native_test(
    NAME core_stack_config_loader
    LAYER contracts
    LABELS unit
    SOURCES unit/test_core_stack_config_loader.cpp
)
set_tests_properties("core_stack_config_loader (contracts)"
    PROPERTIES
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

dedalus_add_native_test(
    NAME provider_composition
    LAYER contracts
    LABELS unit
    SOURCES unit/test_provider_composition.cpp
)

dedalus_add_native_test(
    NAME ppm_frame_annotation_sink
    LAYER contracts
    LABELS unit
    SOURCES unit/test_ppm_frame_annotation_sink.cpp
)

dedalus_add_native_test(
    NAME airsim_provider_boundary
    LAYER contracts
    LABELS unit airsim
    SOURCES unit/test_airsim_provider_boundary.cpp
)
set_tests_properties("airsim_provider_boundary (contracts)"
    PROPERTIES
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

dedalus_add_native_test(
    NAME airsim_velocity_command_sink
    LAYER contracts
    LABELS unit
    SOURCES unit/test_airsim_velocity_command_sink.cpp
)

dedalus_add_native_test(
    NAME px4_mavlink_command_sink
    LAYER contracts
    LABELS unit
    SOURCES unit/test_px4_mavlink_command_sink.cpp
)

# Layer 2: deterministic native unit tests, usually with synthetic in-memory data.
dedalus_add_native_test(
    NAME perception_world_model_flow
    LAYER unit
    LABELS synthetic
    SOURCES unit/test_perception_world_model_flow.cpp
)

dedalus_add_native_test(
    NAME video_only_world_model_flow
    LAYER unit
    LABELS synthetic
    SOURCES unit/test_video_only_world_model_flow.cpp
)

dedalus_add_native_test(
    NAME recorded_frame_world_model_flow
    LAYER unit
    LABELS synthetic
    SOURCES unit/test_recorded_frame_world_model_flow.cpp
)
set_tests_properties("recorded_frame_world_model_flow (unit)"
    PROPERTIES
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

dedalus_add_native_test(
    NAME pipeline_profiler
    LAYER unit
    LABELS synthetic
    SOURCES unit/test_pipeline_profiler.cpp
)

dedalus_add_native_test(
    NAME trajectory_mission_controller
    LAYER unit
    LABELS synthetic
    SOURCES unit/test_trajectory_mission_controller.cpp
)

dedalus_add_native_test(
    NAME mission_runtime
    LAYER unit
    LABELS synthetic
    SOURCES unit/test_mission_runtime.cpp
)

dedalus_add_native_test(
    NAME latest_world_snapshot
    LAYER unit
    LABELS synthetic
    SOURCES unit/test_latest_world_snapshot.cpp
)

find_package(Python3 COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
    # Layer 3: synthetic / recorded data and generated artifact tests.
    dedalus_add_python_test(
        NAME replay_recording_smoke
        LAYER synthetic
        LABELS integration
        COMMAND_ARGS
            ${PROJECT_SOURCE_DIR}/tests/integration/test_replay_recording_smoke.py
            ${PROJECT_SOURCE_DIR}
            ${PROJECT_BINARY_DIR}
    )

    dedalus_add_python_test(
        NAME replay_artifact_validator
        LAYER synthetic
        LABELS integration
        COMMAND_ARGS
            ${PROJECT_SOURCE_DIR}/tests/integration/test_replay_artifact_validator.py
            ${PROJECT_SOURCE_DIR}
            ${PROJECT_BINARY_DIR}
    )

    dedalus_add_python_test(
        NAME ppm_sequence_mp4_export
        LAYER synthetic
        LABELS integration
        COMMAND_ARGS
            ${PROJECT_SOURCE_DIR}/tests/integration/test_ppm_sequence_mp4_export.py
            ${PROJECT_SOURCE_DIR}
            ${PROJECT_BINARY_DIR}
    )

    dedalus_add_python_test(
        NAME replay_mission_smoke
        LAYER synthetic
        LABELS integration
        COMMAND_ARGS
            ${PROJECT_SOURCE_DIR}/tests/integration/test_replay_mission_smoke.py
            ${PROJECT_SOURCE_DIR}
            ${PROJECT_BINARY_DIR}
    )

    dedalus_add_python_test(
        NAME mission_artifact_validator
        LAYER synthetic
        LABELS integration
        COMMAND_ARGS
            ${PROJECT_SOURCE_DIR}/tests/integration/test_mission_artifact_validator.py
            ${PROJECT_SOURCE_DIR}
    )

    # Layer 4: archive-grade scenario/campaign tests.
    dedalus_add_python_test(
        NAME mission_scenario_runner
        LAYER scenario
        LABELS synthetic integration
        COMMAND_ARGS
            ${PROJECT_SOURCE_DIR}/tests/integration/test_mission_scenario_runner.py
            ${PROJECT_SOURCE_DIR}
            ${PROJECT_BINARY_DIR}
    )

    dedalus_add_python_test(
        NAME mission_campaign_runner
        LAYER scenario
        LABELS synthetic integration
        COMMAND_ARGS
            ${PROJECT_SOURCE_DIR}/tests/integration/test_mission_campaign_runner.py
            ${PROJECT_SOURCE_DIR}
            ${PROJECT_BINARY_DIR}
    )

    dedalus_add_python_test(
        NAME mission_abort_scenario
        LAYER scenario
        LABELS synthetic integration
        COMMAND_ARGS
            ${PROJECT_SOURCE_DIR}/tests/integration/test_mission_abort_scenario.py
            ${PROJECT_SOURCE_DIR}
            ${PROJECT_BINARY_DIR}
    )
endif()
