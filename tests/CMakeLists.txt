function(dedalus_add_native_test)
    set(options)
    set(one_value_args NAME LAYER)
    set(multi_value_args SOURCES TAGS)
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

    set_tests_properties("${display_name}"
        PROPERTIES
            LABELS "${DEDALUS_TEST_LAYER}"
            DEDALUS_LAYER "${DEDALUS_TEST_LAYER}"
            DEDALUS_TAGS "${DEDALUS_TEST_TAGS}"
    )
endfunction()

function(dedalus_add_python_test)
    set(options)
    set(one_value_args NAME LAYER)
    set(multi_value_args COMMAND_ARGS TAGS)
    cmake_parse_arguments(DEDALUS_TEST "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

    if(NOT DEDALUS_TEST_NAME)
        message(FATAL_ERROR "dedalus_add_python_test requires NAME")
    endif()
    if(NOT DEDALUS_TEST_LAYER)
        message(FATAL_ERROR "dedalus_add_python_test(${DEDALUS_TEST_NAME}) requires LAYER")
    endif()
    if(NOT DEDALUS_TEST_COMMAND_ARGS)
        message(FATAL_ERROR "dedalus_add_python_test requires COMMAND_ARGS")
    endif()

    set(display_name "${DEDALUS_TEST_NAME} (${DEDALUS_TEST_LAYER})")
    add_test(
        NAME "${display_name}"
        COMMAND ${Python3_EXECUTABLE} ${DEDALUS_TEST_COMMAND_ARGS}
    )

    set_tests_properties("${display_name}"
        PROPERTIES
            LABELS "${DEDALUS_TEST_LAYER}"
            DEDALUS_LAYER "${DEDALUS_TEST_LAYER}"
            DEDALUS_TAGS "${DEDALUS_TEST_TAGS}"
    )
endfunction()

# Validation layers execute in declaration order under plain `ctest`:
#   contracts -> unit -> synthetic -> scenario
#
# `LAYER` is the primary validation layer and is used for:
#   - CTest display name, e.g. `world_snapshot_json (contracts)`
#   - the only CTest LABELS value, so the Label Time Summary stays layer-only
#   - the DEDALUS_LAYER property for structured inspection tooling
#
# Optional TAGS are stored in the DEDALUS_TAGS property only. They are intentionally not
# CTest LABELS because CTest prints every label in the Label Time Summary. Keeping labels
# layer-only makes the default test report readable and makes `ctest -L <layer>` precise.
#
# No CTest test is labeled airsim_live today. Real live AirSim/PX4 validation remains an
# explicit operator-run smoke/campaign command outside normal CTest.

# Layer 1: contracts / protocols / boundaries.
dedalus_add_native_test(
    NAME world_snapshot_json
    LAYER contracts
    TAGS unit
    SOURCES unit/test_world_snapshot_json.cpp
)

dedalus_add_native_test(
    NAME behavior_spec
    LAYER contracts
    TAGS unit
    SOURCES unit/test_behavior_spec.cpp
)

dedalus_add_native_test(
    NAME target_selector
    LAYER contracts
    TAGS unit
    SOURCES unit/test_target_selector.cpp
)

dedalus_add_native_test(
    NAME core_stack_config_loader
    LAYER contracts
    TAGS unit
    SOURCES unit/test_core_stack_config_loader.cpp
)
set_tests_properties("core_stack_config_loader (contracts)"
    PROPERTIES
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

dedalus_add_native_test(
    NAME provider_composition
    LAYER contracts
    TAGS unit
    SOURCES unit/test_provider_composition.cpp
)

dedalus_add_native_test(
    NAME ppm_frame_annotation_sink
    LAYER contracts
    TAGS unit
    SOURCES unit/test_ppm_frame_annotation_sink.cpp
)

dedalus_add_native_test(
    NAME airsim_provider_boundary
    LAYER contracts
    TAGS unit airsim
    SOURCES unit/test_airsim_provider_boundary.cpp
)
set_tests_properties("airsim_provider_boundary (contracts)"
    PROPERTIES
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

dedalus_add_native_test(
    NAME airsim_velocity_command_sink
    LAYER contracts
    TAGS unit
    SOURCES unit/test_airsim_velocity_command_sink.cpp
)

dedalus_add_native_test(
    NAME px4_mavlink_command_sink
    LAYER contracts
    TAGS unit
    SOURCES unit/test_px4_mavlink_command_sink.cpp
)

# Layer 2: deterministic native unit tests, usually with synthetic in-memory data.
dedalus_add_native_test(
    NAME world_to_image_projector
    LAYER unit
    TAGS visualization reprojection
    SOURCES unit/test_world_to_image_projector.cpp
)

dedalus_add_native_test(
    NAME ghost_scenario
    LAYER unit
    TAGS synthetic simulation
    SOURCES unit/test_ghost_scenario.cpp
)
set_tests_properties("ghost_scenario (unit)"
    PROPERTIES
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

dedalus_add_native_test(
    NAME perception_world_model_flow
    LAYER unit
    TAGS synthetic
    SOURCES unit/test_perception_world_model_flow.cpp
)
set_tests_properties("perception_world_model_flow (unit)"
    PROPERTIES
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

dedalus_add_native_test(
    NAME video_only_world_model_flow
    LAYER unit
    TAGS synthetic
    SOURCES unit/test_video_only_world_model_flow.cpp
)

dedalus_add_native_test(
    NAME recorded_frame_world_model_flow
    LAYER unit
    TAGS synthetic
    SOURCES unit/test_recorded_frame_world_model_flow.cpp
)
set_tests_properties("recorded_frame_world_model_flow (unit)"
    PROPERTIES
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

dedalus_add_native_test(
    NAME pipeline_profiler
    LAYER unit
    TAGS synthetic
    SOURCES unit/test_pipeline_profiler.cpp
)

dedalus_add_native_test(
    NAME world_snapshot_publisher
    LAYER unit
    TAGS world_model publisher artifacts
    SOURCES unit/test_world_snapshot_publisher.cpp
)

dedalus_add_native_test(
    NAME world_snapshot_stream_server
    LAYER unit
    TAGS world_model stream tcp
    SOURCES unit/test_world_snapshot_stream_server.cpp
)

dedalus_add_native_test(
    NAME trajectory_mission_controller
    LAYER unit
    TAGS synthetic
    SOURCES unit/test_trajectory_mission_controller.cpp
)

dedalus_add_native_test(
    NAME object_behavior_mission_controller
    LAYER unit
    TAGS synthetic behavior
    SOURCES unit/test_object_behavior_mission_controller.cpp
)

dedalus_add_native_test(
    NAME mission_runtime
    LAYER unit
    TAGS synthetic
    SOURCES unit/test_mission_runtime.cpp
)

dedalus_add_native_test(
    NAME latest_world_snapshot
    LAYER unit
    TAGS synthetic
    SOURCES unit/test_latest_world_snapshot.cpp
)

find_package(Python3 COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
    # Layer 3: synthetic / recorded data and generated artifact tests.
    dedalus_add_python_test(
        NAME ghost_scenario_eval_cli
        LAYER synthetic
        TAGS integration simulation
        COMMAND_ARGS
            ${PROJECT_SOURCE_DIR}/tests/integration/test_ghost_scenario_eval_cli.py
            ${PROJECT_SOURCE_DIR}
            ${PROJECT_BINARY_DIR}
    )

    dedalus_add_python_test(
        NAME replay_recording_smoke
        LAYER synthetic
        TAGS integration
        COMMAND_ARGS
            ${PROJECT_SOURCE_DIR}/tests/integration/test_replay_recording_smoke.py
            ${PROJECT_SOURCE_DIR}
            ${PROJECT_BINARY_DIR}
    )

    dedalus_add_python_test(
        NAME replay_artifact_validator
        LAYER synthetic
        TAGS integration
        COMMAND_ARGS
            ${PROJECT_SOURCE_DIR}/tests/integration/test_replay_artifact_validator.py
            ${PROJECT_SOURCE_DIR}
            ${PROJECT_BINARY_DIR}
    )

    dedalus_add_python_test(
        NAME ppm_sequence_mp4_export
        LAYER synthetic
        TAGS integration
        COMMAND_ARGS
            ${PROJECT_SOURCE_DIR}/tests/integration/test_ppm_sequence_mp4_export.py
            ${PROJECT_SOURCE_DIR}
            ${PROJECT_BINARY_DIR}
    )

    dedalus_add_python_test(
        NAME world_reprojection_artifacts
        LAYER synthetic
        TAGS integration visualization reprojection
        COMMAND_ARGS
            ${PROJECT_SOURCE_DIR}/tests/integration/test_world_reprojection_artifacts.py
            ${PROJECT_SOURCE_DIR}
            ${PROJECT_BINARY_DIR}
    )

    dedalus_add_python_test(
        NAME object_behavior_mission_smoke
        LAYER synthetic
        TAGS integration behavior
        COMMAND_ARGS
            ${PROJECT_SOURCE_DIR}/tests/integration/test_object_behavior_mission_smoke.py
            ${PROJECT_SOURCE_DIR}
            ${PROJECT_BINARY_DIR}
    )

    dedalus_add_python_test(
        NAME replay_mission_smoke
        LAYER synthetic
        TAGS integration
        COMMAND_ARGS
            ${PROJECT_SOURCE_DIR}/tests/integration/test_replay_mission_smoke.py
            ${PROJECT_SOURCE_DIR}
            ${PROJECT_BINARY_DIR}
    )

    dedalus_add_python_test(
        NAME mission_artifact_validator
        LAYER synthetic
        TAGS integration
        COMMAND_ARGS
            ${PROJECT_SOURCE_DIR}/tests/integration/test_mission_artifact_validator.py
            ${PROJECT_SOURCE_DIR}
    )

    # Layer 4: archive-grade scenario/campaign tests.
    dedalus_add_python_test(
        NAME mission_scenario_runner
        LAYER scenario
        TAGS synthetic integration
        COMMAND_ARGS
            ${PROJECT_SOURCE_DIR}/tests/integration/test_mission_scenario_runner.py
            ${PROJECT_SOURCE_DIR}
            ${PROJECT_BINARY_DIR}
    )

    dedalus_add_python_test(
        NAME mission_campaign_runner
        LAYER scenario
        TAGS synthetic integration
        COMMAND_ARGS
            ${PROJECT_SOURCE_DIR}/tests/integration/test_mission_campaign_runner.py
            ${PROJECT_SOURCE_DIR}
            ${PROJECT_BINARY_DIR}
    )

    dedalus_add_python_test(
        NAME mission_abort_scenario
        LAYER scenario
        TAGS synthetic integration
        COMMAND_ARGS
            ${PROJECT_SOURCE_DIR}/tests/integration/test_mission_abort_scenario.py
            ${PROJECT_SOURCE_DIR}
            ${PROJECT_BINARY_DIR}
    )
endif()
