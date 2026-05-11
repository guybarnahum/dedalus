add_executable(test_world_snapshot_json
    unit/test_world_snapshot_json.cpp
)

target_link_libraries(test_world_snapshot_json
    PRIVATE
        dedalus_core
)

target_compile_features(test_world_snapshot_json PRIVATE cxx_std_20)

add_test(NAME world_snapshot_json COMMAND test_world_snapshot_json)

add_executable(test_milestone_1c_pipeline
    unit/test_milestone_1c_pipeline.cpp
)

target_link_libraries(test_milestone_1c_pipeline
    PRIVATE
        dedalus_core
)

target_compile_features(test_milestone_1c_pipeline PRIVATE cxx_std_20)

add_test(NAME milestone_1c_pipeline COMMAND test_milestone_1c_pipeline)
