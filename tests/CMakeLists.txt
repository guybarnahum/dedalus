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
