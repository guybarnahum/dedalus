add_library(dedalus_core
    sensors/synthetic_frame_source.cpp
    perception/perception_pipeline.cpp
    world_model/in_memory_world_model.cpp
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
