add_library(dedalus_core
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
