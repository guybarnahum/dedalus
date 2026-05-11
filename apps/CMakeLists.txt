add_executable(dedalus_core_stack
    dedalus_core_stack.cpp
)

target_link_libraries(dedalus_core_stack
    PRIVATE
        dedalus_core
)

target_compile_features(dedalus_core_stack PRIVATE cxx_std_20)

add_executable(dedalus_dump_world
    dedalus_dump_world.cpp
)

target_link_libraries(dedalus_dump_world
    PRIVATE
        dedalus_core
)

target_compile_features(dedalus_dump_world PRIVATE cxx_std_20)
