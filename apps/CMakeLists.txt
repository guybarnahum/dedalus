add_executable(dedalus_core_stack
    dedalus_core_stack.cpp
)

target_link_libraries(dedalus_core_stack
    PRIVATE
        dedalus_core
)

target_compile_features(dedalus_core_stack PRIVATE cxx_std_20)
