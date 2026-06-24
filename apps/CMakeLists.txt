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

add_executable(dedalus_replay_recording
    dedalus_replay_recording.cpp
)

target_link_libraries(dedalus_replay_recording
    PRIVATE
        dedalus_core
)

target_compile_features(dedalus_replay_recording PRIVATE cxx_std_20)

add_executable(dedalus_mission_loop
    dedalus_mission_loop.cpp
)

target_link_libraries(dedalus_mission_loop
    PRIVATE
        dedalus_core
)

target_compile_features(dedalus_mission_loop PRIVATE cxx_std_20)

add_executable(dedalus_ghost_scenario_eval
    dedalus_ghost_scenario_eval.cpp
)

target_link_libraries(dedalus_ghost_scenario_eval
    PRIVATE
        dedalus_core
)

target_compile_features(dedalus_ghost_scenario_eval PRIVATE cxx_std_20)

# Standalone live/replay viewer — connects to the runtime TCP event stream and
# relays to browser clients via HTTP/SSE.  Intentionally has no dedalus_core
# dependency: it is a pure POSIX string relay and must remain deployable on any
# host without the full drone stack.
add_executable(dedalus_viewer
    dedalus_viewer.cpp
)

# SQLite3 is the only external dep — intentionally no dedalus_core link so the
# viewer binary remains deployable on any host without the full drone stack.
target_link_libraries(dedalus_viewer PRIVATE SQLite3::SQLite3)

target_compile_features(dedalus_viewer PRIVATE cxx_std_20)

target_compile_options(dedalus_viewer PRIVATE -Wall -Wextra -Wpedantic)
