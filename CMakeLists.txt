cmake_minimum_required(VERSION 3.22)

project(dedalus VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(DEDALUS_BUILD_APPS "Build Dedalus runtime/debug apps" ON)
option(DEDALUS_BUILD_TESTS "Build Dedalus unit tests" ON)

find_package(SQLite3 REQUIRED)
# Older/patched distro CMake packages find SQLite3 but don't create the
# imported target.  Create it from the plain variables if missing.
if(NOT TARGET SQLite3::SQLite3)
    add_library(SQLite3::SQLite3 UNKNOWN IMPORTED)
    set_target_properties(SQLite3::SQLite3 PROPERTIES
        IMPORTED_LOCATION             "${SQLite3_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${SQLite3_INCLUDE_DIRS}")
endif()

add_subdirectory(src)

if(DEDALUS_BUILD_APPS)
    add_subdirectory(apps)
endif()

if(DEDALUS_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
