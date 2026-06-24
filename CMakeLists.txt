cmake_minimum_required(VERSION 3.22)

project(dedalus VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(DEDALUS_BUILD_APPS "Build Dedalus runtime/debug apps" ON)
option(DEDALUS_BUILD_TESTS "Build Dedalus unit tests" ON)

find_package(SQLite3 REQUIRED)

add_subdirectory(src)

if(DEDALUS_BUILD_APPS)
    add_subdirectory(apps)
endif()

if(DEDALUS_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
