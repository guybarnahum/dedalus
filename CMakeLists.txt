cmake_minimum_required(VERSION 3.22)

project(dedalus VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(DEDALUS_BUILD_APPS "Build Dedalus runtime/debug apps" ON)
option(DEDALUS_BUILD_TESTS "Build Dedalus unit tests" ON)
option(DEDALUS_ENABLE_ONNX_DEPTH "Build ONNXDepthEngine (requires ONNX Runtime)" OFF)

# CUDA depth kernels — auto-detected via CUDAToolkit probe.
# Override: -DDEDALUS_CUDA=ON (force on) or -DDEDALUS_CUDA=OFF (force off).
find_package(CUDAToolkit QUIET)
option(DEDALUS_CUDA "Enable CUDA depth kernels" ${CUDAToolkit_FOUND})
# TensorRT inference engine — requires DEDALUS_CUDA + TensorRT headers/libs.
option(DEDALUS_TENSORRT "Enable TensorRTDepthEngine (requires DEDALUS_CUDA + TensorRT)" OFF)

if(DEDALUS_CUDA)
    find_package(CUDAToolkit REQUIRED)  # promote to REQUIRED now that the option is ON
    enable_language(CUDA)
    set(CMAKE_CUDA_STANDARD 17)
    set(CMAKE_CUDA_STANDARD_REQUIRED ON)
    # Default: L4 (sm_89) + A100 (sm_80) + A10/RTX30xx (sm_86).
    # Override on the command line: -DCMAKE_CUDA_ARCHITECTURES=89
    if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
        set(CMAKE_CUDA_ARCHITECTURES "80;86;89" CACHE STRING
            "CUDA architectures to compile for")
    endif()
endif()

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
