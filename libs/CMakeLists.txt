# OrthoStream algorithm libraries (ADR-011). They build and test without cFS:
#
#   scripts/build_libs.sh
#
# Third-party packages come from the vcpkg install prefix (ADR-010), passed
# as CMAKE_PREFIX_PATH.
cmake_minimum_required(VERSION 3.20)

project(OrthoStreamLibs
    VERSION 0.1.0
    DESCRIPTION "OrthoStream algorithm libraries"
    LANGUAGES C CXX)

include(CTest)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(ORTHOSTREAM_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" ON)
option(ORTHOSTREAM_BUILD_TOOLS "Build the C++ tools under tools/" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
# cFS applications are shared modules; the libraries linked into them must be
# position-independent (ADR-010).
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library(orthostream_warnings INTERFACE)
target_compile_options(orthostream_warnings INTERFACE
    -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion
    $<$<BOOL:${ORTHOSTREAM_WARNINGS_AS_ERRORS}>:-Werror>)

if(BUILD_TESTING)
    find_package(GTest CONFIG REQUIRED)
    include(GoogleTest)
endif()

add_subdirectory(geo)
add_subdirectory(imaging)
add_subdirectory(selection)
add_subdirectory(dense)

if(ORTHOSTREAM_BUILD_TOOLS)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../tools ${CMAKE_CURRENT_BINARY_DIR}/tools)
endif()
