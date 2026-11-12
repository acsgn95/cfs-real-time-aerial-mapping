# Benchmark of dense reconstruction on Dorfberg keyframe batches (ADR-005
# section 6): time, peak GPU memory, and depth accuracy against the
# reference DSM.

find_package(GDAL CONFIG REQUIRED)
find_package(PROJ CONFIG REQUIRED)
find_package(CUDAToolkit REQUIRED)

# Defaults for the PROJ data and grid directories; both can be overridden on
# the command line.
find_file(ORTHOSTREAM_BENCH_PROJ_DB proj.db
    HINTS "${PROJ_DIR}" "${PROJ_DIR}/../../share/proj"
    NO_DEFAULT_PATH REQUIRED)
get_filename_component(ORTHOSTREAM_BENCH_PROJ_DATA_DIR "${ORTHOSTREAM_BENCH_PROJ_DB}" DIRECTORY)
get_filename_component(ORTHOSTREAM_BENCH_GRID_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../build-deps/proj-grids" ABSOLUTE)

add_library(dense_benchmark_core STATIC
    src/dataset.cpp
    src/error_stats.cpp
    src/gpu_memory.cpp
    src/raster.cpp)
target_include_directories(dense_benchmark_core PUBLIC src)
target_link_libraries(dense_benchmark_core
    PUBLIC OrthoStream::geo
    PRIVATE GDAL::GDAL CUDA::nvml orthostream_warnings)

add_executable(dense_benchmark src/main.cpp)
target_compile_definitions(dense_benchmark PRIVATE
    OST_BENCH_PROJ_DATA_DIR="${ORTHOSTREAM_BENCH_PROJ_DATA_DIR}"
    OST_BENCH_GRID_DIR="${ORTHOSTREAM_BENCH_GRID_DIR}")
target_link_libraries(dense_benchmark PRIVATE
    dense_benchmark_core orthostream_tool_support
    OrthoStream::geo OrthoStream::imaging OrthoStream::selection OrthoStream::dense
    orthostream_warnings)

if(BUILD_TESTING)
    add_executable(dense_benchmark_tests tests/test_core.cpp)
    target_link_libraries(dense_benchmark_tests PRIVATE dense_benchmark_core GTest::gtest_main orthostream_warnings)
    gtest_discover_tests(dense_benchmark_tests)
endif()
