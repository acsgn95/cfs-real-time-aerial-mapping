# Support code shared by the host tools: pose files, rasters, error
# statistics, and keyframe selection over a recorded flight.

find_package(GDAL CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED COMPONENTS core imgproc imgcodecs)

get_filename_component(OST_TOOLS_PROJ_DATA_DIR "${ORTHOSTREAM_PROJ_DB}" DIRECTORY)
get_filename_component(OST_TOOLS_GRID_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../build-deps/proj-grids" ABSOLUTE)

add_library(orthostream_tool_support STATIC
    src/dataset.cpp
    src/error_stats.cpp
    src/flight.cpp
    src/raster.cpp
    src/visual.cpp)
target_include_directories(orthostream_tool_support PUBLIC src)
target_compile_definitions(orthostream_tool_support PUBLIC
    OST_TOOLS_PROJ_DATA_DIR="${OST_TOOLS_PROJ_DATA_DIR}"
    OST_TOOLS_GRID_DIR="${OST_TOOLS_GRID_DIR}")
target_link_libraries(orthostream_tool_support
    PUBLIC OrthoStream::geo OrthoStream::selection opencv_core
    PRIVATE GDAL::GDAL opencv_imgproc opencv_imgcodecs orthostream_warnings)

if(BUILD_TESTING)
    add_executable(tool_support_tests tests/test_core.cpp)
    target_link_libraries(tool_support_tests PRIVATE orthostream_tool_support GTest::gtest_main orthostream_warnings)
    gtest_discover_tests(tool_support_tests TEST_PREFIX "tools.")
endif()
