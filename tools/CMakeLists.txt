# C++ tools that run the libraries without cFS (ADR-011 section 7). Built
# from libs/CMakeLists.txt with ORTHOSTREAM_BUILD_TOOLS.

add_subdirectory(common)
if(ORTHOSTREAM_WITH_COLMAP)
    add_subdirectory(dense_benchmark)
    add_subdirectory(offline_pipeline)
else()
    message(STATUS "dense_benchmark and offline_pipeline need ORTHOSTREAM_WITH_COLMAP; not built")
endif()
