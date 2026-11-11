# C++ tools that run the libraries without cFS (ADR-011 section 7). Built
# from libs/CMakeLists.txt with ORTHOSTREAM_BUILD_TOOLS.

if(ORTHOSTREAM_WITH_COLMAP)
    add_subdirectory(dense_benchmark)
else()
    message(STATUS "dense_benchmark needs ORTHOSTREAM_WITH_COLMAP; not built")
endif()
