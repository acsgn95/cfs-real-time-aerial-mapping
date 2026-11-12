# The mapping pipeline on a recorded flight without cFS (ADR-011 section 7).

add_executable(offline_pipeline src/main.cpp)
target_link_libraries(offline_pipeline PRIVATE
    orthostream_tool_support
    OrthoStream::imaging OrthoStream::dense OrthoStream::map_products
    orthostream_warnings)
