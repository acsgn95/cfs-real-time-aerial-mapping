# Pose refinement over a block of a recorded flight, without cFS (ADR-019).

add_executable(refine_poses src/main.cpp)
target_link_libraries(refine_poses PRIVATE
    orthostream_tool_support OrthoStream::imaging OrthoStream::refine orthostream_warnings)
