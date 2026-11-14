# DENSE_APP: depth maps of the keyframe batches (ADR-005, ADR-012). Links the
# C++ library OrthoStream::dense (OpenCV; COLMAP and CUDA when built with
# PatchMatch) statically, with its symbols kept inside the module (ADR-011
# section 8).
project(ORTHOSTREAM_DENSE_APP C CXX)

if(NOT CFE_EDS_ENABLED)
  message(FATAL_ERROR "dense_app requires an EDS build (ADR-020)")
endif()

find_package(OrthoStreamDense CONFIG REQUIRED)

add_cfe_app(dense_app fsw/src/dense_app.c fsw/src/dense_app_worker.c)
target_include_directories(dense_app PUBLIC fsw/inc)
target_include_directories(dense_app PRIVATE
  $<TARGET_PROPERTY:OrthoStream::dense,INTERFACE_INCLUDE_DIRECTORIES>
)
add_cfe_app_dependency(dense_app ost_common)
add_cfe_app_dependency(dense_app ost_datastore)

target_link_libraries(dense_app OrthoStream::dense)
target_link_options(dense_app PRIVATE "LINKER:--exclude-libs,ALL")
set_target_properties(dense_app PROPERTIES LINKER_LANGUAGE CXX)

add_cfe_tables(dense_app fsw/tables/dense_app_tbl.c)
