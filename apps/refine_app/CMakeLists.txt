# REFINE_APP: pose refinement of the keyframe batches (ADR-019). Links the
# C++ library OrthoStream::refine (GTSAM, OpenCV) statically, with its
# symbols kept inside the module (ADR-011 section 8).
project(ORTHOSTREAM_REFINE_APP C CXX)

if(NOT CFE_EDS_ENABLED)
  message(FATAL_ERROR "refine_app requires an EDS build (ADR-020)")
endif()

find_package(OrthoStreamRefine CONFIG REQUIRED)

add_cfe_app(refine_app fsw/src/refine_app.c fsw/src/refine_app_worker.c)
target_include_directories(refine_app PUBLIC fsw/inc)
target_include_directories(refine_app PRIVATE
  $<TARGET_PROPERTY:OrthoStream::refine,INTERFACE_INCLUDE_DIRECTORIES>
)
add_cfe_app_dependency(refine_app ost_common)
add_cfe_app_dependency(refine_app ost_datastore)

target_link_libraries(refine_app OrthoStream::refine)
target_link_options(refine_app PRIVATE "LINKER:--exclude-libs,ALL")
set_target_properties(refine_app PROPERTIES LINKER_LANGUAGE CXX)

add_cfe_tables(refine_app fsw/tables/refine_app_tbl.c)
