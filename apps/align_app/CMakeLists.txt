# ALIGN_APP: time alignment of the images with the telemetry stream
# (ADR-012, ADR-014). Links the C++ library OrthoStream::time_align
# statically, with its symbols kept inside the module (ADR-011 section 8).
project(ORTHOSTREAM_ALIGN_APP C CXX)

if(NOT CFE_EDS_ENABLED)
  message(FATAL_ERROR "align_app requires an EDS build (ADR-020)")
endif()

find_package(OrthoStreamTimeAlign CONFIG REQUIRED)

add_cfe_app(align_app fsw/src/align_app.c)
target_include_directories(align_app PUBLIC fsw/inc)
target_include_directories(align_app PRIVATE
  $<TARGET_PROPERTY:OrthoStream::time_align,INTERFACE_INCLUDE_DIRECTORIES>
)
add_cfe_app_dependency(align_app ost_common)
add_cfe_app_dependency(align_app ost_datastore)

target_link_libraries(align_app OrthoStream::time_align)
target_link_options(align_app PRIVATE "LINKER:--exclude-libs,ALL")
set_target_properties(align_app PROPERTIES LINKER_LANGUAGE CXX)

add_cfe_tables(align_app fsw/tables/align_app_tbl.c)
