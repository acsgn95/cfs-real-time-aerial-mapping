# SELECT_APP: footprints, keyframes, source views and the image pool
# (ADR-006, ADR-012). Links the C++ libraries OrthoStream::selection (with
# libs/terrain: GDAL, PROJ) and OrthoStream::imaging (libjpeg-turbo,
# OpenCV) statically, with their symbols kept inside the module (ADR-011
# section 8).
project(ORTHOSTREAM_SELECT_APP C CXX)

if(NOT CFE_EDS_ENABLED)
  message(FATAL_ERROR "select_app requires an EDS build (ADR-020)")
endif()

find_package(OrthoStreamSelection CONFIG REQUIRED)
find_package(OrthoStreamImaging CONFIG REQUIRED)

add_cfe_app(select_app fsw/src/select_app.c fsw/src/select_app_worker.c)
target_include_directories(select_app PUBLIC fsw/inc)
target_include_directories(select_app PRIVATE
  $<TARGET_PROPERTY:OrthoStream::selection,INTERFACE_INCLUDE_DIRECTORIES>
  $<TARGET_PROPERTY:OrthoStream::imaging,INTERFACE_INCLUDE_DIRECTORIES>
)
add_cfe_app_dependency(select_app ost_common)
add_cfe_app_dependency(select_app ost_datastore)

target_link_libraries(select_app OrthoStream::selection OrthoStream::imaging)
target_link_options(select_app PRIVATE "LINKER:--exclude-libs,ALL")
set_target_properties(select_app PROPERTIES LINKER_LANGUAGE CXX)

add_cfe_tables(select_app fsw/tables/select_app_tbl.c)
