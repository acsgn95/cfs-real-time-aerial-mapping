# MAPPROD_APP: DSM and orthomosaic tiles (ADR-007, ADR-012). Links the C++
# libraries OrthoStream::map_products (GDAL, PROJ, LASzip) and
# OrthoStream::imaging (libjpeg-turbo, OpenCV) statically, with their
# symbols kept inside the module (ADR-011 section 8).
project(ORTHOSTREAM_MAPPROD_APP C CXX)

if(NOT CFE_EDS_ENABLED)
  message(FATAL_ERROR "mapprod_app requires an EDS build (ADR-020)")
endif()

find_package(OrthoStreamMapProducts CONFIG REQUIRED)
find_package(OrthoStreamImaging CONFIG REQUIRED)

add_cfe_app(mapprod_app fsw/src/mapprod_app.c fsw/src/mapprod_app_worker.c)
target_include_directories(mapprod_app PUBLIC fsw/inc)
target_include_directories(mapprod_app PRIVATE
  $<TARGET_PROPERTY:OrthoStream::map_products,INTERFACE_INCLUDE_DIRECTORIES>
  $<TARGET_PROPERTY:OrthoStream::imaging,INTERFACE_INCLUDE_DIRECTORIES>
)
add_cfe_app_dependency(mapprod_app ost_common)
add_cfe_app_dependency(mapprod_app ost_datastore)

target_link_libraries(mapprod_app OrthoStream::map_products OrthoStream::imaging)
target_link_options(mapprod_app PRIVATE "LINKER:--exclude-libs,ALL")
set_target_properties(mapprod_app PROPERTIES LINKER_LANGUAGE CXX)

add_cfe_tables(mapprod_app fsw/tables/mapprod_app_tbl.c)
