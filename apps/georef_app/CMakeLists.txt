# GEOREF_APP: georeferencing of the depth sets (ADR-007, ADR-012). Links the
# C++ library OrthoStream::map_products (PROJ through libs/geo, LASzip)
# statically, with its symbols kept inside the module (ADR-011 section 8).
project(ORTHOSTREAM_GEOREF_APP C CXX)

if(NOT CFE_EDS_ENABLED)
  message(FATAL_ERROR "georef_app requires an EDS build (ADR-020)")
endif()

find_package(OrthoStreamMapProducts CONFIG REQUIRED)

add_cfe_app(georef_app fsw/src/georef_app.c fsw/src/georef_app_worker.c)
target_include_directories(georef_app PUBLIC fsw/inc)
target_include_directories(georef_app PRIVATE
  $<TARGET_PROPERTY:OrthoStream::map_products,INTERFACE_INCLUDE_DIRECTORIES>
)
add_cfe_app_dependency(georef_app ost_common)
add_cfe_app_dependency(georef_app ost_datastore)

target_link_libraries(georef_app OrthoStream::map_products)
target_link_options(georef_app PRIVATE "LINKER:--exclude-libs,ALL")
set_target_properties(georef_app PROPERTIES LINKER_LANGUAGE CXX)

add_cfe_tables(georef_app fsw/tables/georef_app_tbl.c)
