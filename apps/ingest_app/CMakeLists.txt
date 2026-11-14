# INGEST_APP: source adapter for the injector link (ADR-003, ADR-012).
# Links the C++ library OrthoStream::link statically, with its symbols kept
# inside the module (ADR-011 section 8).
project(ORTHOSTREAM_INGEST_APP C CXX)

if(NOT CFE_EDS_ENABLED)
  message(FATAL_ERROR "ingest_app requires an EDS build (ADR-020)")
endif()

find_package(OrthoStreamLink CONFIG REQUIRED)

add_cfe_app(ingest_app
  fsw/src/ingest_app.c
  fsw/src/ingest_app_net.c
)
target_include_directories(ingest_app PUBLIC fsw/inc)
target_include_directories(ingest_app PRIVATE
  $<TARGET_PROPERTY:OrthoStream::link,INTERFACE_INCLUDE_DIRECTORIES>
)
add_cfe_app_dependency(ingest_app ost_common)
add_cfe_app_dependency(ingest_app ost_datastore)

target_link_libraries(ingest_app OrthoStream::link)
target_link_options(ingest_app PRIVATE "LINKER:--exclude-libs,ALL")
set_target_properties(ingest_app PROPERTIES LINKER_LANGUAGE CXX)

add_cfe_tables(ingest_app fsw/tables/ingest_app_tbl.c)
