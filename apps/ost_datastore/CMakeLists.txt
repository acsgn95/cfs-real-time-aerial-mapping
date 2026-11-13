# ost_datastore: the shared data store (ADR-013) as a cFS library module
# (CFE_LIB). It links the C++ library OrthoStream::datastore statically and
# exports only its own C functions OST_DS_* (ADR-011 section 8).
project(ORTHOSTREAM_OST_DATASTORE C CXX)

find_package(OrthoStreamDatastore CONFIG REQUIRED)

add_cfe_app(ost_datastore fsw/src/ost_datastore.c)
# datastore_c_api.h (types and statistics) is part of the public interface.
target_include_directories(ost_datastore PUBLIC
  fsw/public_inc
  $<TARGET_PROPERTY:OrthoStream::datastore,INTERFACE_INCLUDE_DIRECTORIES>
)
add_cfe_app_dependency(ost_datastore ost_common)

target_link_libraries(ost_datastore OrthoStream::datastore)
# Symbols of the static archives stay inside this module.
target_link_options(ost_datastore PRIVATE "LINKER:--exclude-libs,ALL")
set_target_properties(ost_datastore PROPERTIES LINKER_LANGUAGE CXX)
