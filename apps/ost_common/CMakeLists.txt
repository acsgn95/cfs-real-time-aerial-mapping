# ost_common: shared interface library of the OrthoStream applications
# (ADR-020 section 1). A cFS library module (CFE_LIB) loaded before the
# applications. Its EDS file (eds/ost.xml) defines package OST.
project(ORTHOSTREAM_OST_COMMON C)

if (NOT CFE_EDS_ENABLED)
  message(FATAL_ERROR "ost_common requires a mission built with CFE_EDS_ENABLED=ON (ADR-020)")
endif ()

add_cfe_app(ost_common
  fsw/src/ost_common.c
  fsw/src/ost_fault.c
  fsw/src/ost_fault_catalog.c
)

# ost_status.h and ost_diag.h are shared with the C++ libraries under libs/
# and have no cFE dependency (interfaces/include).
target_include_directories(ost_common PUBLIC
  fsw/public_inc
  ${CMAKE_CURRENT_SOURCE_DIR}/../../interfaces/include
)
