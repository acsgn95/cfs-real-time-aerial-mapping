# RECORDER_APP: measurement records of a run (ADR-012, ADR-017). No
# OrthoStream library: messages are encoded with the mission's EDS runtime,
# as TO_LAB encodes them for the ground link.
project(ORTHOSTREAM_RECORDER_APP C)

if(NOT CFE_EDS_ENABLED)
  message(FATAL_ERROR "recorder_app requires an EDS build (ADR-020)")
endif()

add_cfe_app(recorder_app fsw/src/recorder_app.c fsw/src/recorder_app_file.c fsw/src/recorder_app_encode.c)
target_include_directories(recorder_app PUBLIC fsw/inc)
add_cfe_app_dependency(recorder_app ost_common)

add_cfe_tables(recorder_app fsw/tables/recorder_app_tbl.c)
