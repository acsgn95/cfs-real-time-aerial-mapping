######################################################################
#
# OrthoStream: install step for cpu1 (ADR-009)
#
# Derived from vendor/cfs/sample_defs/cpu1/install_custom.cmake, reduced
# to the applications this mission builds.
#
######################################################################

# Container start script for native builds
if (${SIMULATION} MATCHES "^native")
    install(PROGRAMS ${CMAKE_CURRENT_LIST_DIR}/container-start DESTINATION cpu1)
endif()

# The lab application tables may refer to the message IDs of the other lab
# applications
set(APP_INCLUDE_TABLE_LIST
    $<TARGET_PROPERTY:ci_lab,INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:to_lab,INCLUDE_DIRECTORIES>
)
target_include_directories(to_lab.table INTERFACE ${APP_INCLUDE_TABLE_LIST})
target_include_directories(sch_lab.table INTERFACE ${APP_INCLUDE_TABLE_LIST})

# Startup script generated from the application list of targets.cmake
install(SCRIPT ${MISSION_DEFS}/generate_startup.cmake)
install(CODE "generate_cfs_startup_script(\"${TGTNAME}/${INSTALL_SUBDIR}\" ${${TGTNAME}_APPLIST})")
