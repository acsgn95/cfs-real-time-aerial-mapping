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

# Geodata for the terrain prior of SELECT_APP (ADR-006 section 2), under
# /cf/geodata as its default table expects: proj.db of the PROJ package in
# use, the EGM2008 grid (scripts/fetch_proj_grids.sh) and the Copernicus
# mosaic of the mission area (tools/dataset/fetch_dem.py), installed as
# terrain.vrt. Missing files are left out; SELECT_APP then reports
# TERRAIN_UNAVAILABLE.
get_filename_component(OST_REPO_ROOT "${MISSION_DEFS}/../.." ABSOLUTE)
set(OST_DEPS_PREFIX "${OST_REPO_ROOT}/build-deps/vcpkg_installed/x64-linux-orthostream")
set(OST_DEM_DIR "${OST_REPO_ROOT}/data/raw/dem/cop30")
install(FILES "${OST_DEPS_PREFIX}/share/proj/proj.db" DESTINATION cpu1/cf/geodata/proj OPTIONAL)
install(FILES "${OST_REPO_ROOT}/build-deps/proj-grids/us_nga_egm08_25.tif" DESTINATION cpu1/cf/geodata/grids OPTIONAL)
install(FILES "${OST_DEM_DIR}/dorfberg.vrt" DESTINATION cpu1/cf/geodata/dem RENAME terrain.vrt OPTIONAL)
if(EXISTS "${OST_DEM_DIR}/tiles")
    install(DIRECTORY "${OST_DEM_DIR}/tiles" DESTINATION cpu1/cf/geodata/dem)
endif()
