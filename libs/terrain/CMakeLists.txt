# terrain: the terrain prior of the image footprints (ADR-006 section 2):
# a window of a digital elevation model (GDAL) with heights converted to
# ellipsoidal heights (PROJ, through libs/geo).

find_package(GDAL CONFIG REQUIRED)

add_library(orthostream_terrain src/terrain.cpp)
add_library(OrthoStream::terrain ALIAS orthostream_terrain)
set_target_properties(orthostream_terrain PROPERTIES EXPORT_NAME terrain)

target_include_directories(orthostream_terrain PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_features(orthostream_terrain PUBLIC cxx_std_17)
target_link_libraries(orthostream_terrain
    PUBLIC OrthoStream::geo
    PRIVATE GDAL::GDAL $<BUILD_INTERFACE:orthostream_warnings>)

install(TARGETS orthostream_terrain EXPORT OrthoStreamTerrainTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT OrthoStreamTerrainTargets
    NAMESPACE OrthoStream::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OrthoStreamTerrain)
configure_package_config_file(cmake/OrthoStreamTerrainConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamTerrainConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OrthoStreamTerrain)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamTerrainConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMinorVersion)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamTerrainConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamTerrainConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OrthoStreamTerrain)

if(BUILD_TESTING)
    # proj.db found by libs/geo (cache variable ORTHOSTREAM_PROJ_DB).
    get_filename_component(OST_TERRAIN_PROJ_DATA_DIR "${ORTHOSTREAM_PROJ_DB}" DIRECTORY)
    add_executable(terrain_tests tests/test_terrain.cpp)
    target_compile_definitions(terrain_tests PRIVATE
        OST_TERRAIN_TEST_PROJ_DATA_DIR="${OST_TERRAIN_PROJ_DATA_DIR}"
        OST_TERRAIN_TEST_REPO="${CMAKE_CURRENT_SOURCE_DIR}/../..")
    target_link_libraries(terrain_tests PRIVATE orthostream_terrain GDAL::GDAL GTest::gtest_main orthostream_warnings)
    gtest_discover_tests(terrain_tests)
endif()
