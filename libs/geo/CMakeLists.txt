# geo: frames, camera model, Metashape conversion and PROJ transformations
# (ADR-004, ADR-008, ADR-011).

find_package(Eigen3 CONFIG REQUIRED)
find_package(PROJ CONFIG REQUIRED)

add_library(orthostream_geo
    src/camera.cpp
    src/crs.cpp
    src/frames.cpp
    src/geo_c_api.cpp
    src/metashape.cpp
    src/triangulation.cpp)
add_library(OrthoStream::geo ALIAS orthostream_geo)
set_target_properties(orthostream_geo PROPERTIES EXPORT_NAME geo)

target_include_directories(orthostream_geo PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_features(orthostream_geo PUBLIC cxx_std_17)
target_link_libraries(orthostream_geo
    PUBLIC Eigen3::Eigen
    PRIVATE PROJ::proj $<BUILD_INTERFACE:orthostream_warnings>)

install(TARGETS orthostream_geo EXPORT OrthoStreamGeoTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT OrthoStreamGeoTargets
    NAMESPACE OrthoStream::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OrthoStreamGeo)
configure_package_config_file(cmake/OrthoStreamGeoConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamGeoConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OrthoStreamGeo)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamGeoConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMinorVersion)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamGeoConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamGeoConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OrthoStreamGeo)

if(BUILD_TESTING)
    # proj.db of the PROJ package in use; vcpkg installs it next to the
    # package configuration.
    find_file(ORTHOSTREAM_PROJ_DB proj.db
        HINTS "${PROJ_DIR}" "${PROJ_DIR}/../../share/proj"
        NO_DEFAULT_PATH REQUIRED)
    get_filename_component(ORTHOSTREAM_PROJ_DATA_DIR "${ORTHOSTREAM_PROJ_DB}" DIRECTORY)

    add_executable(geo_tests
        tests/c_api_usage.c
        tests/test_c_api.cpp
        tests/test_camera.cpp
        tests/test_crs.cpp
        tests/test_dorfberg_gcp.cpp
        tests/test_frames.cpp)
    target_compile_definitions(geo_tests PRIVATE
        OST_GEO_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data"
        OST_GEO_TEST_PROJ_DATA_DIR="${ORTHOSTREAM_PROJ_DATA_DIR}")
    # Grids fetched by scripts/fetch_proj_grids.sh; tests that need them are
    # skipped without them.
    set(ORTHOSTREAM_PROJ_GRID_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../build-deps/proj-grids"
        CACHE PATH "Directory with PROJ transformation grids")
    if(EXISTS "${ORTHOSTREAM_PROJ_GRID_DIR}/ch_swisstopo_chgeo2004_ETRS89_LHN95.tif")
        target_compile_definitions(geo_tests PRIVATE OST_GEO_TEST_GRID_DIR="${ORTHOSTREAM_PROJ_GRID_DIR}")
    endif()
    target_link_libraries(geo_tests PRIVATE
        orthostream_geo PROJ::proj GTest::gtest_main orthostream_warnings)
    gtest_discover_tests(geo_tests)
endif()
