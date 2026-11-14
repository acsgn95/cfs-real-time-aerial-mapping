# selection: image footprints, overlap, keyframe and source view selection
# (ADR-006), and the incremental selector with its image pool and C
# interface for SELECT_APP (terrain prior from libs/terrain).

find_package(GDAL CONFIG REQUIRED)

add_library(orthostream_selection src/footprint.cpp src/views.cpp src/selector.cpp src/selection_c_api.cpp)
add_library(OrthoStream::selection ALIAS orthostream_selection)
set_target_properties(orthostream_selection PROPERTIES EXPORT_NAME selection)

target_include_directories(orthostream_selection PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_features(orthostream_selection PUBLIC cxx_std_17)
target_link_libraries(orthostream_selection
    PUBLIC OrthoStream::geo OrthoStream::interfaces
    PRIVATE OrthoStream::terrain $<BUILD_INTERFACE:orthostream_warnings>)

install(TARGETS orthostream_selection EXPORT OrthoStreamSelectionTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT OrthoStreamSelectionTargets
    NAMESPACE OrthoStream::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OrthoStreamSelection)
configure_package_config_file(cmake/OrthoStreamSelectionConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamSelectionConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OrthoStreamSelection)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamSelectionConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMinorVersion)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamSelectionConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamSelectionConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OrthoStreamSelection)

if(BUILD_TESTING)
    get_filename_component(OST_SELECTION_PROJ_DATA_DIR "${ORTHOSTREAM_PROJ_DB}" DIRECTORY)
    add_executable(selection_tests
        tests/c_api_usage.c tests/test_footprint.cpp tests/test_views.cpp tests/test_selector.cpp)
    target_compile_definitions(selection_tests PRIVATE
        OST_SELECTION_TEST_PROJ_DATA_DIR="${OST_SELECTION_PROJ_DATA_DIR}")
    target_link_libraries(selection_tests PRIVATE
        orthostream_selection GDAL::GDAL GTest::gtest_main orthostream_warnings)
    gtest_discover_tests(selection_tests)
endif()
