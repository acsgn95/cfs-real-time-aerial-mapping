# selection: image footprints, overlap, keyframe and source view selection
# (ADR-006). The spatial index of the image pool follows with SELECT_APP.

add_library(orthostream_selection src/footprint.cpp src/views.cpp)
add_library(OrthoStream::selection ALIAS orthostream_selection)
set_target_properties(orthostream_selection PROPERTIES EXPORT_NAME selection)

target_include_directories(orthostream_selection PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_features(orthostream_selection PUBLIC cxx_std_17)
target_link_libraries(orthostream_selection
    PUBLIC OrthoStream::geo
    PRIVATE $<BUILD_INTERFACE:orthostream_warnings>)

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
    add_executable(selection_tests tests/test_footprint.cpp tests/test_views.cpp)
    target_link_libraries(selection_tests PRIVATE orthostream_selection GTest::gtest_main orthostream_warnings)
    gtest_discover_tests(selection_tests)
endif()
