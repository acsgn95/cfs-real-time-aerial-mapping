# time_align: association of images with the telemetry stream (ADR-011,
# ADR-014 section 4). Depends only on Eigen: the geodetic conversion is the
# closed form, so ALIGN_APP does not carry PROJ.

find_package(Eigen3 CONFIG REQUIRED)

add_library(orthostream_time_align src/aligner.cpp src/time_align_c_api.cpp)
add_library(OrthoStream::time_align ALIAS orthostream_time_align)
set_target_properties(orthostream_time_align PROPERTIES EXPORT_NAME time_align)

target_include_directories(orthostream_time_align PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_features(orthostream_time_align PUBLIC cxx_std_17)
target_link_libraries(orthostream_time_align
    PUBLIC Eigen3::Eigen OrthoStream::interfaces
    PRIVATE $<BUILD_INTERFACE:orthostream_warnings>)

install(TARGETS orthostream_time_align EXPORT OrthoStreamTimeAlignTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT OrthoStreamTimeAlignTargets
    NAMESPACE OrthoStream::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OrthoStreamTimeAlign)
configure_package_config_file(cmake/OrthoStreamTimeAlignConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamTimeAlignConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OrthoStreamTimeAlign)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamTimeAlignConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMinorVersion)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamTimeAlignConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamTimeAlignConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OrthoStreamTimeAlign)

if(BUILD_TESTING)
    add_executable(time_align_tests tests/c_api_usage.c tests/test_aligner.cpp)
    target_link_libraries(time_align_tests PRIVATE orthostream_time_align GTest::gtest_main orthostream_warnings m)
    gtest_discover_tests(time_align_tests)
endif()
