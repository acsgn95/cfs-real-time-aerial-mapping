# imaging: JPEG decoding with DCT scaling, undistortion and resampling
# (ADR-001 section 1d, ADR-012 section 4).

find_package(libjpeg-turbo CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED COMPONENTS core imgproc calib3d)

if(TARGET libjpeg-turbo::turbojpeg-static)
    set(ORTHOSTREAM_TURBOJPEG libjpeg-turbo::turbojpeg-static)
else()
    set(ORTHOSTREAM_TURBOJPEG libjpeg-turbo::turbojpeg)
endif()

add_library(orthostream_imaging
    src/imaging_c_api.cpp
    src/jpeg.cpp
    src/undistort.cpp)
add_library(OrthoStream::imaging ALIAS orthostream_imaging)
set_target_properties(orthostream_imaging PROPERTIES EXPORT_NAME imaging)

target_include_directories(orthostream_imaging PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_features(orthostream_imaging PUBLIC cxx_std_17)
target_link_libraries(orthostream_imaging
    PUBLIC OrthoStream::geo
    PRIVATE ${ORTHOSTREAM_TURBOJPEG} opencv_core opencv_imgproc opencv_calib3d
            $<BUILD_INTERFACE:orthostream_warnings>)

install(TARGETS orthostream_imaging EXPORT OrthoStreamImagingTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT OrthoStreamImagingTargets
    NAMESPACE OrthoStream::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OrthoStreamImaging)
configure_package_config_file(cmake/OrthoStreamImagingConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamImagingConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OrthoStreamImaging)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamImagingConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMinorVersion)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamImagingConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamImagingConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OrthoStreamImaging)

if(BUILD_TESTING)
    add_executable(imaging_tests
        tests/c_api_usage.c
        tests/test_c_api.cpp
        tests/test_jpeg.cpp
        tests/test_undistort.cpp)
    target_include_directories(imaging_tests PRIVATE tests)
    target_link_libraries(imaging_tests PRIVATE
        orthostream_imaging ${ORTHOSTREAM_TURBOJPEG} GTest::gtest_main orthostream_warnings)
    gtest_discover_tests(imaging_tests)
endif()
