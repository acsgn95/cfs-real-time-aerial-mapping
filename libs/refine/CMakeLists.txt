# refine: optional pose refinement (ADR-019): image features, matching, tie
# point tracks and a bundle adjustment with telemetry priors. Built with
# ORTHOSTREAM_WITH_REFINE (vcpkg feature `refine`, GTSAM 4.3.0, ADR-010).

find_package(GTSAM CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED COMPONENTS core imgproc calib3d features2d flann)

add_library(orthostream_refine src/adjustment.cpp src/features.cpp src/refiner.cpp)
add_library(OrthoStream::refine ALIAS orthostream_refine)
set_target_properties(orthostream_refine PROPERTIES EXPORT_NAME refine)

target_include_directories(orthostream_refine PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_features(orthostream_refine PUBLIC cxx_std_17)
target_link_libraries(orthostream_refine
    PUBLIC OrthoStream::geo
    PRIVATE gtsam opencv_core opencv_imgproc opencv_calib3d opencv_features2d opencv_flann
            $<BUILD_INTERFACE:orthostream_warnings>)

install(TARGETS orthostream_refine EXPORT OrthoStreamRefineTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT OrthoStreamRefineTargets
    NAMESPACE OrthoStream::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OrthoStreamRefine)
configure_package_config_file(cmake/OrthoStreamRefineConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamRefineConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OrthoStreamRefine)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamRefineConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMinorVersion)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamRefineConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamRefineConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OrthoStreamRefine)

if(BUILD_TESTING)
    add_executable(refine_tests tests/test_refine.cpp)
    target_link_libraries(refine_tests PRIVATE orthostream_refine opencv_core opencv_imgproc
                          GTest::gtest_main orthostream_warnings)
    gtest_discover_tests(refine_tests)
endif()
