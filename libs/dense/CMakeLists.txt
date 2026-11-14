# dense: depth estimation per keyframe batch (ADR-005). PatchMatch through
# COLMAP when ORTHOSTREAM_WITH_COLMAP is on; otherwise PatchMatch reports
# "not available" (ADR-011 section 5). The C interface for DENSE_APP adds
# the terrain prior of libs/terrain (depth range and terrain gate).

option(ORTHOSTREAM_WITH_COLMAP "Build GPU PatchMatch with COLMAP and CUDA" ON)

find_package(OpenCV CONFIG REQUIRED COMPONENTS core imgproc calib3d)
find_package(GDAL CONFIG REQUIRED)

add_library(orthostream_dense src/depth_estimator.cpp src/sgbm.cpp src/dense_c_api.cpp)
add_library(OrthoStream::dense ALIAS orthostream_dense)
set_target_properties(orthostream_dense PROPERTIES EXPORT_NAME dense)

target_include_directories(orthostream_dense PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_features(orthostream_dense PUBLIC cxx_std_17)
target_link_libraries(orthostream_dense
    PUBLIC OrthoStream::geo OrthoStream::interfaces
    PRIVATE OrthoStream::terrain opencv_core opencv_imgproc opencv_calib3d $<BUILD_INTERFACE:orthostream_warnings>)

if(ORTHOSTREAM_WITH_COLMAP)
    find_package(COLMAP CONFIG REQUIRED)
    find_package(CUDAToolkit REQUIRED)
    target_sources(orthostream_dense PRIVATE src/patch_match_colmap.cpp)
    target_link_libraries(orthostream_dense PRIVATE colmap::colmap_mvs colmap::colmap_mvs_cuda CUDA::cudart)
else()
    target_sources(orthostream_dense PRIVATE src/patch_match_unavailable.cpp)
endif()

install(TARGETS orthostream_dense EXPORT OrthoStreamDenseTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT OrthoStreamDenseTargets
    NAMESPACE OrthoStream::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OrthoStreamDense)
configure_package_config_file(cmake/OrthoStreamDenseConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamDenseConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OrthoStreamDense)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamDenseConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMinorVersion)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamDenseConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamDenseConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OrthoStreamDense)

if(BUILD_TESTING)
    get_filename_component(OST_DENSE_PROJ_DATA_DIR "${ORTHOSTREAM_PROJ_DB}" DIRECTORY)
    add_executable(dense_tests tests/test_patch_match.cpp tests/test_dense_c_api.cpp tests/c_api_usage.c)
    target_compile_definitions(dense_tests PRIVATE OST_DENSE_TEST_PROJ_DATA_DIR="${OST_DENSE_PROJ_DATA_DIR}")
    target_link_libraries(dense_tests PRIVATE orthostream_dense GDAL::GDAL GTest::gtest_main orthostream_warnings)
    if(ORTHOSTREAM_WITH_COLMAP)
        # The test checks for a GPU before running PatchMatch: COLMAP ends the
        # process on CUDA errors.
        find_package(CUDAToolkit REQUIRED)
        target_link_libraries(dense_tests PRIVATE CUDA::cudart)
        target_compile_definitions(dense_tests PRIVATE OST_DENSE_TEST_CUDA)
    endif()
    gtest_discover_tests(dense_tests)
endif()
