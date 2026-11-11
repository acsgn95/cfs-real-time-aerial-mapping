# map_products: DSM fusion, orthomosaic, point cloud and tiles (ADR-007).

find_package(GDAL CONFIG REQUIRED)

add_library(orthostream_map_products src/product_frame.cpp src/dsm.cpp src/writer.cpp)
add_library(OrthoStream::map_products ALIAS orthostream_map_products)
set_target_properties(orthostream_map_products PROPERTIES EXPORT_NAME map_products)

target_include_directories(orthostream_map_products PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_features(orthostream_map_products PUBLIC cxx_std_17)
target_link_libraries(orthostream_map_products
    PUBLIC OrthoStream::geo
    PRIVATE GDAL::GDAL $<BUILD_INTERFACE:orthostream_warnings>)

install(TARGETS orthostream_map_products EXPORT OrthoStreamMapProductsTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT OrthoStreamMapProductsTargets
    NAMESPACE OrthoStream::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OrthoStreamMapProducts)
configure_package_config_file(cmake/OrthoStreamMapProductsConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamMapProductsConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OrthoStreamMapProducts)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamMapProductsConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMinorVersion)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamMapProductsConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamMapProductsConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OrthoStreamMapProducts)

if(BUILD_TESTING)
    # proj.db found by libs/geo (cache variable ORTHOSTREAM_PROJ_DB).
    get_filename_component(OST_MP_PROJ_DATA_DIR "${ORTHOSTREAM_PROJ_DB}" DIRECTORY)
    add_executable(map_products_tests tests/test_product_frame.cpp tests/test_dsm.cpp tests/test_writer.cpp)
    target_compile_definitions(map_products_tests PRIVATE OST_MP_TEST_PROJ_DATA_DIR="${OST_MP_PROJ_DATA_DIR}")
    if(EXISTS "${ORTHOSTREAM_PROJ_GRID_DIR}/ch_swisstopo_chgeo2004_ETRS89_LHN95.tif")
        target_compile_definitions(map_products_tests PRIVATE OST_MP_TEST_GRID_DIR="${ORTHOSTREAM_PROJ_GRID_DIR}")
    endif()
    target_link_libraries(map_products_tests PRIVATE
        orthostream_map_products GDAL::GDAL GTest::gtest_main orthostream_warnings)
    gtest_discover_tests(map_products_tests)
endif()
