# link: decoding of the injector's UDP telemetry and TCP image frames
# (ADR-003, ADR-012, ADR-016 section 7). No dependency besides the shared
# interface headers.

add_library(orthostream_link src/link_c_api.cpp)
add_library(OrthoStream::link ALIAS orthostream_link)
set_target_properties(orthostream_link PROPERTIES EXPORT_NAME link)

target_include_directories(orthostream_link PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_features(orthostream_link PUBLIC cxx_std_17)
target_link_libraries(orthostream_link
    PUBLIC OrthoStream::interfaces
    PRIVATE $<BUILD_INTERFACE:orthostream_warnings>)

install(TARGETS orthostream_link EXPORT OrthoStreamLinkTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT OrthoStreamLinkTargets
    NAMESPACE OrthoStream::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OrthoStreamLink)
configure_package_config_file(cmake/OrthoStreamLinkConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamLinkConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OrthoStreamLink)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamLinkConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMinorVersion)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamLinkConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamLinkConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OrthoStreamLink)

if(BUILD_TESTING)
    add_executable(link_tests tests/c_api_usage.c tests/test_link.cpp)
    target_compile_definitions(link_tests PRIVATE
        OST_LINK_TEST_VECTORS="${CMAKE_CURRENT_SOURCE_DIR}/../../interfaces/link/test_vectors.txt")
    target_link_libraries(link_tests PRIVATE orthostream_link GTest::gtest_main orthostream_warnings)
    gtest_discover_tests(link_tests)
endif()
