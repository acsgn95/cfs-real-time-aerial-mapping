# datastore: shared data store for bulk data (ADR-013). Plain C++17 without
# third-party dependencies; the cFS library module apps/ost_datastore links it.

add_library(orthostream_datastore
    src/store.cpp
    src/datastore_c_api.cpp)
add_library(OrthoStream::datastore ALIAS orthostream_datastore)
set_target_properties(orthostream_datastore PROPERTIES EXPORT_NAME datastore)

target_include_directories(orthostream_datastore PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_features(orthostream_datastore PUBLIC cxx_std_17)
find_package(Threads REQUIRED)
target_link_libraries(orthostream_datastore
    PUBLIC OrthoStream::interfaces Threads::Threads
    PRIVATE $<BUILD_INTERFACE:orthostream_warnings>)

install(TARGETS orthostream_datastore EXPORT OrthoStreamDatastoreTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT OrthoStreamDatastoreTargets
    NAMESPACE OrthoStream::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OrthoStreamDatastore)
configure_package_config_file(cmake/OrthoStreamDatastoreConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamDatastoreConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OrthoStreamDatastore)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamDatastoreConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMinorVersion)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamDatastoreConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamDatastoreConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OrthoStreamDatastore)

if(BUILD_TESTING)
    add_executable(datastore_tests
        tests/c_api_usage.c
        tests/test_c_api.cpp
        tests/test_store.cpp)
    target_link_libraries(datastore_tests PRIVATE
        orthostream_datastore GTest::gtest_main orthostream_warnings)
    gtest_discover_tests(datastore_tests)
endif()
