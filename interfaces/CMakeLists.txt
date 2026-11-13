# Shared C headers of the OrthoStream libraries and applications without a
# cFE dependency (ADR-020 section 5 and 6): status codes and diagnostics.
# Built as part of libs/ (libs/CMakeLists.txt adds this directory); the cFS
# library module apps/ost_common uses the same headers.

add_library(orthostream_interfaces INTERFACE)
add_library(OrthoStream::interfaces ALIAS orthostream_interfaces)
set_target_properties(orthostream_interfaces PROPERTIES EXPORT_NAME interfaces)
target_include_directories(orthostream_interfaces INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

install(TARGETS orthostream_interfaces EXPORT OrthoStreamInterfacesTargets)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT OrthoStreamInterfacesTargets
    NAMESPACE OrthoStream::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OrthoStreamInterfaces)
configure_package_config_file(cmake/OrthoStreamInterfacesConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamInterfacesConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OrthoStreamInterfaces)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamInterfacesConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMinorVersion)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamInterfacesConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/OrthoStreamInterfacesConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OrthoStreamInterfaces)
