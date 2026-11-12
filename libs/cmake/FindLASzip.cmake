# FindLASzip: the LASzip C API (laszip_api.h) built by vcpkg, which ships no
# CMake package configuration. Defines the imported target LASzip::LASzip.
#
# liblaszip contains the implementation of the C API. liblaszip_api is only a
# loader that opens a shared LASzip with dlopen at run time; it is not used,
# because the libraries are linked statically (ADR-010).

find_path(LASzip_INCLUDE_DIR laszip/laszip_api.h)
find_library(LASzip_LIBRARY NAMES laszip)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LASzip REQUIRED_VARS LASzip_INCLUDE_DIR LASzip_LIBRARY)

if(LASzip_FOUND AND NOT TARGET LASzip::LASzip)
    add_library(LASzip::LASzip UNKNOWN IMPORTED)
    set_target_properties(LASzip::LASzip PROPERTIES
        IMPORTED_LOCATION "${LASzip_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${LASzip_INCLUDE_DIR}"
        INTERFACE_COMPILE_DEFINITIONS "LASZIP_API_VERSION")
endif()
mark_as_advanced(LASzip_INCLUDE_DIR LASzip_LIBRARY)
