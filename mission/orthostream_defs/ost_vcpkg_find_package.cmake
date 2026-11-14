######################################################################
#
# vcpkg package wrappers in the cFS build (ADR-010)
#
# The libraries (libs/) are built with vcpkg's toolchain, which runs the
# share/<package>/vcpkg-cmake-wrapper.cmake of a package when find_package
# looks for it: the wrappers add the link dependencies of static libraries
# (for example SQLite3::SQLite3 for PROJ, LZMA and JPEG for TIFF). The cFS
# build has no vcpkg toolchain, so modules that link GDAL or PROJ would
# miss them. This file installs the same find_package override, for the
# packages of build-deps only; all other packages are found as before.
#
######################################################################

get_property(OST_VCPKG_FIND_PACKAGE_SET GLOBAL PROPERTY OST_VCPKG_FIND_PACKAGE_SET)
if(NOT OST_VCPKG_FIND_PACKAGE_SET)
    set_property(GLOBAL PROPERTY OST_VCPKG_FIND_PACKAGE_SET TRUE)
    get_filename_component(OST_VCPKG_SHARE
        "${MISSION_DEFS}/../../build-deps/vcpkg_installed/x64-linux-orthostream/share" ABSOLUTE)
    set_property(GLOBAL PROPERTY OST_VCPKG_SHARE "${OST_VCPKG_SHARE}")

    # The wrappers read ARGS (package name and options) before they call
    # _find_package, so nested calls do not disturb them.
    macro(find_package ost_vcpkg_name)
        string(TOLOWER "${ost_vcpkg_name}" ost_vcpkg_lower)
        get_property(ost_vcpkg_share GLOBAL PROPERTY OST_VCPKG_SHARE)
        if(EXISTS "${ost_vcpkg_share}/${ost_vcpkg_lower}/vcpkg-cmake-wrapper.cmake")
            set(ARGS "${ost_vcpkg_name};${ARGN}")
            include("${ost_vcpkg_share}/${ost_vcpkg_lower}/vcpkg-cmake-wrapper.cmake")
        else()
            _find_package(${ost_vcpkg_name} ${ARGN})
        endif()
    endmacro()
endif()
