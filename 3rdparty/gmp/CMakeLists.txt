#
# Copyright 2023, Technology Innovation Institute
#
# SPDX-License-Identifier: BSD-2-Clause
#

cmake_minimum_required(VERSION 3.7.2)

project(libgmp NONE)
include(ExternalProject)

# Some helper variables
set(GMP_EP_NAME            "${PROJECT_NAME}_ep" CACHE STRING "" FORCE)
set(GMP_EP_BASE            "${PROJECT_NAME}_ep" CACHE STRING "" FORCE)
set(GMP_EP_BUILD_IN_SOURCE TRUE CACHE BOOL "" FORCE)

# GMP version
set(GMP_EP_VERSION         "6.2.1" CACHE STRING "GMP library version" FORCE)
set(GMP_EP_VERSION_HASH    "SHA256=2c7f4f0d370801b2849c48c9ef3f59553b5f1d3791d070cffb04599f9fc67b41" CACHE STRING "GMP source package SHA256 hash" FORCE)
set(GMP_EP_URL             "https://gmplib.org/download/gmp/gmp-6.2.1.tar.lz" CACHE STRING "GMP library source URL" FORCE)

# Config options
option(GMP_BUILD           "Build GMP library" ON)
option(GMP_BUILD_SHARED    "Build shared GMP library" OFF)
option(GMP_BUILD_STATIC    "Build static GMP library" ON)
option(GMP_ENABLE_CXX      "Enable C++ support" OFF)
option(GMP_PARALLEL_BUILD  "Build GMP with multiple jobs" ON)

# TODO: add setting the --build option for different targets.
# Currently the configure script guesses it on the fly.
# Note that RISC-V seems to be not supported.

# Set EP_BASE directory property, documentation says
# project will be organized as follows:
# TMP_DIR      = <base>/tmp/<name>
# STAMP_DIR    = <base>/Stamp/<name>
# DOWNLOAD_DIR = <base>/Download/<name>
# SOURCE_DIR   = <base>/Source/<name>
# BINARY_DIR   = <base>/Build/<name>
# INSTALL_DIR  = <base>/Install/<name>
# LOG_DIR      = <STAMP_DIR>
set_directory_properties(PROPERTIES EP_BASE "${GMP_EP_BASE}")

# Remove all compile options that the ancient
# M4/automake/whatever doesn't understand, or
# which break some function testing during configuration.
set(GMP_FAULTY_COMPILE_OPTIONS "-fno-pic|-fno-pie")

# Build outputs
list(APPEND GMP_LIBS        "${GMP_EP_BASE}/Source/${GMP_EP_NAME}/lib/libgmp.a")
list(APPEND GMP_LIB_HEADERS "${GMP_EP_BASE}/Source/${GMP_EP_NAME}/include")

# Find GMP sources
#find_file(GMP_PATH gmp PATHS ${project_dir}/projects CMAKE_FIND_ROOT_PATH_BOTH)
#mark_as_advanced(FORCE GMP_PATH)
#if("${GMP_PATH}" STREQUAL "GMP_PATH-NOTFOUND")
#    message(
#        FATAL_ERROR "Failed to find GMP. Consider setting -GMP_PATH=/path/to/gmp"
#    )
#endif()

# Initialize settings variables
set(GMP_COMPILE_OPTIONS "")
set(GMP_CFLAGS          "")
set(GMP_CPPFLAGS        "")
set(GMP_MAKEFLAGS       "")
set(GMP_CONFIG_OPTIONS  "")

# Library config
if(GMP_BUILD_SHARED)
    list(APPEND GMP_CONFIG_OPTIONS --enable-shared=yes)
else()
    list(APPEND GMP_CONFIG_OPTIONS --enable-shared=no)
endif()

if(GMP_BUILD_STATIC)
    list(APPEND GMP_CONFIG_OPTIONS --enable-static=yes)
else()
    list(APPEND GMP_CONFIG_OPTIONS --enable-static=no)
endif()

# Set reasonable settings
list(APPEND GMP_CONFIG_OPTIONS --with-pic)
list(APPEND GMP_CONFIG_OPTIONS --enable-alloca=malloc-reentrant)
list(APPEND GMP_CONFIG_OPTIONS --with-readline=no)

# Patch for disabling non-usable features,
# the check in configure script seems broken when
# ran against muslc headers.
list(APPEND GMP_PATCHES "${CMAKE_CURRENT_LIST_DIR}/0001_gmp_remove_obstack_vprintf.patch")

# Parse compiler
set(GMP_C_COMPILER "${CMAKE_C_COMPILER}")

# Add the target triple to the compile flags if we are using clang
# It is not part of the CMAKE_C_FLAGS variable
if(CMAKE_C_COMPILER_ID STREQUAL "Clang")
    list(APPEND GMP_COMPILE_OPTIONS "${CMAKE_C_COMPILE_OPTIONS_TARGET}${CMAKE_C_COMPILER_TARGET}")
endif()

# We make an attempt to extract compiler options and  C_FLAGS from CMake.
# Also inherit the CMAKE build type flags from one of the following variables
# depending on the configured build type:
# - CMAKE_C_FLAGS_DEBUG
# - CMAKE_C_FLAGS_RELEASE
# - CMAKE_C_FLAGS_RELWITHDEBINFO
# - CMAKE_C_FLAGS_MINSIZEREL
get_property(GMP_COMPILE_OPTIONS DIRECTORY PROPERTY COMPILE_OPTIONS)
separate_arguments(cmake_c_flags_sep NATIVE_COMMAND "${CMAKE_C_FLAGS}")
string(TOUPPER ${CMAKE_BUILD_TYPE} cmake_build_type)
separate_arguments(cmake_c_build_flags_sep NATIVE_COMMAND "${CMAKE_C_FLAGS_${cmake_build_type}}")

# Remove all compile options that the ancient
# M4/automake/whatever doesn't understand, or
# which break some function testing.
list(APPEND flags "${cmake_c_flags_sep}")
list(APPEND flags "${cmake_c_build_flags_sep}")
list(APPEND flags "${GMP_COMPILE_OPTIONS}")

foreach(FLAG ${flags})
    if(NOT ${FLAG} MATCHES "${GMP_FAULTY_COMPILE_OPTIONS}")
        list(APPEND GMP_CFLAGS ${FLAG})
    endif()
endforeach()

# Set C standard if defined
if(DEFINED CMAKE_C_STANDARD)
    if(CMAKE_C_STANDARD MATCHES "90")
        list(APPEND GMP_CFLAGS "-std=c90")
    elseif(CMAKE_C_STANDARD MATCHES "99")
        list(APPEND GMP_CFLAGS "-std=c99")
    elseif(CMAKE_C_STANDARD MATCHES "11")
        list(APPEND GMP_CFLAGS "-std=c11")
    elseif(CMAKE_C_STANDARD MATCHES "17")
        list(APPEND GMP_CFLAGS "-std=c17")
    elseif(CMAKE_C_STANDARD MATCHES "23")
        list(APPEND GMP_CFLAGS "-std=c23")
    endif()
endif()

# We need to pass location of muslc headers to the build,
# otherwise the build stubbornly tries to use system headers.
# Don't know if the preprocessor flags are strictly necessary,
# as the include path is already in C_FLAGS.
# 
# We need to however tell the compiler that we don't have
# standard libraries, so set that
list(APPEND GMP_CFLAGS "-I$<TARGET_PROPERTY:muslc,INTERFACE_INCLUDE_DIRECTORIES>")
list(APPEND GMP_CFLAGS "-nostdinc")
list(APPEND GMP_CPPFLAGS "-I$<TARGET_PROPERTY:muslc,INTERFACE_INCLUDE_DIRECTORIES>")

# Remove duplicate settings
list(REMOVE_DUPLICATES GMP_CFLAGS)

# Now COMPILE_OPTIONS and CMAKE_C_FLAGS should all be parsed.
message(STATUS "GMP_C_COMPILER      : ${GMP_C_COMPILER}")
message(STATUS "GMP_CFLAGS          : ${GMP_CFLAGS}")
message(STATUS "GMP_CPPFLAGS        : ${GMP_CPPFLAGS}")
message(STATUS "GMP_COMPILE_OPTIONS : ${GMP_COMPILE_OPTIONS}")
message(STATUS "GMP_CONFIG_OPTIONS  : ${GMP_CONFIG_OPTIONS}")

# Extract CXX flags from CMake if needed
if(GMP_ENABLE_CXX)

    # Not building for C++ by default
    list(APPEND GMP_CONFIG_OPTIONS --enable-cxx=yes)

    set(GMP_CXX_COMPILER "${CMAKE_CXX_COMPILER}")
    set(GMP_CXXFLAGS     "")

    separate_arguments(cmake_cxx_flags_sep NATIVE_COMMAND "${CMAKE_CXX_FLAGS}")
    string(TOUPPER ${CMAKE_BUILD_TYPE} cmake_build_type)
    separate_arguments(cmake_cxx_build_flags_sep NATIVE_COMMAND "${CMAKE_CXX_FLAGS_${cmake_build_type}}")

    # Remove all compile options that the ancient
    # M4/automake/whatever doesn't understand, or
    # which break some function testing.
    list(APPEND flags "${cmake_cxx_flags_sep}")
    list(APPEND flags "${cmake_cxx_build_flags_sep}")
    list(APPEND flags "${GMP_COMPILE_OPTIONS}")
    foreach(FLAG ${flags})
        if(NOT ${FLAG} MATCHES "${GMP_FAULTY_COMPILE_OPTIONS}")
            list(APPEND GMP_CXXFLAGS ${FLAG})
        endif()
    endforeach()

    # Set C++ standard if defined
    if(DEFINED CMAKE_CXX_STANDARD)
        if(CMAKE_CXX_STANDARD MATCHES "98")
            list(APPEND GMP_CXXFLAGS "-std=c++98")
        elseif(CMAKE_CXX_STANDARD MATCHES "11")
            list(APPEND GMP_CXXFLAGS "-std=c++11")
        elseif(CMAKE_CXX_STANDARD MATCHES "14")
            list(APPEND GMP_CXXFLAGS "-std=c++14")
        elseif(CMAKE_CXX_STANDARD MATCHES "17")
            list(APPEND GMP_CXXFLAGS "-std=c++17")
        elseif(CMAKE_CXX_STANDARD MATCHES "20")
            list(APPEND GMP_CXXFLAGS "-std=c++20")
        elseif(CMAKE_CXX_STANDARD MATCHES "23")
            list(APPEND GMP_CXXFLAGS "-std=c++23")
        elseif(CMAKE_CXX_STANDARD MATCHES "26")
            list(APPEND GMP_CXXFLAGS "-std=c++26")
        endif()
    endif()

    # We need to pass location of muslc headers to the build,
    # otherwise the build stubbornly tries to use system headers.
    # Don't know if the include path here is necessary, as it is 
    # already in C_FLAGS.
    # 
    # We need to however tell the compiler that we don't have
    # standard libraries, so set that
    list(APPEND GMP_CXXFLAGS "-nostdinc++")
    list(APPEND GMP_CXXFLAGS "-I$<TARGET_PROPERTY:muslc,INTERFACE_INCLUDE_DIRECTORIES>")

    # Remove duplicate settings
    list(REMOVE_DUPLICATES GMP_CXXFLAGS)

    # Now CMAKE_CXX_FLAGS should all be parsed
    message(STATUS "GMP_CXX_COMPILER    : ${GMP_CXX_COMPILER}")
    message(STATUS "GMP_CXXFLAGS        : ${GMP_CXXFLAGS}")
endif()

# Parallel build
if(GMP_PARALLEL_BUILD)
    include(ProcessorCount)
    ProcessorCount(N)
    if(N EQUAL 0)
        message(WARNING "Could not detect ProcessorCount for building libgmp!")
    else()
        # We set tasks to number of CPUs + 2 to account for blocking IO, this is
        # a similar heuristic to what Ninja uses by default. Note that this gets
        # used in a rule generated by add_custom_command and is additive with
        # any existing Ninja jobs.
        math(EXPR Nplus2 "${N} + 2")
        list(APPEND GMP_MAKEFLAGS "-j${Nplus2}")
    endif()
endif()


# Convert lists to strings, CMake doesn't seem to handle
# lists correctly for pasting into env command.
string(REPLACE ";" " " GMP_COMPILE_OPTIONS "${GMP_COMPILE_OPTIONS}")
string(REPLACE ";" " " GMP_CFLAGS "${GMP_CFLAGS}")
string(REPLACE ";" " " GMP_CPPFLAGS "${GMP_CPPFLAGS}")
string(REPLACE ";" " " GMP_MAKEFLAGS "${GMP_MAKEFLAGS}")
string(REPLACE ";" " " GMP_CONFIG_OPTIONS "${GMP_CONFIG_OPTIONS}")
if(GMP_ENABLE_CXX)
    string(REPLACE ";" " " GMP_CXXFLAGS "${GMP_CXXFLAGS}")
endif()


# Need this to avoid CMake error about not existing directory
#file(MAKE_DIRECTORY ${GMP_LIB_HEADERS})


# Build GMP with its own build system
ExternalProject_Add(
    "${GMP_EP_NAME}"
    BUILD_IN_SOURCE ${GMP_EP_BUILD_IN_SOURCE}
    URL             "${GMP_EP_URL}"
    URL_HASH        "${GMP_EP_VERSION_HASH}"
    UPDATE_COMMAND  ""
    PATCH_COMMAND   patch -p1 < "${GMP_PATCHES}"
    TEST_COMMAND    ""
    CONFIGURE_COMMAND
        ${CMAKE_COMMAND} -E env 
        CC=${GMP_C_COMPILER}
        CXX=${GMP_CXX_COMPILER}
        CFLAGS=${GMP_CFLAGS}
        CXXFLAGS=${GMP_CXXFLAGS}
        CPPFLAGS=${GMP_CPPFLAGS} 
        <SOURCE_DIR>/configure
        --prefix=<SOURCE_DIR>
        ${GMP_CONFIG_OPTIONS}
    BUILD_COMMAND
        ${CMAKE_COMMAND} -E env
        MAKEFLAGS=${GMP_MAKEFLAGS}
        make
    EXCLUDE_FROM_ALL
    BUILD_BYPRODUCTS
    "${GMP_LIBS}"
    DEPENDS
    muslc)


# Create a dependency library from the external project
set(GMP_SOURCES "${GMP_EP_NAME}_sources")
add_library(${GMP_SOURCES} STATIC IMPORTED GLOBAL)
add_dependencies(${GMP_SOURCES} "${GMP_EP_NAME}")
set_target_properties(${GMP_SOURCES} PROPERTIES IMPORTED_LOCATION "${GMP_LIBS}")

# Create a library which is the header files, that depends on the sources
add_library(gmp INTERFACE)
add_dependencies(gmp ${GMP_SOURCES})
set_target_properties(gmp PROPERTIES INTERFACE_LINK_LIBRARIES ${GMP_SOURCES})
set_target_properties(gmp PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${GMP_LIB_HEADERS}")