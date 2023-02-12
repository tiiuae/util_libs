#
# Copyright 2023, Technology Innovation Institute
#
# SPDX-License-Identifier: BSD-2-Clause
#

# Use the same version as mbedtls wants.
cmake_minimum_required(VERSION 3.5.1)

set(ENABLE_PROGRAMS            OFF CACHE BOOL "Don't build mbedtls programs" FORCE)
set(UNSAFE_BUILD               OFF CACHE BOOL "Don't allow unsafe builds for mbedtls" FORCE)
set(MBEDTLS_FATAL_WARNINGS     OFF CACHE BOOL "Don't treat compiler warnings treated as errors on mbedtls" FORCE)
set(ENABLE_TESTING             OFF CACHE BOOL "Don't build mbedtls tests" FORCE)
set(USE_STATIC_MBEDTLS_LIBRARY ON CACHE BOOL  "Build mbed TLS static library" FORCE)
set(USE_SHARED_MBEDTLS_LIBRARY OFF CACHE BOOL "Don't build mbed TLS shared library" FORCE)
set(LINK_WITH_PTHREAD          OFF CACHE BOOL "Don't link mbed TLS library to pthread" FORCE)
set(LINK_WITH_TRUSTED_STORAGE  OFF CACHE BOOL "Don't link mbed TLS library to trusted_storage" FORCE)

# Ugly hack to add muslc dependency. This is to keep
# changes to the mbedtls sources as small as possible.
# This will most likely break in unexpected ways many times :)
# Used by ./library/CMakeLists.txt
set(libs ${libs} muslc)

# WIP: Custom configuration, don't know if this works without testing
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -I${CMAKE_CURRENT_SOURCE_DIR}/configs -DMBEDTLS_CONFIG_FILE='<config-sel4.h>'")
