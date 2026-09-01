# ---------------------------------------------------------------------------------------------------------------------
#
# Copyright (C) Microsoft Corporation.  
# Copyright (C) 2025 IAMAI CONSULTING CORP
#
# MIT License. All rights reserved.
#
# Module Name:
#
#   unreal-linux-toolchain.cmake
#
# Abstract:
#
#   Basic CMake Linux toolchain file following https://cmake.org/cmake/help/latest/manual/cmake-toolchains.7.html#cross-compiling-for-linux
#
# ---------------------------------------------------------------------------------------------------------------------

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR x86_64)

# Use the Linux SDK bundled with the selected Unreal Engine. The SDK directory
# name changes between engine releases (for example, v21 in UE 5.2 and v25 in
# UE 5.6), so do not pin it to one engine version.
set(UE_TOOLCHAIN_ROOT "$ENV{UE_ROOT}/Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/Linux_x64")
file(GLOB UE_TOOLCHAIN_CANDIDATES LIST_DIRECTORIES true "${UE_TOOLCHAIN_ROOT}/v*_clang-*")
list(LENGTH UE_TOOLCHAIN_CANDIDATES UE_TOOLCHAIN_COUNT)
if(UE_TOOLCHAIN_COUNT EQUAL 0)
    message(FATAL_ERROR "No Unreal Linux toolchain found under ${UE_TOOLCHAIN_ROOT}. Run Unreal's Setup.sh first.")
elseif(UE_TOOLCHAIN_COUNT GREATER 1)
    message(FATAL_ERROR "Multiple Unreal Linux toolchains found under ${UE_TOOLCHAIN_ROOT}; unable to select one: ${UE_TOOLCHAIN_CANDIDATES}")
endif()
list(GET UE_TOOLCHAIN_CANDIDATES 0 UE_TOOLCHAIN)

set(CMAKE_SYSROOT "${UE_TOOLCHAIN}/x86_64-unknown-linux-gnu")
set(CMAKE_C_COMPILER "${CMAKE_SYSROOT}/bin/clang")
set(CMAKE_CXX_COMPILER "${CMAKE_SYSROOT}/bin/clang++")
# Unreal supplies its own libc++ headers. Newer bundled Clang SDKs also contain
# a second libc++ copy, so suppress the compiler's default C++ include paths to
# avoid mixing two incompatible libc++ versions.
set(CMAKE_CXX_FLAGS "-nostdinc++ -I$ENV{UE_ROOT}/Engine/Source/ThirdParty/Unix/LibCxx/include -I$ENV{UE_ROOT}/Engine/Source/ThirdParty/Unix/LibCxx/include/c++/v1")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
