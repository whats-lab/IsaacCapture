# SPDX-FileCopyrightText: Copyright (c) 2026 WHATs LAB Corp. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(STATUS "Skipping airglove plugin (Linux only)")
    return()
endif()

include(GNUInstallDirs)
include(FetchContent)

set(AIRGLOVE_CLIENT_DIR "" CACHE PATH "AirGlove-Client checkout (include/, native/<platform>/); empty fetches the pinned revision")
set(AIRGLOVE_CLIENT_REPOSITORY "https://github.com/whats-lab/AirGlove-Client.git" CACHE STRING "AirGlove-Client git repository")
set(AIRGLOVE_CLIENT_TAG "v0.1.0" CACHE STRING "AirGlove-Client release tag to fetch")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(_airglove_platform linux-arm64)
else()
    set(_airglove_platform linux-x64)
endif()

if(NOT AIRGLOVE_CLIENT_DIR)
    FetchContent_Declare(airglove_client
        GIT_REPOSITORY "${AIRGLOVE_CLIENT_REPOSITORY}"
        GIT_TAG "${AIRGLOVE_CLIENT_TAG}"
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(airglove_client)
    set(AIRGLOVE_CLIENT_DIR "${airglove_client_SOURCE_DIR}")
endif()

set(_airglove_library "${AIRGLOVE_CLIENT_DIR}/native/${_airglove_platform}/libairglove_client.so")
if(NOT EXISTS "${_airglove_library}")
    message(FATAL_ERROR "airglove_client library not found: ${_airglove_library}")
endif()

add_library(airglove_client SHARED IMPORTED)
set_target_properties(airglove_client PROPERTIES
    IMPORTED_LOCATION "${_airglove_library}"
    INTERFACE_INCLUDE_DIRECTORIES "${AIRGLOVE_CLIENT_DIR}/include"
)

add_executable(airglove_plugin
    main.cpp
    airglove_plugin.cpp
    airglove_plugin.hpp
)

find_package(Threads REQUIRED)

target_link_libraries(airglove_plugin PRIVATE
    airglove_client
    Teleop::plugin_utils
    deviceio::deviceio_session
    deviceio::deviceio_trackers
    oxr::oxr_core
    oxr::oxr_utils
    OpenXR::openxr_loader
    Teleop::openxr_extensions
    Threads::Threads
    log_bridge::log_bridge_core
)

set_target_properties(airglove_plugin PROPERTIES
    INSTALL_RPATH "$ORIGIN;$ORIGIN/../../${CMAKE_INSTALL_LIBDIR}"
)

install(TARGETS airglove_plugin
    RUNTIME DESTINATION plugins/airglove
    COMPONENT airglove
)

install(FILES plugin.yaml README.md "${_airglove_library}"
    DESTINATION plugins/airglove
    COMPONENT airglove
)
