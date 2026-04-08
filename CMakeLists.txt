#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)

# Configure build metadata from git (best-effort)
find_package(Git)

if(GIT_EXECUTABLE)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} branch --show-current
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_BRANCH
    )
  # Prefer a https URL for display (falls back to 'unknown')
  execute_process(
    COMMAND ${GIT_EXECUTABLE} config --get remote.origin.url
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_REPO_URL_SSH
    )
  execute_process(
    COMMAND ${GIT_EXECUTABLE} config --get remote.origin.https.url
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_REPO_URL_HTTPS
    )
  execute_process(
    COMMAND ${GIT_EXECUTABLE} log -1 --pretty=%an
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_AUTHOR
    )
endif()

if(GIT_BRANCH STREQUAL "")
  set(GIT_BRANCH unknown)
  message(WARNING "Unknown Git Branch")
endif()
# Choose URL: origin.https.url > origin.url
if(NOT GIT_REPO_URL_HTTPS STREQUAL "")
  set(GIT_REPO_URL ${GIT_REPO_URL_HTTPS})
else()
  set(GIT_REPO_URL ${GIT_REPO_URL_SSH})
endif()

# Normalize GitHub SSH URLs to HTTPS for display in firmware info
if(GIT_REPO_URL MATCHES "^git@github\\.com:(.+)$")
  set(GIT_REPO_URL "https://github.com/${CMAKE_MATCH_1}")
elseif(GIT_REPO_URL MATCHES "^ssh://git@github\\.com/(.+)$")
  set(GIT_REPO_URL "https://github.com/${CMAKE_MATCH_1}")
endif()
string(REGEX REPLACE "\\.git$" "" GIT_REPO_URL "${GIT_REPO_URL}")

if(GIT_REPO_URL STREQUAL "")
  set(GIT_REPO_URL unknown)
  message(WARNING "Unknown Git Repo URL")
endif()
if(GIT_AUTHOR STREQUAL "")
  set(GIT_AUTHOR unknown)
  message(WARNING "Unknown Git Author")
endif()

string(STRIP ${GIT_BRANCH} GIT_BRANCH)
string(STRIP ${GIT_REPO_URL} GIT_REPO_URL)
string(STRIP ${GIT_AUTHOR} GIT_AUTHOR)

# Extra metadata for USB HID build-info (docs/usb-build-info-hid.md)
set(GIT_HASH_SHORT "000000000000")
set(FW_GIT_DIRTY_VAL 0)
set(GIT_COMMIT_COUNT_CAPPED 0)

if(GIT_EXECUTABLE)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --short=12 HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_HASH_SHORT
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if(GIT_HASH_SHORT STREQUAL "")
    set(GIT_HASH_SHORT "000000000000")
  endif()

  execute_process(
    COMMAND ${GIT_EXECUTABLE} describe --dirty --always
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_DESCRIBE_OUT
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  string(FIND "${GIT_DESCRIBE_OUT}" "-dirty" GIT_DIRTY_POS)
  if(NOT GIT_DIRTY_POS EQUAL -1)
    set(FW_GIT_DIRTY_VAL 1)
  endif()

  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-list --count HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_COMMIT_COUNT
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if(GIT_COMMIT_COUNT MATCHES "^[0-9]+$")
    if(GIT_COMMIT_COUNT GREATER 65535)
      set(GIT_COMMIT_COUNT_CAPPED 65535)
    else()
      set(GIT_COMMIT_COUNT_CAPPED ${GIT_COMMIT_COUNT})
    endif()
  endif()
endif()

if(DEFINED ENV{SOURCE_DATE_EPOCH})
  set(FW_BUILD_UNIX_TIME "$ENV{SOURCE_DATE_EPOCH}")
else()
  execute_process(
    COMMAND date -u +%s
    OUTPUT_VARIABLE FW_BUILD_UNIX_TIME
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
endif()
if(NOT FW_BUILD_UNIX_TIME MATCHES "^[0-9]+$")
  set(FW_BUILD_UNIX_TIME 0)
endif()

# Pass metadata to C compilation units
add_compile_definitions(
  FW_GIT_BRANCH="${GIT_BRANCH}"
  FW_GIT_REPO_URL="${GIT_REPO_URL}"
  FW_GIT_AUTHOR="${GIT_AUTHOR}"
  FW_GIT_HASH_SHORT="${GIT_HASH_SHORT}"
  FW_GIT_DIRTY=${FW_GIT_DIRTY_VAL}
  FW_BUILD_UNIX_TIME=${FW_BUILD_UNIX_TIME}
  FW_VERSION_TWEAK=${GIT_COMMIT_COUNT_CAPPED}
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(NONE)

FILE(GLOB_RECURSE app_sources src/*.c)
# NORDIC SDK APP START
target_sources(app PRIVATE ${app_sources})
# NORDIC SDK APP END

target_include_directories(app PRIVATE src)

# Force recompile for updated build timestamp
add_custom_target(touch_util_h ALL
    COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_SOURCE_DIR}/src/build_defines.h
)
