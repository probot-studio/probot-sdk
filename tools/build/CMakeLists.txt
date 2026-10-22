# prosdk build (design/PORT.md section 9). Configure through tools/build/build.sh:
#   esp32s3: -DCMAKE_TOOLCHAIN_FILE=tools/build/esp32s3.cmake -> src/esp32s3/libprosdk.a
#   host:    system g++ (gnu++20, no RTTI, exceptions)      -> host tests
# One OBJECT library per module (tools/build/modules/<module>.cmake); the archive is assembled
# from them. Vendored source lists come from tools/upstream/generated.txt.
cmake_minimum_required(VERSION 3.20)
project(prosdk LANGUAGES C CXX)

get_filename_component(PROSDK_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(PROSDK_GENERATED "${PROSDK_ROOT}/tools/upstream/generated.txt")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${PROSDK_GENERATED}" "${CMAKE_CURRENT_LIST_DIR}/header_check.py")

if(CMAKE_CROSSCOMPILING)
  set(PROSDK_TARGET_HW 1)
else()
  set(PROSDK_TARGET_HW 0)
endif()

# ccache (static binary in tools/bin, cache in .build/ccache)
find_program(PROSDK_CCACHE ccache HINTS "${PROSDK_ROOT}/tools/bin" NO_DEFAULT_PATH)
if(NOT PROSDK_CCACHE)
  find_program(PROSDK_CCACHE ccache)
endif()
if(PROSDK_CCACHE)
  set(_launcher "${CMAKE_COMMAND};-E;env;CCACHE_DIR=${PROSDK_ROOT}/.build/ccache;CCACHE_BASEDIR=${PROSDK_ROOT};${PROSDK_CCACHE}")
  set(CMAKE_C_COMPILER_LAUNCHER "${_launcher}")
  set(CMAKE_CXX_COMPILER_LAUNCHER "${_launcher}")
endif()

# What a sketch translation unit gets: the include root (and, on the host, the target's language
# mode). arduino-cli passes no prosdk -D flags to the sketch, so public headers must not depend on
# them; the header check uses only this.
add_library(prosdk_sketch_options INTERFACE)
target_include_directories(prosdk_sketch_options INTERFACE "${PROSDK_ROOT}/src")
if(NOT CMAKE_CROSSCOMPILING)
  # host: same language mode as the target (gnu++20, no RTTI, exceptions)
  target_compile_options(prosdk_sketch_options INTERFACE
    $<$<COMPILE_LANGUAGE:CXX>:-std=gnu++20 -fno-rtti -fexceptions>
    -O1 -g -ffunction-sections -fdata-sections)
endif()

# Usage requirements shared by every prosdk translation unit of the archive and the host tests.
add_library(prosdk_options INTERFACE)
target_link_libraries(prosdk_options INTERFACE prosdk_sketch_options)
# Eigen and Sleipnir configuration is not passed here: it lives in src/prosdk/platform/config.hpp,
# which every Eigen module header and Sleipnir header includes first (design/PORT.md 7.1).
target_compile_definitions(prosdk_options INTERFACE
  PROSDK_TARGET_HW=${PROSDK_TARGET_HW})

# Generated (vendored) files of one module whose repo-relative path matches REGEX.
file(STRINGS "${PROSDK_GENERATED}" _prosdk_generated REGEX "^[A-Za-z0-9_]+ ")
function(prosdk_generated_files out module regex)
  set(res)
  foreach(line IN LISTS _prosdk_generated)
    if(line MATCHES "^([^ ]+) (.+)$")
      set(_mod "${CMAKE_MATCH_1}")
      set(_path "${CMAKE_MATCH_2}")
      if(_mod STREQUAL module AND _path MATCHES "${regex}")
        list(APPEND res "${PROSDK_ROOT}/${_path}")
      endif()
    endif()
  endforeach()
  set(${out} "${res}" PARENT_SCOPE)
endfunction()

# prosdk_add_module(<name> SOURCES ...): one OBJECT library, part of libprosdk.a.
function(prosdk_add_module name)
  cmake_parse_arguments(A "" "" "SOURCES" ${ARGN})
  add_library(prosdk_${name} OBJECT ${A_SOURCES})
  target_link_libraries(prosdk_${name} PUBLIC prosdk_options)
  set_property(GLOBAL APPEND PROPERTY PROSDK_MODULES ${name})
endfunction()

# prosdk_add_test(<name> SOURCES ... [INCLUDES ...]): host Catch2 executable linked to prosdk.
function(prosdk_add_test name)
  cmake_parse_arguments(A "" "" "SOURCES;INCLUDES" ${ARGN})
  if(CMAKE_CROSSCOMPILING OR NOT A_SOURCES)
    return()
  endif()
  add_executable(${name} ${A_SOURCES})
  target_include_directories(${name} PRIVATE ${A_INCLUDES})
  target_link_libraries(${name} PRIVATE prosdk prosdk_catch2 Threads::Threads)
  set_property(GLOBAL APPEND PROPERTY PROSDK_TESTS ${name})
endfunction()

if(NOT CMAKE_CROSSCOMPILING)
  find_package(Threads REQUIRED)
  prosdk_generated_files(_catch2_src catch2 "^test/thirdparty/catch2/cpp/.*\\.cpp$")
  add_library(prosdk_catch2 STATIC ${_catch2_src})
  target_include_directories(prosdk_catch2 PUBLIC "${PROSDK_ROOT}/test/thirdparty/catch2/include")
  target_link_libraries(prosdk_catch2 PUBLIC prosdk_options)
endif()

include("${CMAKE_CURRENT_LIST_DIR}/modules/platform.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/modules/wpiutil.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/modules/telemetry.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/modules/tunables.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/modules/hal.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/modules/datalog.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/modules/eigen.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/modules/gcem.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/modules/sleipnir.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/modules/wpimath.cmake")

# libprosdk.a from every module's objects
get_property(_modules GLOBAL PROPERTY PROSDK_MODULES)
set(_objects)
foreach(m IN LISTS _modules)
  list(APPEND _objects "$<TARGET_OBJECTS:prosdk_${m}>")
endforeach()
add_library(prosdk STATIC ${_objects})
target_link_libraries(prosdk PUBLIC prosdk_options)
set_target_properties(prosdk PROPERTIES OUTPUT_NAME prosdk)
if(CMAKE_CROSSCOMPILING)
  # Arduino precompiled library layout: src/{build.mcu}/lib<name>.a
  set_target_properties(prosdk PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${PROSDK_ROOT}/src/esp32s3")
  add_custom_target(size
    COMMAND "${PROSDK_SIZE}" -t "$<TARGET_FILE:prosdk>"
    DEPENDS prosdk VERBATIM USES_TERMINAL)
endif()

# header-check: every public header compiled on its own after #include <Arduino.h> (target) or
# alone (host), with exactly the flags a sketch TU gets (prosdk_sketch_options: no prosdk -D
# flags, so PROSDK_TARGET_HW comes from prosdk/platform/config.hpp as in a sketch). The TU list is
# generated from generated.txt plus the hand-written headers, at configure time (re-runs when
# generated.txt changes).
execute_process(
  COMMAND python3 "${CMAKE_CURRENT_LIST_DIR}/header_check.py"
          --out "${CMAKE_BINARY_DIR}/header-check" --target-hw ${PROSDK_TARGET_HW}
  OUTPUT_VARIABLE _hc_sources OUTPUT_STRIP_TRAILING_WHITESPACE
  RESULT_VARIABLE _hc_rc)
if(NOT _hc_rc EQUAL 0)
  message(FATAL_ERROR "header_check.py failed")
endif()
# re-run configure (and header_check.py) when hand-written headers are added or removed
file(GLOB _hand_headers CONFIGURE_DEPENDS "${PROSDK_ROOT}/src/prosdk.h"
  "${PROSDK_ROOT}/src/prosdk/cxx/*.hpp" "${PROSDK_ROOT}/src/prosdk/platform/*.hpp")
string(REPLACE "\n" ";" _hc_sources "${_hc_sources}")
add_library(header_check OBJECT EXCLUDE_FROM_ALL ${_hc_sources})
target_link_libraries(header_check PRIVATE prosdk_sketch_options)
set_target_properties(header_check PROPERTIES UNITY_BUILD OFF)

if(NOT CMAKE_CROSSCOMPILING)
  get_property(_tests GLOBAL PROPERTY PROSDK_TESTS)
  add_custom_target(check
    COMMAND "${CMAKE_COMMAND}" -E echo "running host tests"
    DEPENDS ${_tests})
  foreach(t IN LISTS _tests)
    add_custom_command(TARGET check POST_BUILD COMMAND $<TARGET_FILE:${t}> VERBATIM)
  endforeach()
endif()
