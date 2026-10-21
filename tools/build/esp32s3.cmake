# CMake toolchain file: the Arduino-ESP32 3.3.10 esp32s3 compile recipe (design/PORT.md
# sections 6, 7 and 9). The flags come from tools/build/arduino_recipe.py as GCC response files,
# so order and quoting are exactly Arduino's: @cpp_flags -w -Os -Werror=return-type <board
# defines> @defines -iprefix <sdk>/include/ @includes -I<memory type> -I<core> -I<variant>.
# The effective standard is the last -std in cpp_flags (gnu++2a = gnu++20); nothing is added.

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR xtensa)

get_filename_component(PROSDK_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(_prosdk_recipe_dir "${PROSDK_ROOT}/.build/recipe/esp32s3")
execute_process(
  COMMAND python3 "${PROSDK_ROOT}/tools/build/arduino_recipe.py" --out-dir "${_prosdk_recipe_dir}"
  RESULT_VARIABLE _prosdk_recipe_rc)
if(NOT _prosdk_recipe_rc EQUAL 0)
  message(FATAL_ERROR "tools/build/arduino_recipe.py failed (${_prosdk_recipe_rc})")
endif()
include("${_prosdk_recipe_dir}/recipe.cmake")

set(_bin "${PROSDK_ARDUINO_TOOLCHAIN_BIN}/xtensa-esp32s3-elf-")
set(CMAKE_C_COMPILER "${_bin}gcc")
set(CMAKE_CXX_COMPILER "${_bin}g++")
set(CMAKE_ASM_COMPILER "${_bin}gcc")
# Arduino archives with gcc-ar (recipe.ar.pattern)
set(CMAKE_AR "${_bin}gcc-ar" CACHE FILEPATH "" FORCE)
set(CMAKE_RANLIB "${_bin}gcc-ranlib" CACHE FILEPATH "" FORCE)
set(PROSDK_SIZE "${_bin}size")

set(CMAKE_CXX_FLAGS_INIT "@${PROSDK_ARDUINO_CXX_RSP}")
set(CMAKE_C_FLAGS_INIT "@${PROSDK_ARDUINO_C_RSP}")
# no per-configuration flags: -Os and -g come from the recipe
foreach(_lang C CXX)
  foreach(_cfg DEBUG RELEASE MINSIZEREL RELWITHDEBINFO)
    set(CMAKE_${_lang}_FLAGS_${_cfg}_INIT "")
  endforeach()
endforeach()

# Arduino order: recipe flags first, the library include path (-I.../src) last.
set(CMAKE_CXX_COMPILE_OBJECT "<CMAKE_CXX_COMPILER> <FLAGS> <DEFINES> <INCLUDES> -o <OBJECT> -c <SOURCE>")
set(CMAKE_C_COMPILE_OBJECT "<CMAKE_C_COMPILER> <FLAGS> <DEFINES> <INCLUDES> -o <OBJECT> -c <SOURCE>")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
