# hal (vendored by tools/upstream/modules/hal.py)
#   hal         backend-independent HAL code (src/prosdk/impl/hal) plus hand-written common
#               replacements (src/prosdk/platform/hal/*.cpp); archive and host
#   hal_esp32   the ESP32 backend (hand-written, src/prosdk/platform/hal/esp32/*.cpp); target only.
#               Until it exists the archive leaves the backend's HAL_* symbols unresolved.
#   hal_sim     the WPILib simulation backend (host/hal/sim, outside the Arduino tree); host only
prosdk_generated_files(_hal_src hal "^src/prosdk/impl/hal/.*\\.cpp$")
file(GLOB _hal_common_src CONFIGURE_DEPENDS "${PROSDK_ROOT}/src/prosdk/platform/hal/*.cpp")
prosdk_add_module(hal SOURCES ${_hal_src} ${_hal_common_src})

if(CMAKE_CROSSCOMPILING)
  file(GLOB _hal_esp32_src CONFIGURE_DEPENDS "${PROSDK_ROOT}/src/prosdk/platform/hal/esp32/*.cpp")
  if(_hal_esp32_src)
    prosdk_add_module(hal_esp32 SOURCES ${_hal_esp32_src})
  endif()
else()
  prosdk_generated_files(_hal_sim_src hal "^host/hal/sim/.*\\.cpp$")
  prosdk_add_module(hal_sim SOURCES ${_hal_sim_src})
endif()

prosdk_generated_files(_hal_tests hal "^test/hal/cpp/.*\\.cpp$")
prosdk_add_test(hal_tests SOURCES ${_hal_tests})
