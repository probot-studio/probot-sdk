# platform: hand-written ESP32/host glue (threads, console, backtrace) in src/prosdk/platform.
file(GLOB _platform_src CONFIGURE_DEPENDS "${PROSDK_ROOT}/src/prosdk/platform/*.cpp")
prosdk_add_module(platform SOURCES ${_platform_src})

file(GLOB _cxx_tests CONFIGURE_DEPENDS "${PROSDK_ROOT}/test/prosdk/*.cpp")
prosdk_add_test(prosdk_tests SOURCES ${_cxx_tests})
