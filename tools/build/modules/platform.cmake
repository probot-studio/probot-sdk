# platform: hand-written ESP32/host glue (threads, console, backtrace) in src/prosdk/platform, and
# the probot-core dashboard backends (telemetry, tunables, alerts) in src/prosdk/platform/dashboard.
file(GLOB _platform_src CONFIGURE_DEPENDS "${PROSDK_ROOT}/src/prosdk/platform/*.cpp"
  "${PROSDK_ROOT}/src/prosdk/platform/dashboard/*.cpp")
# re-run configure (header_check.py) when a dashboard header is added or removed
file(GLOB _dashboard_headers CONFIGURE_DEPENDS "${PROSDK_ROOT}/src/prosdk/platform/dashboard/*.hpp")
prosdk_add_module(platform SOURCES ${_platform_src})

file(GLOB _cxx_tests CONFIGURE_DEPENDS "${PROSDK_ROOT}/test/prosdk/*.cpp")
# DataLogManagerConsoleTest reads its log back with the DataLog reader (datalog test support)
prosdk_generated_files(_cxx_datalog_support datalog "^test/datalog/support/.*\\.cpp$")
prosdk_add_test(prosdk_tests SOURCES ${_cxx_tests} ${_cxx_datalog_support}
  INCLUDES "${PROSDK_ROOT}/test/datalog/support/include")

# own executable: it installs the fake core table (CoreOps is installed once per process)
file(GLOB _dashboard_tests CONFIGURE_DEPENDS "${PROSDK_ROOT}/test/prosdk/dashboard/*.cpp")
prosdk_add_test(prosdk_dashboard_tests SOURCES "${PROSDK_ROOT}/test/prosdk/main.cpp" ${_dashboard_tests})

# own executable: the hosted robot program against a fake core table and the HAL sim (CoreOps is
# installed once per process, and the e-stop latches)
file(GLOB _robot_tests CONFIGURE_DEPENDS "${PROSDK_ROOT}/test/prosdk/robot/*.cpp")
prosdk_add_test(prosdk_robot_tests SOURCES ${_robot_tests})
