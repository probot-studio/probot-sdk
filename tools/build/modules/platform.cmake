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

# own executable: the fake core store without dashboard persistence (a probot-core before R12
# persistence: CoreOps ABI 6 dashClearPersisted null)
file(GLOB _dashboard_legacy_tests CONFIGURE_DEPENDS
  "${PROSDK_ROOT}/test/prosdk/dashboard_legacy/*.cpp")
prosdk_add_test(prosdk_dashboard_legacy_tests SOURCES "${PROSDK_ROOT}/test/prosdk/main.cpp"
  ${_dashboard_legacy_tests} "${PROSDK_ROOT}/test/prosdk/dashboard/FakeDashboardStore.cpp"
  INCLUDES "${PROSDK_ROOT}/test/prosdk/dashboard")
if(TARGET prosdk_dashboard_legacy_tests)
  target_compile_definitions(prosdk_dashboard_legacy_tests PRIVATE PROSDK_FAKE_CORE_NO_PERSIST=1)
endif()

# own executable: the hosted robot program against a fake core table and the HAL sim (CoreOps is
# installed once per process, and the e-stop latches)
file(GLOB _robot_tests CONFIGURE_DEPENDS "${PROSDK_ROOT}/test/prosdk/robot/*.cpp")
prosdk_add_test(prosdk_robot_tests SOURCES ${_robot_tests})

# own executable: the same robot host tests against a fake core without grid restarts (a
# probot-core before R15: CoreOps ABI 7 loopGridRestarts null)
prosdk_add_test(prosdk_robot_legacy_tests SOURCES ${_robot_tests})
if(TARGET prosdk_robot_legacy_tests)
  target_compile_definitions(prosdk_robot_legacy_tests PRIVATE PROSDK_FAKE_CORE_NO_GRID_RESTARTS=1)
endif()

# own executable: DataLogManager sharing its log folder through a fake core that records
# share()/unshare() (CoreOps is installed once per process), and the same tests against a core
# without file sharing (a probot-core before R11: CoreOps ABI 8 filesShare/filesUnshare null)
file(GLOB _files_tests CONFIGURE_DEPENDS "${PROSDK_ROOT}/test/prosdk/files/*.cpp")
prosdk_add_test(prosdk_files_tests SOURCES "${PROSDK_ROOT}/test/prosdk/main.cpp" ${_files_tests})
prosdk_add_test(prosdk_files_legacy_tests SOURCES "${PROSDK_ROOT}/test/prosdk/main.cpp"
  ${_files_tests})
if(TARGET prosdk_files_legacy_tests)
  target_compile_definitions(prosdk_files_legacy_tests PRIVATE PROSDK_FAKE_CORE_NO_FILES_SHARE=1)
endif()
