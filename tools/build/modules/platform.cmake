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
# with the probot-core clone ($PROBOT_CORE, default .upstream/probot-core) the fake passes the calls
# on to core's own header-only probot::files::share()/unshare(), so core's rules decide
set(_probot_core "$ENV{PROBOT_CORE}")
if(NOT _probot_core)
  set(_probot_core "${PROSDK_ROOT}/.upstream/probot-core")
endif()
if(TARGET prosdk_files_tests AND EXISTS "${_probot_core}/src/probot/files/files.hpp")
  target_include_directories(prosdk_files_tests PRIVATE "${_probot_core}/src")
  target_compile_definitions(prosdk_files_tests PRIVATE PROSDK_TEST_PROBOT_CORE_FILES=1)
elseif(TARGET prosdk_files_tests)
  message(STATUS "prosdk_files_tests: no probot-core with R11 at ${_probot_core}, core's rules untested")
endif()
prosdk_add_test(prosdk_files_legacy_tests SOURCES "${PROSDK_ROOT}/test/prosdk/main.cpp"
  ${_files_tests})
if(TARGET prosdk_files_legacy_tests)
  target_compile_definitions(prosdk_files_legacy_tests PRIVATE PROSDK_FAKE_CORE_NO_FILES_SHARE=1)
endif()

# own executable: probot-core R18 match info -> HAL (prosdk/platform/hal/CoreMatchInfo.hpp) against
# fake tables, with a core table whose ABI 9 fields are null installed (CoreOps is installed once
# per process). With a probot-core clone that has R18 ($PROBOT_CORE, as above) one more case reads
# core's own probot::ds match store; core's <probot/ds.hpp> needs its Arduino test stubs.
file(GLOB _match_tests CONFIGURE_DEPENDS "${PROSDK_ROOT}/test/prosdk/match/*.cpp")
prosdk_add_test(prosdk_match_tests SOURCES "${PROSDK_ROOT}/test/prosdk/main.cpp" ${_match_tests})
if(TARGET prosdk_match_tests AND EXISTS "${_probot_core}/src/probot/robot/match_info.hpp")
  target_include_directories(prosdk_match_tests PRIVATE "${_probot_core}/src"
    "${_probot_core}/tests/stubs")
  target_compile_definitions(prosdk_match_tests PRIVATE PROSDK_TEST_PROBOT_CORE_MATCH=1)
elseif(TARGET prosdk_match_tests)
  message(STATUS "prosdk_match_tests: no probot-core with R18 at ${_probot_core}, core's store untested")
endif()

# own executable: probot-core's own gamepad mapping profiles (header-only, no stubs needed) through
# the bridge's copy and the HAL conversion: the R20 Home/Guide button as GUIDE, or none on an older
# core. Needs the probot-core clone ($PROBOT_CORE, as above); it must have R20 unless
# PROBOT_CORE_BEFORE_R20=1 is set while configuring (then it must not).
file(GLOB _gamepad_tests CONFIGURE_DEPENDS "${PROSDK_ROOT}/test/prosdk/gamepad/*.cpp")
prosdk_add_test(prosdk_gamepad_tests SOURCES "${PROSDK_ROOT}/test/prosdk/main.cpp" ${_gamepad_tests})
if(TARGET prosdk_gamepad_tests AND
   EXISTS "${_probot_core}/src/probot/io/joystick_mappings/mapping.hpp")
  target_include_directories(prosdk_gamepad_tests PRIVATE "${_probot_core}/src")
  target_compile_definitions(prosdk_gamepad_tests PRIVATE PROSDK_TEST_PROBOT_CORE_GAMEPAD=1)
  if("$ENV{PROBOT_CORE_BEFORE_R20}" STREQUAL "1")
    target_compile_definitions(prosdk_gamepad_tests PRIVATE PROSDK_TEST_CORE_BEFORE_R20=1)
  endif()
elseif(TARGET prosdk_gamepad_tests)
  message(STATUS "prosdk_gamepad_tests: no probot-core at ${_probot_core}, core's profiles untested")
endif()

# own executable: the ESP32 Driver Station backend (src/prosdk/platform/hal/esp32/DriverStation.cpp,
# otherwise target only) on the host: its pump against a fake core table, through the HAL's Driver
# Station functions. Its GetDefaultDriverStationImpl is renamed here, since the sim backend defines
# the host's.
prosdk_add_test(prosdk_ds_pump_tests SOURCES "${PROSDK_ROOT}/test/prosdk/main.cpp"
  "${PROSDK_ROOT}/test/prosdk/dspump/DriverStationPumpTest.cpp"
  "${PROSDK_ROOT}/src/prosdk/platform/hal/esp32/DriverStation.cpp")
if(TARGET prosdk_ds_pump_tests)
  set_source_files_properties("${PROSDK_ROOT}/src/prosdk/platform/hal/esp32/DriverStation.cpp"
    TARGET_DIRECTORY prosdk_ds_pump_tests
    PROPERTIES COMPILE_DEFINITIONS "GetDefaultDriverStationImpl=GetEsp32DriverStationForTest")
endif()
