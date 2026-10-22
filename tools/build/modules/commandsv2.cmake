# commandsv2: the command-based framework (vendored by tools/upstream/modules/commandsv2.py),
# including the generated Command*Controller wrappers (src/prosdk/impl/commandsv2/generated).
prosdk_generated_files(_commandsv2_src commandsv2 "^src/prosdk/impl/commandsv2/.*\\.cpp$")
prosdk_add_module(commandsv2 SOURCES ${_commandsv2_src})

# host tests (Catch2, against the HAL sim backend)
prosdk_generated_files(_commandsv2_tests commandsv2 "^test/commandsv2/cpp/.*\\.cpp$")
prosdk_add_test(commandsv2_tests SOURCES ${_commandsv2_tests})

# prosdk tests of the command framework on the probot-core dashboard backends (fake core store,
# own executable: CoreOps is installed once per process)
file(GLOB _commandsv2_prosdk_tests CONFIGURE_DEPENDS "${PROSDK_ROOT}/test/prosdk/commands2/*.cpp")
prosdk_add_test(prosdk_commands2_tests SOURCES ${_commandsv2_prosdk_tests}
  "${PROSDK_ROOT}/test/prosdk/dashboard/FakeDashboardStore.cpp"
  INCLUDES "${PROSDK_ROOT}/test/prosdk/dashboard")
