# wpilibc: the WPILib robot library (vendored by tools/upstream/modules/wpilibc.py), including the
# generated HID controllers (src/prosdk/impl/wpilibc/generated). Hand-written glue it relies on
# (RobotHost hooks, Preferences store, storage directories, GetWPILibVersion) is in
# src/prosdk/platform and part of the platform module.
prosdk_generated_files(_wpilibc_src wpilibc "^src/prosdk/impl/wpilibc/.*\\.cpp$")
prosdk_add_module(wpilibc SOURCES ${_wpilibc_src})

# host tests (Catch2, against the HAL sim backend), incl. the generated controller tests.
# DataLogTelemetryBackendTest reads its log back with the DataLog reader (datalog test support).
prosdk_generated_files(_wpilibc_tests wpilibc "^test/wpilibc/(cpp|generated)/.*\\.cpp$")
prosdk_generated_files(_wpilibc_datalog_support datalog "^test/datalog/support/.*\\.cpp$")
prosdk_add_test(wpilibc_tests SOURCES ${_wpilibc_tests} ${_wpilibc_datalog_support}
  INCLUDES "${PROSDK_ROOT}/test/wpilibc/include" "${PROSDK_ROOT}/test/datalog/support/include")
