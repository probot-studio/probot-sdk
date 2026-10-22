# datalog writer side (vendored by tools/upstream/modules/datalog.py)
prosdk_generated_files(_datalog_src datalog "^src/prosdk/impl/datalog/.*\\.cpp$")
prosdk_add_module(datalog SOURCES ${_datalog_src})

# host tests; the reader side is test support only (test/datalog/support)
prosdk_generated_files(_datalog_tests datalog "^test/datalog/(cpp|support)/.*\\.cpp$")
prosdk_add_test(datalog_tests SOURCES ${_datalog_tests}
  INCLUDES "${PROSDK_ROOT}/test/datalog/support/include")
