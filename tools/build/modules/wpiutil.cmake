# wpiutil (vendored by tools/upstream/modules/wpiutil.py)
prosdk_generated_files(_wpiutil_src wpiutil "^src/prosdk/impl/wpiutil/.*\\.(cpp|c)$")
prosdk_add_module(wpiutil SOURCES ${_wpiutil_src})

prosdk_generated_files(_wpiutil_tests wpiutil "^test/wpiutil/(cpp|generated)/.*\\.cpp$")
prosdk_add_test(wpiutil_tests SOURCES ${_wpiutil_tests}
  INCLUDES "${PROSDK_ROOT}/test/wpiutil/include" "${PROSDK_ROOT}/test/wpiutil/generated")
