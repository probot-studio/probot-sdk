# telemetry (vendored by tools/upstream/modules/telemetry.py)
prosdk_generated_files(_telemetry_src telemetry "^src/prosdk/impl/telemetry/.*\\.cpp$")
prosdk_add_module(telemetry SOURCES ${_telemetry_src})

prosdk_generated_files(_telemetry_tests telemetry "^test/telemetry/cpp/.*\\.cpp$")
prosdk_add_test(telemetry_tests SOURCES ${_telemetry_tests})
