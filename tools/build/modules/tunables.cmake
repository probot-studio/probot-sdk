# tunables (vendored by tools/upstream/modules/tunables.py)
prosdk_generated_files(_tunables_src tunables "^src/prosdk/impl/tunables/.*\\.cpp$")
prosdk_add_module(tunables SOURCES ${_tunables_src})

prosdk_generated_files(_tunables_tests tunables "^test/tunables/cpp/.*\\.cpp$")
prosdk_add_test(tunables_tests SOURCES ${_tunables_tests})
