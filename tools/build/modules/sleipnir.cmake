# sleipnir (vendored by tools/upstream/modules/sleipnir.py): explicit template instantiations
# (src/*.cpp); configuration from src/prosdk/platform/config.hpp.
prosdk_generated_files(_sleipnir_src sleipnir "^src/prosdk/impl/sleipnir/.*\\.cpp$")
prosdk_add_module(sleipnir SOURCES ${_sleipnir_src})
