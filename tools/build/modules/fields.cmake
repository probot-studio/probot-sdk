# fields (vendored by tools/upstream/modules/fields.py): Field, FieldTag, FieldImage, the
# generated FieldId registry and the field JSON files as resource sources. The image registry
# (GetFieldImages) is hand-written in src/prosdk/platform/FieldImages.cpp (platform module).
prosdk_generated_files(_fields_src fields "^src/prosdk/impl/fields/.*\\.cpp$")
prosdk_add_module(fields SOURCES ${_fields_src})

prosdk_generated_files(_fields_tests fields "^test/fields/cpp/.*\\.cpp$")
prosdk_add_test(fields_tests SOURCES ${_fields_tests})
