# apriltag (vendored by tools/upstream/modules/apriltag.py): the prosdk::apriltag C++ classes and
# the AprilTag C library (src/prosdk/impl/apriltag/thirdparty, C sources).
prosdk_generated_files(_apriltag_src apriltag "^src/prosdk/impl/apriltag/.*\\.(cpp|c)$")
prosdk_add_module(apriltag SOURCES ${_apriltag_src})

prosdk_generated_files(_apriltag_tests apriltag "^test/apriltag/cpp/.*\\.cpp$")
prosdk_add_test(apriltag_tests SOURCES ${_apriltag_tests})
