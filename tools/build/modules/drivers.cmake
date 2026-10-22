# drivers: the WPILib device drivers library (vendored by tools/upstream/modules/drivers.py: the
# generated PWM motor controllers and GoBildaPinpoint) plus the hand-written prosdk drivers in
# src/prosdk/drivers (Servo, BTS7960, VNH5019, MPU6050, PCA9685, ProHub; design/drivers.md).
prosdk_generated_files(_drivers_src drivers "^src/prosdk/impl/drivers/.*\\.cpp$")
# *.c: vendored C sources under src/prosdk/drivers keep their upstream names
file(GLOB_RECURSE _drivers_hand_src CONFIGURE_DEPENDS "${PROSDK_ROOT}/src/prosdk/drivers/*.cpp"
     "${PROSDK_ROOT}/src/prosdk/drivers/*.c")
# re-run configure (header_check.py) when a hand-written driver header is added or removed
file(GLOB_RECURSE _drivers_hand_headers CONFIGURE_DEPENDS "${PROSDK_ROOT}/src/prosdk/drivers/*.hpp")
prosdk_add_module(drivers SOURCES ${_drivers_src} ${_drivers_hand_src})

# upstream host tests (Catch2, against the HAL sim backend)
prosdk_generated_files(_drivers_tests drivers "^test/drivers/cpp/.*\\.cpp$")
prosdk_add_test(drivers_tests SOURCES ${_drivers_tests})

# tests of the hand-written drivers (own executable: declared order, HAL sim)
file(GLOB _drivers_prosdk_tests CONFIGURE_DEPENDS "${PROSDK_ROOT}/test/prosdk/drivers/*.cpp")
prosdk_add_test(prosdk_drivers_tests SOURCES ${_drivers_prosdk_tests})
