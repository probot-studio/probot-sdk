# wpimath (vendored by tools/upstream/modules/wpimath.py): wpi/math, wpi/units and the
# committed nanopb output.
prosdk_generated_files(_wpimath_src wpimath "^src/prosdk/impl/wpimath/.*\\.cpp$")
prosdk_add_module(wpimath SOURCES ${_wpimath_src})

# Host tests: the units tests as their own binary, everything else in wpimath_tests.
prosdk_generated_files(_wpimath_tests wpimath "^test/wpimath/cpp/.*\\.cpp$")
set(_units_tests)
foreach(f IN LISTS _wpimath_tests)
  if(f MATCHES "/test/wpimath/cpp/Units[A-Za-z]*Test\\.cpp$")
    list(APPEND _units_tests "${f}")
  endif()
endforeach()
list(REMOVE_ITEM _wpimath_tests ${_units_tests})
prosdk_add_test(wpimath_tests SOURCES ${_wpimath_tests}
  INCLUDES "${PROSDK_ROOT}/test/wpimath/include")
prosdk_add_test(units_tests SOURCES ${_units_tests} "${PROSDK_ROOT}/test/wpimath/cpp/main.cpp"
  INCLUDES "${PROSDK_ROOT}/test/wpimath/include")
# Eigen has no operator<< here (EIGEN_NO_IO); Catch2 needs one to print Eigen operands.
if(TARGET wpimath_tests)
  target_compile_options(wpimath_tests PRIVATE
    -include "${PROSDK_ROOT}/test/prosdk/support/EigenOstream.hpp")
endif()
