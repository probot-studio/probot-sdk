# Host tests of the examples (design/examples.md): upstream's wpilibcExamples tests, ported to
# test/examples/<Name>, each in its own executable with the example's .cpp tabs (the .ino holds
# only PROSDK_ROBOT_MAIN, which is not compiled here). Host only; not part of libprosdk.a.
foreach(_ex ArmSimulation ElevatorSimulation)
  file(GLOB _ex_src CONFIGURE_DEPENDS "${PROSDK_ROOT}/examples/${_ex}/*.cpp")
  prosdk_add_test(example_${_ex}_tests SOURCES
    "${PROSDK_ROOT}/test/examples/main.cpp"
    "${PROSDK_ROOT}/test/examples/${_ex}/${_ex}Test.cpp" ${_ex_src}
    INCLUDES "${PROSDK_ROOT}/examples/${_ex}")
  if(TARGET example_${_ex}_tests)
    target_compile_definitions(example_${_ex}_tests PRIVATE RUNNING_WPILIB_TESTS=1)
  endif()
endforeach()
