// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/system/LinearSystemLoop.hpp"

#include "prosdk/util/SymbolExports.hpp"

namespace prosdk::math {

template class EXPORT_TEMPLATE_DEFINE(WPILIB_DLLEXPORT)
    LinearSystemLoop<1, 1, 1>;
template class EXPORT_TEMPLATE_DEFINE(WPILIB_DLLEXPORT)
    LinearSystemLoop<2, 1, 1>;

}  // namespace prosdk::math
