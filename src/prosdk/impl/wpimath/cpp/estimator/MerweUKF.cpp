// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/estimator/MerweUKF.hpp"

#include "prosdk/math/estimator/MerweScaledSigmaPoints.hpp"
#include "prosdk/math/estimator/UnscentedKalmanFilter.hpp"
#include "prosdk/util/SymbolExports.hpp"

namespace prosdk::math {

template class EXPORT_TEMPLATE_DEFINE(WPILIB_DLLEXPORT)
    UnscentedKalmanFilter<3, 3, 1, MerweScaledSigmaPoints<3>>;
template class EXPORT_TEMPLATE_DEFINE(WPILIB_DLLEXPORT)
    UnscentedKalmanFilter<5, 3, 3, MerweScaledSigmaPoints<5>>;

}  // namespace prosdk::math
