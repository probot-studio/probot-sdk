// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <prosdk/cxx/expected.hpp>

#include <prosdk/thirdparty/eigen/Eigen/Core>

#include "prosdk/math/linalg/DARE.hpp"

template prosdk::cxx::expected<Eigen::Matrix<double, 2, 2>, prosdk::math::DAREError>
prosdk::math::DARE<2, 3>(const Eigen::Matrix<double, 2, 2>& A,
                      const Eigen::Matrix<double, 2, 3>& B,
                      const Eigen::Matrix<double, 2, 2>& Q,
                      const Eigen::Matrix<double, 3, 3>& R,
                      bool checkPreconditions);
template prosdk::cxx::expected<Eigen::Matrix<double, 2, 2>, prosdk::math::DAREError>
prosdk::math::DARE<2, 3>(const Eigen::Matrix<double, 2, 2>& A,
                      const Eigen::Matrix<double, 2, 3>& B,
                      const Eigen::Matrix<double, 2, 2>& Q,
                      const Eigen::Matrix<double, 3, 3>& R,
                      const Eigen::Matrix<double, 2, 3>& N,
                      bool checkPreconditions);
