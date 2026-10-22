// Copyright (c) Sleipnir contributors

#pragma once

#include <prosdk/platform/config.hpp>  // prosdk: configuration

#include <concepts>
#include <type_traits>

#include <prosdk/thirdparty/eigen/Eigen/Core>

#include "prosdk/thirdparty/sleipnir/sleipnir/autodiff/sleipnir_base.hpp"

namespace slp {

template <typename T>
concept SleipnirType = std::derived_from<std::decay_t<T>, SleipnirBase>;

template <typename T>
concept MatrixLike = requires(std::decay_t<T> t) {
  t.rows();
  t.cols();
};

template <typename T>
concept ScalarLike =
    !MatrixLike<T> && std::constructible_from<std::decay_t<T>, int>;

template <typename T>
concept EigenMatrixLike =
    std::derived_from<std::decay_t<T>, Eigen::MatrixBase<std::decay_t<T>>> &&
    MatrixLike<T>;

template <typename T, typename Scalar>
concept SleipnirMatrixLike =
    SleipnirType<T> && MatrixLike<T> &&
    std::same_as<typename std::decay_t<T>::Scalar, Scalar>;

template <typename T, typename Scalar>
concept SleipnirScalarLike =
    SleipnirType<T> && ScalarLike<T> &&
    std::same_as<typename std::decay_t<T>::Scalar, Scalar>;

}  // namespace slp
