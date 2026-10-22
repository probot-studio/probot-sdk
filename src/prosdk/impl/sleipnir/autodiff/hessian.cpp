// Copyright (c) Sleipnir contributors

#include "prosdk/thirdparty/sleipnir/sleipnir/autodiff/hessian.hpp"

#include <prosdk/thirdparty/eigen/Eigen/Core>

#include "prosdk/thirdparty/sleipnir/sleipnir/util/symbol_exports.hpp"

template class EXPORT_TEMPLATE_DEFINE(SLEIPNIR_DLLEXPORT)
slp::Hessian<double, Eigen::Lower | Eigen::Upper>;
