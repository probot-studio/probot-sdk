// Copyright (c) Sleipnir contributors

#include "prosdk/thirdparty/sleipnir/sleipnir/optimization/solver/ipm.hpp"

#include "prosdk/thirdparty/sleipnir/sleipnir/util/symbol_exports.hpp"

template SLEIPNIR_DLLEXPORT slp::ExitStatus slp::ipm(
    const IPMMatrixCallbacks<double>& matrix_callbacks,
    std::span<std::function<bool(const IterationInfo<double>& info)>>
        iteration_callbacks,
    const Options& options,
#ifdef SLEIPNIR_ENABLE_BOUND_PROJECTION
    const Eigen::ArrayX<bool>& bound_constraint_mask,
#endif
    Eigen::Vector<double, Eigen::Dynamic>& x);
