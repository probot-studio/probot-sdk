//===- IOSandbox.h ----------------------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef WPIUTIL_WPI_IOSANDBOX_H
#define WPIUTIL_WPI_IOSANDBOX_H

#if defined(LLVM_ENABLE_IO_SANDBOX) && LLVM_ENABLE_IO_SANDBOX

#include "prosdk/util/Compiler.hpp"
#include "prosdk/util/ErrorHandling.hpp"
#include "prosdk/util/SaveAndRestore.hpp"

namespace prosdk::util::sys::sandbox {
inline LLVM_THREAD_LOCAL bool Enabled = false;
struct [[nodiscard, maybe_unused]] ScopedSetting {
  SaveAndRestore<bool> Impl;
};
inline ScopedSetting scopedEnable() { return {{Enabled, true}}; }
inline ScopedSetting scopedDisable() { return {{Enabled, false}}; }
inline void violationIfEnabled() {
  if (Enabled)
    reportFatalInternalError("IO sandbox violation");
}
} // namespace prosdk::util::sys::sandbox

#else

namespace prosdk::util::sys::sandbox {
struct [[nodiscard, maybe_unused]] ScopedSetting {};
inline ScopedSetting scopedEnable() { return {}; }
inline ScopedSetting scopedDisable() { return {}; }
inline void violationIfEnabled() {}
} // namespace prosdk::util::sys::sandbox

#endif

#endif
