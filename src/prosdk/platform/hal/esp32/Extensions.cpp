// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// ESP32-S3 HAL backend: extensions. There is no dynamic loading on the ESP32-S3, so
// HAL_LoadExtensions loads nothing; the in-process registry (HAL_RegisterExtension and its
// listeners) works as on the host. HAL_OnShutdown is in HAL.cpp.

#include "prosdk/hal/Extensions.h"

#include <utility>
#include <vector>

#include "prosdk/util/mutex.hpp"

namespace {
prosdk::util::mutex gRegistryMutex;
std::vector<std::pair<const char*, void*>> gRegistry;
std::vector<std::pair<void*, void (*)(void*, const char*, void*)>> gListeners;
}  // namespace

extern "C" {

int HAL_LoadExtensions(void) {
  return 0;
}

int HAL_LoadOneExtension(const char* library) {
  (void)library;
  return -1;  // no shared libraries on the ESP32-S3
}

void HAL_RegisterExtension(const char* name, void* data) {
  std::scoped_lock lock(gRegistryMutex);
  gRegistry.emplace_back(name, data);
  for (auto&& listener : gListeners) {
    listener.second(listener.first, name, data);
  }
}

void HAL_RegisterExtensionListener(void* param,
                                   void (*func)(void*, const char* name, void* data)) {
  std::scoped_lock lock(gRegistryMutex);
  gListeners.emplace_back(param, func);
  for (auto&& extension : gRegistry) {
    func(param, extension.first, extension.second);
  }
}

void HAL_SetShowExtensionsNotFoundMessages(HAL_Bool showMessage) {
  (void)showMessage;
}

}  // extern "C"
