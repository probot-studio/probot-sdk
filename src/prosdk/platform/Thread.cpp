// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.

#include "prosdk/platform/Thread.hpp"

#if PROSDK_TARGET_HW
#include <esp_err.h>
#include <esp_pthread.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif

namespace prosdk::platform {

#if PROSDK_TARGET_HW

static_assert(PROSDK_THREAD_PRIORITY < 4,
              "prosdk threads must run below the probot-core control loop (priority 4)");

struct ScopedThreadConfig::Saved {
  bool hadConfig = false;
  esp_pthread_cfg_t config{};
};

ScopedThreadConfig::ScopedThreadConfig(const ThreadConfig& config)
    : m_saved(std::make_unique<Saved>()) {
  m_saved->hadConfig = esp_pthread_get_cfg(&m_saved->config) == ESP_OK;
  esp_pthread_cfg_t cfg = esp_pthread_get_default_config();
  cfg.stack_size = config.stackSize;
  cfg.prio = config.priority < 4 ? config.priority : 3;
  cfg.inherit_cfg = false;
  if (config.name != nullptr) {
    cfg.thread_name = config.name;
  }
  // ThreadConfig uses -1 for "no affinity"; ESP-IDF expects tskNO_AFFINITY
  cfg.pin_to_core = config.core < 0 ? static_cast<int>(tskNO_AFFINITY) : config.core;
  esp_pthread_set_cfg(&cfg);
}

ScopedThreadConfig::~ScopedThreadConfig() {
  if (m_saved->hadConfig) {
    esp_pthread_set_cfg(&m_saved->config);
  } else {
    esp_pthread_cfg_t cfg = esp_pthread_get_default_config();
    esp_pthread_set_cfg(&cfg);
  }
}

#else  // host

struct ScopedThreadConfig::Saved {};

ScopedThreadConfig::ScopedThreadConfig(const ThreadConfig&) {}

ScopedThreadConfig::~ScopedThreadConfig() = default;

#endif

}  // namespace prosdk::platform
