// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// ESP32-S3 HAL backend: usage reporting. There is no usage/version store on the robot (systemcore
// forwards these to its system server); they are accepted and dropped.

#include "prosdk/hal/UsageReporting.h"

extern "C" {

void HAL_ReportUsage(const struct WPI_String* resource, const struct WPI_String* data) {
  (void)resource;
  (void)data;
}

int32_t HAL_PublishCanVersion(uint8_t busId, uint32_t deviceId, const struct WPI_String* name,
                              const struct WPI_String* version) {
  (void)busId;
  (void)deviceId;
  (void)name;
  (void)version;
  return 0;
}

int32_t HAL_PublishVersion(const struct WPI_String* name, const struct WPI_String* version) {
  (void)name;
  (void)version;
  return 0;
}

int32_t HAL_PublishWpilibVersion(const struct WPI_String* version) {
  (void)version;
  return 0;
}

}  // extern "C"
