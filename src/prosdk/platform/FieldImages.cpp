// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Built-in field image registry of the fields library (prosdk/fields/field_images.hpp).
//
// Upstream builds the field images (PNG/JPG, about 9 MB) into the optional `fieldimages` library
// (only with WPILIB_WITH_GUI). prosdk does not vendor them (design/API-DIFFERENCES.md section 4):
// the robot has no display and they do not fit in flash. The registry is therefore empty and
// GetFieldImage() returns the empty view upstream documents for "not available". The field
// metadata, including FieldImage::GetPath() and the pixel bounds, is unchanged.

#include "prosdk/fields/field_images.hpp"

namespace prosdk::fields {

std::span<const FieldImageResource> GetFieldImages() {
  return {};
}

std::string_view GetFieldImage(std::string_view) {
  return {};
}

}  // namespace prosdk::fields
