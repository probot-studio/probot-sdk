// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/units/area.hpp"
#include "prosdk/units/base.hpp"
#include "prosdk/units/mass.hpp"

namespace prosdk::units {
UNIT_ADD(moment_of_inertia, kilogram_square_meter, kilogram_square_meters,
         kg_sq_m, compound_unit<mass::kilograms, area::square_meters>)
UNIT_ADD(moment_of_inertia, pound_square_inch, pound_square_inches, lb_sq_in,
         compound_unit<mass::pounds, area::square_inches>)

using namespace moment_of_inertia;
}  // namespace prosdk::units
