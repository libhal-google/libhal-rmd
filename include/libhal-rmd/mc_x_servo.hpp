// Copyright 2023 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <libhal/servo.hpp>

#include "mc_x.hpp"

namespace hal {
namespace rmd {
class mc_x_servo;
}  // namespace rmd

/**
 * @brief Create a hal::rotation_sensor driver using the MC-X driver
 *
 * @param p_mc_x - reference to a MC-X driver. This object's lifetime must
 * exceed the lifetime of the returned object.
 * @param p_max_speed - maximum speed of the motor when moving to an angle
 * @return result<mc_x_rotation> - rotation sensor implementation based on the
 * MC-X driver
 */
result<rmd::mc_x_servo> make_servo(rmd::mc_x& p_mc_x, hal::rpm p_max_speed);
}  // namespace hal

namespace hal::rmd {
/**
 * @brief Control a mc_x motor driver like a hal::servo
 *
 */
class mc_x_servo : public hal::servo
{
private:
  mc_x_servo(mc_x& p_mc_x, hal::rpm p_max_speed);
  result<hal::servo::position_t> driver_position(
    hal::degrees p_position) override;
  friend result<mc_x_servo> hal::make_servo(rmd::mc_x& p_mc_x,
                                            hal::rpm p_max_speed);
  mc_x* m_mc_x = nullptr;
  hal::rpm m_max_speed;
};
}  // namespace hal::rmd
