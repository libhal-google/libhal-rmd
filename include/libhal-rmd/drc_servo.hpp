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

#include "drc.hpp"

namespace hal {
namespace rmd {
class drc_servo;
}  // namespace rmd

/**
 * @brief Create a hal::servo driver using the drc driver
 *
 * @param p_drc - reference to a drc driver. This object's lifetime must
 * exceed the lifetime of the returned object.
 * @param p_max_speed - maximum speed of the motor when moving to an angle
 * @return result<drc_servo> - motor implementation based on the drc driver
 */
result<rmd::drc_servo> make_servo(rmd::drc& p_drc, hal::rpm p_max_speed);
}  // namespace hal

namespace hal::rmd {
/**
 * @brief Servo interface adaptor for DRC
 *
 */
class drc_servo : public hal::servo
{
private:
  drc_servo(drc& p_drc, hal::rpm p_max_speed);
  result<position_t> driver_position(hal::degrees p_position) override;
  friend result<drc_servo> hal::make_servo(rmd::drc& p_drc,
                                           hal::rpm p_max_speed);
  drc* m_drc = nullptr;
  hal::rpm m_max_speed;
};
}  // namespace hal::rmd
