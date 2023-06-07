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

#include <libhal/rotation_sensor.hpp>

#include "mc_x.hpp"

namespace hal {
namespace rmd {
class mc_x_rotation;
}  // namespace rmd
/**
 * @brief Create a hal::rotation_sensor driver using the MC-X driver
 *
 * @param p_mc_x - reference to a MC-X driver. This object's lifetime must
 * exceed the lifetime of the returned object.
 * @return result<mc_x_rotation> - rotation sensor implementation based on the
 * MC-X driver
 */
result<rmd::mc_x_rotation> make_rotation_sensor(rmd::mc_x& p_mc_x);
}  // namespace hal

namespace hal::rmd {
/**
 * @brief Reports the rotation of the DRC motor
 *
 */
class mc_x_rotation : public hal::rotation_sensor
{
private:
  mc_x_rotation(mc_x& p_mc_x);
  result<hal::rotation_sensor::read_t> driver_read() override;
  friend result<mc_x_rotation> hal::make_rotation_sensor(rmd::mc_x& p_mc_x);
  mc_x* m_mc_x = nullptr;
};
}  // namespace hal::rmd
