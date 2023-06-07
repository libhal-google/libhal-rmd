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

#include <libhal/temperature_sensor.hpp>

#include "drc.hpp"

namespace hal {
namespace rmd {
class drc_temperature_sensor;
}  // namespace rmd

/**
 * @brief Create a hal::temperature_sensor driver using the drc driver
 *
 * @param p_drc - reference to a drc driver. This object's lifetime must exceed
 * the lifetime of the returned object.
 * @return result<drc_temperature_sensor> - temperature sensor implementation
 * based on the drc driver.
 */
result<rmd::drc_temperature_sensor> make_temperature_sensor(rmd::drc& p_drc);

}  // namespace hal

namespace hal::rmd {
/**
 * @brief Temperature sensor adaptor for DRC motors
 *
 */
class drc_temperature_sensor : public hal::temperature_sensor
{
private:
  drc_temperature_sensor(drc& p_drc);
  result<read_t> driver_read() override;
  friend result<drc_temperature_sensor> hal::make_temperature_sensor(
    rmd::drc& p_drc);
  drc* m_drc = nullptr;
};
}  // namespace hal::rmd
