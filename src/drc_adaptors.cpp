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

#include <cmath>

#include <libhal-rmd/drc.hpp>

namespace hal::rmd {
drc_servo::drc_servo(drc& p_drc, hal::rpm p_max_speed)
  : m_drc(&p_drc)
  , m_max_speed(p_max_speed)
{
}

result<hal::servo::position_t> drc_servo::driver_position(
  hal::degrees p_position)
{
  HAL_CHECK(m_drc->position_control(p_position, m_max_speed));
  return hal::servo::position_t{};
}

drc_temperature_sensor::drc_temperature_sensor(drc& p_drc)
  : m_drc(&p_drc)
{
}

result<hal::temperature_sensor::read_t> drc_temperature_sensor::driver_read()
{
  HAL_CHECK(m_drc->feedback_request(hal::rmd::drc::read::status_2));

  return hal::temperature_sensor::read_t{
    .temperature = m_drc->feedback().temperature(),
  };
}

drc_rotation_sensor::drc_rotation_sensor(drc& p_drc)
  : m_drc(&p_drc)
{
}

result<hal::rotation_sensor::read_t> drc_rotation_sensor::driver_read()
{
  HAL_CHECK(m_drc->feedback_request(hal::rmd::drc::read::multi_turns_angle));

  return hal::rotation_sensor::read_t{
    .angle = m_drc->feedback().angle(),
  };
}

result<rmd::drc_rotation_sensor> make_rotation_sensor(rmd::drc& p_drc)
{
  return rmd::drc_rotation_sensor(p_drc);
}

result<rmd::drc_servo> make_servo(rmd::drc& p_drc, hal::rpm p_max_speed)
{
  return rmd::drc_servo(p_drc, std::abs(p_max_speed));
}

result<rmd::drc_temperature_sensor> make_temperature_sensor(rmd::drc& p_drc)
{
  return rmd::drc_temperature_sensor(p_drc);
}

drc_motor::drc_motor(rmd::drc& p_drc, hal::rpm p_max_speed)
  : m_drc(&p_drc)
  , m_max_speed(p_max_speed)
{
}

result<hal::motor::power_t> drc_motor::driver_power(float p_power)
{
  HAL_CHECK(m_drc->velocity_control(m_max_speed * p_power));
  return hal::motor::power_t{};
}

result<drc_motor> make_motor(rmd::drc& p_drc, hal::rpm p_max_speed)
{
  return drc_motor(p_drc, std::abs(p_max_speed));
}

result<int> make_servo(hal::rpm p_max_speed)
{
  return static_cast<int>(5 * p_max_speed);
}
}  // namespace hal::rmd