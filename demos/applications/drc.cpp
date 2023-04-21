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

#include <cinttypes>

#include <libhal-rmd/drc.hpp>
#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include "../hardware_map.hpp"

hal::status application(hardware_map& p_map)
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto& clock = *p_map.clock;
  auto& console = *p_map.console;
  auto& can = *p_map.can;

  hal::print(console, "RMD DRC Smart Servo Application Starting...\n\n");

  auto router = HAL_CHECK(hal::can_router::create(can));
  auto drc = HAL_CHECK(hal::rmd::drc::create(router, clock, 6.0f, 0x141));

  auto print_feedback = [&drc, &console]() {
    (void)drc.feedback_request(hal::rmd::drc::read::status_2);
    (void)drc.feedback_request(hal::rmd::drc::read::multi_turns_angle);
    (void)drc.feedback_request(hal::rmd::drc::read::status_1_and_error_flags);

    hal::print<2048>(console,
                     "[%u] =================================\n"
                     "raw_multi_turn_angle = %f\n"
                     "raw_current = %d\n"
                     "raw_speed = %d\n"
                     "raw_volts = %d\n"
                     "encoder = %d\n"
                     "raw_motor_temperature = %d"
                     "\n"
                     "over_voltage_protection_tripped = %d\n"
                     "over_temperature_protection_tripped = %d\n"
                     "-------\n"
                     "angle() = %f °deg\n"
                     "current() = %f A\n"
                     "speed() = %f rpm\n"
                     "volts() = %f V\n"
                     "temperature() = %f °C\n\n",

                     drc.feedback().message_number,
                     static_cast<float>(drc.feedback().raw_multi_turn_angle),
                     drc.feedback().raw_current,
                     drc.feedback().raw_speed,
                     drc.feedback().raw_volts,
                     drc.feedback().encoder,
                     drc.feedback().raw_motor_temperature,
                     drc.feedback().over_voltage_protection_tripped(),
                     drc.feedback().over_temperature_protection_tripped(),
                     drc.feedback().angle(),
                     drc.feedback().current(),
                     drc.feedback().speed(),
                     drc.feedback().volts(),
                     drc.feedback().temperature());
  };

  HAL_CHECK(hal::delay(clock, 500ms));

  while (true) {
    HAL_CHECK(drc.velocity_control(10.0_rpm));
    (void)hal::delay(clock, 5000ms);
    print_feedback();

    HAL_CHECK(drc.velocity_control(-10.0_rpm));
    (void)hal::delay(clock, 5000ms);
    print_feedback();

    HAL_CHECK(drc.position_control(0.0_deg, 50.0_rpm));
    (void)hal::delay(clock, 5000ms);
    print_feedback();

    HAL_CHECK(drc.position_control(-45.0_deg, 50.0_rpm));
    (void)hal::delay(clock, 5000ms);
    print_feedback();

    HAL_CHECK(drc.position_control(90.0_deg, 50.0_rpm));
    (void)hal::delay(clock, 5000ms);
    print_feedback();

    HAL_CHECK(drc.position_control(180.0_deg, 50.0_rpm));
    (void)hal::delay(clock, 5000ms);
    print_feedback();

    HAL_CHECK(drc.position_control(-360.0_deg, 50.0_rpm));
    (void)hal::delay(clock, 5000ms);
    print_feedback();

    HAL_CHECK(drc.position_control(0.0_deg, 50.0_rpm));
    (void)hal::delay(clock, 5000ms);
    print_feedback();
  }

  return hal::success();
}
