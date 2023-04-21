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

#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-armcortex/startup.hpp>
#include <libhal-armcortex/system_control.hpp>

#include <libhal-lpc40xx/can.hpp>
#include <libhal-lpc40xx/constants.hpp>
#include <libhal-lpc40xx/system_controller.hpp>
#include <libhal-lpc40xx/uart.hpp>

#include "../../hardware_map.hpp"

hal::result<hardware_map> initialize_target()
{
  using namespace hal::literals;
  hal::cortex_m::initialize_data_section();

  HAL_CHECK(hal::lpc40xx::clock::maximum(10.0_MHz));

  auto& clock = hal::lpc40xx::clock::get();
  auto cpu_frequency = clock.get_frequency(hal::lpc40xx::peripheral::cpu);
  static hal::cortex_m::dwt_counter counter(cpu_frequency);

  auto& uart0 = HAL_CHECK((hal::lpc40xx::uart::get<0, 64>(hal::serial::settings{
    .baud_rate = 38400,
  })));

  auto& can = HAL_CHECK((hal::lpc40xx::can::get<2>(hal::can::settings{
    .baud_rate = 1.0_MHz,
  })));

  return hardware_map{
    .console = &uart0,
    .can = &can,
    .clock = &counter,
    .reset = []() { hal::cortex_m::system_control::reset(); },
  };
}
