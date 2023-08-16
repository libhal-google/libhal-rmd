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

#include <libhal-rmd/drc.hpp>

class do_nothing_can : public hal::can
{
public:
  hal::status driver_configure(
    [[maybe_unused]] const settings& p_settings) override
  {
    return hal::success();
  }

  hal::status driver_bus_on() override
  {
    return hal::success();
  }

  hal::result<hal::can::send_t> driver_send(
    [[maybe_unused]] const message_t& p_message) override
  {
    return send_t{};
  }
  void driver_on_receive(hal::callback<handler>) override
  {
  }
};

class do_nothing_steady_clock : public hal::steady_clock
{
public:
  frequency_t driver_frequency() override
  {
    return frequency_t{ .operating_frequency = 10'000'000.0f };
  }
  uptime_t driver_uptime() override
  {
    return uptime_t{ .ticks = m_counter++ };
  }
  std::uint64_t m_counter = 0;
};

int main()
{
  do_nothing_can can;
  do_nothing_steady_clock steady_clock;
  auto router = hal::can_router::create(can).value();
  [[maybe_unused]] auto servo =
    hal::rmd::drc::create(router, steady_clock, 6.0f, 0x140);

  return 0;
}

namespace boost {
void throw_exception(std::exception const& e)
{
  hal::halt();
}
}  // namespace boost
