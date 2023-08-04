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

#include <libhal-mock/can.hpp>
#include <libhal-mock/steady_clock.hpp>
#include <libhal-util/enum.hpp>

#include <boost/ut.hpp>

namespace hal::rmd {
namespace {
struct drc_inert_can : public hal::can
{
private:
  status driver_configure([[maybe_unused]] const settings& p_settings) override
  {
    return hal::success();
  }

  status driver_bus_on() override
  {
    return hal::success();
  }

  result<send_t> driver_send(
    [[maybe_unused]] const message_t& p_message) override
  {
    return send_t{};
  }

  void driver_on_receive(
    [[maybe_unused]] hal::callback<handler> p_handler) override
  {
  }
};
}  // namespace

void drc_adaptors_test()
{
  using namespace boost::ut;
  using namespace std::literals;
  using namespace hal::literals;
};
}  // namespace hal::rmd