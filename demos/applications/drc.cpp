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
  auto drc = HAL_CHECK(hal::rmd::drc::create(router, 6.0f, 0x142));

  while (true) {
    HAL_CHECK(hal::delay(clock, 500ms));

    drc.velocity_control(5.0_rpm);

    HAL_CHECK(hal::delay(clock, 2000ms));
    hal::print<64>(console,
                   "[%zu] raw_speed = %" PRId16 "\n",
                   drc.feedback().message_number,
                   drc.feedback().raw_speed);

    drc.velocity_control(-5.0_rpm);

    HAL_CHECK(hal::delay(clock, 2000ms));
    hal::print<64>(console,
                   "[%zu] raw_speed = %" PRId16 "\n",
                   drc.feedback().message_number,
                   drc.feedback().raw_speed);
  }

  return hal::success();
}
