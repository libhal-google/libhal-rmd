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
  hal::result<frequency_t> driver_frequency() override
  {
    return frequency_t{ .operating_frequency = 10'000'000.0f };
  }
  hal::result<uptime_t> driver_uptime() override
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
