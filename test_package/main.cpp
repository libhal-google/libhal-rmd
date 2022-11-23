#include <libhal/can/interface.hpp>
#include <libhal/can/router.hpp>
#include <librmd/drc.hpp>

class do_nothing_can : public hal::can
{
public:
  hal::status driver_configure(
    [[maybe_unused]] const settings& p_settings) noexcept override
  {
    return hal::success();
  }
  hal::status driver_send(
    [[maybe_unused]] const message_t& p_message) noexcept override
  {
    return hal::success();
  }
  hal::status driver_on_receive(
    [[maybe_unused]] std::function<handler> p_handler) noexcept override
  {
    return hal::success();
  }
};

int main()
{
  do_nothing_can can;
  auto router = hal::can_router::create(can).value();
  auto servo = hal::rmd::drc::create(router, 6.0f, 0x140).value();

  return 0;
}