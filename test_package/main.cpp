#include <libhal-rmd/drc.hpp>
#include <libhal-util/can.hpp>
#include <libhal/can.hpp>
#include <libhal/functional.hpp>

class do_nothing_can : public hal::can
{
public:
  hal::status driver_configure(
    [[maybe_unused]] const settings& p_settings) override
  {
    return hal::success();
  }
  hal::status driver_send([[maybe_unused]] const message_t& p_message) override
  {
    return hal::success();
  }
  void driver_on_receive(hal::callback<handler>) override
  {
  }
};

int main()
{
  do_nothing_can can;
  auto router = hal::can_router::create(can).value();
  auto servo = hal::rmd::drc::create(router, 6.0f, 0x140).value();

  return 0;
}
