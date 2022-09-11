#include <libhal/can/interface.hpp>
#include <librmd/x_series.hpp>

class do_nothing_can : hal::can
{
public:
  hal::status driver_configure(
    [[maybe_unused]] const settings& p_settings) noexcept override
  {
    return hal::success;
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
  hal::rmd::x_series x_servo(can);
  return 0;
}
