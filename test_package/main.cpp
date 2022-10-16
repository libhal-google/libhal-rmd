#include <libhal/can/interface.hpp>
#include <libhal/can/router.hpp>
#include <libhal/static_memory_resource.hpp>
#include <librmd/drc.hpp>

class do_nothing_can : hal::can
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
  hal::static_memory_resource<1024> memory_resource;

  hal::can_router router(can, memory_resource);
  auto& servo = hal::rmd::drc::create<0x140, 0>(router, 6.0f).value();

  return 0;
}
