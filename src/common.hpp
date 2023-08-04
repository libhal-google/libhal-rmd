#pragma once

#include <array>

#include <libhal/can.hpp>

namespace hal {
inline constexpr can::message_t message(hal::can::id_t p_device_id,
                                        std::array<hal::byte, 8> p_payload)
{
  can::message_t message{ .id = p_device_id, .length = 8 };
  message.payload = p_payload;
  return message;
}
}  // namespace hal
