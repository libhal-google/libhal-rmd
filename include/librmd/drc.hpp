#pragma once

#include <atomic>
#include <cstdint>

#include <libhal/can/interface.hpp>
#include <libhal/can/router.hpp>
#include <libhal/enum.hpp>
#include <libhal/map.hpp>
#include <libhal/move_interceptor.hpp>
#include <libhal/units.hpp>

namespace hal::rmd {
/**
 * @brief Driver for RMD motors equip with the DRC motor drivers
 *
 *
 */
class drc : public move_interceptor<drc>
{
public:
  friend class move_interceptor<drc>;

  /// Operating baudrate of all RMD-X smart servos
  static constexpr hertz baudrate_hz = 1'000'000;

  static constexpr float dps_per_lsb_speed = 0.01f;
  static constexpr float dps_per_lsb_angle = 1.0f;

  /// Commands that can be issued to a RMD-X motor
  enum class read : hal::byte
  {
    pid_data = 0x30,
    acceleration_data = 0x33,
    encoder_data = 0x90,
    multi_turns_angle = 0x92,
    single_circle_angle = 0x94,
    status_1_and_error_flags = 0x9A,
    status_2 = 0x9C,
    status_3 = 0x9D,
  };

  /// Commands for actuate the motor
  enum class actuate : hal::byte
  {
    torque = 0xA1,
    speed = 0xA2,
    position_1 = 0xA3,
    position_2 = 0xA4,
    position_3 = 0xA5,
    position_4 = 0xA6,
  };

  /// Commands for updating motor configuration data
  enum class write : hal::byte
  {
    pid_to_ram = 0x31,
    pid_to_rom = 0x32,
    acceleration_data_to_ram = 0x34,
    encoder_offset = 0x91,
    current_position_to_rom_as_motor_zero = 0x19,
  };

  /// Commands for controlling the motor as a whole
  enum class system : hal::byte
  {
    clear_error_flag = 0x9B,
    off = 0x80,
    stop = 0x81,
    running = 0x88,
  };

  /// Structure containing all of the forms of feedback acquired by an RMD-X
  /// motor
  struct feedback_t
  {
    /// Current flowing through the motor windings
    //  (-2048 <-> 2048 ==> -33A <-> 33A)
    int16_t raw_current{ 0 };
    /// Rotational velocity of the motor (1 degrees per second (dps)/LSB)
    int16_t raw_speed{ 0 };
    /// Motor's supply voltage (0.1V/LSB)
    int16_t raw_volts{ 0 };
    /// Signed 16-bit raw encoder count value of the motor
    int16_t encoder{ 0 };
    /// Core temperature of the motor (1C/LSB)
    int8_t raw_motor_temperature{ 0 };
    /// Error code indicating an over voltage protection event on from the motor
    /// winding output.
    bool over_voltage_protection_tripped{ false };
    /// Error code indicating an over temperature protection event on from the
    /// motor winding output.
    bool over_temperature_protection_tripped{ false };

    auto current()
    {
      static constexpr float raw_current_range = 2048.0f;
      static constexpr auto current_range = 33.0_A;
      return hal::map(static_cast<float>(raw_current),
                      std::make_pair(-raw_current_range, raw_current_range),
                      std::make_pair(-current_range, current_range));
    }

    auto speed()
    {
      static constexpr auto velocity_per_lsb = 1.0_deg_per_sec;
      return static_cast<float>(raw_speed) * velocity_per_lsb;
    }

    auto volts()
    {
      static constexpr float volts_per_lsb = 0.1f;
      return static_cast<float>(raw_volts) * volts_per_lsb;
    }

    auto temperature()
    {
      static constexpr float celsius_per_lsb = 1.0f;
      return static_cast<float>(raw_motor_temperature) * celsius_per_lsb;
    }
  };

  static result<drc> create(can_router& p_router,
                            float p_gear_ratio,
                            can::id_t device_id)
  {
    drc drc_driver(p_router, p_gear_ratio, device_id);
    HAL_CHECK(drc_driver.system_control(system::off));
    HAL_CHECK(drc_driver.system_control(system::running));
    return drc_driver;
  }

  drc(drc& p_old_self) = delete;
  drc& operator=(drc& p_old_self) = delete;
  drc(drc&& p_old_self) = default;
  drc& operator=(drc&& p_old_self) = default;

  status velocity_control(rpm p_speed) noexcept;
  status position_control(angle p_angle, rpm speed) noexcept;
  status feedback_request(read p_command) noexcept;
  status system_control(system p_system_command) noexcept;

  const feedback_t& feedback() const noexcept
  {
    return m_feedback;
  }

  void operator()(const can::message_t& p_message) noexcept;

private:
  drc(can_router& p_router, float p_gear_ratio, can::id_t p_device_id)
    : m_feedback{}
    , m_router(&p_router)
    , m_route_item(p_router.add_message_callback(p_device_id))
    , m_gear_ratio(p_gear_ratio)
    , m_device_id(p_device_id)
  {
    intercept(this);
  }

  /**
   * @brief Update the callback location if this object is moved
   *
   * @param p_old_self - the old version of this driver
   */
  void intercept(drc* p_old_self)
  {
    p_old_self->m_route_item.get().handler = std::ref(*this);
  }

  can::message_t message(std::array<hal::byte, 8> p_payload) const noexcept
  {
    can::message_t message{ .id = m_device_id, .length = 8 };
    message.payload = p_payload;
    return message;
  }

  result<int32_t> rpm_to_drc_speed(rpm p_rpm, float p_dps_per_lsb);

  template<std::integral T>
  result<T> bounds_check(std::floating_point auto p_float)
  {
    using float_t = decltype(p_float);
    constexpr auto min = static_cast<float_t>(std::numeric_limits<T>::min());
    constexpr auto max = static_cast<float_t>(std::numeric_limits<T>::max());

    if (min < p_float && p_float < max) {
      return static_cast<T>(p_float);
    }

    return new_error(std::errc::result_out_of_range);
  }

  feedback_t m_feedback{};
  can_router* m_router;
  can_router::route_item m_route_item;
  float m_gear_ratio;
  can::id_t m_device_id;
};

inline result<std::int32_t> drc::rpm_to_drc_speed(rpm p_rpm,
                                                  float p_dps_per_lsb)
{
  static constexpr float dps_per_rpm = (1.0f / 1.0_deg_per_sec);

  const float dps_float = (p_rpm * m_gear_ratio * dps_per_rpm) / p_dps_per_lsb;

  return bounds_check<std::int32_t>(dps_float);
}

inline status drc::velocity_control(rpm p_rpm) noexcept
{
  const auto speed_data = HAL_CHECK(rpm_to_drc_speed(p_rpm, dps_per_lsb_speed));

  return m_router->bus().send(message({
    value(actuate::speed),
    0x00,
    0x00,
    0x00,
    static_cast<hal::byte>((speed_data >> 0) & 0xFF),
    static_cast<hal::byte>((speed_data >> 8) & 0xFF),
    static_cast<hal::byte>((speed_data >> 16) & 0xFF),
    static_cast<hal::byte>((speed_data >> 24) & 0xFF),
  }));
}

inline status drc::position_control(angle p_angle, rpm p_rpm) noexcept
{
  static constexpr float deg_per_lsb = 0.01f;
  const auto angle = (p_angle * m_gear_ratio) / deg_per_lsb;
  const auto angle_data = HAL_CHECK(bounds_check<std::int32_t>(angle));
  const auto speed_data = HAL_CHECK(rpm_to_drc_speed(p_rpm, dps_per_lsb_angle));

  return m_router->bus().send(message({
    value(actuate::position_2),
    0x00,
    static_cast<hal::byte>((speed_data >> 0) & 0xFF),
    static_cast<hal::byte>((speed_data >> 8) & 0xFF),
    static_cast<hal::byte>((angle_data >> 0) & 0xFF),
    static_cast<hal::byte>((angle_data >> 8) & 0xFF),
    static_cast<hal::byte>((angle_data >> 16) & 0xFF),
    static_cast<hal::byte>((angle_data >> 24) & 0xFF),
  }));
}

inline status drc::feedback_request(read p_command) noexcept
{
  return m_router->bus().send(message({
    value(p_command),
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
  }));
}

inline status drc::system_control(system p_system_command) noexcept
{
  return m_router->bus().send(message({
    value(p_system_command),
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
  }));
}

inline void drc::operator()(const can::message_t& p_message) noexcept
{
  if (p_message.length != 8 || p_message.id != m_device_id) {
    return;
  }

  switch (p_message.payload[0]) {
    case value(read::status_2):
    case value(actuate::torque):
    case value(actuate::speed):
    case value(actuate::position_1):
    case value(actuate::position_2):
    case value(actuate::position_3):
    case value(actuate::position_4): {
      auto& data = p_message.payload;
      m_feedback.raw_motor_temperature = data[1];
      m_feedback.raw_current = static_cast<int16_t>((data[3] << 8) | data[2]);
      m_feedback.raw_speed = static_cast<int16_t>((data[5] << 8) | data[4]);
      m_feedback.encoder = static_cast<int16_t>((data[7] << 8) | data[6]);
      break;
    }
    case value(read::status_1_and_error_flags): {
      auto& data = p_message.payload;
      m_feedback.raw_motor_temperature = static_cast<int8_t>(data[1]);
      m_feedback.raw_volts = static_cast<uint16_t>((data[4] << 8) | data[3]);
      m_feedback.over_voltage_protection_tripped = data[7] & 0b1;
      m_feedback.over_temperature_protection_tripped = data[7] & 0b100;
      break;
    }
    default:
      return;
  }

  return;
}
}  // namespace hal::rmd
