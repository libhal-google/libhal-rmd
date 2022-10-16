#pragma once

#include <atomic>
#include <cstdint>

#include <libhal/can/interface.hpp>
#include <libhal/can/router.hpp>
#include <libhal/enum.hpp>
#include <libhal/map.hpp>
#include <libhal/units.hpp>

namespace hal::rmd {
/**
 * @brief Driver for RMD motors equip with the DRC motor drivers
 *
 *
 */
class drc
{
public:
  /// Operating baudrate of all RMD-X smart servos
  static constexpr hertz baudrate_hz = 1'000'000;

  static constexpr float dps_per_lsb_speed = 0.01f;
  static constexpr float dps_per_lsb_angle = 1.0f;

  /// Commands that can be issued to a RMD-X motor
  enum class read_commands : hal::byte
  {
    read_pid_data = 0x30,
    read_acceleration_data_command = 0x33,
    read_encoder_data_command = 0x90,
    read_multi_turns_angle_command = 0x92,
    read_single_circle_angle_command = 0x94,
    read_motor_status_1_and_error_flag_commands = 0x9A,
    read_motor_status_2 = 0x9C,
    read_motor_status_3 = 0x9D,
  };

  /// Commands for actuation the motor
  enum class actuation_commands : hal::byte
  {
    torque_closed_loop_command = 0xA1,
    speed_closed_loop_command = 0xA2,
    position_closed_loop_command_1 = 0xA3,
    position_closed_loop_command_2 = 0xA4,
    position_closed_loop_command_3 = 0xA5,
    position_closed_loop_command_4 = 0xA6,
  };

  /// Commands for changing the settings of the motor
  enum class settings_commands : hal::byte
  {
    write_pid_to_ram_command = 0x31,
    write_pid_to_rom_command = 0x32,
    write_acceleration_data_to_ram_command = 0x34,
    write_encoder_offset_command = 0x91,
    write_current_position_to_rom_as_motor_zero_command = 0x19,
  };

  /// Commands for controlling the motor as a whole
  enum class system_commands : hal::byte
  {
    clear_motor_error_flag_command = 0x9B,
    motor_off_command = 0x80,
    motor_stop_command = 0x81,
    motor_running_command = 0x88,
  };

  /// Structure containing all of the forms of feedback acquired by an RMD-X
  /// motor
  struct feedback_t
  {
    /// Current flowing through the motor windings
    //  (-2048 <-> 2048 ==> -33A <-> 33A)
    std::atomic<int16_t> raw_current;
    /// Rotational velocity of the motor (1 degrees per second (dps)/LSB)
    std::atomic<int16_t> raw_speed;
    /// Motor's supply voltage (0.1V/LSB)
    std::atomic<int16_t> raw_volts;
    /// Signed 16-bit raw encoder count value of the motor
    std::atomic<int16_t> encoder;
    /// Core temperature of the motor (1C/LSB)
    std::atomic<int8_t> raw_motor_temperature;
    /// Error code indicating an over voltage protection event on from the motor
    /// winding output.
    std::atomic<bool> over_voltage_protection_tripped;
    /// Error code indicating an over temperature protection event on from the
    /// motor winding output.
    std::atomic<bool> over_temperature_protection_tripped;

    auto current()
    {
      static constexpr float raw_current_range = 2048.0f;
      static constexpr auto current_range = 33.0_A;
      return hal::map(static_cast<float>(raw_current.load()),
                      std::make_pair(-raw_current_range, raw_current_range),
                      std::make_pair(-current_range, current_range));
    }

    auto speed()
    {
      static constexpr auto velocity_per_lsb = 1.0_deg_per_sec;
      return static_cast<float>(raw_speed.load()) * velocity_per_lsb;
    }

    auto volts()
    {
      static constexpr float volts_per_lsb = 0.1f;
      return static_cast<float>(raw_volts.load()) * volts_per_lsb;
    }

    auto temperature()
    {
      static constexpr float celsius_per_lsb = 1.0f;
      return static_cast<float>(raw_motor_temperature.load()) * celsius_per_lsb;
    }
  };

  template<can::id_t device_id, int bus_number = 0>
  static result<drc&> create(can_router& p_router, float p_gear_ratio)
  {
    static drc drc_driver(p_router, p_gear_ratio, device_id);
    HAL_CHECK(
      p_router.on_receive(device_id, [](const can::message_t& p_message) {
        drc_driver.response_handler(p_message);
      }));
    HAL_CHECK(drc_driver.system_control(system_commands::motor_off_command));
    HAL_CHECK(
      drc_driver.system_control(system_commands::motor_running_command));
    return drc_driver;
  }

  status velocity_control(rpm p_speed) noexcept;
  status position_control(angle p_angle, rpm speed) noexcept;
  status request_feedback(read_commands p_read_command) noexcept;
  status system_control(system_commands p_system_command) noexcept;

  const feedback_t& feedback() const noexcept
  {
    return m_feedback;
  }

private:
  void response_handler(const can::message_t& p_message) noexcept;

  drc(can_router& p_router, float p_gear_ratio, can::id_t p_device_id)
    : m_feedback{}
    , m_router(&p_router)
    , m_gear_ratio(p_gear_ratio)
    , m_device_id(p_device_id)
  {
  }

  can::message_t message(std::array<hal::byte, 8> p_payload) const noexcept
  {
    can::message_t message{ .id = m_device_id, .length = 8 };
    message.payload = p_payload;
    return message;
  }

  int32_t rpm_to_drc_speed(rpm p_rpm, float dps_per_lsb);

  feedback_t m_feedback;
  can_router* m_router;
  float m_gear_ratio;
  can::id_t m_device_id;
};

inline int32_t drc::rpm_to_drc_speed(rpm p_rpm, float dps_per_lsb)
{
  static constexpr float dps_per_rpm = (1.0f / 1.0_deg_per_sec);

  const float dps_float = (p_rpm * m_gear_ratio * dps_per_rpm) / dps_per_lsb;
  const std::int32_t dps = static_cast<std::int32_t>(dps_float);

  return dps;
}

inline status drc::velocity_control(rpm p_rpm) noexcept
{
  const auto speed_data = rpm_to_drc_speed(p_rpm, dps_per_lsb_speed);

  return m_router->bus().send(message({
    value(actuation_commands::speed_closed_loop_command),
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
  const auto angle_data = static_cast<std::int32_t>(angle);
  const auto speed_data = rpm_to_drc_speed(p_rpm, dps_per_lsb_angle);

  return m_router->bus().send(message({
    value(actuation_commands::position_closed_loop_command_2),
    0x00,
    static_cast<hal::byte>((speed_data >> 0) & 0xFF),
    static_cast<hal::byte>((speed_data >> 8) & 0xFF),
    static_cast<hal::byte>((angle_data >> 0) & 0xFF),
    static_cast<hal::byte>((angle_data >> 8) & 0xFF),
    static_cast<hal::byte>((angle_data >> 16) & 0xFF),
    static_cast<hal::byte>((angle_data >> 24) & 0xFF),
  }));
}

inline status drc::request_feedback(read_commands p_read_command) noexcept
{
  return m_router->bus().send(message({
    value(p_read_command),
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
  }));
}

inline status drc::system_control(system_commands p_system_command) noexcept
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

inline void drc::response_handler(const can::message_t& p_message) noexcept
{
  if (p_message.length != 8 || p_message.id != m_device_id) {
    return;
  }

  switch (p_message.payload[0]) {
    case value(read_commands::read_motor_status_2):
    case value(actuation_commands::torque_closed_loop_command):
    case value(actuation_commands::speed_closed_loop_command):
    case value(actuation_commands::position_closed_loop_command_1):
    case value(actuation_commands::position_closed_loop_command_2):
    case value(actuation_commands::position_closed_loop_command_3):
    case value(actuation_commands::position_closed_loop_command_4): {
      auto& data = p_message.payload;
      m_feedback.raw_motor_temperature = data[1];
      m_feedback.raw_current = static_cast<int16_t>((data[3] << 8) | data[2]);
      m_feedback.raw_speed = static_cast<int16_t>((data[5] << 8) | data[4]);
      m_feedback.encoder = static_cast<int16_t>((data[7] << 8) | data[6]);
      break;
    }
    case value(read_commands::read_motor_status_1_and_error_flag_commands): {
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
