#pragma once

#include <cstdint>

#include <libhal/can/interface.hpp>
#include <libhal/can/network.hpp>
#include <libhal/enum.hpp>
#include <libhal/units.hpp>

namespace hal::rmd {
struct gear_ratio_t
{
  std::uint32_t input;
  std::uint32_t output;
};
/**
 * @brief Driver for RMD motors equip with the DRC motor drivers
 *
 *
 */
class drc
{
public:
  /// Operating baudrate of all RMD-X smart servos
  static constexpr std::uint32_t baudrate_hz = 1'000'000;

  /// Commands that can be issued to a RMD-X motor
  enum class commands : hal::byte
  {
    read_pid_data = 0x30,
    write_pid_to_ram_command = 0x31,
    write_pid_to_rom_command = 0x32,
    read_acceleration_data_command = 0x33,
    write_acceleration_data_to_ram_command = 0x34,
    read_encoder_data_command = 0x90,
    write_encoder_offset_command = 0x91,
    write_current_position_to_rom_as_motor_zero_command = 0x19,
    read_multi_turns_angle_command = 0x92,
    read_single_circle_angle_command = 0x94,
    read_motor_status_1_and_error_flag_commands = 0x9A,
    clear_motor_error_flag_command = 0x9B,
    read_motor_status_2 = 0x9C,
    read_motor_status_3 = 0x9D,
    motor_off_command = 0x80,
    motor_stop_command = 0x81,
    motor_running_command = 0x88,
    torque_closed_loop_command = 0xA1,
    speed_closed_loop_command = 0xA2,
    position_closed_loop_command_1 = 0xA3,
    position_closed_loop_command_2 = 0xA4,
    position_closed_loop_command_3 = 0xA5,
    position_closed_loop_command_4 = 0xA6,
  };

  /// Structure containing all of the forms of feedback acquired by an RMD-X
  /// motor
  struct feedback_t
  {
    /// Core temperature of the motor (1C/LSB)
    int8_t raw_motor_temperature;
    /// Current flowing through the motor windings
    //  (-2048 <-> 2048 ==> -33A <-> 33A)
    int16_t raw_current;
    /// Rotational velocity of the motor (1 degrees per second (dps)/LSB)
    int16_t raw_speed;
    /// Motor's supply voltage (0.1V/LSB)
    int16_t raw_volts;
    /// Signed 16-bit raw encoder count value of the motor
    int16_t raw_encoder_position;
    /// Error code indicating an over voltage protection event on from the motor
    /// winding output.
    bool over_voltage_protection_tripped;
    /// Error code indicating an over temperature protection event on from the
    /// motor winding output.
    bool over_temperature_protection_tripped;
    /// When true, indicates that an attempt to read the feedback from the motor
    /// failed.
    bool missed_feedback = true;
  };

  result<drc> create(can_network& p_network,
                     std::uint32_t p_max_rpm,
                     gear_ratio_t p_gear_ratio,
                     can::id_t p_device_id = 0x140)
  {
    HAL_CHECK(
      p_network.bus().configure(can::settings{ .clock_rate = baudrate_hz }));

    m_node = HAL_CHECK(p_network.register_message_id(m_device_id));

    HAL_CHECK(
      p_network.bus().send(to_rmd_message(p_device_id,
                                          { value(commands::motor_off_command),
                                            0x00,
                                            0x00,
                                            0x00,
                                            0x00,
                                            0x00,
                                            0x00,
                                            0x00 })));

    HAL_CHECK(p_network.bus().send(
      to_rmd_message(p_device_id,
                     { value(commands::motor_running_command),
                       0x00,
                       0x00,
                       0x00,
                       0x00,
                       0x00,
                       0x00,
                       0x00 })));

    return drc(m_node, p_max_rpm, p_gear_ratio, p_device_id);
  }

  drc(can_network::node_t* p_node,
      std::uint32_t p_max_rpm,
      gear_ratio_t p_gear_ratio,
      can::id_t p_device_id = 0x140)
    : m_feedback{}
    , m_node(p_node)
    , m_max_rpm(p_max_rpm)
    , m_gear_ratio(p_gear_ratio)
    , m_device_id(p_device_id)
  {
  }

  void speed(int32_t p_speed);
  void angle(int32_t p_angle /*, full_scale<int32_t> speed */);
  feedback_t feedback() const noexcept
  {
    return m_feedback;
  }

private:
  static can::message_t to_rmd_message(
    can::id_t p_id,
    std::array<hal::byte, 8> p_payload) noexcept
  {
    can::message_t message{ .id = p_id, .length = 8 };
    message.payload = p_payload;
    return message;
  }
  can::message_t to_rmd_message(
    std::array<hal::byte, 8> p_payload) const noexcept
  {
    can::message_t message{ .id = m_device_id, .length = 8 };
    message.payload = p_payload;
    return message;
  }

  feedback_t m_feedback;
  can_network::node_t* m_node;
  gear_ratio_t m_gear_ratio;
  std::uint32_t m_max_rpm;
  can::id_t m_device_id;
};

inline void drc::speed(int32_t p_rpm)
{
  int32_t command_speed = p_rpm * m_gear_ratio.input;
  m_network.bus().send(to_rmd_message({
    value(commands::speed_closed_loop_command),
    0x00,
    0x00,
    0x00,
    static_cast<hal::byte>((command_speed >> 0) & 0xFF),
    static_cast<hal::byte>((command_speed >> 8) & 0xFF),
    static_cast<hal::byte>((command_speed >> 16) & 0xFF),
    static_cast<hal::byte>((command_speed >> 24) & 0xFF),
  }));
}

inline void drc::angle(int32_t p_angle /* , full_scale<int32_t> speed */)
{
  int32_t command_speed = 100;
  p_angle *= m_gear_ratio.input;

  m_network.bus().send(to_rmd_message({
    value(commands::position_closed_loop_command_2),
    0x00,
    static_cast<hal::byte>((command_speed >> 0) & 0xFF),
    static_cast<hal::byte>((command_speed >> 8) & 0xFF),
    static_cast<hal::byte>((p_angle >> 0) & 0xFF),
    static_cast<hal::byte>((p_angle >> 8) & 0xFF),
    static_cast<hal::byte>((p_angle >> 16) & 0xFF),
    static_cast<hal::byte>((p_angle >> 24) & 0xFF),
  }));
}
}  // namespace hal::rmd
