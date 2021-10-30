#pragma once

#include <libembeddedhal/can.hpp>
#include <libembeddedhal/driver.hpp>
#include <libembeddedhal/utility/can_network.hpp>
#include <libembeddedhal/utility/enum.hpp>
#include <libembeddedhal/utility/full_scale.hpp>

namespace embed {
struct gear_ratio_t
{
  uint32_t input;
  uint32_t output;
};

class rmd_x : public driver<>
{
public:
  /// Operating baudrate of all RMD-X smart servos
  static constexpr uint32_t baudrate_hz = 1'000'000;

  /// Commands that can be issued to a RMD-X motor
  enum class Commands : uint8_t
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

  rmd_x(can_network& p_network,
        uint32_t p_max_rpm,
        gear_ratio_t p_gear_ratio,
        can::id_t p_device_id = 0x140)
    : m_feedback{}
    , m_node{}
    , m_network(p_network)
    , m_max_rpm(p_max_rpm)
    , m_gear_ratio(p_gear_ratio)
    , m_device_id(p_device_id)
  {}

  bool driver_initialize() override;
  void speed(int32_t speed);
  void angle(int32_t angle /*, full_scale<int32_t> speed */);
  void RequestFeedbackFromMotor();
  feedback_t feedback() const noexcept { return m_feedback; }

private:
  can::message_t to_rmd_message(std::array<uint8_t, 8> p_payload) const noexcept
  {
    can::message_t message{ .id = m_device_id,
                            .length = 8,
                            .payload = p_payload };
    return message;
  }

  feedback_t m_feedback;
  can_network::node_t* m_node;
  can_network& m_network;
  gear_ratio_t m_gear_ratio;
  uint32_t m_max_rpm;
  can::id_t m_device_id;
};

inline bool rmd_x::driver_initialize()
{
  m_network.bus().settings().clock_rate_hz = baudrate_hz;
  bool success = m_network.initialize();

  if (!success) {
    return false;
  }

  m_node = m_network.register_message_id(m_device_id);

  if (!m_node) {
    return false;
  }

  m_network.bus().send(to_rmd_message({ value(Commands::motor_off_command),
                                        0x00,
                                        0x00,
                                        0x00,
                                        0x00,
                                        0x00,
                                        0x00,
                                        0x00 }));

  m_network.bus().send(to_rmd_message({ value(Commands::motor_running_command),
                                        0x00,
                                        0x00,
                                        0x00,
                                        0x00,
                                        0x00,
                                        0x00,
                                        0x00 }));

  return true;
}

inline void rmd_x::speed(int32_t rpm)
{
  int32_t command_speed = rpm * m_gear_ratio.input;
  m_network.bus().send(to_rmd_message({
    value(Commands::speed_closed_loop_command),
    0x00,
    0x00,
    0x00,
    static_cast<uint8_t>((command_speed >> 0) & 0xFF),
    static_cast<uint8_t>((command_speed >> 8) & 0xFF),
    static_cast<uint8_t>((command_speed >> 16) & 0xFF),
    static_cast<uint8_t>((command_speed >> 24) & 0xFF),
  }));
}

inline void rmd_x::angle(int32_t angle /* , full_scale<int32_t> speed */)
{
  int32_t command_speed = 100;
  angle *= m_gear_ratio.input;

  m_network.bus().send(to_rmd_message({
    value(Commands::position_closed_loop_command_2),
    0x00,
    static_cast<uint8_t>((command_speed >> 0) & 0xFF),
    static_cast<uint8_t>((command_speed >> 8) & 0xFF),
    static_cast<uint8_t>((angle >> 0) & 0xFF),
    static_cast<uint8_t>((angle >> 8) & 0xFF),
    static_cast<uint8_t>((angle >> 16) & 0xFF),
    static_cast<uint8_t>((angle >> 24) & 0xFF),
  }));
}
} // namespace embed
