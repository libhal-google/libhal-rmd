#pragma once

#include <atomic>
#include <cstdint>

#include <libhal-util/can.hpp>
#include <libhal-util/enum.hpp>
#include <libhal-util/map.hpp>
#include <libhal/can.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>

namespace hal::rmd {
/**
 * @brief Driver for RMD series motors equip with the MC-X motor driver
 *
 */
class mc_x
{
public:
  /// Operating baudrate of all RMD-X smart servos
  static constexpr hertz baudrate_hz = 1'000'000;

  static constexpr float dps_per_lsb_speed = 0.01f;
  static constexpr float dps_per_lsb_angle = 1.0f;

  /// 🟡 Commands that can be issued to a RMD-X motor
  enum class read : hal::byte
  {
    multi_turns_angle = 0x92,
    status_1_and_error_flags = 0x9A,
    status_2 = 0x9C,
  };

  /// Commands for actuate the motor
  enum class actuate : hal::byte
  {
    torque = 0xA1,
    speed = 0xA2,
    position = 0xA5,
  };

  /// Commands for updating motor configuration data
  enum class write : hal::byte
  {
    // None supported currently
  };

  /// Commands for controlling the motor as a whole
  enum class system : hal::byte
  {
    off = 0x80,
    stop = 0x81,
  };

  /// Structure containing all of the forms of feedback acquired by an RMD-X
  /// motor
  struct feedback_t
  {
    /// Error state flag indicating the motor is stalling or has stalled
    static constexpr std::uint16_t motor_stall_mask = 0x0002;
    /// Error state flag indicating the motor is suffering from low pressure...
    /// I'm not sure what that means
    static constexpr std::uint16_t low_pressure_mask = 0x0004;
    /// Error state flag indicating the motor has seen a voltage spike above its
    /// maximum rating.
    static constexpr std::uint16_t over_voltage_mask = 0x0008;
    /// Error state flag indicating the motor drew more current than it was
    /// prepared to handle.
    static constexpr std::uint16_t over_current_mask = 0x0010;
    /// Error state flag indicating the motor drew more power than it was
    /// prepared to handle.
    static constexpr std::uint16_t power_overrun_mask = 0x0040;
    /// NOTE: I don't know what this means exactly...
    static constexpr std::uint16_t speeding_mask = 0x0100;
    /// Error state flag indicating the motor temperature has exceeded its
    /// rating.
    static constexpr std::uint16_t over_temperature_mask = 0x1000;
    /// Error state flag indicating the encoder calibration failed.
    static constexpr std::uint16_t encoder_calibration_error_mask = 0x2000;

    /// Every time a message from our motor is received this number increments.
    /// This can be used to indicate if the feedback has updated since the last
    /// time it was read.
    std::uint32_t message_number = 0;
    /// Represents the multi-turn absolute angle of the encoder relative to its
    /// zero starting point (0.01°/LSB)
    std::int64_t raw_multi_turn_angle{ 0 };
    /// 16-bit value containing error flag information
    std::int16_t raw_error_state{ 0 };
    /// Current flowing through the motor windings
    ///  (-2048 <-> 2048 ==> -33A <-> 33A)
    std::int16_t raw_current{ 0 };
    /// Rotational velocity of the motor (1 degrees per second (dps)/LSB)
    std::int16_t raw_speed{ 0 };
    /// Motor's supply voltage (0.1V/LSB)
    std::int16_t raw_volts{ 0 };
    /// Signed 16-bit raw encoder count value of the motor
    std::int16_t encoder{ 0 };
    /// Core temperature of the motor (1C/LSB)
    std::int8_t raw_motor_temperature{ 0 };

    auto current() const noexcept
    {
      static constexpr auto amps_per_lsb = 0.1_A;
      return static_cast<float>(raw_current) * amps_per_lsb;
    }

    auto speed() const noexcept
    {
      static constexpr auto velocity_per_lsb = 1.0_deg_per_sec;
      return static_cast<float>(raw_speed) * velocity_per_lsb;
    }

    auto volts() const noexcept
    {
      static constexpr float volts_per_lsb = 0.1f;
      return static_cast<float>(raw_volts) * volts_per_lsb;
    }

    auto temperature() const noexcept
    {
      static constexpr float celsius_per_lsb = 1.0f;
      return static_cast<float>(raw_motor_temperature) * celsius_per_lsb;
    }

    auto angle() const noexcept
    {
      return static_cast<float>(raw_multi_turn_angle) * dps_per_lsb_speed;
    }

    bool motor_stall() const noexcept
    {
      return raw_error_state & motor_stall_mask;
    }
    bool low_pressure() const noexcept
    {
      return raw_error_state & low_pressure_mask;
    }
    bool over_voltage() const noexcept
    {
      return raw_error_state & over_voltage_mask;
    }
    bool over_current() const noexcept
    {
      return raw_error_state & over_current_mask;
    }
    bool power_overrun() const noexcept
    {
      return raw_error_state & power_overrun_mask;
    }
    bool speeding() const noexcept
    {
      return raw_error_state & speeding_mask;
    }
    bool over_temperature() const noexcept
    {
      return raw_error_state & over_temperature_mask;
    }
    bool encoder_calibration_error() const noexcept
    {
      return raw_error_state & encoder_calibration_error_mask;
    }
  };

  static result<mc_x> create(
    hal::can_router& p_router,
    hal::steady_clock& p_clock,
    float p_gear_ratio,
    can::id_t device_id,
    hal::time_duration p_max_response_time = std::chrono::milliseconds(10))
  {
    mc_x mc_x_driver(
      p_router, p_clock, p_gear_ratio, device_id, p_max_response_time);
    return mc_x_driver;
  }

  mc_x(mc_x& p_other) = delete;
  mc_x& operator=(mc_x& p_other) = delete;
  mc_x(mc_x&& p_other)
    : m_feedback(std::move(p_other.m_feedback))
    , m_clock(std::move(p_other.m_clock))
    , m_router(std::move(p_other.m_router))
    , m_route_item(std::move(p_other.m_route_item))
    , m_gear_ratio(std::move(p_other.m_gear_ratio))
    , m_device_id(std::move(p_other.m_device_id))
    , m_max_response_time(std::move(p_other.m_max_response_time))
  {
    m_route_item.get().handler = std::ref(*this);
  }

  mc_x& operator=(mc_x&& p_other)
  {
    m_feedback = std::move(p_other.m_feedback);
    m_clock = std::move(p_other.m_clock);
    m_router = std::move(p_other.m_router);
    m_route_item = std::move(p_other.m_route_item);
    m_gear_ratio = std::move(p_other.m_gear_ratio);
    m_device_id = std::move(p_other.m_device_id);
    m_max_response_time = std::move(p_other.m_max_response_time);

    m_route_item.get().handler = std::ref(*this);

    return *this;
  }

  const feedback_t& feedback() const
  {
    return m_feedback;
  }

  status feedback_request(read p_command);
  status velocity_control(rpm p_speed);
  status position_control(degrees p_angle, rpm speed);
  status system_control(system p_system_command);

  void operator()(const can::message_t& p_message);

private:
  mc_x(hal::can_router& p_router,
       hal::steady_clock& p_clock,
       float p_gear_ratio,
       can::id_t p_device_id,
       hal::time_duration p_max_response_time)
    : m_feedback{}
    , m_clock(&p_clock)
    , m_router(&p_router)
    , m_route_item(p_router.add_message_callback(p_device_id))
    , m_gear_ratio(p_gear_ratio)
    , m_device_id(p_device_id)
    , m_max_response_time(p_max_response_time)
  {
    m_route_item.get().handler = std::ref(*this);
  }

  can::message_t message(std::array<hal::byte, 8> p_payload) const
  {
    can::message_t message{ .id = m_device_id, .length = 8 };
    message.payload = p_payload;
    return message;
  }

  result<int32_t> rpm_to_mc_x_speed(rpm p_rpm, float p_dps_per_lsb);

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
  hal::steady_clock* m_clock;
  hal::can_router* m_router;
  hal::can_router::route_item m_route_item;
  float m_gear_ratio;
  can::id_t m_device_id;
  hal::time_duration m_max_response_time;
};

inline result<std::int32_t> mc_x::rpm_to_mc_x_speed(rpm p_rpm,
                                                    float p_dps_per_lsb)
{
  static constexpr float dps_per_rpm = (1.0f / 1.0_deg_per_sec);
  const float dps_float = (p_rpm * m_gear_ratio * dps_per_rpm) / p_dps_per_lsb;
  return bounds_check<std::int32_t>(dps_float);
}

inline status mc_x::velocity_control(rpm p_rpm)
{
  const auto speed_data =
    HAL_CHECK(rpm_to_mc_x_speed(p_rpm, dps_per_lsb_speed));

  HAL_CHECK(m_router->bus().send(message({
    hal::value(actuate::speed),
    0x00,
    0x00,
    0x00,
    static_cast<hal::byte>((speed_data >> 0) & 0xFF),
    static_cast<hal::byte>((speed_data >> 8) & 0xFF),
    static_cast<hal::byte>((speed_data >> 16) & 0xFF),
    static_cast<hal::byte>((speed_data >> 24) & 0xFF),
  })));

  return hal::success();
}

inline status mc_x::position_control(degrees p_angle, rpm p_rpm)
{
  static constexpr float deg_per_lsb = 0.01f;
  const auto angle = (p_angle * m_gear_ratio) / deg_per_lsb;
  const auto angle_data = HAL_CHECK(bounds_check<std::int32_t>(angle));
  const auto speed_data =
    HAL_CHECK(rpm_to_mc_x_speed(std::abs(p_rpm), dps_per_lsb_angle));

  HAL_CHECK(m_router->bus().send(message({
    hal::value(actuate::position),
    0x00,
    static_cast<hal::byte>((speed_data >> 0) & 0xFF),
    static_cast<hal::byte>((speed_data >> 8) & 0xFF),
    static_cast<hal::byte>((angle_data >> 0) & 0xFF),
    static_cast<hal::byte>((angle_data >> 8) & 0xFF),
    static_cast<hal::byte>((angle_data >> 16) & 0xFF),
    static_cast<hal::byte>((angle_data >> 24) & 0xFF),
  })));

  return hal::success();
}

inline status mc_x::feedback_request(read p_command)
{
  HAL_CHECK(m_router->bus().send(message({
    hal::value(p_command),
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
  })));

  return hal::success();
}

inline status mc_x::system_control(system p_system_command)
{
  HAL_CHECK(m_router->bus().send(message({
    hal::value(p_system_command),
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
  })));

  return hal::success();
}

inline void mc_x::operator()(const can::message_t& p_message)
{
  m_feedback.message_number++;

  if (p_message.length != 8 || p_message.id != m_device_id) {
    return;
  }

  switch (p_message.payload[0]) {
    case hal::value(read::status_2):
    case hal::value(actuate::torque):
    case hal::value(actuate::speed):
    case hal::value(actuate::position): {
      auto& data = p_message.payload;
      m_feedback.raw_motor_temperature = static_cast<int8_t>(data[1]);
      m_feedback.raw_current = static_cast<int16_t>((data[3] << 8) | data[2]);
      m_feedback.raw_speed = static_cast<int16_t>((data[5] << 8) | data[4]);
      m_feedback.encoder = static_cast<int16_t>((data[7] << 8) | data[6]);
      break;
    }
    case hal::value(read::status_1_and_error_flags): {
      auto& data = p_message.payload;
      m_feedback.raw_motor_temperature = static_cast<int8_t>(data[1]);
      // data[3] = Brake release command
      m_feedback.raw_volts = static_cast<int16_t>((data[5] << 8) | data[4]);
      auto error_state = data[7] << 8 | data[6];
      m_feedback.raw_error_state = error_state;
      break;
    }
    case hal::value(read::multi_turns_angle): {
      auto& data = p_message.payload;
      m_feedback.raw_multi_turn_angle = static_cast<std::int32_t>(
        data[7] << 24 | data[6] << 16 | data[5] << 8 | data[4]);
      break;
    }
    default:
      return;
  }

  return;
}
}  // namespace hal::rmd
