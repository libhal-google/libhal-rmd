#pragma once

#include <atomic>
#include <cstdint>

#include <libhal-util/bit.hpp>
#include <libhal-util/can.hpp>
#include <libhal-util/enum.hpp>
#include <libhal-util/map.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <libhal/servo.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>

namespace hal::rmd {
/**
 * @brief Driver for RMD motors equip with the DRC motor drivers
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
  enum class read : hal::byte
  {
    /**
     * @brief Status1 + error flag information read request command
     *
     * Sending this request will update the following fields in the feedback_t:
     *
     *    - raw_multi_turn_angle
     */
    multi_turns_angle = 0x92,
    /**
     * @brief Status1 + error flag information read request command
     *
     * Sending this request will update the following fields in the feedback_t:
     *
     *    - raw_motor_temperature
     *    - over_voltage_protection_tripped
     *    - over_temperature_protection_tripped
     */
    status_1_and_error_flags = 0x9A,
    /**
     * @brief Status2 read request command
     *
     * Sending this request will update the following fields in the feedback_t:
     *
     *    - raw_motor_temperature
     *    - raw_current
     *    - raw_speed
     *    - encoder
     */
    status_2 = 0x9C,
  };

  /// Commands for actuate the motor
  enum class actuate : hal::byte
  {
    speed = 0xA2,
    position_2 = 0xA4,
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
    /// Every time a message from our motor is received this number increments.
    /// This can be used to indicate if the feedback has updated since the last
    /// time it was read.
    std::uint32_t message_number = 0;
    /// Raw multi-turn angle (0.01°/LSB)
    std::int64_t raw_multi_turn_angle{ 0 };
    /// Current flowing through the motor windings
    /// (-2048 <-> 2048 ==> -33A <-> 33A)
    std::int16_t raw_current{ 0 };
    /// Rotational velocity of the motor (1 degrees per second (dps)/LSB)
    std::int16_t raw_speed{ 0 };
    /// Motor's supply voltage (0.1V/LSB)
    std::int16_t raw_volts{ 0 };
    /// Signed 16-bit raw encoder count value of the motor
    std::int16_t encoder{ 0 };
    /// Core temperature of the motor (1C/LSB)
    std::int8_t raw_motor_temperature{ 0 };
    /// 8-bit value containing error flag information
    std::uint8_t raw_error_state{ 0 };

    static constexpr std::uint8_t over_voltage_protection_tripped_mask = 0b1;
    static constexpr std::uint8_t over_temperature_protection_tripped_mask =
      0b100;

    auto current() const noexcept
    {
      static constexpr float raw_current_range = 2048.0f;
      static constexpr auto current_range = 33.0_A;
      return hal::map(static_cast<float>(raw_current),
                      std::make_pair(-raw_current_range, raw_current_range),
                      std::make_pair(-current_range, current_range));
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

    /**
     * @brief Return if the motor has detected an over voltage event
     *
     * In order for this field to be updated a feedback_request with
     * status_1_and_error_flags must be issued.
     *
     * @return true - over voltage protection tripped
     * @return false - over voltage protection has not tripped
     */
    bool over_voltage_protection_tripped() const noexcept
    {
      return raw_error_state & over_temperature_protection_tripped_mask;
    }

    /**
     * @brief Return if the motor has detected an over temperature event
     *
     * In order for this field to be updated a feedback_request with
     * status_1_and_error_flags must be issued.
     *
     * @return true - over temperature protection tripped
     * @return false - over temperature protection has not tripped
     */
    bool over_temperature_protection_tripped() const noexcept
    {
      return raw_error_state & over_temperature_protection_tripped_mask;
    }
  };

  /**
   * @brief Create a new drc device driver
   *
   * This factory function will power cycle the motor
   *
   * @param p_router - can router to use
   * @param p_clock - clocked used to determine timeouts
   * @param p_gear_ratio - gear ratio of the motor
   * @param device_id - The CAN ID of the motor
   * @param p_max_response_time - maximum amount of time to wait for a response
   * from the motor.
   * @return result<drc> - the drc driver or an error
   * @throws std::errc::timed_out - a response is not returned within the max
   * response time when attempting to power cycle.
   */
  [[nodiscard]] static result<drc> create(
    hal::can_router& p_router,
    hal::steady_clock& p_clock,
    float p_gear_ratio,
    can::id_t device_id,
    hal::time_duration p_max_response_time = std::chrono::milliseconds(10))
  {
    drc drc_driver(
      p_router, p_clock, p_gear_ratio, device_id, p_max_response_time);
    HAL_CHECK(drc_driver.system_control(system::off));
    HAL_CHECK(drc_driver.system_control(system::running));
    return drc_driver;
  }

  drc(drc& p_other) = delete;
  drc& operator=(drc& p_other) = delete;
  drc(drc&& p_other)
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

  drc& operator=(drc&& p_other)
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

  /**
   * @brief Request feedback from the motor
   *
   * @param p_command - the request to command the motor to respond with
   * @return status - success or failure
   * @throws std::errc::timed_out - if a response is not returned within the max
   * response time set at creation.
   */
  [[nodiscard]] status feedback_request(read p_command);

  /**
   * @brief Rotate motor shaft at the designated speed
   *
   * @param p_speed - speed in rpm to move the motor shaft at. Positive values
   * rotate the motor shaft clockwise, negative values rotate the motor shaft
   * counter-clockwise assuming you are looking directly at the motor shaft.
   * @return status - success or failure
   * @throws std::errc::timed_out - if a response is not returned within the max
   * response time set at creation.
   */
  [[nodiscard]] status velocity_control(rpm p_speed);

  /**
   * @brief Move motor shaft to a specific angle
   *
   * @param p_angle - angle position in degrees to move to
   * @param speed - maximum speed in rpm's
   * @return status - success or failure
   * @throws std::errc::timed_out - if a response is not returned within the max
   * response time set at creation.
   */
  [[nodiscard]] status position_control(degrees p_angle, rpm speed);

  /**
   * @brief Send system control commands to the device
   *
   * @param p_system_command - system control command to send to the device
   * @return status - success or failure status
   * @throws std::errc::timed_out - if a response is not returned within the max
   * response time set at creation.
   */
  [[nodiscard]] status system_control(system p_system_command);

  const feedback_t& feedback() const
  {
    return m_feedback;
  }

  /**
   * @brief Handle messages from the CANBUS with this devices ID
   *
   * Meant mostly for testing purposes.
   *
   * @param p_message - message received from the bus
   */
  void operator()(const can::message_t& p_message);

private:
  drc(hal::can_router& p_router,
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

    return new_error(std::errc::invalid_argument,
                     hal::servo::range_error{
                       .min = min,
                       .max = max,
                     });
  }

  struct response_waiter
  {
    using message_t = decltype(feedback_t::message_number);
    response_waiter(drc* p_this)
      : m_this(p_this)
    {
      m_original_message_number = m_this->m_feedback.message_number;
    }
    hal::status wait()
    {
      auto timeout = HAL_CHECK(
        hal::create_timeout(*m_this->m_clock, m_this->m_max_response_time));
      while (true) {
        if (m_original_message_number != m_this->m_feedback.message_number) {
          return hal::success();
        }
        HAL_CHECK(timeout());
      }
    }
    message_t m_original_message_number{ 0 };
    drc* m_this;
  };

  feedback_t m_feedback{};
  hal::steady_clock* m_clock;
  hal::can_router* m_router;
  hal::can_router::route_item m_route_item;
  float m_gear_ratio;
  can::id_t m_device_id;
  hal::time_duration m_max_response_time;
};

inline result<std::int32_t> drc::rpm_to_drc_speed(rpm p_rpm,
                                                  float p_dps_per_lsb)
{
  static constexpr float dps_per_rpm = (1.0f / 1.0_deg_per_sec);

  const float dps_float = (p_rpm * m_gear_ratio * dps_per_rpm) / p_dps_per_lsb;

  return bounds_check<std::int32_t>(dps_float);
}

inline status drc::velocity_control(rpm p_rpm)
{
  const auto speed_data = HAL_CHECK(rpm_to_drc_speed(p_rpm, dps_per_lsb_speed));

  response_waiter response(this);

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

  return response.wait();
}

inline status drc::position_control(degrees p_angle, rpm p_rpm)
{
  static constexpr float deg_per_lsb = 0.01f;
  const auto angle = (p_angle * m_gear_ratio) / deg_per_lsb;
  const auto angle_data = HAL_CHECK(bounds_check<std::int32_t>(angle));
  const auto speed_data = HAL_CHECK(rpm_to_drc_speed(p_rpm, dps_per_lsb_angle));

  response_waiter response(this);

  HAL_CHECK(m_router->bus().send(message({
    hal::value(actuate::position_2),
    0x00,
    static_cast<hal::byte>((speed_data >> 0) & 0xFF),
    static_cast<hal::byte>((speed_data >> 8) & 0xFF),
    static_cast<hal::byte>((angle_data >> 0) & 0xFF),
    static_cast<hal::byte>((angle_data >> 8) & 0xFF),
    static_cast<hal::byte>((angle_data >> 16) & 0xFF),
    static_cast<hal::byte>((angle_data >> 24) & 0xFF),
  })));

  return response.wait();
}

inline status drc::feedback_request(read p_command)
{
  response_waiter response(this);

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

  return response.wait();
}

inline status drc::system_control(system p_system_command)
{
  response_waiter response(this);

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

  return response.wait();
}

inline void drc::operator()(const can::message_t& p_message)
{
  m_feedback.message_number++;

  if (p_message.length != 8 || p_message.id != m_device_id) {
    return;
  }

  switch (p_message.payload[0]) {
    case hal::value(read::status_2):
    case hal::value(actuate::speed):
    case hal::value(actuate::position_2): {
      auto& data = p_message.payload;
      m_feedback.raw_motor_temperature = static_cast<std::int8_t>(data[1]);
      m_feedback.raw_current =
        static_cast<std::int16_t>((data[3] << 8) | data[2] << 0);
      m_feedback.raw_speed =
        static_cast<std::int16_t>((data[5] << 8) | data[4] << 0);
      m_feedback.encoder =
        static_cast<std::int16_t>((data[7] << 8) | data[6] << 0);
      break;
    }
    case hal::value(read::status_1_and_error_flags): {
      auto& data = p_message.payload;
      m_feedback.raw_motor_temperature = static_cast<int8_t>(data[1]);
      m_feedback.raw_volts =
        static_cast<std::int16_t>((data[4] << 8) | data[3]);
      m_feedback.raw_error_state = data[7];
      break;
    }
    case hal::value(read::multi_turns_angle): {
      auto& data = p_message.payload;

      m_feedback.raw_multi_turn_angle = hal::bit::value(0U)
                                          .insert<bit::byte_m<0>>(data[1])
                                          .insert<bit::byte_m<1>>(data[2])
                                          .insert<bit::byte_m<2>>(data[3])
                                          .insert<bit::byte_m<3>>(data[4])
                                          .insert<bit::byte_m<4>>(data[5])
                                          .insert<bit::byte_m<5>>(data[6])
                                          .insert<bit::byte_m<6>>(data[7])
                                          .to<std::int64_t>();
      break;
    }
    default:
      return;
  }

  return;
}
}  // namespace hal::rmd
