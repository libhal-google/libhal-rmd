// Copyright 2023 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <cstdint>

#include <libhal-canrouter/can_router.hpp>
#include <libhal/angular_velocity_sensor.hpp>
#include <libhal/can.hpp>
#include <libhal/motor.hpp>
#include <libhal/rotation_sensor.hpp>
#include <libhal/servo.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/temperature_sensor.hpp>
#include <libhal/units.hpp>

namespace hal::rmd {
/**
 * @brief Driver for RMD motors equip with the DRC motor drivers
 *
 */
class drc
{
public:
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

    hal::ampere current() const noexcept;
    hal::rpm speed() const noexcept;
    hal::volts volts() const noexcept;
    hal::celsius temperature() const noexcept;
    hal::degrees angle() const noexcept;

    /**
     * @brief Return if the motor has detected an over voltage event
     *
     * In order for this field to be updated a feedback_request with
     * status_1_and_error_flags must be issued.
     *
     * @return true - over voltage protection tripped
     * @return false - over voltage protection has not tripped
     */
    bool over_voltage_protection_tripped() const noexcept;

    /**
     * @brief Return if the motor has detected an over temperature event
     *
     * In order for this field to be updated a feedback_request with
     * status_1_and_error_flags must be issued.
     *
     * @return true - over temperature protection tripped
     * @return false - over temperature protection has not tripped
     */
    bool over_temperature_protection_tripped() const noexcept;
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
    hal::time_duration p_max_response_time = std::chrono::milliseconds(10));

  drc(drc& p_other) = delete;
  drc& operator=(drc& p_other) = delete;
  drc(drc&& p_other) noexcept;
  drc& operator=(drc&& p_other) noexcept;

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

  const feedback_t& feedback() const;

  /**
   * @brief Handle messages from the canbus with this devices ID
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
      hal::time_duration p_max_response_time);

  friend struct response_waiter;

  feedback_t m_feedback{};
  hal::steady_clock* m_clock;
  hal::can_router* m_router;
  hal::can_router::route_item m_route_item;
  float m_gear_ratio;
  can::id_t m_device_id;
  hal::time_duration m_max_response_time;
};

/**
 * @brief Rotation sensor adaptor for DRC motors
 *
 */
class drc_rotation_sensor : public hal::rotation_sensor
{
private:
  drc_rotation_sensor(drc& p_drc);
  result<hal::rotation_sensor::read_t> driver_read() override;
  friend result<drc_rotation_sensor> make_rotation_sensor(drc& p_drc);
  drc* m_drc = nullptr;
};

/**
 * @brief Create a hal::rotation_sensor driver using the drc driver
 *
 * @param p_drc - reference to a drc driver. This object's lifetime must
 * exceed the lifetime of the returned object.
 * @return drc_rotation_sensor - motor implementation based on the drc driver
 */
result<drc_rotation_sensor> make_rotation_sensor(drc& p_drc);

/**
 * @brief Temperature sensor adaptor for DRC motors
 *
 */
class drc_temperature_sensor : public hal::temperature_sensor
{
private:
  drc_temperature_sensor(drc& p_drc);
  result<read_t> driver_read() override;
  friend result<drc_temperature_sensor> make_temperature_sensor(drc& p_drc);
  drc* m_drc = nullptr;
};
/**
 * @brief Create a hal::temperature_sensor driver using the drc driver
 *
 * @param p_drc - reference to a drc driver. This object's lifetime must exceed
 * the lifetime of the returned object.
 * @return result<drc_temperature_sensor> - temperature sensor implementation
 * based on the drc driver.
 */
result<drc_temperature_sensor> make_temperature_sensor(drc& p_drc);

/**
 * @brief Motor interface adaptor for DRC
 *
 */
class drc_motor : public hal::motor
{
public:
  friend result<drc_motor> make_motor(drc& p_drc, hal::rpm p_max_speed);

private:
  drc_motor(drc& p_drc, hal::rpm p_max_speed);
  result<power_t> driver_power(float p_power) override;

  drc* m_drc = nullptr;
  hal::rpm m_max_speed;
};

/**
 * @brief Create a hal::motor implementation from the drc driver
 *
 * @param p_drc - reference to a drc driver. This object's lifetime must NOT
 * exceed the lifetime of the return drc motor.
 * @param p_max_speed - maximum speed of the motor represented by +1.0 and
 * -1.0
 * @return drc_motor - motor implementation based on the drc driver
 */
result<drc_motor> make_motor(drc& p_drc, hal::rpm p_max_speed);

/**
 * @brief Servo interface adaptor for DRC
 *
 */
class drc_servo : public hal::servo
{
private:
  drc_servo(drc& p_drc, hal::rpm p_max_speed);
  result<position_t> driver_position(hal::degrees p_position) override;
  friend result<drc_servo> make_servo(drc& p_drc, hal::rpm p_max_speed);
  drc* m_drc = nullptr;
  hal::rpm m_max_speed;
};

/**
 * @brief Create a hal::servo driver using the drc driver
 *
 * @param p_drc - reference to a drc driver. This object's lifetime must
 * exceed the lifetime of the returned object.
 * @param p_max_speed - maximum speed of the servo when moving to an angle
 * @return result<drc_servo> - servo implementation based on the drc driver
 */
result<drc_servo> make_servo(drc& p_drc, hal::rpm p_max_speed);

/**
 * @brief angular velocity sensor adaptor for DRC
 *
 */
class drc_angular_velocity_sensor : public hal::angular_velocity_sensor
{
private:
  drc_angular_velocity_sensor(drc& p_drc);
  result<angular_velocity_sensor::read_t> driver_read() override;
  friend result<drc_angular_velocity_sensor> make_angular_velocity_sensor(
    drc& p_drc);
  drc* m_drc = nullptr;
};

/**
 * @brief Create a hal::angular_velocity_sensor driver using the drc driver
 *
 * @param p_drc - reference to a drc driver. This object's lifetime must exceed
 * the lifetime of the returned object
 * @return result<angular_velocity_sensor> - angular_velocity_sensor
 * implementation based on the drc driver
 */
result<drc_angular_velocity_sensor> make_angular_velocity_sensor(drc& p_drc);

}  // namespace hal::rmd

namespace hal {
using rmd::make_angular_velocity_sensor;
using rmd::make_motor;
using rmd::make_rotation_sensor;
using rmd::make_servo;
using rmd::make_temperature_sensor;
}  // namespace hal
