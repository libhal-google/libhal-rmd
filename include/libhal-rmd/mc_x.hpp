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

#include <atomic>
#include <cstdint>

#include <libhal-util/can.hpp>
#include <libhal-util/enum.hpp>
#include <libhal-util/map.hpp>
#include <libhal-util/steady_clock.hpp>
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
  /// Commands that can be issued to a RMD-X motor
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

    hal::ampere current() const noexcept;
    hal::rpm speed() const noexcept;
    hal::volts volts() const noexcept;
    hal::celsius temperature() const noexcept;
    hal::degrees angle() const noexcept;
    bool motor_stall() const noexcept;
    bool low_pressure() const noexcept;
    bool over_voltage() const noexcept;
    bool over_current() const noexcept;
    bool power_overrun() const noexcept;
    bool speeding() const noexcept;
    bool over_temperature() const noexcept;
    bool encoder_calibration_error() const noexcept;
  };

  /**
   * @brief Create a new mc_x device driver
   *
   * @param p_router - can router to use
   * @param p_clock - clocked used to determine timeouts
   * @param p_gear_ratio - gear ratio of the motor
   * @param device_id - The CAN ID of the motor
   * @param p_max_response_time - maximum amount of time to wait for a response
   * from the motor.
   * @return result<mc_x> - the mc_x driver or an error (no errors are currently
   * generated from this function)
   */
  [[nodiscard]] static result<mc_x> create(
    hal::can_router& p_router,
    hal::steady_clock& p_clock,
    float p_gear_ratio,
    can::id_t device_id,
    hal::time_duration p_max_response_time = std::chrono::milliseconds(10));

  mc_x(mc_x& p_other) = delete;
  mc_x& operator=(mc_x& p_other) = delete;
  mc_x(mc_x&& p_other);
  mc_x& operator=(mc_x&& p_other);

  /**
   * @brief Get feedback about the motor
   *
   * This object contains cached data from each response returned from the
   * motor. It is updated when any of the control or feedback APIs are called.
   * This object will not update without one of those APIs being called.
   *
   * @return const feedback_t& - information about the motor
   * @throws std::errc::timed_out - if a response is not returned within the max
   * response time set at creation.
   */
  const feedback_t& feedback() const;

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
   * @param speed - speed in rpm's
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

  /**
   * @brief Handle messages from the can bus with this devices ID
   *
   * Meant mostly for testing purposes.
   *
   * @param p_message - message received from the bus
   */
  void operator()(const can::message_t& p_message);

private:
  mc_x(hal::can_router& p_router,
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
}  // namespace hal::rmd
