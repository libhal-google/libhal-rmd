#pragma once

#include <cmath>

#include <libhal/motor.hpp>
#include <libhal/rotation_sensor.hpp>
#include <libhal/servo.hpp>
#include <libhal/temperature_sensor.hpp>

#include "mc_x.hpp"

namespace hal::rmd {
/**
 * @brief Control a mc_x motor driver like a hal::servo
 *
 */
class mc_x_servo : public hal::servo
{
public:
  /**
   * @brief Create a servo adapter for a mc_x object
   *
   * @param p_mc_x - DRC object
   * @param p_max_speed - maximum servo movement speed
   * @return result<mc_x_servo>
   */
  static result<mc_x_servo> create(mc_x& p_mc_x, hal::rpm p_max_speed)
  {
    return mc_x_servo(p_mc_x, std::abs(p_max_speed));
  }

private:
  mc_x_servo(mc_x& p_mc_x, hal::rpm p_max_speed)
    : m_mc_x(&p_mc_x)
    , m_max_speed(p_max_speed)
  {
  }

  result<hal::servo::position_t> driver_position(
    hal::degrees p_position) override
  {
    HAL_CHECK(m_mc_x->position_control(p_position, m_max_speed));
    return hal::servo::position_t{};
  }

  mc_x* m_mc_x = nullptr;
  hal::rpm m_max_speed;
};

/**
 * @brief Control a mc_x motor driver like a hal::motor
 *
 */
class mc_x_motor : public hal::motor
{
public:
  /**
   * @brief Create a motor adapter for a mc_x object
   *
   * @param p_mc_x - DRC object
   * @param p_max_speed - maximum motor movement speed
   * @return result<mc_x_motor>
   */
  static result<mc_x_motor> create(mc_x& p_mc_x, hal::rpm p_max_speed)
  {
    return mc_x_motor(p_mc_x, std::abs(p_max_speed));
  }

private:
  mc_x_motor(mc_x& p_mc_x, hal::rpm p_max_speed)
    : m_mc_x(&p_mc_x)
    , m_max_speed(p_max_speed)
  {
  }

  result<hal::motor::power_t> driver_power(float p_power) override
  {
    m_mc_x->velocity_control(m_max_speed * p_power);
    return hal::motor::power_t{};
  }

  mc_x* m_mc_x = nullptr;
  hal::rpm m_max_speed;
};

/**
 * @brief Reports the temperature of the DRC motor
 *
 */
class mc_x_temperature : public hal::temperature_sensor
{
public:
  /**
   * @brief Create a temperature adapter for a mc_x object
   *
   * @param p_mc_x - DRC object
   * @return result<mc_x_temperature>
   */
  static result<mc_x_temperature> create(mc_x& p_mc_x)
  {
    return mc_x_temperature(p_mc_x);
  }

private:
  mc_x_temperature(mc_x& p_mc_x)
    : m_mc_x(&p_mc_x)
  {
  }

  result<hal::temperature_sensor::read_t> driver_read() override
  {
    HAL_CHECK(
      m_mc_x->feedback_request(hal::rmd::mc_x::read::multi_turns_angle));

    return hal::temperature_sensor::read_t{
      .temperature = m_mc_x->feedback().temperature(),
    };
  }

  mc_x* m_mc_x = nullptr;
};

/**
 * @brief Reports the rotation of the DRC motor
 *
 */
class mc_x_rotation : public hal::rotation_sensor
{
public:
  /**
   * @brief Create a rotation adapter for a mc_x object
   *
   * @param p_mc_x - DRC object
   * @return result<mc_x_rotation>
   */
  static result<mc_x_rotation> create(mc_x& p_mc_x)
  {
    return mc_x_rotation(p_mc_x);
  }

private:
  mc_x_rotation(mc_x& p_mc_x)
    : m_mc_x(&p_mc_x)
  {
  }

  result<hal::rotation_sensor::read_t> driver_read() override
  {
    HAL_CHECK(m_mc_x->feedback_request(hal::rmd::mc_x::read::status_2));

    return hal::rotation_sensor::read_t{
      .angle = m_mc_x->feedback().angle(),
    };
  }

  mc_x* m_mc_x = nullptr;
};
}  // namespace hal::rmd
