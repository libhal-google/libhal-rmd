#pragma once

#include <cmath>

#include <libhal/motor.hpp>
#include <libhal/rotation_sensor.hpp>
#include <libhal/servo.hpp>
#include <libhal/temperature_sensor.hpp>

#include "drc.hpp"

namespace hal::rmd {
/**
 * @brief Control a drc motor driver like a hal::servo
 *
 */
class drc_servo : public hal::servo
{
public:
  /**
   * @brief Create a servo adapter for a drc object
   *
   * @param p_drc - DRC object
   * @param p_max_speed - maximum servo movement speed
   * @return result<drc_servo>
   */
  static result<drc_servo> create(drc& p_drc, hal::rpm p_max_speed)
  {
    return drc_servo(p_drc, std::abs(p_max_speed));
  }

private:
  drc_servo(drc& p_drc, hal::rpm p_max_speed)
    : m_drc(&p_drc)
    , m_max_speed(p_max_speed)
  {
  }

  result<hal::servo::position_t> driver_position(
    hal::degrees p_position) override
  {
    HAL_CHECK(m_drc->position_control(p_position, m_max_speed));
    return hal::servo::position_t{};
  }

  drc* m_drc = nullptr;
  hal::rpm m_max_speed;
};

/**
 * @brief Control a drc motor driver like a hal::motor
 *
 */
class drc_motor : public hal::motor
{
public:
  /**
   * @brief Create a motor adapter for a drc object
   *
   * @param p_drc - DRC object
   * @param p_max_speed - maximum motor movement speed
   * @return result<drc_motor>
   */
  static result<drc_motor> create(drc& p_drc, hal::rpm p_max_speed)
  {
    return drc_motor(p_drc, std::abs(p_max_speed));
  }

private:
  drc_motor(drc& p_drc, hal::rpm p_max_speed)
    : m_drc(&p_drc)
    , m_max_speed(p_max_speed)
  {
  }

  result<hal::motor::power_t> driver_power(float p_power) override
  {
    m_drc->velocity_control(m_max_speed * p_power);
    return hal::motor::power_t{};
  }

  drc* m_drc = nullptr;
  hal::rpm m_max_speed;
};

/**
 * @brief Reports the temperature of the DRC motor
 *
 */
class drc_temperature : public hal::temperature_sensor
{
public:
  /**
   * @brief Create a temperature adapter for a drc object
   *
   * @param p_drc - DRC object
   * @return result<drc_temperature>
   */
  static result<drc_temperature> create(drc& p_drc)
  {
    return drc_temperature(p_drc);
  }

private:
  drc_temperature(drc& p_drc)
    : m_drc(&p_drc)
  {
  }

  result<hal::temperature_sensor::read_t> driver_read() override
  {
    HAL_CHECK(m_drc->feedback_request(hal::rmd::drc::read::status_2));

    return hal::temperature_sensor::read_t{
      .temperature = m_drc->feedback().temperature(),
    };
  }

  drc* m_drc = nullptr;
};

/**
 * @brief Reports the rotation of the DRC motor
 *
 */
class drc_rotation : public hal::rotation_sensor
{
public:
  /**
   * @brief Create a rotation adapter for a drc object
   *
   * @param p_drc - DRC object
   * @return result<drc_rotation>
   */
  static result<drc_rotation> create(drc& p_drc)
  {
    return drc_rotation(p_drc);
  }

private:
  drc_rotation(drc& p_drc)
    : m_drc(&p_drc)
  {
  }

  result<hal::rotation_sensor::read_t> driver_read() override
  {
    HAL_CHECK(m_drc->feedback_request(hal::rmd::drc::read::status_2));

    return hal::rotation_sensor::read_t{
      .angle = m_drc->feedback().angle(),
    };
  }

  drc* m_mc_x = nullptr;
};
}  // namespace hal::rmd
