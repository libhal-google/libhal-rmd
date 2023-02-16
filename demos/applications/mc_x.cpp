#include <cinttypes>

#include <libhal-rmd/mc_x.hpp>
#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include "../hardware_map.hpp"

hal::status application(hardware_map& p_map)
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto& clock = *p_map.clock;
  auto& console = *p_map.console;
  auto& can = *p_map.can;

  hal::print(console, "RMD MC-X Smart Servo Application Starting...\n\n");

  auto router = HAL_CHECK(hal::can_router::create(can));
  auto mc_x = HAL_CHECK(hal::rmd::mc_x::create(router, clock, 36.0f, 0x141));

  auto print_feedback = [&mc_x, &clock, &console]() {
    mc_x.feedback_request(hal::rmd::mc_x::read::status_2);
    mc_x.feedback_request(hal::rmd::mc_x::read::multi_turns_angle);
    mc_x.feedback_request(hal::rmd::mc_x::read::status_1_and_error_flags);

    hal::print<2048>(console,
                     "[%u] =================================\n"
                     "raw_multi_turn_angle = %f\n"
                     "raw_current = %d\n"
                     "raw_speed = %d\n"
                     "raw_volts = %d\n"
                     "encoder = %d\n"
                     "raw_motor_temperature = %d"
                     "\n"
                     "-------\n"
                     "angle() = %f °deg\n"
                     "current() = %f A\n"
                     "speed() = %f rpm\n"
                     "volts() = %f V\n"
                     "temperature() = %f °C\n"
                     "motor_stall() = %d\n"
                     "low_pressure() = %d\n"
                     "over_voltage() = %d\n"
                     "over_current() = %d\n"
                     "power_overrun() = %d\n"
                     "speeding() = %d\n"
                     "over_temperature() = %d\n"
                     "encoder_calibration_error() = %d\n"
                     "\n",

                     mc_x.feedback().message_number,
                     static_cast<float>(mc_x.feedback().raw_multi_turn_angle),
                     mc_x.feedback().raw_current,
                     mc_x.feedback().raw_speed,
                     mc_x.feedback().raw_volts,
                     mc_x.feedback().encoder,
                     mc_x.feedback().raw_motor_temperature,
                     mc_x.feedback().angle(),
                     mc_x.feedback().current(),
                     mc_x.feedback().speed(),
                     mc_x.feedback().volts(),
                     mc_x.feedback().temperature(),
                     mc_x.feedback().motor_stall(),
                     mc_x.feedback().low_pressure(),
                     mc_x.feedback().over_voltage(),
                     mc_x.feedback().over_current(),
                     mc_x.feedback().power_overrun(),
                     mc_x.feedback().speeding(),
                     mc_x.feedback().over_temperature(),
                     mc_x.feedback().encoder_calibration_error());
  };

  while (true) {
    mc_x.velocity_control(1.0_rpm);
    (void)hal::delay(clock, 5000ms);
    print_feedback();

    mc_x.velocity_control(-1.0_rpm);
    (void)hal::delay(clock, 5000ms);
    print_feedback();

    // mc_x.position_control(0.0_deg, 20.0_rpm);
    // (void)hal::delay(clock, 2000ms);
    // print_feedback();

    // mc_x.position_control(45.0_deg, 20.0_rpm);
    // (void)hal::delay(clock, 2000ms);
    // print_feedback();

    // mc_x.position_control(90.0_deg, 20.0_rpm);
    // (void)hal::delay(clock, 2000ms);
    // print_feedback();

    // mc_x.position_control(180.0_deg, 20.0_rpm);
    // (void)hal::delay(clock, 2000ms);
    // print_feedback();

    // mc_x.position_control(360.0_deg, 20.0_rpm);
    // (void)hal::delay(clock, 5000ms);
    // print_feedback();

    // mc_x.position_control(0.0_deg, 20.0_rpm);
    // (void)hal::delay(clock, 2000ms);
    // print_feedback();
  }

  return hal::success();
}
