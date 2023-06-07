#pragma once

#include <libhal/units.hpp>

namespace hal::rmd {
/// Operating baudrate of all RMD-X smart servos
static constexpr hertz baudrate_hz = 1'000'000;
static constexpr float dps_per_lsb_speed = 0.01f;
static constexpr float dps_per_lsb_angle = 1.0f;
/// Messages returned from these motor drivers are the same as motor ID plus
/// this offset.
static constexpr std::uint32_t response_id_offset = 0x100;
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
}  // namespace hal::rmd
