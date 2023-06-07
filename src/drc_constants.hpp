#pragma once

#include <libhal/units.hpp>

namespace hal::rmd {
/// Operating baudrate of all RMD-X smart servos
static constexpr hertz baudrate_hz = 1'000'000;
static constexpr float dps_per_lsb_speed = 0.01f;
static constexpr float dps_per_lsb_angle = 1.0f;
static constexpr std::uint8_t over_voltage_protection_tripped_mask = 0b1;
static constexpr std::uint8_t over_temperature_protection_tripped_mask = 0b100;
}  // namespace hal::rmd
