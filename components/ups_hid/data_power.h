#pragma once

#include <string>
#include <cmath>
#include "constants_hid.h"

namespace esphome {
namespace ups_hid {

struct PowerData {
  // Input power metrics
  float input_voltage{NAN};            // Current input voltage (V)
  float input_voltage_nominal{NAN};    // Nominal input voltage (V)
  float input_transfer_low{NAN};       // Low transfer voltage threshold (V)
  float input_transfer_high{NAN};      // High transfer voltage threshold (V)
  float frequency{NAN};                // Input frequency (Hz)
  
  // Output power metrics
  float output_voltage{NAN};           // Current output voltage (V)
  float output_voltage_nominal{NAN};   // Nominal output voltage (V)
  float load_percent{NAN};             // Current load percentage (0-100%)
  
  // Power ratings and capabilities
  float realpower_nominal{NAN};        // Nominal real power rating (W)
  float apparent_power_nominal{NAN};   // Nominal apparent power rating (VA)

  // MEASURED instantaneous output power. Distinct from the *_nominal ratings
  // above, and distinct from the maintainer's "UPS Load Power" template sensor,
  // which is an ESTIMATE: realpower_nominal x load_percent / 100, with a 700 W
  // fallback when the nominal is unknown. These two are read from the device.
  //
  // ⚠ NOT declared by every unit -- D (CP825LCD, 383-byte descriptor) has neither
  // report, while A/B/C/E do. They are read via the descriptor usage map, never a
  // hardcoded report ID, so a device that lacks them spends no HID traffic.
  //
  // SCALE: the raw uint16 IS watts / VA. The descriptor advertises Unit Exponent
  // +7 on both, and it is deliberately IGNORED -- NUT ignores it too, and the
  // reading confirms it: E's ConfigActivePower reads 450 on a CP825AVRLCDa, whose
  // nameplate is 825 VA / 450 W. Applying the exponent would give 4.5e9 W.
  float realpower{NAN};                // Measured real power out (W)   -- UPS.Output.ActivePower
  float apparent_power{NAN};           // Measured apparent power (VA)  -- UPS.Output.ApparentPower
  
  // Power status information
  std::string status{};                // Power status text (Online, On Battery, etc.)
  
  // Power quality indicators
  bool input_voltage_valid() const {
    return !std::isnan(input_voltage) && input_voltage > 50.0f && input_voltage < 300.0f;
  }
  
  bool output_voltage_valid() const {
    return !std::isnan(output_voltage) && output_voltage > 50.0f && output_voltage < 300.0f;
  }
  
  bool frequency_valid() const {
    return !std::isnan(frequency) && frequency >= FREQUENCY_MIN_VALID && frequency <= FREQUENCY_MAX_VALID;
  }
  
  bool is_input_out_of_range() const {
    if (!input_voltage_valid()) return false;
    return (!std::isnan(input_transfer_low) && input_voltage < input_transfer_low) ||
           (!std::isnan(input_transfer_high) && input_voltage > input_transfer_high);
  }
  
  bool is_overloaded() const {
    return !std::isnan(load_percent) && load_percent > 95.0f;
  }
  
  bool has_load_info() const {
    return !std::isnan(load_percent);
  }
  
  // Validation and utility methods
  bool is_valid() const {
    return input_voltage_valid() || output_voltage_valid() || has_load_info();
  }
  
  void reset() { 
    *this = PowerData{}; 
  }
  
  // Copy constructor and assignment for safe copying
  PowerData() = default;
  PowerData(const PowerData&) = default;
  PowerData& operator=(const PowerData&) = default;
  PowerData(PowerData&&) = default;
  PowerData& operator=(PowerData&&) = default;
};

}  // namespace ups_hid
}  // namespace esphome