/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include <AP_ADC/AP_ADC.h>                 // ArduPilot Mega Analog to Digital Converter Library
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

# define AP_BATT_VOLT_PIN                  10
# define AP_BATT_CURR_PIN                  11
# define AP_BATT_VOLTDIVIDER_DEFAULT       10.1f
# define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0f

// Other values normally set directly by mission planner
// # define AP_BATT_VOLTDIVIDER_DEFAULT 15.70   // Volt divider for AttoPilot 50V/90A sensor
// # define AP_BATT_VOLTDIVIDER_DEFAULT 4.127   // Volt divider for AttoPilot 13.6V/45A sensor
// # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT 27.32  // Amp/Volt for AttoPilot 50V/90A sensor
// # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT 13.66  // Amp/Volt for AttoPilot 13.6V/45A sensor

class AP_BattMonitor_Analog : public AP_BattMonitor_Backend
{
public:

    /// Constructor
    AP_BattMonitor_Analog(AP_BattMonitor &mon, uint8_t instance, AP_BattMonitor::BattMonitor_State &mon_state);

    /// Read the battery voltage and current.  Should be called at 10hz
    void read();

protected:

    AP_HAL::AnalogSource *_volt_pin_analog_source;
    AP_HAL::AnalogSource *_curr_pin_analog_source;
};
