// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

#if CLI_ENABLED == ENABLED

#define PWM_CALIB_MIN 1000
#define PWM_CALIB_MAX 2000
#define PWM_HIGHEST_MAX 2200
#define PWM_LOWEST_MAX 1200
#define PWM_HIGHEST_MIN 1800
#define PWM_LOWEST_MIN 800


/***************************************************************************/
// CLI reports
/***************************************************************************/

void Copter::report_batt_monitor()
{
    cliSerial->printf("\nBatt Mon:\n");
    print_divider();
    if (battery.num_instances() == 0) {
        print_enabled(false);
    } else if (!battery.has_current()) {
        cliSerial->printf("volts");
    } else {
        cliSerial->printf("volts and cur");
    }
    print_blanks(2);
}

void Copter::report_frame()
{
    cliSerial->printf("Frame\n");
    print_divider();

 #if FRAME_CONFIG == QUAD_FRAME
    cliSerial->printf("Quad frame\n");
 #elif FRAME_CONFIG == TRI_FRAME
    cliSerial->printf("TRI frame\n");
 #elif FRAME_CONFIG == HEXA_FRAME
    cliSerial->printf("Hexa frame\n");
 #elif FRAME_CONFIG == Y6_FRAME
    cliSerial->printf("Y6 frame\n");
 #elif FRAME_CONFIG == OCTA_FRAME
    cliSerial->printf("Octa frame\n");
 #elif FRAME_CONFIG == HELI_FRAME
    cliSerial->printf("Heli frame\n");
 #endif

    print_blanks(2);
}

void Copter::report_radio()
{
    cliSerial->printf("Radio\n");
    print_divider();
    // radio
    print_radio_values();
    print_blanks(2);
}

void Copter::report_ins()
{
    cliSerial->printf("INS\n");
    print_divider();

    print_gyro_offsets();
    print_accel_offsets_and_scaling();
    print_blanks(2);
}

void Copter::report_flight_modes()
{
    cliSerial->printf("Flight modes\n");
    print_divider();

    for(int16_t i = 0; i < 6; i++ ) {
        print_switch(i, flight_modes[i], BIT_IS_SET(g.simple_modes, i));
    }
    print_blanks(2);
}

void Copter::report_optflow()
{
 #if OPTFLOW == ENABLED
    cliSerial->printf("OptFlow\n");
    print_divider();

    print_enabled(optflow.enabled());

    print_blanks(2);
 #endif     // OPTFLOW == ENABLED
}

/***************************************************************************/
// CLI utilities
/***************************************************************************/

void Copter::print_radio_values()
{
    cliSerial->printf("CH1: %d | %d\n", (int)channel_roll->radio_min, (int)channel_roll->radio_max);
    cliSerial->printf("CH2: %d | %d\n", (int)channel_pitch->radio_min, (int)channel_pitch->radio_max);
    cliSerial->printf("CH3: %d | %d\n", (int)channel_throttle->radio_min, (int)channel_throttle->radio_max);
    cliSerial->printf("CH4: %d | %d\n", (int)channel_yaw->radio_min, (int)channel_yaw->radio_max);
    cliSerial->printf("CH5: %d | %d\n", (int)g.rc_5.radio_min, (int)g.rc_5.radio_max);
    cliSerial->printf("CH6: %d | %d\n", (int)g.rc_6.radio_min, (int)g.rc_6.radio_max);
    cliSerial->printf("CH7: %d | %d\n", (int)g.rc_7.radio_min, (int)g.rc_7.radio_max);
    cliSerial->printf("CH8: %d | %d\n", (int)g.rc_8.radio_min, (int)g.rc_8.radio_max);
}

void Copter::print_switch(uint8_t p, uint8_t m, bool b)
{
    cliSerial->printf("Pos %d:\t",p);
    print_flight_mode(cliSerial, m);
    cliSerial->printf(",\t\tSimple: ");
    if(b)
        cliSerial->printf("ON\n");
    else
        cliSerial->printf("OFF\n");
}

void Copter::print_accel_offsets_and_scaling(void)
{
    const Vector3f &accel_offsets = ins.get_accel_offsets();
    const Vector3f &accel_scale = ins.get_accel_scale();
    cliSerial->printf("A_off: %4.2f, %4.2f, %4.2f\nA_scale: %4.2f, %4.2f, %4.2f\n",
                    (double)accel_offsets.x,                           // Pitch
                    (double)accel_offsets.y,                           // Roll
                    (double)accel_offsets.z,                           // YAW
                    (double)accel_scale.x,                             // Pitch
                    (double)accel_scale.y,                             // Roll
                    (double)accel_scale.z);                            // YAW
}

void Copter::print_gyro_offsets(void)
{
    const Vector3f &gyro_offsets = ins.get_gyro_offsets();
    cliSerial->printf("G_off: %4.2f, %4.2f, %4.2f\n",
                    (double)gyro_offsets.x,
                    (double)gyro_offsets.y,
                    (double)gyro_offsets.z);
}

#endif // CLI_ENABLED

// report_compass - displays compass information.  Also called by compassmot.pde
void Copter::report_compass()
{
    cliSerial->printf("Compass\n");
    print_divider();

    print_enabled(g.compass_enabled);

    // mag declination
    cliSerial->printf("Mag Dec: %4.4f\n",
            (double)degrees(compass.get_declination()));

    // mag offsets
    Vector3f offsets;
    for (uint8_t i=0; i<compass.get_count(); i++) {
        offsets = compass.get_offsets(i);
        // mag offsets
        cliSerial->printf("Mag%d off: %4.4f, %4.4f, %4.4f\n",
                        (int)i,
                        (double)offsets.x,
                        (double)offsets.y,
                        (double)offsets.z);
    }

    // motor compensation
    cliSerial->print("Motor Comp: ");
    if( compass.get_motor_compensation_type() == AP_COMPASS_MOT_COMP_DISABLED ) {
        cliSerial->print("Off\n");
    }else{
        if( compass.get_motor_compensation_type() == AP_COMPASS_MOT_COMP_THROTTLE ) {
            cliSerial->print("Throttle");
        }
        if( compass.get_motor_compensation_type() == AP_COMPASS_MOT_COMP_CURRENT ) {
            cliSerial->print("Current");
        }
        Vector3f motor_compensation;
        for (uint8_t i=0; i<compass.get_count(); i++) {
            motor_compensation = compass.get_motor_compensation(i);
            cliSerial->printf("\nComMot%d: %4.2f, %4.2f, %4.2f\n",
                        (int)i,
                        (double)motor_compensation.x,
                        (double)motor_compensation.y,
                        (double)motor_compensation.z);
        }
    }
    print_blanks(1);
}

void Copter::print_blanks(int16_t num)
{
    while(num > 0) {
        num--;
        cliSerial->println("");
    }
}

void Copter::print_divider(void)
{
    for (int i = 0; i < 40; i++) {
        cliSerial->print("-");
    }
    cliSerial->println();
}

void Copter::print_enabled(bool b)
{
    if(b)
        cliSerial->print("en");
    else
        cliSerial->print("dis");
    cliSerial->print("abled\n");
}

void Copter::report_version()
{
    cliSerial->printf("FW Ver: %d\n",(int)g.k_format_version);
    print_divider();
    print_blanks(2);
}
