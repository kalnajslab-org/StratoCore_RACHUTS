/*
 *  PIBConfigs.h
 *  Author:  Alex St. Clair
 *  Created: April 2020
 *
 *  This class manages configuration storage in EEPROM on the PIB
 *
 *  To add a configuration value:
 *    1) Add a public EEPROMData<T> object in the header file
 *    2) Set the hard-coded backup value in the constructor
 *    3) Register the object in the RegisterAll method
 *    *note* maintain the order of objects in all three locations
 */

#ifndef PIBCONFIGS_H
#define PIBCONFIGS_H

#include "TeensyEEPROM.h"

class PIBConfigs : public TeensyEEPROM {
private:
    void RegisterAll();

public:
    PIBConfigs();

    // constants, manually change version number here to force update
    static const uint16_t CONFIG_VERSION = 0x5C02;
    static const uint16_t BASE_ADDRESS = 0x0000;

    // ------------------ Configurations ------------------

    // profile triggers
    EEPROMData<float> sza_minimum;
    EEPROMData<uint32_t> time_trigger;
    EEPROMData<bool> sza_trigger; // true if SZA triggers profile, false if profile_time

    // profile sizing (in revolutions)
    EEPROMData<float> profile_size;
    EEPROMData<float> dock_amount;
    EEPROMData<float> dock_overshoot;
    EEPROMData<float> redock_out;
    EEPROMData<float> redock_in;

    // profile speeds (in rpm)
    EEPROMData<float> deploy_velocity;
    EEPROMData<float> retract_velocity;
    EEPROMData<float> dock_velocity;

    // PU configuration
    EEPROMData<float> flash_temp;
    EEPROMData<float> heater1_temp;
    EEPROMData<float> heater2_temp;
    EEPROMData<uint32_t> profile_rate;
    EEPROMData<uint32_t> dwell_rate;
    EEPROMData<uint8_t> flash_power;
    EEPROMData<uint8_t> tsen_power;
    EEPROMData<uint8_t> profile_TSEN;
    EEPROMData<uint8_t> profile_ROPC;
    EEPROMData<uint8_t> profile_FLASH;
    EEPROMData<uint32_t> docked_rate;
    EEPROMData<uint8_t> docked_TSEN;
    EEPROMData<uint8_t> docked_ROPC;
    EEPROMData<uint8_t> docked_FLASH;

    // profile timing (seconds)
    EEPROMData<uint16_t> dwell_time;
    EEPROMData<uint16_t> preprofile_time;
    EEPROMData<uint16_t> puwarmup_time;
    EEPROMData<uint16_t> motion_timeout;
    EEPROMData<uint16_t> profile_period;

    // autonomous configurations
    EEPROMData<uint8_t> num_profiles; // per night
    EEPROMData<uint8_t> num_redock;   // before erroring out

    // PU tracking
    EEPROMData<bool> pu_docked;

    // MCB TM mode
    EEPROMData<bool> real_time_mcb;

    // ----------------------------------------------------

};

#endif /* PIBCONFIGS_H */