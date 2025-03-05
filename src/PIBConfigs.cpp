/*
 *  PIBConfigs.cpp
 *  Author:  Alex St. Clair
 *  Created: April 2020
 *
 *  This class manages configuration storage in EEPROM on the PIB
 */

#include "PIBConfigs.h"
#include "StratoGroundPort.h"

PIBConfigs::PIBConfigs()
    : TeensyEEPROM(CONFIG_VERSION, BASE_ADDRESS)
    // ------------ Hard-Coded Config Defaults ------------
    , sza_minimum(105)
    , time_trigger(UINT32_MAX)
    , sza_trigger(false)
    , profile_size(7500.0f)
    , dock_amount(200.0f)
    , dock_overshoot(100.0f)
    , redock_out(5)
    , redock_in(10)
    , deploy_velocity(250.0f)
    , retract_velocity(250.0f)
    , dock_velocity(80.0f)
    , flash_temp(-20.0f)
    , heater1_temp(0.0f)
    , heater2_temp(-15.0f)
    , profile_rate(1)
    , dwell_rate(10)
    , flash_power(1)
    , tsen_power(1)
    , profile_TSEN(1)
    , profile_ROPC(1)
    , profile_FLASH(1)
    , docked_rate(10)
    , docked_TSEN(1)
    , docked_ROPC(1)
    , docked_FLASH(1)
    , dwell_time(900)
    , preprofile_time(180)
    , puwarmup_time(900)
    , motion_timeout(30)
    , profile_period(7200)
    , num_profiles(3)
    , num_redock(3)
    , pu_docked(false)
    , real_time_mcb(false)
    , lora_tx_tm(false)
    , lora_tx_status(1800)
    , profile_id(1)
    , ra_override(false)
    , pu_auto_offload(false)
    // ----------------------------------------------------
{ }

void PIBConfigs::RegisterAll()
{
    bool success = true;

    success &= Register(&sza_minimum);
    success &= Register(&time_trigger);
    success &= Register(&sza_trigger);
    success &= Register(&profile_size);
    success &= Register(&dock_amount);
    success &= Register(&dock_overshoot);
    success &= Register(&redock_out);
    success &= Register(&redock_in);
    success &= Register(&deploy_velocity);
    success &= Register(&retract_velocity);
    success &= Register(&dock_velocity);
    success &= Register(&flash_temp);
    success &= Register(&heater1_temp);
    success &= Register(&heater2_temp);
    success &= Register(&profile_rate);
    success &= Register(&dwell_rate);
    success &= Register(&flash_power);
    success &= Register(&tsen_power);
    success &= Register(&profile_TSEN);
    success &= Register(&profile_ROPC);
    success &= Register(&profile_FLASH);
    success &= Register(&docked_rate);
    success &= Register(&docked_TSEN);
    success &= Register(&docked_ROPC);
    success &= Register(&docked_FLASH);
    success &= Register(&dwell_time);
    success &= Register(&preprofile_time);
    success &= Register(&puwarmup_time);
    success &= Register(&motion_timeout);
    success &= Register(&profile_period);
    success &= Register(&num_profiles);
    success &= Register(&num_redock);
    success &= Register(&pu_docked);
    success &= Register(&real_time_mcb);
    success &= Register(&lora_tx_tm);
    success &= Register(&lora_tx_status);
    success &= Register(&profile_id);
    success &= Register(&ra_override);
    success &= Register(&pu_auto_offload);

    if (!success) {
        debug_serial->println("Error registering EEPROM configs");
    }
}