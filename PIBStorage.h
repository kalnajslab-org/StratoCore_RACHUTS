/*
 *  PIBStorage.h
 *  Author:  Alex St. Clair
 *  Created: August 2019
 *
 *  This class manages configuration storage in EEPROM
 */

#include "WProgram.h"
#include <EEPROM.h>
#include <StdInt.h>

#define EEPROM_BASE_ADDRESS     0
#define EEPROM_MAX_ADDRESS      4095
#define EEPROM_VERSION          0xAC6F0003 // increment for each version change

struct PIBConfigs_t {
    uint32_t eeprom_version;
    float sza_minimum; // start of night after astronomical twilight
    float profile_size; // revolutions
    float dock_amount; // revolutions
    float dock_overshoot;
    float deploy_velocity;
    float retract_velocity;
    float dock_velocity;
    uint32_t time_trigger; // absolute time in seconds
    uint16_t dwell_time; // seconds
    uint16_t profile_period; // seconds
    uint8_t num_profiles; // per night
    bool sza_trigger; // true if SZA triggers profile, false if profile_time
    bool pu_docked;
};

extern PIBConfigs_t pib_config; // software copy of EEPROM

class PIBStorage {
public:
    PIBStorage() { };
    ~PIBStorage() { };

    // Basic EEPROM methods -------------------------------------------------------
    bool LoadFromEEPROM(void);

    // macros for safe updating of EEPROM. Usage: EEPROM_UPDATE_TYPE(storageManager,field_name,new_value);
    #define EEPROM_UPDATE_BOOL(object,name,data)   ((object).Update_bool(offsetof(PIBConfigs_t,name),data))
    #define EEPROM_UPDATE_UINT8(object,name,data)   ((object).Update_uint8(offsetof(PIBConfigs_t,name),data))
    #define EEPROM_UPDATE_UINT16(object,name,data)  ((object).Update_uint16(offsetof(PIBConfigs_t,name),data))
    #define EEPROM_UPDATE_UINT32(object,name,data)  ((object).Update_uint32(offsetof(PIBConfigs_t,name),data))
    #define EEPROM_UPDATE_FLOAT(object,name,data)   ((object).Update_float(offsetof(PIBConfigs_t,name),data))

    // use the macros instead of these to update specific variables
    bool Update_bool(uint16_t offset, bool data);
    bool Update_uint8(uint16_t offset, uint8_t data);
    bool Update_uint16(uint16_t offset, uint16_t data);
    bool Update_uint32(uint16_t offset, uint32_t data);
    bool Update_float(uint16_t offset, float data);

private:
    void ReconfigureEEPROM(void);

};
