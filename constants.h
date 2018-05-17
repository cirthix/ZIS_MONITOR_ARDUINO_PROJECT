  #ifndef CONSTANTS_h
#define CONSTANTS_h

#include <Arduino.h>

const uint32_t SERIAL_BAUD = 38400; // Set to 0 to disable serial port and all debug output.

#define IIC_SPEED_DEFAULT 100 // Speed in KHz
#define EDID_IIC_SPEED IIC_SPEED_DEFAULT
#define EDID_IIC_ADDRESS 0x50
#define IIC_SPEED_EP369 93.75 // Speed in KHz
#define IIC_SPEED_EP269 IIC_SPEED_DEFAULT // Speed in KHz


// How this new EDID setup will work is that two factory EDIDs are loaded into the EEPROM on the atmega.  
// EDID_SAFE will be bone-stock EDID for the selected panel.  EDID_USER will be preloaded with overclocked settings.  EDID_USER will be selected by default.
const uint8_t SELECTED_EDID_FACTORY_SAFE = 0 ;
const uint8_t SELECTED_EDID_FACTORY_OC = 1 ;
const uint8_t SELECTED_EDID_SAFE = 2 ;
const uint8_t SELECTED_EDID_USER = 3 ;

const uint8_t MAX_USER_EDIDS = 1; // IS EQUAL TO THE POTENTIAL NUMBER OF USER EDIDS PRESENT.  FOR NOW, JUST LEAVE THIS AT ONE, THOUGH THE EEPROM IS BIG ENOUGH TO STORE TWO AND MOST OF THE CODE HAS BEEN WRITTEN ALREADY.


#define ACTIVE_LOW  LOW
#define ACTIVE_HIGH HIGH

#ifndef ENABLED
#define ENABLED 1 
#endif

#ifndef DISABLED
#define DISABLED 0
#endif

#define BOARD_IS_EP269_REV2             1
#define BOARD_IS_EP269_REV3             2
#define BOARD_IS_EP269_MAY              3
#define BOARD_IS_EP269_REV4             4
#define BOARD_IS_EP269_REV4_EDID_FIX    5
#define BOARD_IS_EP269_REV4_MAXIM       6
#define BOARD_IS_EP369_REV1             7
#define BOARD_IS_EP269_REV2016          8
#define BOARD_IS_EP369_REV2016          9
#define BOARD_IS_EP269_REV2016ss        10
#define BOARD_IS_EP369_REV2016ss        11
#define BOARD_IS_EP369_REV2017          12

#define PANEL_IS_M270HHF           1
#define PANEL_IS_V390DKZIS         2
#define PANEL_IS_M280GJZIS         3
#define PANEL_IS_LM270WQ           6
#define PANEL_IS_M320DVN01         7
#define PANEL_IS_M240HW01          8
#define PANEL_IS_LM270WF3          9

#define BLDRIVER_IS_NONEXISTANT 1  // Not yet tested!
#define BLDRIVER_IS_GENERIC 2
#define BLDRIVER_IS_VG278H 3
#define BLDRIVER_IS_XSTAR_DP2414LED 4

#define PANELPSU_IS_SHORTED_TO_INPUT 1
#define PANELPSU_IS_DIRECT_NOCHECK 2
#define PANELPSU_IS_DIRECT 3
#define PANELPSU_IS_TPS5332 4 // This isnt properly tested :/

//////////////////////////////////////////////////////////////////////// CHANGE SYSTEM CONFIGURATION PARAMETERS HERE ////////////////////////////////////////////////////////////////////////
#define BOARD_VERSION BOARD_IS_EP369_REV2017
#define PANEL_VERSION PANEL_IS_V390DKZIS
#define BLDRIVER_VERSION BLDRIVER_IS_GENERIC
#define PANELPSU_VERSION PANELPSU_IS_DIRECT
//////////////////////////////////////////////////////////////////////// CHANGE SYSTEM CONFIGURATION PARAMETERS HERE ////////////////////////////////////////////////////////////////////////


#include "SUPPORTED_BOARDS.h"
#include "SUPPORTED_PANELS.h"
#include "SUPPORTED_OVERCLOCKS.h"
#include "SUPPORTED_BLDRIVERS.h"
#include "SUPPORTED_PSUS.h"


#define SERIAL_COMMANDS_SIMPLE ENABLED // ENABLE THIS TO ALLOW VIRTUAL BUTTON PRESSES VIA SERIAL PORT (THIS FEATURE IS USEFUL FOR DEBUGGING)
#define SERIAL_COMMANDS_EXTENDED DISABLED // ENABLE THIS TO ALLOW COMPLEX SERIAL COMMANDS (EDID UPDATE, ETC)

#if SERIAL_COMMANDS_SIMPLE!=ENABLED
#if SERIAL_COMMANDS_EXTENDED==ENABLED
#error "SIMPLE SERIAL COMMANDS MUST BE ENABLED IF EXTENDED ONES ARE ENABLED"
#endif
#if SERIAL_COMMANDS_SIMPLE!=DISABLED
#error "SERIAL_COMMANDS_SIMPLE not specified"
#endif
#endif


#if OVERCLOCKING!=ENABLED
#if OVERCLOCKING!=DISABLED
#error "OVERCLOCKING not specified"
#endif
#endif




// Primary EDID should never be changed.
// Secondary EDID is allwed to be user-reconfigured.

const uint16_t SIZE_EDID                = 256;  // IF A 128-BYTE EDID IS USED, JUST ZERO-OUT THE REMAINING 128 BYTES.

// Take advantage of the fact that EDIDs contain a signature to store some extra bytes
const uint8_t EDID_SIGNATURE_SIZE = 8;
PROGMEM const uint8_t EDID_SIGNATURE[]      = {0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};

const uint16_t ADDRESS_MAGIC_BYTE               = 10;
const uint16_t ADDRESS_POWER_STATE              = 11;
const uint16_t ADDRESS_SELECTED_EDID            = 12;
const uint16_t ADDRESS_BACKLIGHT_MODE           = 13;
const uint16_t ADDRESS_BACKLIGHT_LEVEL_STABLE   = 14;
const uint16_t ADDRESS_BACKLIGHT_LEVEL_PULSE    = 15;
const uint16_t ADDRESS_PWM_FREQUENCY            = 16;
const uint16_t ADDRESS_WAS_SLAVE                = 17;
const uint16_t ADDRESS_OSD_ENABLED              = 18;
const uint16_t ADDRESS_USE_OCTESTMODE           = 19;

#define I2C_TIMEOUT 10

const uint8_t BUTTON_SENSE_TIME = 5;   // 1 TICK IS 1 MICROCSECOND

const uint8_t TargetPowerSaveSHUTDOWN = 0; // Shutdown disables the dp recievers
const uint8_t TargetPowerSaveLOWPOWER = 1; // Lowpower mode disables the fpga, and by extension, the backlight
const uint8_t TargetPowerSaveFULLY_ON = 2; // System fully operational

const uint8_t SystemState_Init = 0 ;
const uint8_t SystemState_PowerOff = 1 ;
const uint8_t SystemState_Rx = 2 ;
const uint8_t SystemState_Tx = 3 ;
const uint8_t SystemState_Panel = 4 ;
const uint8_t SystemState_Backlight = 5 ;
const uint8_t SystemState_On = 6 ;

const uint8_t BACKLIGHT_MODE_PULSE = 0;
const uint8_t BACKLIGHT_MODE_OFF = 1;
const uint8_t BACKLIGHT_MODE_STABLE = 2;
const uint8_t MINIMUM_REFRESH_RATE = 25;

// THIS MUST EXACTLY MATCH THE EP369S INTERNAL FIRMWARE VALUE
const uint8_t ZWSMOD_EP369S_ADDRESS_SPECIAL = 0x00;
const uint8_t ZWSMOD_EP369S_VALUE_SPECIAL = 0x01;
const uint8_t ZWSMOD_EP369S_ADDRESS_CONFIGURATION = 0x01;
const uint8_t CONFIGMASK_EPMI_DW0   = 0b00000001;
const uint8_t CONFIGMASK_EPMI_DW1   = 0b00000010;
const uint8_t CONFIGMASK_EPMI_MAP   = 0b00000100;
const uint8_t CONFIGMASK_EPMI_LR    = 0b00001000;
const uint8_t CONFIGMASK_EPMI_EO    = 0b00010000;
const uint8_t CONFIGMASK_EPMI_DMODE = 0b00100000;
const uint8_t CONFIGMASK_EPMI_TMODE = 0b01000000;
const uint8_t CONFIGMASK_EPMI_RS    = 0b10000000;

const uint16_t SAVED_FREQUENCY_SCALING = 64;
const uint8_t FACTORY_DEFAULT_BACKLIGHT_STABLE_BRIGHTNESS = DEFAULT_BRIGHTNESS_LEVEL ; // This default level is set in SUPPORTED_BLDRIVERS.h
const uint8_t FACTORY_DEFAULT_BACKLIGHT_PULSE_BRIGHTNESS = PWM_MAX_DUTYCYCLE * 0.75 ; // There is no safety issue here, it is checked elsewhere
const uint8_t FACTORY_DEFAULT_BACKLIGHT_MODE = BACKLIGHT_MODE_STABLE ;
const uint8_t FACTORY_DEFAULT_BACKLIGHT_FREQUENCY = (10000) / SAVED_FREQUENCY_SCALING ;
const uint8_t FACTORY_DEFAULT_POWERSTATE = TargetPowerSaveFULLY_ON ;
const uint8_t FACTORY_DEFAULT_SELECTED_EDID = PANEL_DEFAULT_EDID;
const uint8_t FACTORY_DEFAULT_USE_OCTESTMODE = 1;
const uint8_t FACTORY_DEFAULT_USE_OSD = 0;

const uint16_t OneMillisecond        =1;
const uint16_t TenMilliseconds       =10;
const uint16_t HundredMilliseconds   =100;
const uint16_t OneSecond             =1000;
const uint16_t TenSeconds            =10000;

#endif
