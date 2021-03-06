#ifndef INPUTHANDLING_h
#define INPUTHANDLING_h
#include "constants.h"

#define FILTERDEPTH_INPUT 8

const uint8_t ASCII_CODE_FOR_SPECIAL_COMMANDS       = 0x24; // '$' is the special debug code
const uint8_t ASCII_CODE_FOR_SLAVE_ON               = 0x11; // Nonprintable character 0x11 is "Device control 1" in the ascii table.
const uint8_t ASCII_CODE_FOR_SLAVE_OFF              = 0x12; // Nonprintable character 0x12 is "Device control 2" in the ascii table.
const uint8_t ASCII_CODE_FOR_POWER_BUTTON           = 0x2A; // '*' is the power button
const uint8_t ASCII_CODE_FOR_BRIGHTNESS_INCREASE    = 0x2B; // '+' increases brightness
const uint8_t ASCII_CODE_FOR_BRIGHTNESS_DECREASE    = 0x2D; // '-' decreases brightness
const uint8_t ASCII_CODE_FOR_FACTORY_PROGRAM        = 0x2F; // '/' sets factory defaults (if in off state)
const uint8_t ASCII_CODE_FOR_PWM_FREQ_DECREASE      = 0x38; // '8' decreases pwm frequency
const uint8_t ASCII_CODE_FOR_PWM_FREQ_INCREASE      = 0x39; // '9' increases pwm frequency
const uint8_t ASCII_CODE_FOR_EDID_ROTATE            = 0x2E; // '.' swaps the edid
const uint8_t ASCII_CODE_FOR_EDID_0                 = 0x30; // '0' Sets internal edid #0
const uint8_t ASCII_CODE_FOR_EDID_1                 = 0x31; // '1' Sets internal edid #1
const uint8_t ASCII_CODE_FOR_EDID_2                 = 0x32; // '2' Sets internal edid #2
const uint8_t ASCII_CODE_FOR_EDID_3                 = 0x33; // '3' Sets internal edid #3
const uint8_t ASCII_CODE_FOR_EDID_4                 = 0x34; // '4' Sets internal edid #4
const uint8_t ASCII_CODE_FOR_EDID_5                 = 0x35; // '5' Sets internal edid #5
const uint8_t ASCII_CODE_FOR_EDID_6                 = 0x36; // '6' Sets internal edid #6
const uint8_t ASCII_CODE_FOR_EDID_7                 = 0x37; // '7' Sets internal edid #7
const uint8_t ASCII_CODE_FOR_STROBE_ROTATE          = 0x21; // '!' toggles strobing mode (if supported)
const uint8_t ASCII_CODE_FOR_PANEL_OSD              = 0x23; // '#' toggles the GPIO for enabling the OSD (if supported)
const uint8_t ASCII_CODE_FOR_OCTESTMODE_ON          = 0x5B; // '[' Enables oc test mode
const uint8_t ASCII_CODE_FOR_OCTESTMODE_OFF         = 0x5D; // ']' Disables oc test mode
const uint8_t ASCII_CODE_FOR_OSD_ON                 = 0x3C; // '<' Enables the OSD
const uint8_t ASCII_CODE_FOR_OSD_OFF                = 0x3E; // '>' Disables the OSD
const uint8_t ASCII_CODE_FOR_POWER_ON               = 0x7B; // '{' Powers on the device
const uint8_t ASCII_CODE_FOR_POWER_OFF              = 0x7D; // '}' Powers off the device


const uint8_t COMMAND_CODE_FOR_UNDEFINED              = 0xFF; // This command does nothing
const uint8_t COMMAND_CODE_FOR_NOTHING                = 0x00; // This command does nothing
const uint8_t COMMAND_CODE_FOR_SERIAL                 = 0x01; // This command is used as a placeholder to differentiate between serial commands and button commands
const uint8_t COMMAND_CODE_FOR_SPECIAL_COMMANDS       = ASCII_CODE_FOR_SPECIAL_COMMANDS       ;
const uint8_t COMMAND_CODE_FOR_SLAVE_ON               = ASCII_CODE_FOR_SLAVE_ON   ;
const uint8_t COMMAND_CODE_FOR_SLAVE_OFF              = ASCII_CODE_FOR_SLAVE_OFF  ;
const uint8_t COMMAND_CODE_FOR_POWER_BUTTON           = ASCII_CODE_FOR_POWER_BUTTON           ;
const uint8_t COMMAND_CODE_FOR_BRIGHTNESS_INCREASE    = ASCII_CODE_FOR_BRIGHTNESS_INCREASE    ;
const uint8_t COMMAND_CODE_FOR_BRIGHTNESS_DECREASE    = ASCII_CODE_FOR_BRIGHTNESS_DECREASE    ;
const uint8_t COMMAND_CODE_FOR_FACTORY_PROGRAM        = ASCII_CODE_FOR_FACTORY_PROGRAM        ;
const uint8_t COMMAND_CODE_FOR_PWM_FREQ_DECREASE      = ASCII_CODE_FOR_PWM_FREQ_DECREASE      ;
const uint8_t COMMAND_CODE_FOR_PWM_FREQ_INCREASE      = ASCII_CODE_FOR_PWM_FREQ_INCREASE      ;
const uint8_t COMMAND_CODE_FOR_EDID_ROTATE            = ASCII_CODE_FOR_EDID_ROTATE            ;
const uint8_t COMMAND_CODE_FOR_EDID_0                 = ASCII_CODE_FOR_EDID_0                 ;
const uint8_t COMMAND_CODE_FOR_EDID_1                 = ASCII_CODE_FOR_EDID_1                 ;
const uint8_t COMMAND_CODE_FOR_EDID_2                 = ASCII_CODE_FOR_EDID_2                 ;
const uint8_t COMMAND_CODE_FOR_EDID_3                 = ASCII_CODE_FOR_EDID_3                 ;
const uint8_t COMMAND_CODE_FOR_EDID_4                 = ASCII_CODE_FOR_EDID_4                 ;
const uint8_t COMMAND_CODE_FOR_EDID_5                 = ASCII_CODE_FOR_EDID_5                 ;
const uint8_t COMMAND_CODE_FOR_EDID_6                 = ASCII_CODE_FOR_EDID_6                 ;
const uint8_t COMMAND_CODE_FOR_EDID_7                 = ASCII_CODE_FOR_EDID_7                 ;
const uint8_t COMMAND_CODE_FOR_STROBE_ROTATE          = ASCII_CODE_FOR_STROBE_ROTATE          ;
const uint8_t COMMAND_CODE_FOR_PANEL_OSD              = ASCII_CODE_FOR_PANEL_OSD              ;
const uint8_t COMMAND_CODE_FOR_CONDITIONAL_ROTATE     = ASCII_CODE_FOR_EDID_ROTATE            ;
const uint8_t COMMAND_CODE_FOR_OCTESTMODE_ON          = ASCII_CODE_FOR_OCTESTMODE_ON          ;
const uint8_t COMMAND_CODE_FOR_OCTESTMODE_OFF         = ASCII_CODE_FOR_OCTESTMODE_OFF         ;
const uint8_t COMMAND_CODE_FOR_OSD_ON                 = ASCII_CODE_FOR_OSD_ON                 ;
const uint8_t COMMAND_CODE_FOR_OSD_OFF                = ASCII_CODE_FOR_OSD_OFF                ;
const uint8_t COMMAND_CODE_FOR_POWER_ON               = ASCII_CODE_FOR_POWER_ON               ;
const uint8_t COMMAND_CODE_FOR_POWER_OFF              = ASCII_CODE_FOR_POWER_OFF              ;


const uint8_t BUTTON_EDID_MASK   =  0 ;
const uint8_t BUTTON_UP_MASK     =  1 ;
const uint8_t BUTTON_DOWN_MASK   =  2 ;
const uint8_t BUTTON_POWER_MASK  =  3 ;

const uint8_t COMBO_FOR_POWER_BUTTON     = 0x01<<BUTTON_POWER_MASK                                                        ;
const uint8_t COMBO_BRIGHTNESS_INCREASE  = (0x01<<BUTTON_UP_MASK)                                                         ;
const uint8_t COMBO_BRIGHTNESS_DECREASE  = (0x01<<BUTTON_DOWN_MASK)                                                       ;
const uint8_t COMBO_FACTORY_PROGRAM      = (0x01<<BUTTON_POWER_MASK) | (0x01<<BUTTON_UP_MASK) | (0x01<<BUTTON_DOWN_MASK)  ;
const uint8_t COMBO_PWM_FREQ_DECREASE    = (0x01<<BUTTON_POWER_MASK) | (0x01<<BUTTON_DOWN_MASK)                           ;
const uint8_t COMBO_PWM_FREQ_INCREASE    = (0x01<<BUTTON_POWER_MASK) | (0x01<<BUTTON_UP_MASK)                             ;
const uint8_t COMBO_CONDITIONAL_ROTATE   = (0x01<<BUTTON_UP_MASK) | (0x01<<BUTTON_DOWN_MASK)                              ;


class InputHandling
{
public:
  InputHandling() ;
  void ReadPhysicalInputs();
  void ResetInputHistory();
  void  RefilterInputState();
  uint8_t GetCurrentFilteredInput();
  uint8_t GetPreviousFilteredInput();
  uint8_t GetCurrentUnFilteredInput();
  void PrintState();
private:
 void SetInputHistory(uint8_t state);
 uint8_t input_history[FILTERDEPTH_INPUT];
 uint8_t input_index ;  
 uint8_t current_filtered_input;
 uint8_t previous_filtered_input;
 uint8_t filter_is_dirty;

 
// These values are for a system with a 10kohm pullup and pulldowns in series with buttons, having values of 0, 10k, 4.7k, and 1k.
const uint16_t abserror          = 15;
const float relerror             = 0.10;
const uint16_t cutoff_zero       = 0;
const uint16_t cutoff_zero_low   = cutoff_zero;
const uint16_t cutoff_zero_high  = cutoff_zero+(abserror+cutoff_zero*relerror);
const uint16_t cutoff_one        = 93;
const uint16_t cutoff_one_low    = cutoff_one-(abserror+cutoff_one*relerror);
const uint16_t cutoff_one_high   = cutoff_one+(abserror+cutoff_one*relerror);
const uint16_t cutoff_two        = 327;
const uint16_t cutoff_two_low    = cutoff_two-(abserror+cutoff_two*relerror);
const uint16_t cutoff_two_high   = cutoff_two+(abserror+cutoff_two*relerror);
const uint16_t cutoff_three      = 511;
const uint16_t cutoff_three_low  = cutoff_three-(abserror+cutoff_three*relerror);
const uint16_t cutoff_three_high = cutoff_three+(abserror+cutoff_three*relerror);


};

#endif
