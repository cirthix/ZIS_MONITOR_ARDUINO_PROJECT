#ifndef SUPPORTEDBOARDS_H
#define SUPPORTEDBOARDS_H


#if BOARD_VERSION==BOARD_IS_EP369_REV2017
inline void board_print_name(){ Serial.println(F("ZisWorks dp2lvds v2017"));}
#define CHIP_IS_EP369 1
#define EXTERNAL_IIC_CONFIGURATION_EEPROM 1
#define BUTTON_POWER  A0
#define BUTTON_A_ANALOG  A1
#define BUTTON_B_ANALOG  A2
#define DUALPURPOSE_INPUT_VOLTAGE_MONITORING_PIN_CONTROL_VREG_VPANEL_VOLTAGE  A3 // This is a dual-purpose pin.  If using the panel DC/DC converter, this pin sets ouptut voltage, high=12v, low=5v. If not using the panel DC/DC converter, this is an analog input.  Multiply by the scaling value to get VIN.
const float MICROCONTROLLER_VOLTAGE=3.3;
const uint16_t ADC_RESOLUTION = 1024;
const uint16_t PULLUP_RESISTOR = 4700;
const uint16_t PULLDOWN_RESISTOR = 1000;
#define SDA_PIN  A4
#define SCL_PIN  A5 
#define DATA_ENABLE  2
#define BLPIN_PWM_D  3
#define CONTROL_VREG_VPANEL  4
  #define CONTROL_DCDC_VPANEL_POLARITY ACTIVE_HIGH
#define CONFIG_EPMI_RS 5
#define BLPIN_BLON  6
#define CONFIG_EPMI_LR  7
#define CONFIG_EPMI_TMODE  8
#define RESET_OTHER_CHIPS  9
#define CONFIG_EPMI_MAP  10
#define BLPIN_PWM_A  11
#define PANEL_GPIO0  12
#define PANEL_GPIO1  13
#define CONFIG_EPMI_DW0  14
#define CONFIG_EPMI_DMODE  15

#elif BOARD_VERSION==BOARD_IS_EP369_REV2016ss
inline void board_print_name(){ Serial.println(F("ZisWorks dp2lvds v2016_SS"));}
#define CHIP_IS_EP369 1
#define EXTERNAL_IIC_CONFIGURATION_EEPROM 1
#define BUTTON_POWER  A0
#define BUTTON_A_ANALOG  A1
#define BUTTON_B_ANALOG  A2
#define DUALPURPOSE_INPUT_VOLTAGE_MONITORING_PIN_CONTROL_VREG_VPANEL_VOLTAGE  A3 // This is a dual-purpose pin.  If using the panel DC/DC converter, this pin sets ouptut voltage, high=12v, low=5v. If not using the panel DC/DC converter, this is an analog input.  Multiply by the scaling value to get VIN.
const float MICROCONTROLLER_VOLTAGE=3.3;
const uint16_t ADC_RESOLUTION = 1024;
const uint16_t PULLUP_RESISTOR = 4700;
const uint16_t PULLDOWN_RESISTOR = 1000;
#define SDA_PIN  A4
#define SCL_PIN  A5 
#define DATA_ENABLE  2
#define BLPIN_PWM_D  3
#define CONTROL_VREG_VPANEL  4
  #define CONTROL_DCDC_VPANEL_POLARITY ACTIVE_HIGH
#define CONFIG_EPMI_RS 5
#define BLPIN_BLON  6
#define RESET_OTHER_CHIPS  7
#define SERIAL_EPMI_RXD  8
#define SERIAL_EPMI_TXD  9
#define CONFIG_EPMI_MAP  10
#define BLPIN_PWM_A  11
#define PANEL_GPIO0  12
#define PANEL_GPIO1  13
#define CONFIG_EPMI_DW0  14
#define CONFIG_EPMI_DMODE  15


#elif BOARD_VERSION==BOARD_IS_EP269_REV2016ss
inline void board_print_name(){ Serial.println(F("ZisWorks dvi2lvds v2016_SS"));}
#define CHIP_IS_EP269 1
#define EXTERNAL_IIC_CONFIGURATION_EEPROM 1

#define BUTTON_POWER  A0
#define BUTTON_A_ANALOG  A1
#define BUTTON_B_ANALOG  A2
#define DUALPURPOSE_INPUT_VOLTAGE_MONITORING_PIN_CONTROL_VREG_VPANEL_VOLTAGE  A3 // This is a dual-purpose pin.  If using the panel DC/DC converter, this pin sets ouptut voltage, high=12v, low=5v. If not using the panel DC/DC converter, this is an analog input.  Multiply by the scaling value to get VIN.
const float MICROCONTROLLER_VOLTAGE=3.3;
const uint16_t ADC_RESOLUTION = 1024;
const uint16_t PULLUP_RESISTOR = 4700;
const uint16_t PULLDOWN_RESISTOR = 1000;
#define EDID_SDA_PIN  A4
#define EDID_SCL_PIN  A5 
#define DATA_ENABLE  2
#define BLPIN_PWM_D  3
#define CONTROL_VREG_VPANEL  4
  #define CONTROL_DCDC_VPANEL_POLARITY ACTIVE_HIGH
#define CONFIG_EPMI_RS  5
#define BLPIN_BLON  6
#define CONTROL_VREG_V1P8  7
  #define CONTROL_VREG_V1P8_POLARITY ACTIVE_HIGH  
#define PGOOD_VREG_V3P3  8
#define CONFIG_EPMI_DMODE  9
#define CONTROL_VREG_V3P3  10
  #define CONTROL_VREG_V3P3_POLARITY ACTIVE_HIGH
#define BLPIN_PWM_A  11
#define PANEL_GPIO0  12
#define PANEL_GPIO1  13
#define SCL_PIN  14 
#define SDA_PIN  15



#elif BOARD_VERSION==BOARD_IS_EP369_REV2016
inline void board_print_name(){ Serial.println(F("ZisWorks dp2lvds v2016"));}
#define CHIP_IS_EP369 1
//#define INTERNAL_IIC_CONFIGURATION_EEPROM 1
#define EXTERNAL_IIC_CONFIGURATION_EEPROM 1
#define BUTTON_POWER  A0
#define BUTTON_UP  A1
#define BUTTON_DOWN  A2
#define RESET_OTHER_CHIPS  A3  // NOTE: THIS PIN WAS ORIGINALLY CONTROLLING THE 3.3V REGULATOR.  WITH THE MICROCONTROLLER POWERED BY THIS REGULATOR INSTEAD OF A DEDICATED LDO, THIS PIN HAS BEEN REPURPOSED TO BE A RESET SIGNAL INSTEAD.
#define SDA_PIN  A4
#define SCL_PIN  A5 
#define DATA_ENABLE  2
#define BLPIN_PWM_D  3
#define CONTROL_VREG_VPANEL  4
  #define CONTROL_DCDC_VPANEL_POLARITY ACTIVE_HIGH
#define CONTROL_VREG_VPANEL_VOLTAGE  5
#define BLPIN_BLON  6
//#define CONTROL_VREG_V1P3  7   // NOTE: DUE TO THE CHANGE OF HAVING THE 3.3V REGULATOR BE ALWAYS ON, V1P3 SHOULD BE CONTROLLED BY THE PGOOD_3V3 SIGNAL INSTEAD OF THE MICROCONTROLLER.
//  #define CONTROL_VREG_V1P3_POLARITY ACTIVE_HIGH
#define PGOOD_VREG_V1P3  8
#define PGOOD_VREG_V3P3  9
#define CONFIG_EPMI_MAP  10
#define BLPIN_PWM_A  11
#define PANEL_GPIO0  12
#define PANEL_GPIO1  13
#define CONFIG_EPMI_DW0  14
#define CONFIG_EPMI_DMODE  15


#elif BOARD_VERSION==BOARD_IS_EP269_REV2016
inline void board_print_name(){ Serial.println(F("ZisWorks dvi2lvds v2016"));}
#define CHIP_IS_EP269 1
#define EXTERNAL_IIC_CONFIGURATION_EEPROM 1
#define BUTTON_POWER  A0
#define BUTTON_UP  A1
#define BUTTON_DOWN  A2
#define CONTROL_VREG_V3P3  A3
  #define CONTROL_VREG_V3P3_POLARITY ACTIVE_HIGH
#define EDID_SDA_PIN  A4
#define EDID_SCL_PIN  A5 
#define DATA_ENABLE  2
#define BLPIN_PWM_D  3
#define CONTROL_VREG_VPANEL  4
  #define CONTROL_DCDC_VPANEL_POLARITY ACTIVE_LOW  // This should be ACTIVE_LOW for boards with the dc/dc and ACTIVE_HIGH for the direct-connected boards.
#define CONFIG_EPMI_RS  5
#define BLPIN_BLON  6
#define CONTROL_VREG_V1P8  7
  #define CONTROL_VREG_V1P8_POLARITY ACTIVE_HIGH  
#define PGOOD_VREG_V3P3  8
#define CONTROL_VREG_VPANEL_VOLTAGE  9

#define CONFIG_EPMI_DMODE  10
#define BLPIN_PWM_A  11
#define PANEL_GPIO0  12
#define PANEL_GPIO1  13
#define SCL_PIN  14 
#define SDA_PIN  15


#elif BOARD_VERSION==BOARD_IS_EP369_REV1

#define CHIP_IS_EP369 1
inline void board_print_name(){ Serial.println(F("ZisWorks dp2lvds prototype"));}
#define INTERNAL_IIC_CONFIGURATION_EEPROM 1
#define BUTTON_POWER  A0
#define CONFIG_EPMI_DW0  A1
#define CONFIG_EPMI_DW1  A2
#define CONTROL_VREG_V3P3  A3
  #define CONTROL_VREG_V3P3_POLARITY ACTIVE_LOW
#define SDA_PIN  A4
#define SCL_PIN  A5 
#define DATA_ENABLE  2
#define BLPIN_PWM_D  3
#define CONTROL_VREG_VPANEL  4
  #define CONTROL_DCDC_VPANEL_POLARITY ACTIVE_HIGH
#define CONFIG_EPMI_INTERRUPT  5
#define BLPIN_BLON  6
#define CONTROL_VREG_V1P3  7
  #define CONTROL_VREG_V1P3_POLARITY ACTIVE_HIGH
#define CONFIG_EPMI_DMODE  8
#define CONFIG_EPMI_RS  9 
#define CONFIG_EPMI_MAP  10
#define BLPIN_PWM_A  11
#define BUTTON_UP  12
#define BUTTON_DOWN  13
#define PANEL_GPIO0  14
#define PANEL_GPIO1  15



#elif BOARD_VERSION==BOARD_IS_EP269_REV4
inline void board_print_name(){ Serial.println(F("ZisWorks dvi2lvds prototype"));}
#define CHIP_IS_EP269 1
#define EXTERNAL_IIC_CONFIGURATION_EEPROM 1
#define BUTTON_POWER  A0
#define BUTTON_UP  A1
#define BUTTON_DOWN  A2
#define CONTROL_VREG_V3P3  A3
  #define CONTROL_VREG_V3P3_POLARITY ACTIVE_LOW
#define SDA_PIN  A4
#define SCL_PIN  A5 
#define DATA_ENABLE  2
#define BLPIN_PWM_D  3
#define CONTROL_VREG_VPANEL  4
  #define CONTROL_DCDC_VPANEL_POLARITY ACTIVE_LOW
#define EDID_SELECT  5
#define BLPIN_BLON  6
#define CONFIG_EPMI_RS  7
#define CONFIG_EPMI_EO  8
#define CONFIG_EPMI_LR  9 
#define CONFIG_EPMI_DMODE  10
#define BLPIN_PWM_A  11
#define PANEL_GPIO0  12
#define PANEL_GPIO1  13


#elif BOARD_VERSION==BOARD_IS_EP269_REV4_MAXIM
inline void board_print_name(){ Serial.println(F("ZisWorks dvi2lvds (MAXIM FIX)"));}
#define CHIP_IS_EP269 1
#define EXTERNAL_IIC_CONFIGURATION_EEPROM 1
#define BUTTON_POWER  A0
#define BUTTON_UP  A1
#define BUTTON_DOWN  A2
#define CONTROL_VREG_V3P3  A3
  #define CONTROL_VREG_V3P3_POLARITY ACTIVE_LOW
#define SDA_PIN  A4
#define SCL_PIN  A5 
#define DATA_ENABLE  2
#define BLPIN_PWM_D  3
#define CONTROL_VREG_VPANEL  4
  #define CONTROL_DCDC_VPANEL_POLARITY ACTIVE_LOW
//#define EDID_SELECT  5
#define BLPIN_BLON  6
#define CONFIG_EPMI_RS  7
#define CONFIG_EPMI_EO  8
#define CONFIG_EPMI_LR  9 
#define CONFIG_EPMI_DMODE  10
//#define BLPIN_PWM_A  11
//#define PANEL_GPIO0  12
//#define PANEL_GPIO1  13

#define BLPIN_PWM_A  BLPIN_PWM_D  // TODO: FIX THIS REQUIREMENT

#elif BOARD_VERSION==BOARD_IS_EP269_REV4_EDID_FIX
inline void board_print_name(){ Serial.println(F("ZisWorks dvi2lvds prototype2016"));}
#define CHIP_IS_EP269 1
#define EXTERNAL_IIC_CONFIGURATION_EEPROM 1
#define BUTTON_POWER  A0
#define BUTTON_UP  A1
#define BUTTON_DOWN  A2
#define CONTROL_VREG_V3P3  A3
  #define CONTROL_VREG_V3P3_POLARITY ACTIVE_LOW
#define SDA_PIN  A4
#define SCL_PIN  A5 
#define DATA_ENABLE  2
#define BLPIN_PWM_D  3
#define CONTROL_VREG_VPANEL  4
  #define CONTROL_DCDC_VPANEL_POLARITY ACTIVE_LOW
//#define EDID_SELECT  5  // Removed from board
#define BLPIN_BLON  6
#define CONFIG_EPMI_RS  7
#define CONFIG_EPMI_EO  8
#define CONFIG_EPMI_LR  9 
#define CONFIG_EPMI_DMODE  10
#define BLPIN_PWM_A  11
#define PANEL_GPIO0  12
#define PANEL_GPIO1  13
#define EDID_SCL_PIN  14
#define EDID_SDA_PIN  15 

#elif BOARD_VERSION==BOARD_IS_EP269_REV3
inline void board_print_name(){ Serial.println(F("Personal dvi2lvds prototype(v3)"));}
#define CHIP_IS_EP269 1
#define EXTERNAL_IIC_CONFIGURATION_EEPROM 1
#define CONFIG_EPMI_TMODE  A0
#define CONFIG_EPMI_DMODE  A1
#define PANEL_GPIO1  A2
#define BUTTON_UP  A3
#define SCL_PIN  A4 // Note: backwards from 'correct' I2c pin mapping
#define SDA_PIN  A5 
#define DATA_ENABLE  2
#define BLPIN_PWM_D  3
#define CONTROL_VREG_VPANEL  4
  #define CONTROL_DCDC_VPANEL_POLARITY ACTIVE_HIGH
#define BUTTON_DOWN  5
#define BLPIN_BLON  6
#define BUTTON_POWER  7
#define BUTTON_EDID  7 
#define CONTROL_VREG_V3P3  8
  #define CONTROL_VREG_V3P3_POLARITY ACTIVE_LOW
#define EDID_SELECT  9
#define CONFIG_EPMI_RS  10
#define BLPIN_PWM_A  11  // On a board that supports top and bottom backlight Pulses, make this 11
#define CONFIG_EPMI_EO  12
#define CONFIG_EPMI_LR  13 

#else
  #error "Unsupported BOARD_VERSION"
#endif


#ifdef CHIP_IS_EP369
  #define IIC_SPEED IIC_SPEED_EP369 // IIC SPEED IN KHz
  #define SOFTIIC_OPTIMIZE_FAST
  #define SOFTIIC_OPTIMIZE_NOPULLUPS
  #define MY_WATCHDOG_TIMEOUT WDTO_8S // ADDITIONAL TIME IS NECESSARY TO ALLOW FOR MODE SWITCHING AND TESTING WITHOUT RESETTING THE DISPLAYPORT LINK.
#elif CHIP_IS_EP269
  #define IIC_SPEED IIC_SPEED_EP269 // IIC SPEED IN KHz
  #define MY_WATCHDOG_TIMEOUT WDTO_4S //WDTO_30MS  // If the link goes down, the host system will not know.  Be aggressive with turning off the backlight for a better user experience.
#else
  #define IIC_SPEED IIC_SPEED_DEFAULT // IIC SPEED IN KHz
#endif

#define UNDEFINED 0
#define DISPLAYPORT 1
#define DLDVI 2
#define HDMI 3

#ifdef CHIP_IS_EP369
  #define HOST_INTERFACE DISPLAYPORT   
#elif CHIP_IS_EP269
  #define HOST_INTERFACE DLDVI 
#else
  #define HOST_INTERFACE UNDEFINED
#endif



#ifdef INTERNAL_IIC_CONFIGURATION_EEPROM
  #ifdef CHIP_IS_EP269 
    #error "EP269 uses an internally generated clock for reading the config eeprom.  This clock is not well-controlled and can vary in speeds from (as observed, range may be wider) 3-800KHz."
    #error "The SoftIIC slave mode cannot handle such speeds.  As a result, an external eeprom must be used."
  #endif
#endif


#ifdef EXTERNAL_IIC_CONFIGURATION_EEPROM
  #ifdef CHIP_IS_EP369 
    #ifndef RESET_OTHER_CHIPS
      #error "EP369 has a built in microcontroller which during most modes of operation (a reset state is the exception) locks up the IIC bus. As a result, an external eeprom may not be programmed in-system."
      #error "Use SoftIIC slave mode to allow for reconfigurable EDID."
    #endif
  #endif
#endif


#ifdef EDID_SDA_PIN  
  #ifdef EDID_SCL_PIN
    #define EXTERNAL_WRITEABLE_EDID
  #endif
#endif


#ifdef CONTROL_VREG_V1P3
#define CONTROL_VREG_SECONDARY CONTROL_VREG_V1P3
#define CONTROL_VREG_SECONDARY_POLARITY CONTROL_VREG_V1P3_POLARITY
#endif

#ifdef CONTROL_VREG_V1P8
#define CONTROL_VREG_SECONDARY CONTROL_VREG_V1P8
#define CONTROL_VREG_SECONDARY_POLARITY CONTROL_VREG_V1P8_POLARITY
#endif

#ifdef PGOOD_VREG_1P8
#define PGOOD_VREG_SECONDARY PGOOD_VREG_1P8
#endif

#ifdef PGOOD_VREG_1P3
#define PGOOD_VREG_SECONDARY PGOOD_VREG_1P3
#endif



#endif


