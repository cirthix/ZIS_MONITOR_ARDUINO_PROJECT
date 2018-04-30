#ifndef SUPPORTEDBLDRIVERS_H
#define SUPPORTEDBLDRIVERS_H

// There are three pins going to the backlight connector.  These are "BLON, "PWM_D", "PWM_A".  These pins can be mapped to support different backlight drivers.


#if BLDRIVER_VERSION==BLDRIVER_IS_NONEXISTANT
inline void board_bldriver_name(){ Serial.println(F("none"));}
// There is no backlight driver attached!
#define DEFAULT_BRIGHTNESS_LEVEL 100
#define Pulse_SUPPORT DISABLED
const uint8_t PWM_MAX_DUTYCYCLE=255;
const uint8_t PWM_MIN_DUTYCYCLE=0;
const uint16_t PWM_MAX_FREQUENCY = 20000;
const uint16_t PWM_MIN_FREQUENCY = 120;

#elif BLDRIVER_VERSION==BLDRIVER_IS_GENERIC
inline void board_bldriver_name(){ Serial.println(F("Generic PSU"));}
#define BACKLIGHT_PWM BLPIN_PWM_D
#define BACKLIGHT_ENABLE BLPIN_BLON
#define BACKLIGHT_PWM_POLARITY ACTIVE_HIGH
#define BACKLIGHT_ENABLE_POLARITY ACTIVE_HIGH
#define CONTROL_PSON BLPIN_PWM_A
#define CONTROL_PSON_POLARITY ACTIVE_LOW
// Safe limits
const uint8_t PWM_MAX_DUTYCYCLE=250;
#define DEFAULT_BRIGHTNESS_LEVEL 200
const uint8_t PWM_MIN_DUTYCYCLE=100;
const uint16_t PWM_MAX_FREQUENCY = 5000; 
const uint16_t PWM_MIN_FREQUENCY = 200; 
#define Pulse_SUPPORT ENABLED

#elif BLDRIVER_VERSION==BLDRIVER_IS_ZISWORKS
inline void board_bldriver_name(){ Serial.println(F("ZisWorks smart backlight driver"));}
#error "Not supported directly yet"


#elif BLDRIVER_VERSION==BLDRIVER_IS_VG278H
inline void board_bldriver_name(){ Serial.println(F("VG278H"));}
#define BACKLIGHT_PWM BLPIN_PWM_D
#define BACKLIGHT_PWM_POLARITY ACTIVE_HIGH
#define ALWAYS_LOW BLPIN_PWM_A // THIS IS ACTUALLY THE '3DMODE' PIN ON THE ASUS SUPPLY
#define BACKLIGHT_ENABLE BLPIN_BLON
#define BACKLIGHT_ENABLE_POLARITY ACTIVE_LOW
// Safe limits
const uint8_t PWM_MAX_DUTYCYCLE=255;
#define DEFAULT_BRIGHTNESS_LEVEL 200
const uint8_t PWM_MIN_DUTYCYCLE=20;
const uint16_t PWM_MAX_FREQUENCY = 20000; 
const uint16_t PWM_MIN_FREQUENCY = 200; 
#define Pulse_SUPPORT ENABLED



#elif BLDRIVER_VERSION==BLDRIVER_IS_XSTAR_DP2414LED
inline void board_bldriver_name(){ Serial.println(F("XSTAR"));}
#define BACKLIGHT_PWM BLPIN_PWM_D
#define BACKLIGHT_ENABLE BLPIN_BLON
#define BACKLIGHT_PWM_POLARITY ACTIVE_LOW
#define BACKLIGHT_ENABLE_POLARITY ACTIVE_HIGH
// Safe limits
const uint8_t PWM_MAX_DUTYCYCLE=255;
#define DEFAULT_BRIGHTNESS_LEVEL 200
const uint8_t PWM_MIN_DUTYCYCLE=20;
const uint16_t PWM_MAX_FREQUENCY = 15000; 
const uint16_t PWM_MIN_FREQUENCY = 200; 
#define Pulse_SUPPORT ENABLED
  
#else
#error "Unsupported BLDRIVER_VERSION"
#endif


#if Pulse_SUPPORT!=ENABLED
#if Pulse_SUPPORT!=DISABLED
  #error "Pulse_SUPPORT not specified"
#endif
#endif




#ifndef DEFAULT_BRIGHTNESS_LEVEL
#define DEFAULT_BRIGHTNESS_LEVEL 0
#endif







#endif
