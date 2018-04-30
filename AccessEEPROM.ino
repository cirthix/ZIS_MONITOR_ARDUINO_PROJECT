// Moves all of the read/write/update functions to eeprom storage into one place
// Most of this code is simple stubs, avoid cluttering the other files.

uint8_t UserConfiguration_LoadShutdown(){return EEPROM.read(ADDRESS_POWER_STATE);}
void UserConfiguration_SaveShutdown(uint8_t myValue){ EEPROM.write(ADDRESS_POWER_STATE, myValue); }
void UserConfiguration_SaveDefaultShutdown(){CheckBeforeWritingEEPROM(ADDRESS_POWER_STATE, FACTORY_DEFAULT_POWERSTATE); }

uint8_t UserConfiguration_LoadEDID(){return EEPROM.read(ADDRESS_SELECTED_EDID);}
void UserConfiguration_SaveEDID(uint8_t myValue){ EEPROM.write(ADDRESS_SELECTED_EDID, myValue); }
void UserConfiguration_SaveDefaultEDID(){CheckBeforeWritingEEPROM(ADDRESS_SELECTED_EDID, FACTORY_DEFAULT_SELECTED_EDID); }

uint8_t UserConfiguration_LoadBacklightMode(){return EEPROM.read(ADDRESS_BACKLIGHT_MODE);}
void UserConfiguration_SaveBacklightMode(uint8_t myValue){ EEPROM.write(ADDRESS_BACKLIGHT_MODE, myValue); }
void UserConfiguration_SaveDefaultBacklightMode(){CheckBeforeWritingEEPROM(ADDRESS_BACKLIGHT_MODE, FACTORY_DEFAULT_BACKLIGHT_MODE); }

uint8_t UserConfiguration_LoadBrightnessStable(){return EEPROM.read(ADDRESS_BACKLIGHT_LEVEL_STABLE);}
void UserConfiguration_SaveBrightnessStable(uint8_t myValue){ EEPROM.write(ADDRESS_BACKLIGHT_LEVEL_STABLE, myValue); }
void UserConfiguration_SaveDefaultBrightnessStable(){CheckBeforeWritingEEPROM(ADDRESS_BACKLIGHT_LEVEL_STABLE, FACTORY_DEFAULT_BACKLIGHT_STABLE_BRIGHTNESS); }

uint8_t UserConfiguration_LoadBrightnessPulse(){return EEPROM.read(ADDRESS_BACKLIGHT_LEVEL_PULSE);}
void UserConfiguration_SaveBrightnessPulse(uint8_t myValue){ EEPROM.write(ADDRESS_BACKLIGHT_LEVEL_PULSE, myValue); }
void UserConfiguration_SaveDefaultBrightnessPulse(){CheckBeforeWritingEEPROM(ADDRESS_BACKLIGHT_LEVEL_PULSE, FACTORY_DEFAULT_BACKLIGHT_PULSE_BRIGHTNESS); }

uint8_t UserConfiguration_LoadFrequencyPWM(){return EEPROM.read(ADDRESS_PWM_FREQUENCY);}
void UserConfiguration_SaveFrequencyPWM(uint8_t myValue){ EEPROM.write(ADDRESS_PWM_FREQUENCY, myValue); }
void UserConfiguration_SaveDefaultFrequencyPWM(){CheckBeforeWritingEEPROM(ADDRESS_PWM_FREQUENCY, FACTORY_DEFAULT_BACKLIGHT_PULSE_BRIGHTNESS); }

uint8_t UserConfiguration_LoadWasSlave(){return EEPROM.read(ADDRESS_WAS_SLAVE);}
void UserConfiguration_SaveWasSlave(uint8_t myValue){ EEPROM.write(ADDRESS_WAS_SLAVE, myValue); }
void UserConfiguration_SaveDefaultWasSlave(){CheckBeforeWritingEEPROM(ADDRESS_WAS_SLAVE, false); }

uint8_t UserConfiguration_LoadOSD(){return EEPROM.read(ADDRESS_OSD_ENABLED);}
void UserConfiguration_SaveOSD(uint8_t myValue){ EEPROM.write(ADDRESS_OSD_ENABLED, myValue); }
void UserConfiguration_SaveDefaultOSD(){CheckBeforeWritingEEPROM(ADDRESS_OSD_ENABLED, FACTORY_DEFAULT_USE_OSD); }

uint8_t UserConfiguration_LoadTestMode(){return EEPROM.read(ADDRESS_USE_OCTESTMODE);}
void UserConfiguration_SaveTestMode(uint8_t myValue){ EEPROM.write(ADDRESS_USE_OCTESTMODE, myValue); }
void UserConfiguration_SaveDefaultTestMode(){CheckBeforeWritingEEPROM(ADDRESS_USE_OCTESTMODE, FACTORY_DEFAULT_USE_OCTESTMODE); }

uint8_t UserConfiguration_LoadMagicByte(){return EEPROM.read(ADDRESS_MAGIC_BYTE);}
void UserConfiguration_SaveDefaultMagicByte(){CheckBeforeWritingEEPROM(ADDRESS_MAGIC_BYTE, MagicByte()); }

void CheckBeforeWritingEEPROM(uint16_t myAddress, uint8_t myValue){
  if(myValue==EEPROM.read(myAddress)) {return;}  
  EEPROM.write(myAddress, myValue);
}

uint8_t DetermineIfFactoryProgrammed(){ return (UserConfiguration_LoadMagicByte() == (MagicByte())); }

// This function is entirely compiled away into a single byte value known at compile-time
uint8_t MagicByte(){
const uint8_t array_size=24;
const uint8_t compile_date_time[array_size] = "" __DATE__ " " __TIME__ ""; // Example string: "Apr  3 2016 01:44:37"
uint8_t hashed_value= 
compile_date_time[0]+
compile_date_time[1]+
compile_date_time[2]+
compile_date_time[3]+
compile_date_time[4]+
compile_date_time[5]+
compile_date_time[6]+
compile_date_time[7]+
compile_date_time[8]+
compile_date_time[9]+
compile_date_time[10]+
compile_date_time[11]+
compile_date_time[12]+
compile_date_time[13]+
compile_date_time[14]+
compile_date_time[15]+
compile_date_time[16]+
compile_date_time[17]+
compile_date_time[18]+
compile_date_time[19];
  
  if(hashed_value== 0xFF || hashed_value==0x00) {return 0xa9;}
return hashed_value;
}
