#include <avr/pgmspace.h>
#include "SUPPORTED_PANELS.h"
#include "SUPPORTED_BOARDS.h"

// This isn't really  a header file so much as a pile of related functions

#define FORCE_INLINE __attribute__((always_inline)) inline

volatile uint8_t successful_bytes;

#ifdef CHIP_IS_EP369 
static uint8_t current_register_address_for_50 = 0x00;
#endif 

#ifdef CHIP_IS_EP269 
static uint8_t current_register_address_for_52 = 0x00;
static uint8_t current_register_address_for_53 = 0x00;
#endif


static void fastprinthexbyte(const uint8_t data) ;
static uint8_t virtualeeprom_ep269_config(uint8_t chipaddress, uint8_t registeraddress) ;
static uint8_t virtualeeprom_ep369_config(uint8_t chipaddress, uint8_t registeraddress);
static uint8_t virtualeeprom(uint8_t chipaddress, uint8_t registeraddress) ;
FORCE_INLINE    static uint8_t addr_check_iic_slave(uint8_t chipaddr);
FORCE_INLINE    static uint8_t get_current_register_address(uint8_t chipaddr);
FORCE_INLINE    static uint8_t set_current_register_address(uint8_t chipaddr, uint8_t regaddr);
FORCE_INLINE    static uint8_t read_iic_slave(uint8_t chipaddress, uint8_t* value);
FORCE_INLINE    static uint8_t write_iic_slave(uint8_t chipaddr, uint8_t value);


uint8_t virtualeeprom_ep269_config_dumped(uint8_t chipaddress, uint8_t registeraddress) {
//   VERY IMPORTANT NOTE: IN ORDER TO ACCESS PROGMEM BASED STRUCTURES, YOU MUST USE pgm_read_byte_near(STRUCTURE_NAME + OFFSET_WITHIN_STRUCTURE);
//   SEE THIS FOR MORE INFORMATION: http://playground.arduino.cc/Main/PROGMEM

PROGMEM const uint8_t EP269_CONFIG_EEPROM_0x53_MINIMAL[]          = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x02, 0x60, 0x34, 0x00, 0x44, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  
  uint8_t retval = 0xFF;
  if(chipaddress==0x53 && registeraddress<(sizeof(EP269_CONFIG_EEPROM_0x53)/sizeof(uint8_t))) {retval= pgm_read_byte_near(EP269_CONFIG_EEPROM_0x53_MINIMAL+registeraddress);}
  return retval;
  
}



#define EP269_MAX_RESOLUTION_X  2560 // more than 2560 is unsupported as per EPMI contact.  EP269 replaces additional area with corrupt/invalid data.
uint8_t virtualeeprom_ep269_config_zisworks(uint8_t chipaddress, uint8_t registeraddress) {
	// There is a procedure for generating the configuration, but it has been removed to avoid breaking NDA
  return virtualeeprom_ep269_config_dumped(chipaddress, registeraddress);
}



uint8_t virtualeeprom_ep269_config(uint8_t chipaddress, uint8_t registeraddress) {return virtualeeprom_ep269_config_zisworks(chipaddress, registeraddress);}


uint8_t virtualeeprom_all_zero_edid(uint8_t chipaddress, uint8_t registeraddress) {
  uint8_t retval = 0x00;
  return retval;  
}



uint8_t virtualeeprom_edid(uint8_t chipaddress, uint8_t registeraddress) {
  uint8_t retval = 0x00;
  if(chipaddress==0x50 && registeraddress<(SIZE_EDID)) {retval= BUFFERED_EDID[registeraddress];}
//           Serial.print("virtaleeprom:: Chip=0x") ;    fastprinthexbyte(chipaddress);  
//           Serial.print(", Reg=0x")   ;  fastprinthexbyte(registeraddress);    
//           Serial.print(", Val=0x") ;     fastprinthexbyte(retval);   
//           Serial.println("");      
  return retval;
  
}





uint8_t virtualeeprom(uint8_t chipaddress, uint8_t registeraddress) {  
#ifdef CHIP_IS_EP269 
return virtualeeprom_ep269_config( chipaddress,  registeraddress);
#endif

#ifdef CHIP_IS_EP369 
if(chipaddress==0x50) {return BUFFERED_EDID[registeraddress];}
else {return 0xff;}
//return virtualeeprom_edid( chipaddress,  registeraddress);
#endif
}










//////////////////////////////////////////////////////////// These functions should be edited to give the iic slave a 'personality'. //////////////////////////////////////////////////////////////// 




uint8_t respond_to_address(uint8_t chipaddr){  
#ifdef CHIP_IS_EP369 
  if((chipaddr>>1)==0x50) {
 //   Serial.println("!");
  return 1;}
#endif
#ifdef CHIP_IS_EP269 
  if((chipaddr>>1)==0x52) {return 1;}
  if((chipaddr>>1)==0x53) {return 1;}
#endif
 return 0;  
}


uint8_t respond_to_command(uint8_t commandaddr){  
 return 0x01;  
}


uint8_t respond_to_data(uint8_t commandaddr){  
 return 0x01;  
}



uint8_t get_current_register_address(uint8_t chipaddr){
#ifdef CHIP_IS_EP369 
  if(chipaddr==0x50) {return current_register_address_for_50;}
#endif
#ifdef CHIP_IS_EP269 
  if(chipaddr==0x52) {return current_register_address_for_52;}
  if(chipaddr==0x53) {return current_register_address_for_53;}
#endif
return 0x00;
}


uint8_t set_current_register_address(uint8_t chipaddr, uint8_t regaddr){
#ifdef CHIP_IS_EP369 
  if(chipaddr==0x50) { current_register_address_for_50=regaddr;}
#endif
#ifdef CHIP_IS_EP269 
  if(chipaddr==0x52) { current_register_address_for_52=regaddr;}
  if(chipaddr==0x53) { current_register_address_for_53=regaddr;}
#endif
return 0x00;
}

uint8_t read_iic_slave(uint8_t chipaddress, uint8_t* value){
  uint8_t registeraddress=get_current_register_address(chipaddress);
  *value=0;
  *value = virtualeeprom( chipaddress, registeraddress);
//  Serial.print(F("Incoming iic read of chipaddr 0x"));  fastprinthexbyte(chipaddr); 
//  Serial.print(F(" @ regaddr= 0x"));  fastprinthexbyte(registeraddress);
//  Serial.print(F(".  Returning value= 0x"));  fastprinthexbyte(*value);
//  Serial.println(F(""));
successful_bytes++;
return 0x00;
}

uint8_t write_iic_slave(uint8_t chipaddr, uint8_t value){
//  uint8_t registeraddress=get_current_register_address(chipaddr);
//  Serial.print(F("Incoming iic write to chipaddr 0x"));  fastprinthexbyte(chipaddr);
//  Serial.print(F(", regaddr=0x"));  fastprinthexbyte(registeraddress);
//  Serial.print(F("value="0x));  fastprinthexbyte(value);
//  Serial.println(F(""));  
successful_bytes++;
return 0x00;
}



