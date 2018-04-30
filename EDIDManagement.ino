



uint8_t slave_has_corresponding_edid(uint8_t selected_edid) {
// uint8_t selected_edid=which_edid_selected();  
          if(selected_edid<NUMBER_OF_FACTORY_EDIDS){            
                switch (selected_edid) {
                  case 0  :  
                      #ifndef EDID_0_SLAVE
                           return false;
                      #else
                          return true;
                      #endif
                      break;
                  case 1  : 
                      #ifndef EDID_1_SLAVE
                           return false;
                      #else
                          return true;
                      #endif
                      break;
                  case 2  : 
                      #ifndef EDID_2_SLAVE
                           return false;
                      #else
                          return true;
                      #endif
                      break;
                  case 3  : 
                      #ifndef EDID_3_SLAVE
                           return false;
                      #else
                          return true;
                      #endif
                      break;
                  case 4  : 
                      #ifndef EDID_4_SLAVE
                           return false;
                      #else
                          return true;
                      #endif
                      break;
                  case 5  : 
                      #ifndef EDID_5_SLAVE
                           return false;
                      #else
                          return true;
                      #endif
                      break;
                  case 6  : 
                      #ifndef EDID_6_SLAVE
                           return false;
                      #else
                          return true;
                      #endif
                      break;
                  case 7  : 
                      #ifndef EDID_7_SLAVE
                           return false;
                      #else
                          return true;
                      #endif
                  default : return false;
                }
          } else {
               return true;           
          }
}

uint8_t which_edid_selected() {
  if (UserConfiguration_LoadEDID() >= NUMBER_OF_FACTORY_EDIDS ){
    return 0; // Default to EDID_0_SLOT
    }
  else {return UserConfiguration_LoadEDID();}
}

void PrintWhichEDIDSelected(){
  uint8_t selected_edid=which_edid_selected();
    if(selected_edid<NUMBER_OF_FACTORY_EDIDS){   
        Serial.print(F("\t->EDID_")); Serial.print(selected_edid); 
        if(I_AM_A_SLAVE==true) {
            Serial.print(F("_SLAVE"));
        } else {
            Serial.print(F("_SLOT"));
        }
    } else {
         Serial.print(F("\t->User0")); 
    }                 
}

void buffer_the_edid(){return buffer_the_edid(which_edid_selected());}

void buffer_the_edid(uint8_t selected_edid){
  for(uint16_t tmpaddr=0; tmpaddr<SIZE_EDID; tmpaddr++){
      BUFFERED_EDID[tmpaddr]=0x00;
      if(tmpaddr<EDID_SIGNATURE_SIZE)          { BUFFERED_EDID[tmpaddr]=pgm_read_byte_near(EDID_SIGNATURE+tmpaddr);      }
      else
          if(selected_edid<NUMBER_OF_FACTORY_EDIDS){        
                switch (selected_edid) {
                  case 0  : 
                      if(I_AM_A_SLAVE==true) {
                          #ifdef EDID_0_SLAVE
                              if(tmpaddr<sizeof(EDID_0_SLAVE))   { BUFFERED_EDID[tmpaddr]=pgm_read_byte_near(EDID_0_SLAVE+tmpaddr); }
                          #else
                              BUFFERED_EDID[tmpaddr]=0x00;
                          #endif
                      } else {
                          #ifdef EDID_0_SLOT
                              if(tmpaddr<sizeof(EDID_0_SLOT))    { BUFFERED_EDID[tmpaddr]=pgm_read_byte_near(EDID_0_SLOT+tmpaddr); }
                          #else
                              BUFFERED_EDID[tmpaddr]=0x00;
                          #endif
                      }
                      break;
                  case 1  : 
                      if(I_AM_A_SLAVE==true) {
                          #ifdef EDID_1_SLAVE
                              if(tmpaddr<sizeof(EDID_1_SLAVE))   { BUFFERED_EDID[tmpaddr]=pgm_read_byte_near(EDID_1_SLAVE+tmpaddr); }
                          #else
                              BUFFERED_EDID[tmpaddr]=0x00;
                          #endif
                      } else {
                          #ifdef EDID_1_SLOT
                              if(tmpaddr<sizeof(EDID_1_SLOT))    { BUFFERED_EDID[tmpaddr]=pgm_read_byte_near(EDID_1_SLOT+tmpaddr); }
                          #else
                              BUFFERED_EDID[tmpaddr]=0x00;
                          #endif
                      }
                      break;
                  case 2  : 
                      if(I_AM_A_SLAVE==true) {
                          #ifdef EDID_2_SLAVE
                              if(tmpaddr<sizeof(EDID_2_SLAVE))   { BUFFERED_EDID[tmpaddr]=pgm_read_byte_near(EDID_2_SLAVE+tmpaddr); }
                          #else
                              BUFFERED_EDID[tmpaddr]=0x00;
                          #endif
                      } else {
                          #ifdef EDID_2_SLOT
                              if(tmpaddr<sizeof(EDID_2_SLOT))    { BUFFERED_EDID[tmpaddr]=pgm_read_byte_near(EDID_2_SLOT+tmpaddr); }
                          #else
                              BUFFERED_EDID[tmpaddr]=0x00;
                          #endif
                      }
                      break;
                  case 3  : 
                      if(I_AM_A_SLAVE==true) {
                          #ifdef EDID_3_SLAVE
                              if(tmpaddr<sizeof(EDID_3_SLAVE))   { BUFFERED_EDID[tmpaddr]=pgm_read_byte_near(EDID_3_SLAVE+tmpaddr); }
                          #else
                              BUFFERED_EDID[tmpaddr]=0x00;
                          #endif
                      } else {
                          #ifdef EDID_3_SLOT
                              if(tmpaddr<sizeof(EDID_3_SLOT))    { BUFFERED_EDID[tmpaddr]=pgm_read_byte_near(EDID_3_SLOT+tmpaddr); }
                          #else
                              BUFFERED_EDID[tmpaddr]=0x00;
                          #endif
                      }
                      break;
                  case 4  : 
                      if(I_AM_A_SLAVE==true) {
                          #ifdef EDID_4_SLAVE
                              if(tmpaddr<sizeof(EDID_4_SLAVE))   { BUFFERED_EDID[tmpaddr]=pgm_read_byte_near(EDID_4_SLAVE+tmpaddr); }
                          #else
                              BUFFERED_EDID[tmpaddr]=0x00;
                          #endif
                      } else {
                          #ifdef EDID_4_SLOT
                              if(tmpaddr<sizeof(EDID_4_SLOT))    { BUFFERED_EDID[tmpaddr]=pgm_read_byte_near(EDID_4_SLOT+tmpaddr); }
                          #else
                              BUFFERED_EDID[tmpaddr]=0x00;
                          #endif
                      }
                      break;
                  case 5  : 
                      if(I_AM_A_SLAVE==true) {
                          #ifdef EDID_5_SLAVE
                              if(tmpaddr<sizeof(EDID_5_SLAVE))   { BUFFERED_EDID[tmpaddr]=pgm_read_byte_near(EDID_5_SLAVE+tmpaddr); }
                          #else
                              BUFFERED_EDID[tmpaddr]=0x00;
                          #endif
                      } else {
                          #ifdef EDID_5_SLOT
                              if(tmpaddr<sizeof(EDID_5_SLOT))    { BUFFERED_EDID[tmpaddr]=pgm_read_byte_near(EDID_5_SLOT+tmpaddr); }
                          #else
                              BUFFERED_EDID[tmpaddr]=0x00;
                          #endif
                      }
                      break;
                  case 6  : 
                      if(I_AM_A_SLAVE==true) {
                          #ifdef EDID_6_SLAVE
                              if(tmpaddr<sizeof(EDID_6_SLAVE))   { BUFFERED_EDID[tmpaddr]=pgm_read_byte_near(EDID_6_SLAVE+tmpaddr); }
                          #else
                              BUFFERED_EDID[tmpaddr]=0x00;
                          #endif
                      } else {
                          #ifdef EDID_6_SLOT
                              if(tmpaddr<sizeof(EDID_6_SLOT))    { BUFFERED_EDID[tmpaddr]=pgm_read_byte_near(EDID_6_SLOT+tmpaddr); }
                          #else
                              BUFFERED_EDID[tmpaddr]=0x00;
                          #endif
                      }
                      break;
                  case 7  : 
                      if(I_AM_A_SLAVE==true) {
                          #ifdef EDID_7_SLAVE
                              if(tmpaddr<sizeof(EDID_7_SLAVE))   { BUFFERED_EDID[tmpaddr]=pgm_read_byte_near(EDID_7_SLAVE+tmpaddr); }
                          #else
                              BUFFERED_EDID[tmpaddr]=0x00;
                          #endif
                      } else {
                          #ifdef EDID_7_SLOT
                              if(tmpaddr<sizeof(EDID_7_SLOT))    { BUFFERED_EDID[tmpaddr]=pgm_read_byte_near(EDID_7_SLOT+tmpaddr); }
                          #else
                              BUFFERED_EDID[tmpaddr]=0x00;
                          #endif
                      }
                      break;
                  default : BUFFERED_EDID[tmpaddr]=0x00;
                }
          } else {
         BUFFERED_EDID[tmpaddr]=0x00;
          }
      }  
 if( check_buffered_edid()) { 
  //dump_buffered_edid();
  rotate_selected_edid();}
}

boolean check_if_allzero_edid(){
boolean allZeroes = true;
  for(uint8_t i=EDID_SIGNATURE_SIZE; i<128; i++){
      if( BUFFERED_EDID[i]!=0) { return false; } 
    }        
     for(uint16_t i=0; i<SIZE_EDID; i++){BUFFERED_EDID[i]=0;}
    return true;
}

uint8_t check_buffered_edid(){
// Return 0 if edid appears valid, 1 otherwise.
const uint8_t NUMBER_OF_EXTENSIONS = 127-1;
Serial.println(F("Checking EDID"));

if(check_if_allzero_edid()==true) { return 0;}  
//dump_buffered_edid();
  for(uint8_t i=0; i<EDID_SIGNATURE_SIZE; i++){
      if( BUFFERED_EDID[i]!=pgm_read_byte_near(EDID_SIGNATURE+i)) {
      Serial.println(F("EDID badsig"));
      return 1;
    }        
  }
  
uint8_t checksum=0; for(uint16_t i=0; i<128; i++){checksum+=BUFFERED_EDID[i];}
if(checksum!=0) {  Serial.println(F("EDID badsum"));  return 1;}
  if(BUFFERED_EDID[NUMBER_OF_EXTENSIONS]==0) {return 0;}  // Only handle one extension block and then return

 checksum=0; for(uint16_t i=128; i<=255; i++){checksum+=BUFFERED_EDID[i];}
if(checksum!=0) {  Serial.println(F("EDID EXT badsum"));  return 1;}
return 0;  
}

void rotate_selected_edid() {
  if (which_edid_selected() <( NUMBER_OF_FACTORY_EDIDS-1)) {
    UserConfiguration_SaveEDID(UserConfiguration_LoadEDID() + 1);
  }
  else {
    UserConfiguration_SaveEDID(0);
  }
  set_selected_edid();
}

void set_selected_edid(uint8_t desired_edid) {  
  if(desired_edid<NUMBER_OF_FACTORY_EDIDS){ UserConfiguration_SaveEDID(desired_edid); }
  set_selected_edid();
}

void set_selected_edid() {
  disconnect_hotplug();
  buffer_the_edid();

#ifdef EXTERNAL_WRITEABLE_EDID
  StopSyncInterrupt();
  write_external_edid();
  StartSyncInterrupt();
  zdelay(100);
  return;
#endif


#ifdef CHIP_IS_EP369 
  #ifdef EXTERNAL_IIC_CONFIGURATION_EEPROM
    StopSyncInterrupt();
    power_down_display();
    power_down_board();  
    write_external_all_zero_config(); 
    power_up_board();  
    if(check_if_allzero_edid()==false) {
      zdelay(1000);   
      power_down_board();  
      write_config_eeprom();
      power_up_board();  
    }
    StartSyncInterrupt();
    zdelay(100);
    softReset();
    return;
  #endif
#endif

select_hardware_edid();

connect_hotplug();
}



void select_hardware_edid(){    
#ifdef EDID_SELECT
uint8_t selected_edid=UserConfiguration_LoadEDID();
  Serial.print(F("HW EDID #"));  Serial.println(selected_edid);
pinMode(EDID_SELECT,OUTPUT);
if(selected_edid==0){  digitalWrite(EDID_SELECT, LOW);  } else{    digitalWrite(EDID_SELECT, HIGH);    }
#endif  
  }



void dump_buffered_edid(){
  Serial.print(F(" EDID Buffer:"));
  for(uint16_t i=0; i<128; i++){
    Serial.print(F(" 0x")); fastprinthexbyte(BUFFERED_EDID[i]);
  }
  uint8_t print_the_extension_block=false;
  for(uint16_t i=128; i<SIZE_EDID; i++){
    if(BUFFERED_EDID[i]!=0x00) {print_the_extension_block=true;}
  }

  if(print_the_extension_block==true) {
    for(uint16_t i=128; i<SIZE_EDID; i++){
      Serial.print(F(" 0x")); fastprinthexbyte(BUFFERED_EDID[i]);
    }
  }
  Serial.println(F(""));
}


void disconnect_hotplug() {  
  #ifdef CHIP_IS_EP369
    digitalWrite(RESET_OTHER_CHIPS, LOW);
    zdelay(10);
  #endif
}

void connect_hotplug() {  
  #ifdef CHIP_IS_EP369
    digitalWrite(RESET_OTHER_CHIPS, HIGH);
    zdelay(10);
  #endif
}

