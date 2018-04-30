#include "INPUT_HANDLING.h"

InputHandling::InputHandling(){
InputHandling::ResetInputHistory();
}

void  InputHandling::ReadPhysicalInputs() {     
  if (input_index < FILTERDEPTH_INPUT-1) {    input_index ++;  } 
  else {    input_index=0;  }
  
  input_history[input_index] = COMMAND_CODE_FOR_NOTHING;
  
  #if SERIAL_COMMANDS_SIMPLE==ENABLED
  // If the serial port has button input, use that and ignore the real button board.  Also, virtual buttons bypass filtering.
              int incomingByte = Serial.read();      
              if (incomingByte > 0) {
                Serial.print("Got rx byte: ");                Serial.print(incomingByte&0xff, HEX);    
                uint8_t myChar = incomingByte & 0xff ;
                if(myChar>0x7f) { InputHandling::SetInputHistory( myChar ); return;} // All bytes with values above 127 are considered special zws serial commands.  Let the software handle it.
                switch (myChar) {            
                  #if SERIAL_COMMANDS_EXTENDED==ENABLED
                  case ASCII_CODE_FOR_SPECIAL_COMMANDS      : InputHandling::SetInputHistory( COMMAND_CODE_FOR_SPECIAL_COMMANDS      );  break;
          #endif
                  case ASCII_CODE_FOR_SLAVE_ON              : InputHandling::SetInputHistory( COMMAND_CODE_FOR_SLAVE_ON              );  break;
                  case ASCII_CODE_FOR_SLAVE_OFF             : InputHandling::SetInputHistory( COMMAND_CODE_FOR_SLAVE_OFF             );  break;
                  case ASCII_CODE_FOR_POWER_BUTTON          : InputHandling::SetInputHistory( COMMAND_CODE_FOR_POWER_BUTTON          );  break;
                  case ASCII_CODE_FOR_BRIGHTNESS_INCREASE   : InputHandling::SetInputHistory( COMMAND_CODE_FOR_BRIGHTNESS_INCREASE   );  break;
                  case ASCII_CODE_FOR_BRIGHTNESS_DECREASE   : InputHandling::SetInputHistory( COMMAND_CODE_FOR_BRIGHTNESS_DECREASE   );  break;
                  case ASCII_CODE_FOR_FACTORY_PROGRAM       : InputHandling::SetInputHistory( COMMAND_CODE_FOR_FACTORY_PROGRAM       );  break;
                  case ASCII_CODE_FOR_PWM_FREQ_DECREASE     : InputHandling::SetInputHistory( COMMAND_CODE_FOR_PWM_FREQ_DECREASE     );  break;
                  case ASCII_CODE_FOR_PWM_FREQ_INCREASE     : InputHandling::SetInputHistory( COMMAND_CODE_FOR_PWM_FREQ_INCREASE     );  break;
                  case ASCII_CODE_FOR_EDID_ROTATE           : InputHandling::SetInputHistory( COMMAND_CODE_FOR_EDID_ROTATE           );  break;
                  case ASCII_CODE_FOR_EDID_0                : InputHandling::SetInputHistory( COMMAND_CODE_FOR_EDID_0                );  break;
                  case ASCII_CODE_FOR_EDID_1                : InputHandling::SetInputHistory( COMMAND_CODE_FOR_EDID_1                );  break;
                  case ASCII_CODE_FOR_EDID_2                : InputHandling::SetInputHistory( COMMAND_CODE_FOR_EDID_2                );  break;
                  case ASCII_CODE_FOR_EDID_3                : InputHandling::SetInputHistory( COMMAND_CODE_FOR_EDID_3                );  break;
                  case ASCII_CODE_FOR_EDID_4                : InputHandling::SetInputHistory( COMMAND_CODE_FOR_EDID_4                );  break;
                  case ASCII_CODE_FOR_EDID_5                : InputHandling::SetInputHistory( COMMAND_CODE_FOR_EDID_5                );  break;
                  case ASCII_CODE_FOR_EDID_6                : InputHandling::SetInputHistory( COMMAND_CODE_FOR_EDID_6                );  break;
                  case ASCII_CODE_FOR_EDID_7                : InputHandling::SetInputHistory( COMMAND_CODE_FOR_EDID_7                );  break;
                  case ASCII_CODE_FOR_STROBE_ROTATE         : InputHandling::SetInputHistory( COMMAND_CODE_FOR_STROBE_ROTATE         );  break;
                  case ASCII_CODE_FOR_PANEL_OSD             : InputHandling::SetInputHistory( COMMAND_CODE_FOR_PANEL_OSD             );  break;
                  case ASCII_CODE_FOR_OCTESTMODE_ON         : InputHandling::SetInputHistory( COMMAND_CODE_FOR_OCTESTMODE_ON         );  break;
                  case ASCII_CODE_FOR_OCTESTMODE_OFF        : InputHandling::SetInputHistory( COMMAND_CODE_FOR_OCTESTMODE_OFF        );  break;
                  case ASCII_CODE_FOR_OSD_ON                : InputHandling::SetInputHistory( COMMAND_CODE_FOR_OSD_ON                );  break;
                  case ASCII_CODE_FOR_OSD_OFF               : InputHandling::SetInputHistory( COMMAND_CODE_FOR_OSD_OFF               );  break;
                  case ASCII_CODE_FOR_POWER_ON              : InputHandling::SetInputHistory( COMMAND_CODE_FOR_POWER_ON              );  break;
                  case ASCII_CODE_FOR_POWER_OFF             : InputHandling::SetInputHistory( COMMAND_CODE_FOR_POWER_OFF             );  break;
                }
                return;
        }
#endif

filter_is_dirty=1;
uint8_t button_combinations=0;
 uint16_t adc_key_in; 

#ifdef BUTTON_A_ANALOG  
adc_key_in = analogRead(BUTTON_A_ANALOG); 
 if ((adc_key_in >= cutoff_zero_low)  && (adc_key_in <= cutoff_zero_high))  { button_combinations|=(0x01<<BUTTON_UP_MASK);}
 if ((adc_key_in >= cutoff_one_low)   && (adc_key_in <= cutoff_one_high))   { input_history[input_index]=COMMAND_CODE_FOR_EDID_2; return;}
 if ((adc_key_in >= cutoff_two_low)   && (adc_key_in <= cutoff_two_high))   { input_history[input_index]=COMMAND_CODE_FOR_EDID_1; return;}
 if ((adc_key_in >= cutoff_three_low) && (adc_key_in <= cutoff_three_high)) { input_history[input_index]=COMMAND_CODE_FOR_EDID_0; return;}
#endif

#ifdef BUTTON_B_ANALOG
adc_key_in = analogRead(BUTTON_B_ANALOG);   
 if ((adc_key_in >= cutoff_zero_low)  && (adc_key_in <= cutoff_zero_high))  { button_combinations|=(0x01<<BUTTON_DOWN_MASK);}
 if ((adc_key_in >= cutoff_one_low)   && (adc_key_in <= cutoff_one_high))   { input_history[input_index]=COMMAND_CODE_FOR_PANEL_OSD; return;}
 if ((adc_key_in >= cutoff_two_low)   && (adc_key_in <= cutoff_two_high))   { input_history[input_index]=COMMAND_CODE_FOR_EDID_4; return;}
 if ((adc_key_in >= cutoff_three_low) && (adc_key_in <= cutoff_three_high)) { input_history[input_index]=COMMAND_CODE_FOR_EDID_3; return;}
#endif

#ifdef BUTTON_POWER
if(digitalRead(BUTTON_POWER)==LOW) {
//  Serial.println(F("B_p"));
  button_combinations|=(0x01<<BUTTON_POWER_MASK);
  }
#endif

#ifdef BUTTON_UP
if(digitalRead(BUTTON_UP)==LOW)    {
//  Serial.println(F("B_+"));
  button_combinations|=(0x01<<BUTTON_UP_MASK);
  }
#endif

#ifdef BUTTON_DOWN
if(digitalRead(BUTTON_DOWN)==LOW)  {
//  Serial.println(F("B_-"));
  button_combinations|=(0x01<<BUTTON_DOWN_MASK);
}
#endif

// Not supported on any board
//#ifdef BUTTON_EDID
//if(digitalRead(BUTTON_EDID)==LOW)  {
////  Serial.println(F("E_-"));
//  button_combinations|=(0x01<<BUTTON_EDID_MASK);
//}
//#endif
            
   switch (button_combinations) {
      case COMBO_FOR_POWER_BUTTON      : input_history[input_index]= COMMAND_CODE_FOR_POWER_BUTTON        ; break;
      case COMBO_BRIGHTNESS_INCREASE   : input_history[input_index]= COMMAND_CODE_FOR_BRIGHTNESS_INCREASE ; break;
      case COMBO_BRIGHTNESS_DECREASE   : input_history[input_index]= COMMAND_CODE_FOR_BRIGHTNESS_DECREASE ; break;
      case COMBO_FACTORY_PROGRAM       : input_history[input_index]= COMMAND_CODE_FOR_FACTORY_PROGRAM     ; break;
      case COMBO_PWM_FREQ_DECREASE     : input_history[input_index]= COMMAND_CODE_FOR_PWM_FREQ_DECREASE   ; break;
      case COMBO_PWM_FREQ_INCREASE     : input_history[input_index]= COMMAND_CODE_FOR_PWM_FREQ_INCREASE   ; break;
      case COMBO_CONDITIONAL_ROTATE    : input_history[input_index]= COMMAND_CODE_FOR_CONDITIONAL_ROTATE  ; break; // Note: edid rotate only works when off, strobe rotate only works when on
      default : ;// do nothing here and the current input history value will be set to COMMAND_CODE_FOR_NOTHING
   }
}



void  InputHandling::ResetInputHistory() {
  InputHandling::SetInputHistory(COMMAND_CODE_FOR_UNDEFINED);
}

void  InputHandling::SetInputHistory(uint8_t state) {
  current_filtered_input = state;
  previous_filtered_input = COMMAND_CODE_FOR_SERIAL;
  for (uint8_t i = 0; i < FILTERDEPTH_INPUT; i++) {
    input_history[i] = state;
  }
  input_index = 0; 
  filter_is_dirty=0;
}



void InputHandling::RefilterInputState()    {
//InputHandling::PrintState();  
  if(filter_is_dirty==0){return;}
  uint8_t found_filtered_input = input_history[0];
  uint8_t myindex = 0;
  while (myindex < FILTERDEPTH_INPUT) {
    if (input_history[myindex] == found_filtered_input) {
      myindex++;
    } else {
      previous_filtered_input = current_filtered_input;
      current_filtered_input = COMMAND_CODE_FOR_UNDEFINED;
      return ;
    }
  }
  previous_filtered_input = current_filtered_input;
  current_filtered_input = found_filtered_input;
    filter_is_dirty=0;
}


  uint8_t InputHandling::GetCurrentFilteredInput(){return current_filtered_input;}
  uint8_t InputHandling::GetPreviousFilteredInput(){return previous_filtered_input;}
  uint8_t InputHandling::GetCurrentUnFilteredInput(){return input_history[input_index];}


void InputHandling::PrintState()    {
Serial.print(F("Filter state:"));
  if(filter_is_dirty==0){Serial.print(F("DIRTY"));} else {Serial.print(F("CLEAN"));}
  Serial.print(F(" ["));
  uint8_t myindex = 0;
  while (myindex < FILTERDEPTH_INPUT) {
      Serial.print(input_history[myindex]);
      if(myindex < FILTERDEPTH_INPUT-1) {      Serial.print(F(", ")); }
      myindex++;
    } 
  Serial.print(F("] ("));
  Serial.print(input_index);
  Serial.print(F(") Filtered: "));
  Serial.print( InputHandling::GetCurrentFilteredInput());
  
  Serial.println(F(""));  
}

