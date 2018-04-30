#include "BACKLIGHT.h"

#define OSP_SET_WIDTH(cycles) (OCR2B = 0xff-(cycles-1))
#define OSP_FIRE() (TCNT2 = STROBE_TRIGGER_POINT)
#define OSP_FORCE_OFF() (TCNT2 = 0)




  //Good timer resources: http://sculland.com/ATmega168/Interrupts-And-Timers/8-Bit-Timer-Setup/ 
  // and: http://www.avrbeginners.net/architecture/timers/timers.html#pwm_mode

//#define TIMNG_DEBUG_MODE 1 // Uses the second pwm output as a timing debug signal instead of second led string pwm


Backlight::Backlight(){
Backlight::init(BACKLIGHT_MODE_OFF,  Backlight::EEpromReadStableBrightness(),  Backlight::EEpromReadPulseBrightness(), Backlight::EEpromReadFrequency());
}


void Backlight::init( uint8_t mode, uint8_t stable_brightness, uint8_t pulse_brightness, uint16_t pwmfrequency){
// Configure timer1 for 65ms rollover with no interrupts or pwm usage
    TCCR1A = 0x00;
    TCCR1B = 0x00;
    TIMSK1 = 0x00|(1<<TOIE1);  // Enable the timer1 overflow interrupt
    TCCR1B = TIMER_1_CLOCK_DIVIDER; // set clock scaler
    
    
    BrightnessStable=Backlight::LimitBrightness(stable_brightness);
    BrightnessPulse=pulse_brightness;
    FrequencyWanted=pwmfrequency;
       
pinMode(BACKLIGHT_PWM, OUTPUT);
  #if BACKLIGHT_PWM_POLARITY==ACTIVE_HIGH
digitalWrite(BACKLIGHT_PWM, LOW);
#else
digitalWrite(BACKLIGHT_PWM, HIGH);
#endif

#ifdef DATA_ENABLE 
pinMode(DATA_ENABLE, INPUT);
#endif

SetModeOff();

}




void Backlight::SetModeInitial(){
  FrequencyReal = 0;
  BrightnessStable=Backlight::LimitBrightness(Backlight::EEpromReadStableBrightness());
  BrightnessPulse=Backlight::EEpromReadPulseBrightness();  
  FrequencyWanted= Backlight::EEpromReadFrequency();
  Backlight::SetMode(Backlight::EEpromReadMode());  
  }


void Backlight::SetModeOff(){ 
//    TCCR2A=0x00;  
//    TCCR2B=0x00;  
    digitalWrite(BACKLIGHT_PWM, LOW);
    SetBLEN_OFF();
    SetHighCurrent_OFF();
    BacklightMode = BACKLIGHT_MODE_OFF;
    FrequencyReal = 0;  
}

void  Backlight::SetModeStable(){    
 // if(Backlight::GetMode() == BACKLIGHT_MODE_OFF ) {Backlight::SetBLEN_ON();}  
  Backlight::SetBLEN_ON();
  Backlight::ForceModeStable(); 
  Backlight::EEpromWriteMode();  
}

void Backlight::ForceModeStable(){
    BacklightMode = BACKLIGHT_MODE_STABLE;
    SetHighCurrent_OFF();
    TCCR2B=0x00;
    TCCR2A=(1<<WGM20); // Set phase-correct pwm mode
    Backlight::SetFrequencyPWM_NOSAVE(10000);
    uint8_t asdfasdf=Backlight::LimitBrightness(Backlight::RdBrightness());
    analogWrite(BACKLIGHT_PWM, asdfasdf);
//    Serial.print(F("asdfasdf:")); Serial.println(asdfasdf);    while(1){;}
}

void Backlight::SetModePulse(){
  Backlight::RecalculateStrobingParameters();
    analogWrite(BACKLIGHT_PWM, 0);
 // if(Backlight::GetMode() == BACKLIGHT_MODE_OFF ) {Backlight::SetBLEN_ON();}  
  Backlight::SetBLEN_ON();

  TCCR2B =  0;      // Halt counter by setting clock select bits to 0 (No clock source).
              // This keeps anything from happeneing while we get set up
  TCNT2 = 0x00;     // Start counting at bottom. 
  OCR2A = 0;      // Set TOP to 0. This effectively keeps us from counting becuase the counter just keeps reseting back to 0.
          // We break out of this by manually setting the TCNT higher than 0, in which case it will count all the way up to MAX and then overflow back to 0 and get locked up again.
          
  TCCR2A = _BV(COM2B0) | _BV(COM2B1) | _BV(WGM20) | _BV(WGM21); // OC2B=Set on Match, clear on BOTTOM. Mode 7 Fast PWM.
  
  Backlight::AdjustTimerStrobing();  // This part of the function is broken off to allow for updates to the duration and delay without fully reconfiguring the timer.
  // Note: TCCR2B is set in AdjustTimerStrobing
//  TCCR2B = _BV(WGM22)| TargetTimer2DividerStrobe;         // Start counting now. WGM22=1 to select Fast PWM mode 7

    SetHighCurrent_ON();
    BacklightMode = BACKLIGHT_MODE_PULSE;
    Backlight::EEpromWriteMode();  
//    Serial.println("In SetModePulse");    Backlight::PrintParameters();
}



void Backlight::AdjustTimerStrobing(){

  if((StrobePulseDelay+StrobePulseDuration)< 8000 ) { TargetTimer2DividerStrobe=0x06; } // Using the 1:256 clock divider for 8ms max duration wtih 32us resolution
  if((StrobePulseDelay+StrobePulseDuration)< 4000 ) { TargetTimer2DividerStrobe=0x05; } // Using the 1:128 clock divider for 4ms max duration wtih 16us resolution
  if((StrobePulseDelay+StrobePulseDuration)< 2000 ) { TargetTimer2DividerStrobe=0x04; } // Using the 1:64  clock divider for 2ms max duration wtih 8us resolution
  if((StrobePulseDelay+StrobePulseDuration)< 1000 ) { TargetTimer2DividerStrobe=0x03; } // Using the 1:32  clock divider for 1ms max duration wtih 4us resolution

  // If not careful, updating this while actively strobing could create a pulse on the pwm output that violates the minimum ontime and minimum offtime.
  // Avoid this by only updating while the timer is idle, but make sure that it is actually counting to avoid a lockup situation
  while((TCNT2!=0) && (Backlight::GetCurrentDivider()!=0x00))  { ; } 
  uint8_t myDuration= 1.0*(StrobePulseDuration*DetermineTimer2TickRate(TargetTimer2DividerStrobe))/1000000;
  uint8_t myDelay= 1.0*(StrobePulseDelay*DetermineTimer2TickRate(TargetTimer2DividerStrobe))/1000000;
  STROBE_TRIGGER_POINT=OCR2B-1-myDelay;
  OSP_SET_WIDTH(myDuration);
  
  TCCR2B = _BV(WGM22)| TargetTimer2DividerStrobe;         // Start counting now. WGM22=1 to select Fast PWM mode 7
}

uint8_t Backlight::GetMode(){ return BacklightMode;} 


void Backlight::SwapMode(){
  switch (BacklightMode) {
  case BACKLIGHT_MODE_OFF:            return Backlight::SetModeOff();      break;
  case BACKLIGHT_MODE_STABLE:         return Backlight::SetModePulse();    break;
  case BACKLIGHT_MODE_PULSE:          return Backlight::SetModeStable();   break;      
  default:                            return Backlight::SetModeOff();      
  }
}

void Backlight::SetMode(uint8_t mode){  
  switch (mode) {
  case BACKLIGHT_MODE_OFF:            return Backlight::SetModeOff();     break;
  case BACKLIGHT_MODE_STABLE:         return Backlight::SetModeStable();  break;
  case BACKLIGHT_MODE_PULSE:          return Backlight::SetModePulse();   break;      
  default:                            return Backlight::SetModeOff();      
  }
}

void  Backlight::EEpromWriteMode(){EEPROM.write(ADDRESS_BACKLIGHT_MODE, Backlight::GetMode());}

void  Backlight::EEpromWriteBrightness(){
  switch (Backlight::GetMode()) {
  case BACKLIGHT_MODE_OFF:                                        break;
  case BACKLIGHT_MODE_STABLE:       Backlight::EEpromWriteStableBrightness();  break;
  case BACKLIGHT_MODE_PULSE:        Backlight::EEpromWritePulseBrightness();   break;      
  default:                          ;
  }
}
  
void  Backlight::EEpromWriteStableBrightness(){EEPROM.write(ADDRESS_BACKLIGHT_LEVEL_STABLE, BrightnessStable);}
void  Backlight::EEpromWritePulseBrightness(){EEPROM.write(ADDRESS_BACKLIGHT_LEVEL_PULSE, BrightnessPulse);}
void  Backlight::EEpromWriteFrequency(){EEPROM.write(ADDRESS_PWM_FREQUENCY, FrequencyReal/SAVED_FREQUENCY_SCALING);}

uint8_t  Backlight::EEpromReadMode(){return EEPROM.read(ADDRESS_BACKLIGHT_MODE);}
uint8_t  Backlight::EEpromReadStableBrightness(){  return  Backlight::LimitBrightness(EEPROM.read(ADDRESS_BACKLIGHT_LEVEL_STABLE));}
uint8_t  Backlight::EEpromReadPulseBrightness(){  return  EEPROM.read(ADDRESS_BACKLIGHT_LEVEL_PULSE);}
uint16_t  Backlight::EEpromReadFrequency(){return EEPROM.read(ADDRESS_PWM_FREQUENCY)*SAVED_FREQUENCY_SCALING;}


void Backlight::SetFrequencyPWM(){return Backlight::SetFrequencyPWM(FrequencyWanted);}
void Backlight::SetFrequencyPWM_NOSAVE(){return Backlight::SetFrequencyPWM_NOSAVE(FrequencyWanted);}

void Backlight::SetFrequencyPWM(uint16_t frequency){  
    Backlight::SetFrequencyPWM_NOSAVE(frequency);  
    Backlight::EEpromWriteFrequency();
    Backlight::PrintFrequency();
}

void Backlight::SetFrequencyPWM_NOSAVE(uint16_t frequency){
    FrequencyWanted=frequency;
  uint8_t DividerWanted=Backlight::GetClosestDivider(FrequencyWanted);
  uint8_t DividerMinimum=GetClosestDivider_BUT_NOT_OVER(PWM_MAX_FREQUENCY);
  uint8_t DividerMaximum=GetClosestDivider_BUT_NOT_UNDER(PWM_MIN_FREQUENCY);
  if(DividerWanted<DividerMinimum) {DividerWanted=DividerMinimum;} 
  if(DividerWanted>DividerMaximum) {DividerWanted=DividerMaximum;} 
    TCCR2B =(DividerWanted & 0b00000111);
    FrequencyReal=Backlight::DetermineTimer2FrequencyPWM(Backlight::GetCurrentDivider());
}


uint8_t Backlight::GetClosestDivider(uint16_t frequency){
    uint8_t best_divider;
    int16_t absolute_error;
    int16_t absolute_error_last;    
absolute_error=frequency - Backlight::DetermineTimer2FrequencyPWM(0x01); absolute_error=abs(absolute_error);                                         best_divider=0x01; FrequencyReal=Backlight::DetermineTimer2FrequencyPWM(0x01);  absolute_error_last=absolute_error;  
absolute_error=frequency - Backlight::DetermineTimer2FrequencyPWM(0x02); absolute_error=abs(absolute_error); if(absolute_error<absolute_error_last) {best_divider=0x02; FrequencyReal=Backlight::DetermineTimer2FrequencyPWM(0x02);} absolute_error_last=absolute_error;  
absolute_error=frequency - Backlight::DetermineTimer2FrequencyPWM(0x03); absolute_error=abs(absolute_error); if(absolute_error<absolute_error_last) {best_divider=0x03; FrequencyReal=Backlight::DetermineTimer2FrequencyPWM(0x03);} absolute_error_last=absolute_error;  
absolute_error=frequency - Backlight::DetermineTimer2FrequencyPWM(0x04); absolute_error=abs(absolute_error); if(absolute_error<absolute_error_last) {best_divider=0x04; FrequencyReal=Backlight::DetermineTimer2FrequencyPWM(0x04);} absolute_error_last=absolute_error;  
absolute_error=frequency - Backlight::DetermineTimer2FrequencyPWM(0x05); absolute_error=abs(absolute_error); if(absolute_error<absolute_error_last) {best_divider=0x05; FrequencyReal=Backlight::DetermineTimer2FrequencyPWM(0x05);} absolute_error_last=absolute_error;  
absolute_error=frequency - Backlight::DetermineTimer2FrequencyPWM(0x06); absolute_error=abs(absolute_error); if(absolute_error<absolute_error_last) {best_divider=0x06; FrequencyReal=Backlight::DetermineTimer2FrequencyPWM(0x06);} absolute_error_last=absolute_error;  
absolute_error=frequency - Backlight::DetermineTimer2FrequencyPWM(0x07); absolute_error=abs(absolute_error); if(absolute_error<absolute_error_last) {best_divider=0x07; FrequencyReal=Backlight::DetermineTimer2FrequencyPWM(0x07);} 

return best_divider;
}


uint8_t Backlight::GetClosestDivider_BUT_NOT_OVER(uint16_t frequency){
if(Backlight::DetermineTimer2FrequencyPWM(0x01)<=frequency) {return 0x01;}
if(Backlight::DetermineTimer2FrequencyPWM(0x02)<=frequency) {return 0x02;}
if(Backlight::DetermineTimer2FrequencyPWM(0x03)<=frequency) {return 0x03;}
if(Backlight::DetermineTimer2FrequencyPWM(0x04)<=frequency) {return 0x04;}
if(Backlight::DetermineTimer2FrequencyPWM(0x05)<=frequency) {return 0x05;}
if(Backlight::DetermineTimer2FrequencyPWM(0x06)<=frequency) {return 0x06;}
                             return 0x07;
}

uint8_t Backlight::GetClosestDivider_BUT_NOT_UNDER(uint16_t frequency){
if(Backlight::DetermineTimer2FrequencyPWM(0x07)>=frequency) {return 0x07;}
if(Backlight::DetermineTimer2FrequencyPWM(0x06)>=frequency) {return 0x06;}
if(Backlight::DetermineTimer2FrequencyPWM(0x05)>=frequency) {return 0x05;}
if(Backlight::DetermineTimer2FrequencyPWM(0x04)>=frequency) {return 0x04;}
if(Backlight::DetermineTimer2FrequencyPWM(0x03)>=frequency) {return 0x03;}
if(Backlight::DetermineTimer2FrequencyPWM(0x02)>=frequency) {return 0x02;}
                             return 0x01;
}



uint8_t  Backlight::GetCurrentDivider1(){return TCCR1B&0b00000111;}
uint8_t  Backlight::GetCurrentDivider(){return TCCR2B&0b00000111;}


void Backlight::SetCurrentDivider(uint8_t divider){TCCR2B = (TCCR2B & 0b11111000) | (divider & 0b00000111); Backlight::EEpromWriteFrequency();}

uint16_t Backlight::DetermineTimer2Prescaler(uint8_t myDivider){
  switch (myDivider) {
  case 0x01:                return 1;
  case 0x02:                return 8;
  case 0x03:                return 32;
  case 0x04:                return 64;
  case 0x05:                return 128;
  case 0x06:                return 256;
  case 0x07:                return 1024;
  default:                  return 0;  
  }
}


uint32_t Backlight::DetermineTimer2TickRate(uint8_t myDivider){ // Returns ticks per second
    return REAL_SPEED / DetermineTimer2Prescaler(myDivider);
}
 // Returns total pwm frequency.  /256 in fast mode, /510 in phase-correct mode
uint16_t Backlight::DetermineTimer2FrequencyPWM(uint8_t myDivider){
    return ((DetermineTimer2TickRate(myDivider))/256);
}


void Backlight::DecrementFrequencyPWM(){  
  uint8_t DividerWanted=Backlight::GetCurrentDivider();  
  uint8_t DividerMaximum=GetClosestDivider_BUT_NOT_UNDER(PWM_MIN_FREQUENCY);
  if(DividerWanted<DividerMaximum) {
    Backlight::SetFrequencyPWM(DetermineTimer2FrequencyPWM(GetCurrentDivider()+1));
  }    
 }

 
void Backlight::IncrementFrequencyPWM(){
  #ifdef BACKLIGHT_ADIM
    return;
  #endif  
  
  uint8_t DividerWanted=Backlight::GetCurrentDivider();  
  uint8_t DividerMinimum=GetClosestDivider_BUT_NOT_OVER(PWM_MAX_FREQUENCY);
  if(DividerWanted>DividerMinimum) { 
    Backlight::SetFrequencyPWM(DetermineTimer2FrequencyPWM(GetCurrentDivider()-1));
  }
 }

 void Backlight::PrintFrequency(){  
    Serial.print(F("BLfreq\t\t"));   Serial.print(FrequencyReal, DEC );    Serial.println(F("Hz"));     
 }

 uint8_t Backlight::RdBrightness(){
  switch (Backlight::GetMode()) {
  case BACKLIGHT_MODE_OFF:          return 0;                       break;
  case BACKLIGHT_MODE_STABLE:       return BrightnessStable;        break;
  case BACKLIGHT_MODE_PULSE:        return BrightnessPulse;         break;     
  default:                          return 0;
  }
 }

  
 void Backlight::WrBrightness(uint8_t value){
  switch (Backlight::GetMode()) {
  case BACKLIGHT_MODE_OFF:                                   break;
  case BACKLIGHT_MODE_STABLE:       BrightnessStable=value;  break;
  case BACKLIGHT_MODE_PULSE:        BrightnessPulse=value;   break;    
  default:                          ;
  }
 }




 void Backlight::IncrementBrightness(){  
    if(Backlight::GetMode()==BACKLIGHT_MODE_OFF){return;}
  uint8_t BrightnessSet=Backlight::RdBrightness();    
  if (Backlight::RdBrightness() < 255) {      
    if(Backlight::GetMode()==BACKLIGHT_MODE_STABLE){
      BrightnessSet=Backlight::LimitBrightness(BrightnessSet+1);
    } else {
      BrightnessSet++;
    }
  }    
  if(Backlight::SetBrightnessNoSave(BrightnessSet)==0) {    Serial.println(F("++"));  }  else {    Serial.println(F(""));      }
}

void Backlight::DecrementBrightness(){
    if(Backlight::GetMode()==BACKLIGHT_MODE_OFF){return;}
  uint8_t BrightnessSet=Backlight::RdBrightness();    
  if (Backlight::RdBrightness() > 0) {      
    if(Backlight::GetMode()==BACKLIGHT_MODE_STABLE){
      BrightnessSet=Backlight::LimitBrightness(BrightnessSet-1);
    } else {
      BrightnessSet--;
    }
  }    
  if(Backlight::SetBrightnessNoSave(BrightnessSet)==0) {    Serial.println(F("--"));  }  else {    Serial.println(F(""));      }
}


uint8_t Backlight::SetBrightness(){return Backlight::SetBrightness(Backlight::RdBrightness());}
uint8_t Backlight::SetBrightness(uint8_t brightness){
  uint8_t retval=Backlight::SetBrightnessNoSave(brightness);
  Serial.println(F(""));
if(retval==0)    {
//  Serial.print(F("Save bright to EEPROM:"));  Serial.flush();  Serial.print(Backlight::RdBrightness(), DEC);  Serial.println(F(""));  Serial.flush();
  Backlight::EEpromWriteBrightness();
}
return retval;
}


uint8_t Backlight::LimitBrightness(uint8_t brightness){
  uint8_t BrightnessSet=brightness;  
    if(brightness> PWM_MAX_DUTYCYCLE ){ BrightnessSet=PWM_MAX_DUTYCYCLE; }
    if(brightness< PWM_MIN_DUTYCYCLE ){ BrightnessSet=PWM_MIN_DUTYCYCLE; } 
  return BrightnessSet;
}


uint8_t Backlight::SetBrightnessNoSave(){return Backlight::SetBrightnessNoSave(Backlight::RdBrightness());}

uint8_t Backlight::SetBrightnessNoSave(uint8_t brightness){
  uint8_t BrightnessSet=brightness;
if(BacklightMode==BACKLIGHT_MODE_STABLE){
  BrightnessSet=Backlight::LimitBrightness(brightness);
  analogWrite( BACKLIGHT_PWM , BrightnessSet);
}

  if(FRAME_TIMINGS_LOCKED == true){
    LOCKED_BLANKING=FILTERED_BLANKING;
  if(BacklightMode==BACKLIGHT_MODE_PULSE) {Backlight::RecalculateStrobingParameters(); }
  }
  Serial.print(F("BrightSet\t"));   Serial.print(BrightnessSet, DEC);  
  Backlight::WrBrightness(BrightnessSet);
return (BrightnessSet!=brightness);
}


uint8_t Backlight::GetBrightness(){return Backlight::RdBrightness();}

uint16_t Backlight::GetFrequencyPWM(){return FrequencyReal;}



static uint16_t GetDividerOfDivider1(uint8_t divider){
  switch (divider) {
  case 0x01:                return ( 1);     break;
  case 0x02:                return ( 8);     break;
  case 0x03:                return ( 64);     break;
  case 0x04:                return ( 256);     break;
  case 0x05:                return ( 1024);     break;
  default:                  return 0;     
  }
}



 uint32_t Backlight::TimerTicksToMicroseconds(uint16_t ticks){
  uint32_t mymicroseconds=ticks;
  mymicroseconds=mymicroseconds*GetDividerOfDivider1(TIMER_1_CLOCK_DIVIDER);
  mymicroseconds=mymicroseconds/(REAL_SPEED/1000000); 
  return mymicroseconds;
}

void Backlight::ResetFrameParameters(){
//  Serial.println(F("Reset frame parameters"));
      FILTERED_BLANKING = 0;
      LOCKED_BLANKING   = 0;
      FRAME_TIMINGS_LOCKED = false;
}



// In the filter function, we switch between a quickly convergent but possibly noisy function and a noise-rejecting slowly converging filter.
void Backlight::RefilterFrameParameters(){  
uint16_t BUF_SAMPLED_BLANKING=Backlight::CalcBlankingTime();
  
  if(FRAME_TIMINGS_LOCKED != true){
      FILTERED_BLANKING=FILTERED_BLANKING-(FILTERED_BLANKING/FILTER_FAST_RATE)+(BUF_SAMPLED_BLANKING/FILTER_FAST_RATE);
  if( (abs(FILTERED_BLANKING-BUF_SAMPLED_BLANKING)<FILTER_CONVERGENCE_HYSTERESIS)
    ) {

// Now we have timings which are supposedly 'locked'.  Do they make sense?
    if(     FILTERED_BLANKING<BLANKING_TIME_MINIMUM ||
            FILTERED_BLANKING>BLANKING_TIME_MAXIMUM )
    {Backlight::ResetFrameParameters(); return;}
    
    
//    Serial.println(F("Got lock"));
//Backlight::PrintParameters();
    FRAME_TIMINGS_LOCKED = true;
    LOCKED_BLANKING=FILTERED_BLANKING;   
    accumulated_error_BLANKING=0;

  if(BacklightDisabledTemporarilyUntilRelock == true){
    BacklightDisabledTemporarilyUntilRelock = false;
    if(Backlight::GetMode()!= BACKLIGHT_MODE_OFF){
      Backlight::SetBLEN_ON();  
      if(Backlight::GetMode()== BACKLIGHT_MODE_STABLE) { Backlight::ForceModeStable();}
    }
  }

  
  }
  } else {
      
    accumulated_error_BLANKING+=BUF_SAMPLED_BLANKING-FILTERED_BLANKING;

    if(abs(accumulated_error_BLANKING)>FILTER_CONVERGENCE_HYSTERESIS){
      if(accumulated_error_BLANKING>0){  FILTERED_BLANKING++;}
  else {    FILTERED_BLANKING--;}  
    accumulated_error_BLANKING=0;
}
int16_t DEVIATION_BLANKING=FILTERED_BLANKING-LOCKED_BLANKING;

  if( (abs(DEVIATION_BLANKING)>FILTER_LOCKLOSS_THRESHOLD)
       ){
//    Serial.println(F("Lost lock"));
    FRAME_TIMINGS_LOCKED = false;
//Backlight::PrintParameters();
  }
    
}

}


void Backlight::RecalculateStrobingParameters(){
  
  uint16_t myPulseDuration =  LOCKED_BLANKING;

  // Ensure within bounds
  if(myPulseDuration > StrobePulseDurationMaximum) {myPulseDuration=StrobePulseDurationMaximum;}
  if(myPulseDuration < StrobePulseDurationMinimum) {myPulseDuration=StrobePulseDurationMinimum;}

  // adjust brightness safely
  float myratio= ( 1.0 * BrightnessPulse) / PWM_MAX_DUTYCYCLE;
  myPulseDuration=((myPulseDuration-StrobePulseDurationMinimum)*myratio)+StrobePulseDurationMinimum; // Guarantees in-bounds result and full-scale usage of 0-255 brightness

// These values are still in microseconds.  Using the update paramteters function will set the timer values approprately
StrobePulseDuration=myPulseDuration;
StrobePulseDelay=LOCKED_BLANKING-myPulseDuration;

  if(BacklightMode==BACKLIGHT_MODE_PULSE){ 
     Backlight::AdjustTimerStrobing();
  }

}


uint8_t GetBit(uint8_t myBits, uint8_t myBit){
  if((myBits&(0x01<<myBit))==0) return 0; else return 1;  
}

void PrintTimer2ConfigurationRegisters(){
uint8_t my_TCCR2A = TCCR2A ;
uint8_t my_TCCR2B = TCCR2B ;
uint8_t my_TCNT2  = TCNT2  ;
uint8_t my_OCR2A  = OCR2A  ;
uint8_t my_OCR2B  = OCR2B  ;
uint8_t my_TIMSK2 = TIMSK2 ;
uint8_t my_TIFR2  = TIFR2  ;
uint8_t my_GTCCR  = GTCCR  ;

  
  Serial.print(F("TCCR2A : ")); Serial.println(my_TCCR2A);
  Serial.print(F("TCCR2B : ")); Serial.println(my_TCCR2B);
  Serial.print(F("TCNT2 : "));  Serial.println(my_TCNT2 );
  Serial.print(F("OCR2A : "));  Serial.println(my_OCR2A );
  Serial.print(F("OCR2B : "));  Serial.println(my_OCR2B );
  Serial.print(F("TIMSK2 : ")); Serial.println(my_TIMSK2);
  Serial.print(F("TIFR2 : "));  Serial.println(my_TIFR2 );
  Serial.print(F("GTCCR : "));  Serial.println(my_GTCCR );
  
  Serial.println(F("Parsed results :"));

  uint8_t my_FOC2=(GetBit(my_TCCR2B, FOC2A)<<1)|(GetBit(my_TCCR2B, FOC2B));
  uint8_t my_COM2A=(GetBit(my_TCCR2A, COM2A1)<<1)|(GetBit(my_TCCR2A, COM2A0));
  uint8_t my_COM2B=(GetBit(my_TCCR2A, COM2B1)<<1)|(GetBit(my_TCCR2A, COM2B0));
  uint8_t my_WGM2=(GetBit(my_TCCR2B, WGM22)<<2)|(GetBit(my_TCCR2A, WGM21)<<1)|(GetBit(my_TCCR2A, WGM20));
  uint8_t my_CS2=(GetBit(my_TCCR2B, CS22)<<2)|(GetBit(my_TCCR2B, CS21)<<1)|(GetBit(my_TCCR2B, CS20));

  
  Serial.print(F("my_CS2 : "));  Serial.println(my_CS2 );
  Serial.print(F("my_COM2A : "));  Serial.println(my_COM2A );
  Serial.print(F("my_COM2B : "));  Serial.println(my_COM2B );
  Serial.print(F("my_WGM2 : "));  Serial.println(my_WGM2 );
  Serial.print(F("my_FOC2 : "));  Serial.println(my_FOC2 );
  
}


void Backlight::PrintParameters(){
//PrintTimer2ConfigurationRegisters();

//     pinMode(BACKLIGHT_ENABLE, OUTPUT);  digitalWrite(BACKLIGHT_ENABLE, HIGH);  pinMode(BACKLIGHT_PWM, OUTPUT);  digitalWrite(BACKLIGHT_PWM, HIGH);  Serial.println(F("ENTERING FORCED ON MODE")); Serial.flush(); wdt_reset();
//delay(2000);
//     pinMode(BACKLIGHT_ENABLE, OUTPUT);  digitalWrite(BACKLIGHT_ENABLE, HIGH);  pinMode(BACKLIGHT_PWM, OUTPUT);  analogWrite(BACKLIGHT_PWM, 116);  Serial.println(F("ENTERING FORCED ON MODEpwm")); Serial.flush(); noInterrupts(); while(1) {wdt_reset();}


  
  Serial.print(F("\nMode      \t"));
  if(BacklightMode==BACKLIGHT_MODE_OFF){Serial.println(F("OFF"));} 
  else if(BacklightMode==BACKLIGHT_MODE_STABLE){Serial.println(F("PWM"));} 
  else if(BacklightMode==BACKLIGHT_MODE_PULSE){ Serial.println(F("Pulse")); }
  else {Serial.print(BacklightMode); Serial.println(F("?"));}  
  pinMode(BACKLIGHT_PWM, OUTPUT);
  analogWrite(BACKLIGHT_PWM, GetBrightness());
  Serial.print(F("Brightness\t")); Serial.println(GetBrightness());    
//  Serial.print(F("RefreshRate\t"));   Serial.print(Backlight::CalcRefreshRate());  Serial.println(F(" Hz"));   

  if(BacklightMode==BACKLIGHT_MODE_STABLE){ Backlight::PrintFrequency(); }
  
  Serial.print(F("VBlankDuration\t"));   Serial.print(Backlight::CalcBlankingTime());  Serial.println(F(" us"));  
  
  Serial.print(F("Frame Lock\t"));  if( FRAME_TIMINGS_LOCKED == true) {  Serial.println(F("YES"));    } else {Serial.println(F("NO"));}
  if(BacklightMode==BACKLIGHT_MODE_PULSE){  
  Serial.print(F("PulseDelay\t"));   Serial.print(StrobePulseDelay);  Serial.println(F(" us")); 
  Serial.print(F("PulseDuration\t"));   Serial.print(StrobePulseDuration);  Serial.println(F(" us")); 
 }
  
  Serial.print(F("\n"));
}



 uint16_t Backlight::CalcRefreshRate(){  
  if( RecentInputPulse != true) {return 0;}
  uint32_t CaptureA=MicrosecondsPerFrame;
  uint32_t CaptureB=MicrosecondsPerFrame;  
  while(CaptureA!=CaptureB) {CaptureA=MicrosecondsPerFrame; CaptureB=MicrosecondsPerFrame;}
  return 1000000/CaptureA; 
 }

uint16_t Backlight::PseudoAtomicRead_VerticalBlankingTimer1Ticks(){
  // This function is written a bit strangely with two captures having to be equal to compare.  
  // The reason for this is to emulate an atomic read, because the target value is volatile and updated in an interrupt.
  uint16_t CaptureA=VerticalBlankingTimer1Ticks;
  uint16_t CaptureB=VerticalBlankingTimer1Ticks;
  while(CaptureA!=CaptureB) {CaptureA=VerticalBlankingTimer1Ticks; CaptureB=VerticalBlankingTimer1Ticks;}
 // Serial.print(F("Timer1ReadValue\t"));   Serial.println(CaptureA);
  return CaptureA;  
}

 uint16_t Backlight::CalcBlankingTime(){ 
  if( RecentInputPulse != true) {return 0;}
  return Backlight::TimerTicksToMicroseconds(Backlight::PseudoAtomicRead_VerticalBlankingTimer1Ticks()); 
 }










 uint8_t Backlight::CheckForActiveVideo() {return RecentInputPulse;}
 void Backlight::ClearRecentPinInterrupt(){ RecentInputPulse=false; }
 void Backlight::SetRecentPinInterrupt(){ RecentInputPulse=true; }







// The framerate times don't have to be exact.
// Measurment of the blanking time should be as accurate as possible.  Use timer1 ticks for consistency.
void Backlight::Interrupt(){  
    if(BacklightMode == BACKLIGHT_MODE_PULSE) {
      OSP_FIRE(); 
    }   
    if(TCNT1> LINE_TIMEOUT_TICKS) {
      // This code runs at the beginning of a frame.
      VerticalBlankingTimer1Ticks=TCNT1;       
      uint32_t myMicros=micros();
      MicrosecondsPerFrame=myMicros-LastFrameMicros;
      LastFrameMicros=myMicros;
    }
    TCNT1=0;
    RecentInputPulse=true;   
}
 
uint8_t Backlight::FastReadDE()   { return digitalRead2(DATA_ENABLE);        }

void Backlight::SetBLEN_OFF(){  
#ifdef BACKLIGHT_ENABLE
  pinMode(BACKLIGHT_ENABLE, OUTPUT);
  #if BACKLIGHT_ENABLE_POLARITY==ACTIVE_HIGH
    digitalWrite(BACKLIGHT_ENABLE, LOW);
    analogWrite(BACKLIGHT_PWM, 0);
  #else
    digitalWrite(BACKLIGHT_ENABLE, HIGH);
  #endif   
#endif  
}

void Backlight::SetBLEN_ON(){
#ifdef BACKLIGHT_ENABLE
  pinMode(BACKLIGHT_ENABLE, OUTPUT);
  #if BACKLIGHT_ENABLE_POLARITY==ACTIVE_HIGH
    digitalWrite(BACKLIGHT_ENABLE, HIGH);
//  Serial.println(F("BLEN SET HIGH"));
  #else
    digitalWrite(BACKLIGHT_ENABLE, LOW);
//  Serial.println(F("nBLEN SET LOW"));
  #endif
#endif  
}



void Backlight::SetHighCurrent_OFF(){  
  #ifdef BACKLIGHT_HIGH_CURRENT_MODE
    pinMode(BACKLIGHT_HIGH_CURRENT_MODE, OUTPUT);
    digitalWrite(BACKLIGHT_HIGH_CURRENT_MODE, LOW);
  #endif  
}


void Backlight::SetHighCurrent_ON(){  
  #ifdef BACKLIGHT_HIGH_CURRENT_MODE
    pinMode(BACKLIGHT_HIGH_CURRENT_MODE, OUTPUT);
    digitalWrite(BACKLIGHT_HIGH_CURRENT_MODE, HIGH);
  #endif  
}












