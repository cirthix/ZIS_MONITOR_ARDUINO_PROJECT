/*
 * THIS CODE WILL FAIL TO COMPILE IN AN UNMODIFIED ARDUINO ENVIRONMENT! 
 * License for use is granted for use with ZisWorks hardware only.  Use of this code for any other purpose is prohibited.
 * TO AVOID DAMAGING YOUR BOARD AND/OR PANEL, CORRECTLY SET THE VALUES UNDER " CHANGE SYSTEM CONFIGURATION PARAMETERS HERE " IN "CONSTANTS.H"
 * This code is intended for use with the ZisWorks "DP2LVDS/DVI2LVDS" board. 
 * Timing in this application is absolutely critical, so some core files had to be modified to avoid the use of interrupts.  
 * Notably, serial buffer transfer and timer0 overflow handling.  These tasks have been pushed into the control loop.
 * 
 * This version is a work-in-progress, though it is in a fairly stable state and is perfectly usable.  
 * 
 * MODIFICATIONS AND LIBRARIES NECESSARY
 * ... a bunch of them.  Will update this when posting the DP2LVDS firmware, many changes are shared.
 * 
 * 
 * Please forgive the coding.
 */
 
 #pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic push

#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <avr/wdt.h>
#include "constants.h"
#include "BACKLIGHT.h"
#include "INPUT_HANDLING.h"
uint8_t BUFFERED_EDID[SIZE_EDID];
#include "IIC_PERSONALITY.h"
#include <SoftIIC.h>

#if OVERCLOCKING==ENABLED
#include "Overclock.h"
#endif


InputHandling Inputs;
Backlight Backlight;

#if OVERCLOCKING==ENABLED
Overclock my_Overclock_object=Overclock(OVERCLOCKED_SPEED, SERIAL_BAUD);
#endif
SoftIIC my_SoftIIC_EPMI=SoftIIC(SCL_PIN, SDA_PIN, IIC_SPEED/CLOCK_RATIO, false, false, true);

#ifdef EXTERNAL_WRITEABLE_EDID  
SoftIIC my_SoftIIC_EDID=SoftIIC(EDID_SCL_PIN, EDID_SDA_PIN, EDID_IIC_SPEED/CLOCK_RATIO, false, false, true);
#endif

#undef SERIAL_EPMI_RXD
#ifdef SERIAL_EPMI_RXD  
  #ifdef SERIAL_EPMI_TXD  
    SoftwareSerial SerialEPMI(SERIAL_EPMI_RXD, SERIAL_EPMI_TXD); 
  #endif
#endif


// Interrupt service run when Timer/Counter1 OVERFLOW
ISR(TIMER1_OVF_vect) { Backlight.ClearRecentPinInterrupt(); }

ISR(WDT_vect) {  
#if OVERCLOCKING==ENABLED
my_Overclock_object.WatchdogFired();
#endif
// Add your other watchdog code here.
}


uint8_t BoardWasOnline=true;
uint8_t PanelWasOnline=true;
uint8_t VideoWasActive=false;

uint32_t currentMillis=0;
uint32_t Task1ms_previousMillis=0;
uint32_t Task10ms_previousMillis=0;
uint32_t Task100ms_previousMillis=0;
uint32_t Task1000ms_previousMillis=0;
uint32_t Task10000ms_previousMillis=0;  
uint8_t VIDEO_SHOULD_BE_ACTIVE=false;
uint8_t I_AM_A_SLAVE=false;
unsigned long FinishedConfigurationTime=0;
uint16_t MeasuredRefreshRate=0;
uint16_t NewMeasuredRefreshRate=0;

void setup()
{



  StopSyncInterrupt();
  wdt_reset();
  Serial.flush();
  Serial.begin(SERIAL_BAUD); // Note: not yet overclocked.
  DisablePullupSerialRX();  
  Serial.print(F("PCB: ")); board_print_name(); 
  Serial.print(F("PS: ")); board_bldriver_name();
  Serial.print(F("CFG: ")); panel_print_name();
  Serial.println(F("" __DATE__ " " __TIME__ "\n"));   
  if ((!DetermineIfFactoryProgrammed())) {   do_factory_configuration();  }  // This is very early because we don't want anything to interfere with factory programming.  It should run standalone.
  
  Serial.println(F("\nINIT"));
  Serial.flush();
  // Note: atmega chips power up with all pins in input mode (high impedance).  When first using a pin as an output, it must be declared as such.
  SetStaticPins(); 
  select_hardware_edid();
 
#if OVERCLOCKING==ENABLED
  Serial.print(F("\nINIT/OC\n\t"));   
  Serial.flush(); 
  my_Overclock_object.SetClockSpeedOC();
  Serial.print(F("Clock: ")); Serial.print(my_Overclock_object.GetClockSpeed());   Serial.println(F("Hz")); 
#endif

// ENABLE THE WATCHDOG TIMER
  Serial.println(F("\nINIT/WDT")); 
  Serial.flush(); // Flush serial output in case somethinge goes horribly wrong, that way we guarantee seeing the last line of output.
    configure_watchdog_timer();     
  Serial.println(F("\t->OK"));

Serial.println(F("\nINIT/MISC"));    
    I_AM_A_SLAVE=UserConfiguration_LoadWasSlave();
//    DetermineMasterSlave();
//    MasterSlaveChangedStates(); // Set the edid and print which one

  Serial.println(F("\nINIT/PWR"));   
   VideoWasActive=false;
   power_down_display();   
   power_up_board();    
  Serial.println(F("\t->OK"));   

 
  Serial.println(F("\nINIT/Interrupt"));    
  Serial.flush(); // Flush serial output in case somethinge goes horribly wrong, that way we guarantee seeing the last line of output.
  TCCR1A=0x00;
  TCCR1B=Backlight.TIMER_1_CLOCK_DIVIDER; //8x divider, 65ms rollover
  StartSyncInterrupt();
  interrupts();
  

  Serial.println(F("\nINIT/DONE\n\n")); 
  Serial.flush();
  FinishedConfigurationTime=millis();

 
}



//ISR(INT0_vect, ISR_NAKED)
ISR(INT0_vect){
    // Serial.print("&");
//    uint8_t cSREG = SREG; // Save SREG state     
    noInterrupts();
    // Avoid function call overhead by accessing the variables directly
    if(Backlight.BacklightMode == BACKLIGHT_MODE_PULSE) { TCNT2 = Backlight.STROBE_TRIGGER_POINT; }   // OSP_FIRE()
    uint16_t myTCNT1=TCNT1;
    if(myTCNT1> Backlight.LINE_TIMEOUT_TICKS) { 
        // This code runs at the beginning of a frame.
        Backlight.VerticalBlankingTimer1Ticks=myTCNT1;       
  //      uint32_t myMicros=micros();
  //      Backlight.MicrosecondsPerFrame=myMicros-Backlight.LastFrameMicros;
  //      Backlight.LastFrameMicros=myMicros;
    }
    TCNT1=0;    
    Backlight.RecentInputPulse=true;   
  
//    SREG=cSREG;
    EIFR=(1<<INTF0); //Just in case the pin toggled while we were inside this interrupt, clear the flag.  Note: interrupt1 is not used
    interrupts();
//    reti();
}




void StartSyncInterrupt(){
//  attachInterrupt(digitalPinToInterrupt(DATA_ENABLE), BacklightInterruptStub, RISING); // This interrupt can be triggered by vsync or de
 EICRA = (1<<ISC01) | (1<<ISC00); //rising edge triggers the interrupt
 EIFR  = (1<<INTF0); // Clear the interrupt before we enable it
 EIMSK = (1<<INT0); //interrupt request enabled  
}

void StopSyncInterrupt(){
 EIMSK =0x00; //interrupt request enabled
//  detachInterrupt(digitalPinToInterrupt(DATA_ENABLE)); // This interrupt can be triggered by vsync or de
}

void loop() {
  currentMillis = millisNoInterruptChanges();
  HandleMillisInTimer0Overflow();
  wdt_reset();
  TaskFastest();
  if ((currentMillis - Task1ms_previousMillis) >= OneMillisecond) {
    Task1ms();
    Task1ms_previousMillis = currentMillis;
  }

  if ((currentMillis - Task10ms_previousMillis) >= TenMilliseconds) {
    Task10ms();
    Task10ms_previousMillis = currentMillis;
  }

  if ((currentMillis - Task100ms_previousMillis) >= HundredMilliseconds) {
    Task100ms();
    Task100ms_previousMillis = currentMillis;
  }

  if ((currentMillis - Task1000ms_previousMillis) >= OneSecond) {
    Task1000ms();
    Task1000ms_previousMillis = currentMillis;
  }

  if ((currentMillis - Task10000ms_previousMillis) >= TenSeconds) {
    Task10000ms();
    Task10000ms_previousMillis = currentMillis;
  }
}

void TaskFastest() {
}


void Task1ms(){
}

void Task10ms(){  
  ProtectAgainstInputPowerFailure();
  Inputs.ReadPhysicalInputs();
  Inputs.RefilterInputState();      
  handle_button_state();
  HandleSystemState();
}

void Task100ms(){
    DetermineMasterSlave();
}

void Task1000ms(){
  if(CheckVideoActive()==false) {  Serial.print(F("\n VIDEO fail\n")); Serial.flush(); }
  if(I_AM_A_SLAVE==false) {
  Backlight.RefilterFrameParameters();
//  Backlight.RecalculateStrobingParameters();  
  }
  

  #ifdef WATCH_FOR_MODE_CHANGE    
    NewMeasuredRefreshRate=GetRefreshRate();
    // Note that the hysteretic filter in the backlight code shoudl reject noise well enough to rely on the refresh rate check function reliably
    if(  NewMeasuredRefreshRate!=MeasuredRefreshRate) {panel_script_refreshratechange();}
    MeasuredRefreshRate=NewMeasuredRefreshRate;
  #endif  
}

void Task10000ms(){
  PrintSystemState();
}

































const uint32_t SystemStateDelay_OffToRx = 10;
const uint32_t SystemStateDelay_RxToTx = 10;
const uint32_t SystemStateDelay_TxToPanel = 10;


uint8_t SystemState = SystemState_Init;
const uint32_t SystemStateHandlerCallRateMilliseconds = 10;
uint32_t SystemStateCounter = 0;
// Note : To keep timing of state transitions proper, call this function consistently.
void HandleSystemState(){
  // This function handles all 'forward' transitions.  Transitions to 'lower' states are handled elsewhere because these situations are caused by external events.
  if(SystemStateCounter <= SystemStateHandlerCallRateMilliseconds ) {
    SystemStateCounter = 0;
  } else {
    SystemStateCounter = SystemStateCounter - SystemStateHandlerCallRateMilliseconds;
  }

boolean WatchStateChanges = true;

    uint8_t myInputPowerFailure = InputPowerFailure();
    uint8_t myUserConfiguration_LoadShutdown = UserConfiguration_LoadShutdown();
    switch (SystemState) {
      case SystemState_Init:     
      if(WatchStateChanges) Serial.println(F("SysStateTransition : Init0"));
          power_down_board();
          power_down_receivers();
          power_down_transmitters();
          SystemStateCounter=0;
          SystemState=SystemState_PowerOff;
        break;
      case SystemState_PowerOff:     
        if( (myUserConfiguration_LoadShutdown!=TargetPowerSaveSHUTDOWN) && (myInputPowerFailure==false ) ){
      if(WatchStateChanges) Serial.println(F("SysStateTransition : Off0"));
          power_up_board();
          SystemStateCounter=SystemStateDelay_OffToRx;
          SystemState=SystemState_Rx;
        }
        break;
      case SystemState_Rx:
        if( (SystemStateCounter==0) && (myUserConfiguration_LoadShutdown==TargetPowerSaveFULLY_ON) && (myInputPowerFailure==false ) ){
      if(WatchStateChanges) Serial.println(F("SysStateTransition : Rx0"));
          power_up_receivers();    
          SystemStateCounter=SystemStateDelay_RxToTx;
          SystemState=SystemState_Tx;
          break;
        }
        if( (myUserConfiguration_LoadShutdown==TargetPowerSaveSHUTDOWN) || (myInputPowerFailure==true ) ){
      if(WatchStateChanges) Serial.println(F("SysStateTransition : Rx1"));
          power_down_board();
          SystemStateCounter=0;
          SystemState=SystemState_PowerOff;
          break;
        }    
        break;
      case SystemState_Tx:       
        if( (myUserConfiguration_LoadShutdown==TargetPowerSaveSHUTDOWN) || (myInputPowerFailure==true ) ){
      if(WatchStateChanges) Serial.println(F("SysStateTransition : Tx0"));
          power_down_receivers();
          power_down_board();
          SystemState=SystemState_PowerOff;
          break;      
        }
        if( myUserConfiguration_LoadShutdown==TargetPowerSaveLOWPOWER ){
      if(WatchStateChanges) Serial.println(F("SysStateTransition : Tx1"));
          power_down_receivers();
          SystemState=SystemState_Rx;
          break;      
        }
        if(SystemStateCounter==0) {
      if(WatchStateChanges) Serial.println(F("SysStateTransition : Tx2"));
          power_up_transmitters();
          SystemStateCounter=SystemStateDelay_TxToPanel;
          SystemState=SystemState_Panel;
          break;    
        }  
        break;
      case SystemState_Panel:            
        if( (myUserConfiguration_LoadShutdown==TargetPowerSaveSHUTDOWN) || (myInputPowerFailure==true ) ){
      if(WatchStateChanges) Serial.println(F("SysStateTransition : P0"));
          power_down_transmitters();
          power_down_receivers();
          power_down_board();
          SystemStateCounter=0;
          SystemState=SystemState_PowerOff;
          break;
        }
        if( myUserConfiguration_LoadShutdown==TargetPowerSaveLOWPOWER ){
      if(WatchStateChanges) Serial.println(F("SysStateTransition : P1"));
          power_down_transmitters();
          power_down_receivers();
          SystemStateCounter=SystemStateDelay_OffToRx;
          SystemState=SystemState_Rx;
          break;
        }
      
        if ( (SystemStateCounter==0) && (CheckVideoActive() == true ) ) {
      if(WatchStateChanges) Serial.println(F("SysStateTransition : P2"));
          power_up_display();
          SystemStateCounter=VIDEO_SIGNAL_TO_BACKLIGHT_ON_DELAY;
          SystemState=SystemState_Backlight;
          break;
        }  
      break;      
      case SystemState_Backlight: 
        if( (myUserConfiguration_LoadShutdown==TargetPowerSaveSHUTDOWN) || (myInputPowerFailure==true ) ){
      if(WatchStateChanges) Serial.println(F("SysStateTransition : B0"));
          power_down_display();
          power_down_transmitters();
          power_down_receivers();
          power_down_board();
          SystemStateCounter=0;
          SystemState=SystemState_PowerOff;
          break;
        }
        if( myUserConfiguration_LoadShutdown==TargetPowerSaveLOWPOWER ){
          if(WatchStateChanges) Serial.println(F("SysStateTransition : B1"));
          power_down_display();
          power_down_transmitters();
          power_down_receivers();
          SystemStateCounter=SystemStateDelay_OffToRx;
          SystemState=SystemState_Rx;
          break;
        }
        if ( CheckVideoActive() == false ) {
          if(WatchStateChanges) Serial.println(F("SysStateTransition : B2"));
          Backlight.SetMode(BACKLIGHT_MODE_OFF);
          SystemStateCounter=0;
          SystemState=SystemState_Panel;
          break;
        }          
        if ( SystemStateCounter==0 ) {
          if(WatchStateChanges) Serial.println(F("SysStateTransition : B3"));
          Backlight.SetMode(BACKLIGHT_MODE_STABLE);
          SystemStateCounter=0;
          SystemState=SystemState_On;
          break;    
        }  
        break;
      case SystemState_On: 
        if( (myUserConfiguration_LoadShutdown==TargetPowerSaveSHUTDOWN) || (myInputPowerFailure==true ) ){
          if(WatchStateChanges) Serial.println(F("SysStateTransition : ON0"));
          Backlight.SetMode(BACKLIGHT_MODE_OFF);
          power_down_display();
          power_down_transmitters();
          power_down_receivers();
          power_down_board();
          SystemStateCounter=0;
          SystemState=SystemState_PowerOff;
          break;
        }
        if( myUserConfiguration_LoadShutdown==TargetPowerSaveLOWPOWER ){
          if(WatchStateChanges) Serial.println(F("SysStateTransition : ON1"));
          Backlight.SetMode(BACKLIGHT_MODE_OFF);
          power_down_display();
          power_down_transmitters();
          power_down_receivers();
          SystemStateCounter=SystemStateDelay_OffToRx;
          SystemState=SystemState_Rx;
          break;
        }
        if ( CheckVideoActive() == false ) {
          if(WatchStateChanges) Serial.println(F("SysStateTransition : ON2"));
          Backlight.SetMode(BACKLIGHT_MODE_OFF);
          power_down_display();
          SystemStateCounter=0;
          SystemState=SystemState_Panel;
          break;
        }          
        break;
      default:                
        if(WatchStateChanges) Serial.print(F("SysStateTransition : DEFAULTED :")); Serial.println(SystemState );
        SystemState = SystemState_PowerOff;
        SystemStateCounter = 0;      
    }
 
}
























void PrintSystemState(){  
  
Serial.print(F("SystemState : ")); 
  switch (SystemState) {
  case SystemState_PowerOff:     Serial.println(F("OFF"));   break;
  case SystemState_Rx    :       Serial.println(F("RX"));    break;
  case SystemState_Tx    :       Serial.println(F("TX"));    break;
  case SystemState_Panel  :      Serial.println(F("Panel")); break;
  case SystemState_Backlight:    Serial.println(F("BL"));    break;
  case SystemState_On:           Serial.println(F("ON"));    break;
  default:                       Serial.print(F("???"));  Serial.println(SystemState);
  }
  
uint8_t myUserConfiguration_LoadShutdown=UserConfiguration_LoadShutdown();
Serial.print(F("TargetState : ")); 
  switch (UserConfiguration_LoadShutdown()) {
  case TargetPowerSaveSHUTDOWN:  Serial.println(F("OFF"));  break;
  case TargetPowerSaveLOWPOWER:  Serial.println(F("LOW"));  break;
  case TargetPowerSaveFULLY_ON:  Serial.println(F("ON"));   break;
  default:                       Serial.print(F("???"));   Serial.println(myUserConfiguration_LoadShutdown);
  }
  
  if(MASTERSLAVE_MODE==true) {
    if(I_AM_A_SLAVE == true){ Serial.println(F("Slave device")); } else { Serial.println(F("Master device")); }
  }
  if(Backlight.GetMode()!=BACKLIGHT_MODE_OFF){Backlight.PrintParameters();} 
}


void  DisablePullupSerialRX(){ pinMode(0, INPUT); }

void DetermineMasterSlave(){  
  if(MASTERSLAVE_MODE==true) {
    uint8_t myMasterSlave=determine_if_slave();
    if(I_AM_A_SLAVE != myMasterSlave){
      // We have changed roles, handle it appropriately
      I_AM_A_SLAVE = myMasterSlave;
      UserConfiguration_SaveWasSlave(I_AM_A_SLAVE);
      MasterSlaveChangedStates();
    }
  }
}

void MasterSlaveChangedStates(){  
  Serial.println(F("Master/Slave detection changed"));
    buffer_the_edid();
    PrintWhichEDIDSelected();
    set_selected_edid();
}


uint8_t determine_if_slave(){  
uint8_t retval=false;
  
  if(MASTERSLAVE_MODE==true){
    #ifdef PANEL_OSD_ENABLE
    pinMode ( PANEL_OSD_ENABLE, INPUT_PULLUP );    
    // The ZWS tcon uses a 1K pulldown on this signal.  This will easily overcome the on-chip pullup resistor and cause a 0 to be read.  The slave board will instead read a 1 because of the on-chip pullup.
    zdelay(10); // Let the line settle
    if(digitalRead(PANEL_OSD_ENABLE)==HIGH) { retval= true;}
    pinMode ( PANEL_OSD_ENABLE, INPUT );  
    #endif

    #ifdef PANEL_SERIAL_CONTROL
    pinMode ( PANEL_SERIAL_CONTROL, INPUT_PULLUP );    
    // The ZWS tcon uses a 1K pulldown on this signal.  This will easily overcome the on-chip pullup resistor and cause a 0 to be read.  The slave board will instead read a 1 because of the on-chip pullup.
    zdelay(10); // Let the line settle
    if(digitalRead(PANEL_SERIAL_CONTROL)==HIGH) { retval= true;}
    pinMode ( PANEL_SERIAL_CONTROL, INPUT );  
    #endif
    
  }  
  return retval;
}




uint16_t GetRefreshRate(){
  return Backlight.CalcRefreshRate();
}

uint8_t CheckVideoActive(){
  #ifdef ACTIVE_VIDEO
    return digitalRead(ACTIVE_VIDEO);
  #endif    
  return Backlight.CheckForActiveVideo();
}

void SetStaticPins(){
#ifdef ALWAYS_LOW
  pinMode(ALWAYS_LOW, OUTPUT);
  digitalWrite(ALWAYS_LOW, LOW);
#endif

#ifdef ALWAYS_HIGH
  pinMode(ALWAYS_HIGH, OUTPUT);
  digitalWrite(ALWAYS_HIGH, HIGH);
#endif


#ifdef PANEL_GPIO0
  pinMode(PANEL_GPIO0, INPUT);
#endif

#ifdef PANEL_GPIO1
  pinMode(PANEL_GPIO1, INPUT);
#endif
}

uint8_t InputPowerFailure(){
  uint8_t retval=false;
  #ifdef INPUT_VOLTAGE_MONITORING_PIN 
  pinMode(INPUT_VOLTAGE_MONITORING_PIN, INPUT);
  
const float INPUT_VOLTAGE_MONITORING_MULTIPLIER =  (ADC_RESOLUTION/MICROCONTROLLER_VOLTAGE)*PULLDOWN_RESISTOR/(PULLUP_RESISTOR+PULLDOWN_RESISTOR);
const uint16_t HIGH_CUTOFF = VIN_MAXIMUM*INPUT_VOLTAGE_MONITORING_MULTIPLIER;
const uint16_t LOW_CUTOFF = VIN_MINIMUM*INPUT_VOLTAGE_MONITORING_MULTIPLIER;
uint16_t voltage_input = analogRead(INPUT_VOLTAGE_MONITORING_PIN);

    if(voltage_input<=LOW_CUTOFF) {retval= true;}
    if(voltage_input>=HIGH_CUTOFF) {retval= true;}    
    if(retval==true) {
      Serial.print(F("Vin=")); Serial.print(1.0*voltage_input/INPUT_VOLTAGE_MONITORING_MULTIPLIER);
      Serial.print(F("  Vmax=")); Serial.print(1.0*HIGH_CUTOFF/INPUT_VOLTAGE_MONITORING_MULTIPLIER);
      Serial.print(F("  Vmin=")); Serial.println(1.0*LOW_CUTOFF/INPUT_VOLTAGE_MONITORING_MULTIPLIER);   
//      Serial.print(F("Vin=")); Serial.print(1.0*voltage_input/INPUT_VOLTAGE_MONITORING_MULTIPLIER);
//      Serial.print(F("  Vmax=")); Serial.print(VIN_MAXIMUM);
//      Serial.print(F("  Vmin=")); Serial.println(VIN_MINIMUM);
//      Serial.print(F("Vin=")); Serial.print(voltage_input);
//      Serial.print(F("  Vmax=")); Serial.print(HIGH_CUTOFF);
//      Serial.print(F("  Vmin=")); Serial.println(LOW_CUTOFF);   
    }
    
  #endif
  return retval;
  }

uint8_t PowerSupplyFailure(){  
  #ifdef PGOOD_VREG_V3P3 
    if(digitalRead(PGOOD_VREG_V3P3)==LOW){return true;}
  #endif
  
  #ifdef PGOOD_VREG_SECONDARY 
    if(digitalRead(PGOOD_VREG_SECONDARY)==LOW){return true;}
  #endif  
  
return InputPowerFailure();  
}


// Replacement for 'delay' which uses the CLOCK_RATIO parameter for overclocked mode and will also touch the watchdog periodically
void zdelay(uint32_t delayvalue){  
const uint8_t Real_DELAY_TIME = 10;
const uint8_t SCALED_DELAY_TIME=(Real_DELAY_TIME*1.0)*CLOCK_RATIO;
uint32_t remdel=delayvalue*CLOCK_RATIO;
while(remdel>SCALED_DELAY_TIME) {
  wdt_reset();
  delay(SCALED_DELAY_TIME);
  remdel=remdel-SCALED_DELAY_TIME; 
}
  wdt_reset();
  delay(remdel);
  wdt_reset();
}



 void debug_function(){
 #if SERIAL_COMMANDS_EXTENDED==DISABLED
    return;
 #endif
  const uint8_t buffersize = 32;
  char command_string[buffersize];
  uint8_t bufferindex=0;
  char* bufferpointer;
  char mychar;
  
  zdelay(100);
  
  while(Serial.available()) {
    mychar=Serial.read();
        if(bufferindex < buffersize-1) // One less than the size of the array
        {
            command_string[bufferindex] = mychar; // Read a character
            bufferindex++; // Increment where to write next
            command_string[bufferindex] = '\0'; // Null terminate the string
        }
        if (mychar!='\n') {zdelay(10);}
  }

  if (command_string[0] != '\0') {
    Serial.print("\nEcho:\"");
    Serial.print(command_string);
    Serial.println("\"");
  }

          uint8_t my_addr_dev=0x00;
          uint8_t my_addr_reg=0x00;
          uint8_t my_data_reg=0x00;


// HERE YOU CAN HANDLE THE COMMAND STRINGS.
  
// Full command for testing is: ".IIC_fix 0x7a 0x49 0xC0"

  if (strncmp(command_string,"id",6)==0) {
  my_SoftIIC_EPMI.MasterDumpAll();
    return;
  }
  
  if (strncmp(command_string,"ir",5)==0) {
    bufferpointer=command_string+5;
    my_addr_dev=strtoul(bufferpointer, &bufferpointer, 16);
    my_addr_reg=strtoul(bufferpointer, &bufferpointer, 16);
    
   my_SoftIIC_EPMI.MasterReadByte( my_addr_dev,  my_addr_reg,  &my_data_reg);
   Serial.print(F("Read: 0x")); fastprinthexbyte(my_addr_dev); Serial.print(F("/0x")); fastprinthexbyte(my_addr_reg);Serial.print(F(":0x")); fastprinthexbyte(my_data_reg);Serial.println("");
  return;
  }  
  
  if (strncmp(command_string,"br",2)==0) {
    bufferpointer=command_string+2;
    my_data_reg=strtoul(bufferpointer, &bufferpointer, 10);
    Backlight.SetBrightness(my_data_reg);
          return;
  }

    if (strncmp(command_string,"bf",2)==0) {
    bufferpointer=command_string+2;
    uint16_t pwmfreq=strtoul(bufferpointer, &bufferpointer, 10);
    Backlight.SetFrequencyPWM(pwmfreq);
          return;
  }

//  if (strncmp(command_string,"pb",8)==0) {  Backlight.PrintParameters();          return;  }

  if (strncmp(command_string,"Factory",7)==0) {
  do_factory_configuration();
  return;
 }
 
  if (strncmp(command_string,"eb",2)==0) {
dump_buffered_edid(); 
  return;
 }
 Serial.println(F("???"));
}




void get_and_handle_buttons_nowait(){  
      Inputs.ReadPhysicalInputs();
      Inputs.RefilterInputState();      
      handle_button_state();
}


void get_and_handle_buttons(){  
      delay(CLOCK_RATIO*BUTTON_SENSE_TIME);  // specifically do not touch watchdog here.  avoid zdelay
      get_and_handle_buttons_nowait();
}





void configure_watchdog_timer(){
wdt_enable(MY_WATCHDOG_TIMEOUT); 
  wdt_reset();
}


void configure_watchdog_timer_slow(){
wdt_enable(WDTO_4S); 
  wdt_reset();
}

void ProtectAgainstInputPowerFailure(){
  if(InputPowerFailure()==true) {
    Serial.println(F("Input power fail!"));
    Serial.flush();
    softReset();
  }  
}



void power_up_display() {
  ProtectAgainstInputPowerFailure();
  Serial.println(F("\tD+"));
#ifdef CONTROL_VREG_VPANEL
pinMode(CONTROL_VREG_VPANEL,OUTPUT);  
if(CONTROL_VREG_VPANEL_POLARITY==ACTIVE_HIGH){  digitalWrite(CONTROL_VREG_VPANEL, HIGH);} else {  digitalWrite(CONTROL_VREG_VPANEL, LOW);}
zdelay(ADDED_DELAY_AFTER_PANEL_POWERUP);
#endif
//wdt_disable();
panel_script_startup();
zdelay(VIDEO_SIGNAL_TO_BACKLIGHT_ON_DELAY);
Serial.println(F("\tBL+"));
  Backlight.SetMode(BACKLIGHT_MODE_STABLE);
PanelWasOnline=true;
}


void power_down_display() {
Serial.println(F("\tBL-"));
  Backlight.SetMode(BACKLIGHT_MODE_OFF);
  Serial.println(F("\tD-")); 
  
panel_script_shutdown();

#ifdef CONTROL_VREG_VPANEL
zdelay(DELAY_BETWEEN_BACKLIGHT_POWEROFF_AND_PANEL_POWEROFF);
pinMode(CONTROL_VREG_VPANEL,OUTPUT);  
if(CONTROL_VREG_VPANEL_POLARITY==ACTIVE_HIGH){  digitalWrite(CONTROL_VREG_VPANEL, LOW);} else {  digitalWrite(CONTROL_VREG_VPANEL, HIGH);}
#endif  
PanelWasOnline=false;
}

void power_down_transmitters() {

#ifdef LOW_POWER_MODE
pinMode(LOW_POWER_MODE,OUTPUT);  
digitalWrite(LOW_POWER_MODE, HIGH);
#endif  
}

void power_up_transmitters() {
  // Set static pin configuration pins

#ifdef CHIP_IS_EP269 
#ifdef RESET_OTHER_CHIPS
  pinMode(RESET_OTHER_CHIPS,OUTPUT); digitalWrite(RESET_OTHER_CHIPS, LOW);
#endif
#endif

#ifdef CHIP_IS_EP269 
  set_static_config_pins(ConfigGenerateEPMI());
#endif

#ifdef CHIP_IS_EP369
  set_static_config_pins(ConfigGenerateEPMI());
#endif


#ifdef LOW_POWER_MODE
pinMode(LOW_POWER_MODE,OUTPUT);  
digitalWrite(LOW_POWER_MODE, LOW);
#endif  


}

void power_up_receivers() {

#ifdef RESET_OTHER_CHIPS
 pinMode(RESET_OTHER_CHIPS,OUTPUT); digitalWrite(RESET_OTHER_CHIPS, HIGH);
#endif

}


void power_down_receivers() {
#ifdef CHIP_IS_EP269 
#ifdef RESET_OTHER_CHIPS
  pinMode(RESET_OTHER_CHIPS,OUTPUT); digitalWrite(RESET_OTHER_CHIPS, LOW);  
#endif
#endif

#ifdef CHIP_IS_EP269 
  set_static_config_pins(0x00);
#endif

#ifdef CHIP_IS_EP369
  set_static_config_pins(0x00);
#endif
}

void power_up_board() {
const uint32_t MILLISECONDS_BETWEEN_V3P3_AND_SECONDARY_POWERUP = 10;
// I don't think that the pgood signals are valid if the input supply is missing, so check that first.
ProtectAgainstInputPowerFailure();
    
  // All board variations are powered by input->3.3v followed by 3.3v-> 1.3/1.8v  
    
#ifdef CONTROL_PSON
pinMode(CONTROL_PSON,OUTPUT);  
if(CONTROL_PSON_POLARITY==ACTIVE_HIGH){  digitalWrite(CONTROL_PSON, HIGH);} else {  digitalWrite(CONTROL_PSON, LOW);}
zdelay(10);
#endif
  
#ifdef CONTROL_VREG_V3P3
pinMode(CONTROL_VREG_V3P3,OUTPUT);  
if(CONTROL_VREG_V3P3_POLARITY==ACTIVE_HIGH){  digitalWrite(CONTROL_VREG_V3P3, HIGH);} else {  digitalWrite(CONTROL_VREG_V3P3, LOW);}
#endif
    
  #ifdef CONTROL_VREG_SECONDARY
    zdelay(MILLISECONDS_BETWEEN_V3P3_AND_SECONDARY_POWERUP);    
  #ifdef PGOOD_VREG_V3P3   
    while(digitalRead(PGOOD_VREG_V3P3)==LOW){} // waits until dc/dc is on
  #endif
  
pinMode(CONTROL_VREG_SECONDARY,OUTPUT);  
if(CONTROL_VREG_SECONDARY_POLARITY==ACTIVE_HIGH){  digitalWrite(CONTROL_VREG_SECONDARY, HIGH);} else {  digitalWrite(CONTROL_VREG_SECONDARY, LOW);}
    #ifdef PGOOD_VREG_SECONDARY 
while(digitalRead(PGOOD_VREG_SECONDARY)==LOW){} // waits until dc/dc is on
  #endif 
#endif

  Serial.println(F("\tB+"));
// Note that there is an observed ~500ms delay between ep369 power-up and the first i2c transaction to get the EDID.  This could change in future ep369 firmwares

BoardWasOnline=true;  
}


void power_down_board() {
  Serial.println(F("\tB-"));  
  #ifdef RESET_OTHER_CHIPS
    pinMode(RESET_OTHER_CHIPS, OUTPUT);
    digitalWrite(RESET_OTHER_CHIPS, LOW);
  #endif
  set_static_config_pins(0x00);
    
  #ifdef CONTROL_VREG_SECONDARY
pinMode(CONTROL_VREG_SECONDARY,OUTPUT);  
if(CONTROL_VREG_SECONDARY_POLARITY==ACTIVE_HIGH){  digitalWrite(CONTROL_VREG_SECONDARY, LOW);} else {  digitalWrite(CONTROL_VREG_SECONDARY, HIGH);}
    #ifdef PGOOD_VREG_1P8 
while(digitalRead(PGOOD_VREG_SECONDARY)==HIGH){} // waits until dc/dc is off
  #endif
  zdelay(10);   
#endif
    

#ifdef CONTROL_VREG_V3P3
pinMode(CONTROL_VREG_V3P3,OUTPUT);  
if(CONTROL_VREG_V3P3_POLARITY==ACTIVE_HIGH){  digitalWrite(CONTROL_VREG_V3P3, LOW);} else {  digitalWrite(CONTROL_VREG_V3P3, HIGH);}
    #ifdef PGOOD_VREG_3P3 
while(digitalRead(PGOOD_VREG_3P3)==HIGH){} // waits until dc/dc is off
  #endif 
#endif

#ifdef CONTROL_PSON
pinMode(CONTROL_PSON,OUTPUT);  
if(CONTROL_PSON_POLARITY==ACTIVE_HIGH){  digitalWrite(CONTROL_PSON, LOW);} else {  digitalWrite(CONTROL_PSON, HIGH);}
#endif
BoardWasOnline=false;

}

void set_static_config_pins(uint8_t myconfig) {
  // Set static pin configuration pins
  
#ifdef CHIP_IS_EP369
#ifdef CONFIG_EPMI_DW0
pinMode(CONFIG_EPMI_DW0,OUTPUT); if((myconfig & CONFIGMASK_EPMI_DW0)>0){digitalWrite(CONFIG_EPMI_DW0, HIGH);} else {digitalWrite(CONFIG_EPMI_DW0, LOW);}
#endif
#ifdef CONFIG_EPMI_DW1
pinMode(CONFIG_EPMI_DW1,OUTPUT);  if((myconfig & CONFIGMASK_EPMI_DW1)>0){digitalWrite(CONFIG_EPMI_DW1, HIGH);} else {digitalWrite(CONFIG_EPMI_DW1, LOW);}
#endif
#ifdef CONFIG_EPMI_MAP
pinMode(CONFIG_EPMI_MAP,OUTPUT); if((myconfig & CONFIGMASK_EPMI_MAP)>0){digitalWrite(CONFIG_EPMI_MAP, HIGH);} else {digitalWrite(CONFIG_EPMI_MAP, LOW);}
#endif
#ifdef CONFIG_EPMI_EO
pinMode(CONFIG_EPMI_EO,OUTPUT);  if((myconfig & CONFIGMASK_EPMI_EO)>0){digitalWrite(CONFIG_EPMI_EO, HIGH);} else {digitalWrite(CONFIG_EPMI_EO, LOW);}
#endif
#ifdef CONFIG_EPMI_LR
pinMode(CONFIG_EPMI_LR,OUTPUT);  if((myconfig & CONFIGMASK_EPMI_LR)>0){digitalWrite(CONFIG_EPMI_LR, HIGH);} else {digitalWrite(CONFIG_EPMI_LR, LOW);}
#endif
#ifdef CONFIG_EPMI_TMODE 
pinMode(CONFIG_EPMI_TMODE,OUTPUT);  if((myconfig & CONFIGMASK_EPMI_TMODE)>0){digitalWrite(CONFIG_EPMI_TMODE, HIGH);} else {digitalWrite(CONFIG_EPMI_TMODE, LOW);}
#endif
#ifdef CONFIG_EPMI_DMODE 
pinMode(CONFIG_EPMI_DMODE,OUTPUT);  if((myconfig & CONFIGMASK_EPMI_DMODE)>0){digitalWrite(CONFIG_EPMI_DMODE, HIGH);} else {digitalWrite(CONFIG_EPMI_DMODE, LOW);}
#endif
#ifdef CONFIG_EPMI_RS 
pinMode(CONFIG_EPMI_RS,OUTPUT);  if((myconfig & CONFIGMASK_EPMI_RS)>0){digitalWrite(CONFIG_EPMI_RS, HIGH);} else {digitalWrite(CONFIG_EPMI_RS, LOW);}
#endif
#endif


#ifdef CHIP_IS_EP269 
#ifdef CONFIG_EPMI_EO 
pinMode(CONFIG_EPMI_EO,OUTPUT);  if((myconfig & CONFIGMASK_EPMI_EO)>0){digitalWrite(CONFIG_EPMI_EO, HIGH);} else {digitalWrite(CONFIG_EPMI_EO, LOW);}
#endif
#ifdef CONFIG_EPMI_LR 
pinMode(CONFIG_EPMI_LR,OUTPUT);  if((myconfig & CONFIGMASK_EPMI_LR)>0){digitalWrite(CONFIG_EPMI_LR, HIGH);} else {digitalWrite(CONFIG_EPMI_LR, LOW);}
#endif
#ifdef CONFIG_EPMI_TMODE 
pinMode(CONFIG_EPMI_TMODE,OUTPUT);  if((myconfig & CONFIGMASK_EPMI_TMODE)>0){digitalWrite(CONFIG_EPMI_TMODE, HIGH);} else {digitalWrite(CONFIG_EPMI_TMODE, LOW);}
#endif
#ifdef CONFIG_EPMI_DMODE 
pinMode(CONFIG_EPMI_DMODE,OUTPUT);  if((myconfig & CONFIGMASK_EPMI_DMODE)>0){digitalWrite(CONFIG_EPMI_DMODE, HIGH);} else {digitalWrite(CONFIG_EPMI_DMODE, LOW);}
#endif
#ifdef CONFIG_EPMI_RS 
pinMode(CONFIG_EPMI_RS,OUTPUT);  if((myconfig & CONFIGMASK_EPMI_RS)>0){digitalWrite(CONFIG_EPMI_RS, HIGH);} else {digitalWrite(CONFIG_EPMI_RS, LOW);}
#endif
#endif
}


uint8_t DetermineIfSlaveStatusChanged(){  return (I_AM_A_SLAVE != UserConfiguration_LoadWasSlave()); }

void do_factory_configuration() {
  Serial.println(F("Fac"));  
  power_down_display();
  StopSyncInterrupt();

 // power_up_board();   while(1){;} // This line is useful when trying to prgram the internal ep369s mcu sometimes
  
  power_down_board();   
  Serial.flush();

  // Program the internal eeprom with safe default values
  UserConfiguration_SaveDefaultShutdown();
  UserConfiguration_SaveDefaultEDID();
  UserConfiguration_SaveDefaultBacklightMode();
  UserConfiguration_SaveDefaultBrightnessStable();
  UserConfiguration_SaveDefaultBrightnessPulse();
  UserConfiguration_SaveDefaultFrequencyPWM();
  UserConfiguration_SaveDefaultWasSlave();
  UserConfiguration_SaveDefaultFrequencyPWM();  
  UserConfiguration_SaveDefaultOSD();
  UserConfiguration_SaveDefaultTestMode();
  buffer_the_edid(FACTORY_DEFAULT_SELECTED_EDID);
  
#ifdef EXTERNAL_WRITEABLE_EDID
  write_external_edid();
#endif
 
#ifdef EXTERNAL_IIC_CONFIGURATION_EEPROM
 write_config_eeprom(); 
#endif

  UserConfiguration_SaveDefaultMagicByte();
  softReset();
}


void softReset() {  
  Serial.println(F("\nRst!"));  Serial.flush();
  noInterrupts();
  power_down_display();
//  power_down_board();
  zdelay(500); // Ensure that the board is powered down sufficiently long to cause a full reset.  
#if OVERCLOCKING==ENABLED
  my_Overclock_object.SetClockSpeedStock();
#endif
  Serial.end();  
wdt_enable(WDTO_15MS); 
 while(1){}
//  asm volatile ("  jmp 0"); // Note: the jmp 0 does not load the bootloader, so we might not get the startup state that we expect.  Using watchdog is cleaner.
}


struct ParsedSerialCommand {
bool Valid;
bool OSD;
uint8_t EDID;
uint8_t PowerSave;
};

void HandleSerialCommandZWS(uint8_t myCommand){
  struct ParsedSerialCommand myParsedSerialCommand=SerialCommandParser(myCommand);
  PrintParsedSerialCommand(myCommand);
  if(myParsedSerialCommand.Valid==false ) {return;}
  if((myParsedSerialCommand.OSD==false) && (UserConfiguration_LoadOSD()!=0)) {OSDtoggleOFF();}  
  if((myParsedSerialCommand.OSD==true) && (UserConfiguration_LoadOSD()==0)) {OSDtoggleON();}  
  if(myParsedSerialCommand.EDID!=UserConfiguration_LoadEDID()) { set_selected_edid(myParsedSerialCommand.EDID);}
  // Don't support off/low/on powerstates yet.  just power on
   if(myParsedSerialCommand.PowerSave==TargetPowerSaveFULLY_ON) {set_on_power_state(); }
   else {set_off_power_state(); }
}

void handle_button_state() {  
//
// if(I_AM_A_SLAVE==true) {
//    if (Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_SLAVE_ON  ) { slave_online(); return; }
//    if (Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_SLAVE_OFF ) { slave_offline(); return; }
//    return;
// }
        
// This optimization breaks on-release effects like saving the brightness and powering on/off the board, SO DON'T USE IT.
//  if (Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_UNDEFINED || Inputs.GetCurrentFilteredInput() == BUTTONSTATE_NOTHING)               {                                          return; }  // exit early if there is nothing to do
  
    uint8_t  mypowerstate=UserConfiguration_LoadShutdown();
      // Handle 'soft' button presses via serial port      
  #if SERIAL_COMMANDS_SIMPLE==ENABLED
  if(Inputs.GetPreviousFilteredInput() == COMMAND_CODE_FOR_SERIAL){
    if(Inputs.GetCurrentFilteredInput()>0x7F) { HandleSerialCommandZWS(Inputs.GetCurrentFilteredInput()); return;}    
    if(Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_UNDEFINED                                               ) {                                                                 return; }      
    if(Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_SPECIAL_COMMANDS                                        ) { debug_function();                                               return; }   
    if(Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_POWER_BUTTON                                            ) { toggle_power_state();                                           return; }  
    if(mypowerstate==TargetPowerSaveFULLY_ON && Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_BRIGHTNESS_INCREASE   ) { Backlight.IncrementBrightness(); Backlight.SetBrightness();     return; }  
    if(mypowerstate==TargetPowerSaveFULLY_ON && Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_BRIGHTNESS_DECREASE   ) { Backlight.DecrementBrightness(); Backlight.SetBrightness();     return; }  
    if(Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_FACTORY_PROGRAM                                         ) { do_factory_configuration();                                     return; }  
    if(mypowerstate==TargetPowerSaveFULLY_ON && Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_PWM_FREQ_DECREASE     ) { Backlight.DecrementFrequencyPWM();                              return; }  
    if(mypowerstate==TargetPowerSaveFULLY_ON && Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_PWM_FREQ_INCREASE     ) { Backlight.IncrementFrequencyPWM();                              return; } 
    if(mypowerstate==TargetPowerSaveFULLY_ON && Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_STROBE_ROTATE         ) { Backlight.SwapMode();                                           return; }   
    if(mypowerstate==TargetPowerSaveFULLY_ON && Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_PANEL_OSD             ) { OSDtoggle();                                                    return; }
    if(Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_EDID_ROTATE                                             ) { rotate_selected_edid();                                         return; }  
    if(Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_EDID_0                                                  ) { if(0==UserConfiguration_LoadEDID()) {return;} set_selected_edid(0); return; }  
    if(Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_EDID_1                                                  ) { if(1==UserConfiguration_LoadEDID()) {return;} set_selected_edid(1); return; }  
    if(Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_EDID_2                                                  ) { if(2==UserConfiguration_LoadEDID()) {return;} set_selected_edid(2); return; }  
    if(Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_EDID_3                                                  ) { if(3==UserConfiguration_LoadEDID()) {return;} set_selected_edid(3); return; }  
    if(Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_EDID_4                                                  ) { if(4==UserConfiguration_LoadEDID()) {return;} set_selected_edid(4); return; }  
    if(Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_EDID_5                                                  ) { if(5==UserConfiguration_LoadEDID()) {return;} set_selected_edid(5); return; }  
    if(Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_EDID_6                                                  ) { if(6==UserConfiguration_LoadEDID()) {return;} set_selected_edid(6); return; }  
    if(Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_EDID_7                                                  ) { if(7==UserConfiguration_LoadEDID()) {return;} set_selected_edid(7); return; }  
    if(Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_OCTESTMODE_ON                                           ) { if(UserConfiguration_LoadTestMode()!=1) {UserConfiguration_SaveTestMode(1);}                           return; }  
    if(Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_OCTESTMODE_OFF                                          ) { if(UserConfiguration_LoadTestMode()!=0) {UserConfiguration_SaveTestMode(0);}                           return; }  
    if(Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_OSD_ON                                                  ) { if(UserConfiguration_LoadOSD()==0) {OSDtoggleON();}                               return; }  
    if(Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_OSD_OFF                                                 ) { if(UserConfiguration_LoadOSD()!=0) {OSDtoggleOFF();}                              return; }  
    if(Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_POWER_ON                                                ) { if(mypowerstate!=TargetPowerSaveFULLY_ON) {set_on_power_state(); }     return; }  
    if(Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_POWER_OFF                                               ) { if(mypowerstate==TargetPowerSaveFULLY_ON) {set_off_power_state();}     return; }  
  return;
  }
  #endif

      // Handle regular button presses via button board   
  if (((Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_NOTHING) ||(Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_UNDEFINED)) && Inputs.GetPreviousFilteredInput() == COMMAND_CODE_FOR_POWER_BUTTON)  { toggle_power_state();               return; } // Note: on button release to avoid cycling. 
  if (mypowerstate==TargetPowerSaveLOWPOWER && Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_FACTORY_PROGRAM && Inputs.GetPreviousFilteredInput() != COMMAND_CODE_FOR_FACTORY_PROGRAM)                  { do_factory_configuration();         return; }
  if (mypowerstate==TargetPowerSaveLOWPOWER && Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_CONDITIONAL_ROTATE && Inputs.GetPreviousFilteredInput() != COMMAND_CODE_FOR_CONDITIONAL_ROTATE)            { rotate_selected_edid();             return; } 
  if (mypowerstate==TargetPowerSaveFULLY_ON && Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_CONDITIONAL_ROTATE && Inputs.GetPreviousFilteredInput() != COMMAND_CODE_FOR_CONDITIONAL_ROTATE)            { Backlight.SwapMode();               return; } 
  if (mypowerstate==TargetPowerSaveFULLY_ON && Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_EDID_ROTATE && Inputs.GetPreviousFilteredInput() != COMMAND_CODE_FOR_EDID_ROTATE)                          { rotate_selected_edid();             return; }
  if (mypowerstate==TargetPowerSaveFULLY_ON && Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_PWM_FREQ_INCREASE && Inputs.GetPreviousFilteredInput() != COMMAND_CODE_FOR_PWM_FREQ_INCREASE)              { Backlight.IncrementFrequencyPWM();  return; }
  if (mypowerstate==TargetPowerSaveFULLY_ON && Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_PWM_FREQ_DECREASE && Inputs.GetPreviousFilteredInput() != COMMAND_CODE_FOR_PWM_FREQ_DECREASE)              { Backlight.DecrementFrequencyPWM();  return; }
  if (mypowerstate==TargetPowerSaveFULLY_ON && Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_BRIGHTNESS_INCREASE )                                                                                      { Backlight.IncrementBrightness();    return; }
  if (mypowerstate==TargetPowerSaveFULLY_ON && Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_BRIGHTNESS_DECREASE )                                                                                      { Backlight.DecrementBrightness();    return; }
  if (mypowerstate==TargetPowerSaveFULLY_ON && Inputs.GetPreviousFilteredInput() == COMMAND_CODE_FOR_BRIGHTNESS_INCREASE && Inputs.GetCurrentFilteredInput() != COMMAND_CODE_FOR_BRIGHTNESS_INCREASE)          { Backlight.SetBrightness();          return; }
  if (mypowerstate==TargetPowerSaveFULLY_ON && Inputs.GetPreviousFilteredInput() == COMMAND_CODE_FOR_BRIGHTNESS_DECREASE && Inputs.GetCurrentFilteredInput() != COMMAND_CODE_FOR_BRIGHTNESS_DECREASE)          { Backlight.SetBrightness();          return; }
  if (mypowerstate==TargetPowerSaveFULLY_ON && Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_PANEL_OSD && Inputs.GetPreviousFilteredInput() != COMMAND_CODE_FOR_PANEL_OSD)                              { OSDtoggle();                        return; }
  if (Inputs.GetPreviousFilteredInput() != COMMAND_CODE_FOR_EDID_0 && Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_EDID_0)                                                                      { set_selected_edid(0);               return; }
  if (Inputs.GetPreviousFilteredInput() != COMMAND_CODE_FOR_EDID_1 && Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_EDID_1)                                                                      { set_selected_edid(1);               return; }
  if (Inputs.GetPreviousFilteredInput() != COMMAND_CODE_FOR_EDID_2 && Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_EDID_2)                                                                      { set_selected_edid(2);               return; }
  if (Inputs.GetPreviousFilteredInput() != COMMAND_CODE_FOR_EDID_3 && Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_EDID_3)                                                                      { set_selected_edid(3);               return; }
  if (Inputs.GetPreviousFilteredInput() != COMMAND_CODE_FOR_EDID_4 && Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_EDID_4)                                                                      { set_selected_edid(4);               return; }
  if (Inputs.GetPreviousFilteredInput() != COMMAND_CODE_FOR_EDID_5 && Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_EDID_5)                                                                      { set_selected_edid(5);               return; }
  if (Inputs.GetPreviousFilteredInput() != COMMAND_CODE_FOR_EDID_6 && Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_EDID_6)                                                                      { set_selected_edid(6);               return; }
  if (Inputs.GetPreviousFilteredInput() != COMMAND_CODE_FOR_EDID_7 && Inputs.GetCurrentFilteredInput() == COMMAND_CODE_FOR_EDID_7)                                                                      { set_selected_edid(7);               return; }
}

void toggle_power_state() {
//  Serial.println(F("PSTATE TOGGLE")); 
  if (UserConfiguration_LoadShutdown() != TargetPowerSaveFULLY_ON) {
    set_on_power_state();  
  }
  else {
    set_off_power_state();
  }
}

void set_off_power_state(){
  return set_low_power_state();
  // Do not support ultra-low-power state right now, because it screws with DP autodetection.  
}

void set_low_power_state(){
    UserConfiguration_SaveShutdown(TargetPowerSaveLOWPOWER);
}

void set_on_power_state(){
    UserConfiguration_SaveShutdown(TargetPowerSaveFULLY_ON);
}



void OSDtoggle(){
  if(UserConfiguration_LoadOSD()!=0) {OSDtoggleOFF(); } else{ OSDtoggleON();}
}

void OSDtoggleON(){
  UserConfiguration_SaveOSD(1);
  OSDoutput();
}

void OSDtoggleOFF(){
  UserConfiguration_SaveOSD(0);
  OSDoutput();
}
void OSDoutput(){
  #ifdef PANEL_OSD_ENABLE
    pinMode(PANEL_OSD_ENABLE, OUTPUT);
    if(UserConfiguration_LoadOSD()!=0){
      digitalWrite(PANEL_OSD_ENABLE, HIGH);
    } else {
      digitalWrite(PANEL_OSD_ENABLE, LOW);
    }  
  #endif
}

void write_config_eeprom(){  
//  Serial.println(F("\nConfig eeprom update is skipped.")); return; // This line is skips the 24c08 firmware update.  Used on the bugged port B7 board.
    #ifdef CHIP_IS_EP269  
        update_eeprom(&my_SoftIIC_EPMI, 0x50, virtualeeprom_ep269_config);
        update_eeprom(&my_SoftIIC_EPMI, 0x51, virtualeeprom_ep269_config);
        update_eeprom(&my_SoftIIC_EPMI, 0x52, virtualeeprom_ep269_config);
        update_eeprom(&my_SoftIIC_EPMI, 0x53, virtualeeprom_ep269_config);
//  my_SoftIIC_EPMI.MasterDumpAll();  Serial.println("\ndone?"); while(1) {;}
    #endif
    #ifdef CHIP_IS_EP369  
//        Serial.print(F("WrExt:")); PrintWhichEDIDSelected(); dump_buffered_edid();
        update_eeprom(&my_SoftIIC_EPMI, 0x50, virtualeeprom_ep369_config);
    #endif        
//  my_SoftIIC_EPMI.MasterDumpAll();
}


void write_external_all_zero_config(){
    #ifdef EXTERNAL_IIC_CONFIGURATION_EEPROM
        update_eeprom(&my_SoftIIC_EPMI, 0x50, virtualeeprom_all_zero_edid);
    #endif
}


void write_external_edid(){
    #ifdef EXTERNAL_WRITEABLE_EDID
//        Serial.print(F("WrExt:")); PrintWhichEDIDSelected(); dump_buffered_edid();
    #ifdef CHIP_IS_EP369  
//        Serial.print(F("WrExt:")); PrintWhichEDIDSelected(); dump_buffered_edid();
        update_eeprom(&my_SoftIIC_EDID, 0x50, virtualeeprom_ep369_config);
        return;
    #endif        
        update_eeprom(&my_SoftIIC_EDID, EDID_IIC_ADDRESS, virtualeeprom_edid);
    #endif
}


 void fastprinthexbyte(uint8_t hexbyte){   
    uint8_t mychar=hexbyte>>4;
    if(mychar<0x0A) { Serial.write('0' + mychar);} else  { Serial.write('7' + mychar);}
    mychar=hexbyte&0x0F;
    if(mychar<0x0A) { Serial.write('0' + mychar);} else  { Serial.write('7' + mychar);}
}
  

uint8_t update_eeprom(SoftIIC* my_SoftIIC, uint8_t eeprom_address, uint8_t (*fp_virtualeeprom)(uint8_t chip_address, uint8_t reg_address )){
    // Try page mode twice, then use byte mode as a fallback method
    uint8_t retval=0;
    retval=update_eeprom_page_mode( my_SoftIIC, eeprom_address,fp_virtualeeprom); if(retval==0) {goto end_of_update_eeprom_function;}
    retval=update_eeprom_page_mode( my_SoftIIC, eeprom_address,fp_virtualeeprom); if(retval==0) {goto end_of_update_eeprom_function;}
    retval=update_eeprom_byte_mode( my_SoftIIC, eeprom_address,fp_virtualeeprom);

    end_of_update_eeprom_function:
//    my_SoftIIC->MasterDumpRegisters(eeprom_address); // This should normally be commented out    
    return retval;
}


const uint8_t EEPROM_WRITE_TIME = 5+1;   // Time between i2c eeprom writes, needed for 24c02/24c08 to complete internal operations
uint8_t update_eeprom_page_mode(SoftIIC* my_SoftIIC, uint8_t eeprom_address, uint8_t (*fp_virtualeeprom)(uint8_t chip_address, uint8_t reg_address )){
    wdt_reset();  
    Serial.print(F("Page8PROG: IIC 0x")); fastprinthexbyte(eeprom_address); Serial.print(F("\t")); Serial.flush();
    const uint8_t pagesize=8;
    uint8_t bytearray[pagesize];
    uint8_t tretval=0xff;
    uint16_t failures = 0;
    uint16_t current_address = 0;
    current_address = 0;
    while(current_address <= 0xff ){            
        Serial.flush();
        for(uint16_t addr=0; addr<pagesize; addr++){bytearray[addr]=fp_virtualeeprom(eeprom_address, current_address+addr);}          
        tretval = my_SoftIIC->MasterWritePage(eeprom_address, current_address, pagesize, 1, bytearray ); 
        if ( tretval != 0) {        failures++;   Serial.print("*");   } else {Serial.print(".");}
        // Serial.print(" 0x"); fastprinthexbyte(fp_virtualeeprom(eeprom_address, current_address));
        current_address=current_address+pagesize;
        zdelay(EEPROM_WRITE_TIME);  // Time between writes
    }
    Serial.print(F(" IIC writes done, "));
    Serial.print(failures);
    Serial.println(F(" errors."));
    return failures;
}
  

uint8_t update_eeprom_byte_mode(SoftIIC* my_SoftIIC, uint8_t eeprom_address, uint8_t (*fp_virtualeeprom)(uint8_t chip_address, uint8_t reg_address )){
    wdt_reset();
    Serial.print(F("BytePROG: IIC 0x")); fastprinthexbyte(eeprom_address); Serial.print(F("\t")); Serial.flush();
    uint8_t tretval=0xff;
    uint16_t failures = 1;
    uint16_t current_address = 0;
    while (failures > 0) {
        failures = 0;
        for (current_address = 0; current_address <= 0xff; current_address++ ) {
            Serial.flush();
            tretval=my_SoftIIC->MasterWriteByte( eeprom_address,  current_address,  fp_virtualeeprom(eeprom_address, current_address), 1 );
            if ( tretval != 0) {        failures++;   Serial.print("*");   } else {Serial.print(".");}
            // Serial.print(" 0x"); fastprinthexbyte(fp_virtualeeprom(eeprom_address, current_address));      
            zdelay(EEPROM_WRITE_TIME);  // Time between writes
        }
        Serial.print(F(" IIC writes done, "));
        Serial.print(failures);
        Serial.println(F(" errors."));
    }
    return failures;
}
  
