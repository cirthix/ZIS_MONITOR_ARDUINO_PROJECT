/* Major assumptions:
We completely control timer1(strobing/interrupt) and timer2(pwm).
Some backlight control pins MUST exist

If strobing is to be supported, the de Pulse pin must exist.
const uint8_t DATA_ENABLE = 2;       // Input pin interrupt 0
const uint8_t BACKLIGHT_PWM = 3; // OC2B_PIN (PIN3)

Other pins may optionally exist:
const uint8_t BACKLIGHT_ENABLE = 6;
const uint8_t SET_HIGH_CURRENT_MODE = 6;


*/

#ifndef BACKLIGHT_h
#define BACKLIGHT_h
#include "constants.h"
#include <EEPROM.h>                          // Included in arduino environment
#include <avr/wdt.h>                         //  Included in arduino environment
#include "arduino2.h"  // include the fast I/O 2 functions.  Info here: http://www.codeproject.com/Articles/732646/Fast-digital-I-O-for-Arduino

#define FORCE_INLINE __attribute__((always_inline)) inline

class Backlight{
public:
  Backlight();
  uint8_t GetMode();
  void SetMode( uint8_t mode);
  void SetModeInitial();
  void SwapMode();
  uint8_t GetBrightness();
  uint8_t SetBrightness();
  uint8_t SetBrightness( uint8_t brightness);
  void IncrementBrightness();
  void DecrementBrightness();
  uint16_t GetFrequencyPWM();
  void SetFrequencyPWM();
  void SetFrequencyPWM(uint16_t frequency);
  void IncrementFrequencyPWM();
  void DecrementFrequencyPWM();
  uint16_t CalcRefreshRate();
  uint16_t CalcBlankingTime();
  void RefilterFrameParameters();
  void RecalculateStrobingParameters(); 
  void PrintParameters();
 void Interrupt();
 uint8_t CheckForActiveVideo();
 void ClearRecentPinInterrupt();
 void SetRecentPinInterrupt();


static const uint8_t TIMER_1_CLOCK_DIVIDER = 0x02;
static const uint16_t TIMER1_CLOCK_DIVIDER_EQ=8;

// These variables arent really intended to be public, but to avoid overhead, the interrupt should avoid function calls that woudl otherwise help with abstraction.
static const uint16_t LINE_TIMEOUT_TICKS             = 40    *(REAL_SPEED/1000000)/TIMER1_CLOCK_DIVIDER_EQ;  // Minimum expected vertical blanking time
volatile uint8_t RecentInputPulse=false;               // Pin interrupt sets to true, timer overflow sets to false.  Should be true whenever there is valid video input
volatile uint16_t VerticalBlankingTimer1Ticks=0;       // Ticks of timer1 between frames
volatile uint16_t MicrosecondsPerFrame=0;             // Frametime in microseconds
volatile uint32_t LastFrameMicros=0;                  // Timestamp (microseconds) of the last frame start
  uint8_t STROBE_TRIGGER_POINT;
uint8_t BacklightMode;  // off/pwm/strobing/dualstrobing
  
private:



static const uint8_t TIMER_MIN_DIVIDER = 0x01; // For timer2
static const uint8_t TIMER_MAX_DIVIDER = 0x07; // For timer2
static const uint16_t FILTER_FAST_RATE               = 2;    // Use a power of 2 for efficient implementation
static const uint16_t FILTER_CONVERGENCE_HYSTERESIS  = 20    *(REAL_SPEED/1000000)/TIMER1_CLOCK_DIVIDER_EQ;
static const uint16_t FILTER_LOCKLOSS_THRESHOLD      = 100   *(REAL_SPEED/1000000)/TIMER1_CLOCK_DIVIDER_EQ;    // Allowed deviation between the locked value and the current filtered value before attempting to re-lock
static const uint16_t BLANKING_TIME_MAXIMUM          = 20000 *(REAL_SPEED/1000000)/TIMER1_CLOCK_DIVIDER_EQ;
static const uint16_t BLANKING_TIME_MINIMUM          = 10    *(REAL_SPEED/1000000)/TIMER1_CLOCK_DIVIDER_EQ;

static const uint16_t StrobePulseDelayMinimum=50;                  // microseconds
static const uint16_t StrobePulseDurationMaximum=2000;                  // microseconds
static const uint16_t StrobePulseDurationMinimum=100;                  // microseconds



uint16_t StrobePulseDelay=StrobePulseDelayMinimum;                         // microseconds
uint16_t StrobePulseDuration=1000;                    // microseconds, to be calculated from VerticalBlankingTimer1Ticks measurement


int16_t   accumulated_error_BLANKING = 0;
uint16_t FILTERED_BLANKING    = 0;
uint16_t LOCKED_BLANKING    = 0;
uint8_t TargetTimer2DividerStrobe;
uint8_t FRAME_TIMINGS_LOCKED = false;
uint8_t BacklightDisabledTemporarilyUntilRelock = false;
uint8_t BrightnessStable;  // Desired stable-mode brightness
uint8_t BrightnessPulse;  // Desired pulse-mode brightness
uint16_t FrequencyReal;  
uint16_t FrequencyWanted; 
  
void init( uint8_t mode, uint8_t stable_brightness, uint8_t pulse_brightness, uint16_t pwmfrequency);

 uint8_t RdBrightness();
 void WrBrightness(uint8_t value);


 void AdjustTimerStrobing();
 void SetBLEN_OFF();
 void SetBLEN_ON();
 void SetHighCurrent_OFF();
 void SetHighCurrent_ON();
 void SetModeOff();
 void SetModeStable();
 void SetModePulse();
 void DeactivateTimer1();
 void DisblePWM();
 void ForceModeStable();
 void PrintFrequency(); 
 
 uint16_t PseudoAtomicRead_VerticalBlankingTimer1Ticks();
 void SetFrequencyPWM_NOSAVE();
 void SetFrequencyPWM_NOSAVE(uint16_t frequency);
uint8_t GetClosestDivider(uint16_t frequency);
uint8_t GetClosestDivider_BUT_NOT_UNDER(uint16_t frequency);
uint8_t GetClosestDivider_BUT_NOT_OVER(uint16_t frequency);
uint8_t GetCurrentDivider();
uint8_t GetCurrentDivider1();
void SetCurrentDivider(uint8_t divider);
uint16_t DetermineTimer2FrequencyPWM(uint8_t divider);
uint16_t DetermineTimer2Prescaler(uint8_t myDivider);
uint32_t DetermineTimer2TickRate(uint8_t myDivider);
uint32_t TimerTicksToMicroseconds(uint16_t ticks);
uint8_t Timer1TicksPerTenMicroSeconds();
uint8_t Timer1TicksPerHundredMicroSeconds();
uint8_t Timer1TicksPerMilliSeconds();
void ResetFrameParameters();

uint8_t LimitBrightness(uint8_t brightness);
uint8_t SetBrightnessNoSave();
uint8_t SetBrightnessNoSave( uint8_t brightness);
 
FORCE_INLINE  uint8_t FastReadDE();
 void  EEpromWriteMode();
 void  EEpromWriteBrightness();
 void  EEpromWriteStableBrightness();
 void  EEpromWritePulseBrightness();
 void  EEpromWriteFrequency();
 uint8_t  EEpromReadMode();
 uint8_t  EEpromReadStableBrightness();
 uint8_t  EEpromReadPulseBrightness();
 uint16_t  EEpromReadFrequency();
 
 void DebugTiming();
};






#endif

