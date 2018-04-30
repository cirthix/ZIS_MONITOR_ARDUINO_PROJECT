#ifndef SUPPORTEDPSUS_H
#define SUPPORTEDPSUS_H

// This file handles the panel dc/dc converter (or lacktherof)

#if PANELPSU_VERSION==PANELPSU_IS_SHORTED_TO_INPUT
// DO NOT DEFINE ANY ADDITIONAL OPTIONS

#elif PANELPSU_VERSION==PANELPSU_IS_DIRECT
const float VIN_MINIMUM = PANEL_VOLTAGE*0.75;  // To allow for measurement error, have a large margin.
const float VIN_MAXIMUM = PANEL_VOLTAGE*1.15;

#ifdef DUALPURPOSE_INPUT_VOLTAGE_MONITORING_PIN_CONTROL_VREG_VPANEL_VOLTAGE
#define INPUT_VOLTAGE_MONITORING_PIN DUALPURPOSE_INPUT_VOLTAGE_MONITORING_PIN_CONTROL_VREG_VPANEL_VOLTAGE
#endif

#define CONTROL_VREG_VPANEL_POLARITY ACTIVE_HIGH
// This circuit is just an n-fet connected to the gate of a p-type passthrough fet with a pullup to the input voltage.  With nfet_gate=high, pfet_gate->low and the circuit conducts.


#elif PANELPSU_VERSION==PANELPSU_IS_DIRECT_NOCHECK
#define CONTROL_VREG_VPANEL_POLARITY ACTIVE_HIGH




#elif PANELPSU_IS_TPS5332
const float VIN_MAXIMUM = 16;
const float VIN_MINIMUM = PANEL_VOLTAGE+1.5;

#ifdef DUALPURPOSE_INPUT_VOLTAGE_MONITORING_PIN_CONTROL_VREG_VPANEL_VOLTAGE
#define CONTROL_VREG_VPANEL_VOLTAGE DUALPURPOSE_INPUT_VOLTAGE_MONITORING_PIN_CONTROL_VREG_VPANEL_VOLTAGE
#endif

#define CONTROL_VREG_VPANEL_POLARITY CONTROL_DCDC_VPANEL_POLARITY
  
#else
#error "Unsupported PANELPSU_VERSION"
#endif





#endif
