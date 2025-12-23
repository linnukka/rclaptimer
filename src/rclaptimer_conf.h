#ifndef RCLAPTIMERCONF_H
#define RCLAPTIMERCONF_H


constexpr uint8_t DISP_DATA_IN = 21;  // pin 12 is connected to the  
constexpr uint8_t DISP_CLK = 23;      // CLK pin 11 is connected to the  
constexpr uint8_t DISP_LOAD = 10;     // pin 10 is connected to LOAD 
constexpr uint8_t DISP_CS = 22;       // pin 10 is connected to LOAD 
constexpr uint8_t USE_EXT_REF_JUMPER_PIN = 19;       // If pin19 is LOW, use external reference level 
constexpr uint8_t DISP_MAX_DEVICES = 1; // We have only a single MAX72XX.
constexpr uint8_t UP_BUTTON_PIN = 0;       // Start button input with pull-up
constexpr uint8_t START_BUTTON_PIN = 2;       // Start button input with pull-up
constexpr uint8_t DOWN_BUTTON_PIN = 4;       // Start button input with pull-up
constexpr uint8_t STOP_INPUT1_PIN = 34;        // Stop sensor input 1 (ADC)
constexpr uint8_t STOP_INPUT2_PIN = 35;        // Stop sensor input 2 (ADC)
constexpr uint8_t STOP_INPUT3_PIN = 36;        // Stop sensor input 3 (ADC)
constexpr uint8_t STOP_INPUT4_PIN = 39;        // Stop sensor input 4 (ADC)
constexpr uint8_t REFERENCE_LEVEL_PIN = 32;    // Reference level input (ADC)
// constexpr bool USE_EXTERNAL_REFERENCE_LEVEL = false; // Use external reference instead of self-calculated
constexpr uint16_t CODED_REFERENCE_LEVEL_MV = 1500;   // Fallback ref (mV) when external ref is disabled
constexpr uint16_t SILENT_LANES_ABOVE_MV_ON_RACE_START = 3000;   // On race start, if lane level is below this (mV), consider it unplugged, not in race
constexpr uint16_t SILENT_LANES_BELOW_MV_ON_RACE_START = 100;   // On race start, if lane level is below this (mV), consider it unplugged, not in race
constexpr float LDR_R_FIXED = 15000; // 15k to GND
constexpr uint8_t DEFAULT_LAP_COUNT = 5; // Default number of laps per race
constexpr uint8_t MAX_LAP_COUNT = 99; // Maximum
constexpr uint16_t STOP_INPUT_TRIGGER_INTERVAL_MS = 500; // Minimum interval between stop-input triggers per lane
#endif