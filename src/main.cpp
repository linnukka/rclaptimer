#include <Arduino.h>

#include <TaskScheduler.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <Preferences.h>

#include "LedController.hpp"
#include "rclaptimer_credentials.h"
#include "rclaptimer_mqttdefs.h"
#include "rclaptimer_timedefs.h"
#include "rclaptimer_conf.h"

static uint8_t buildnro[3] = {0,0,1}; // Release number array

Preferences prefs;
LedController<1,1> lc;

WiFiClient wifiClient;
PubSubClient pubSubClient(wifiClient);
int loopCount = 0;
void networkSetupCallback();
Scheduler ts;
int startlightson=0;
unsigned long raceStartedMillis = 0;
volatile uint16_t referenceLevelMilliVolts = CODED_REFERENCE_LEVEL_MV;
volatile unsigned long laneTimes[4] = {0, 0, 0, 0};
uint8_t stopLowCounts[4] = {0, 0, 0, 0};
bool laneEnabled[4] = {false, false, false, false};
uint8_t laneLapCounts[4] = {0, 0, 0, 0};
unsigned long laneLastTriggerMillis[4] = {0, 0, 0, 0};
constexpr uint8_t STOP_INPUT_PINS[4] = {STOP_INPUT1_PIN, STOP_INPUT2_PIN, STOP_INPUT3_PIN, STOP_INPUT4_PIN};
static uint8_t consecutiveStartButtonLowReads = 0;
static uint8_t consecutiveUpButtonLowReads = 0;
static uint8_t consecutiveDownButtonLowReads = 0;
bool startButtonHandled = false;
bool downButtonHandled = false;
bool upButtonHandled = false;
uint8_t targetLapCount = DEFAULT_LAP_COUNT;
bool lapsConfigurable = false;
uint8_t raceResultOrder[4] = {0, 1, 2, 3};
uint8_t raceResultCount = 0;
uint8_t raceResultIndex = 0;
bool raceResultsInitialized = false;
bool voltageDisplayMode = false;
bool lapCountConfigMode = false;
uint8_t voltageDispLoopCount = 0;

// --------- TASK CALLBACKS -------------
void wifiReconnectCallback();
void pubsubLoopCallback();
void updateStartLightsCallback();
void startRaceCallback();
void raceClockStartDelayCallback();
void updateRaceClockDisplayCallback();
void pollButtonsCallback();
void reEnableRaceClockDisplayCallback();
void pollStopInputsCallback();
void pauseBeforeStartLightsCallback();
void raceCompletedCallback();
void showLanesCallback();
void showVoltagesCallback();
void configureLapsCallback();


// --------- HELPERS -------------
void resetLaneTracking();
void updateReferenceLevel();
void resetRace();
void updateLaneEnablement();

// void mqttReconnectCallback();
void otaHandleCallback(){
  ArduinoOTA.handle();
}

// Task tStartPump(TASK_IMMEDIATE, 1, &startPumpCallback, &ts, false);
// Task tStopPump(pumpcycleinms * TASK_MILLISECOND, 1, &stopPumpCallback, &ts, false);
// Task tUpdateLight(lightUpdateIntervalms * TASK_MILLISECOND, TASK_FOREVER, &updateLightCallback, &ts, false);
// Task tCheckWater(waterCheckIntervalms * TASK_MILLISECOND, TASK_FOREVER, &checkWaterCallback, &ts, false);

// Task tMqttReconnect(mqttreconnectdelayms * TASK_MILLISECOND, TASK_FOREVER, &mqttReconnectCallback, &ts, false);
Task tWifiReconnect(wifiReconnectIntervalms * TASK_MILLISECOND, TASK_FOREVER, &wifiReconnectCallback, &ts, false);

Task tPubsub(pubsubloopintervalms * TASK_MILLISECOND, TASK_FOREVER, &pubsubLoopCallback, &ts, false);
Task tOtaPoll(otahandleintervalms * TASK_MILLISECOND, TASK_FOREVER, &otaHandleCallback, &ts, false);
Task tNetworkSetup(networksetupintervalms * TASK_MILLISECOND, TASK_FOREVER, &networkSetupCallback, &ts, false);
Task tStartLightsUpdate(startlightdelayms * TASK_MILLISECOND, 7, &updateStartLightsCallback, &ts, false);
Task tStartRace(startmindelayms * TASK_MILLISECOND, 1, &startRaceCallback, &ts, false);
Task tRaceClockStartDelay(raceclockdisplaystartdelayintervalms * TASK_MILLISECOND, 1, &raceClockStartDelayCallback, &ts, false);
Task tUpdateRaceClockDisplay(raceclockdisplayupdateintervalms * TASK_MILLISECOND, TASK_FOREVER, &updateRaceClockDisplayCallback, &ts, false);
Task tReEnableRaceClockDisplay(raceclockreenableintervalms * TASK_MILLISECOND, 1, &reEnableRaceClockDisplayCallback, &ts, false);
Task tButtonsPoll(buttonpollintervalms * TASK_MILLISECOND, TASK_FOREVER, &pollButtonsCallback, &ts, false);
// Task tReferenceLevelUpdate(referencelevelupdateintervalms * TASK_MILLISECOND, TASK_FOREVER, &updateReferenceLevelCallback, &ts, false);
Task tStopInputsPoll(stopinputpollintervalms * TASK_MILLISECOND, TASK_FOREVER, &pollStopInputsCallback, &ts, false);
Task tPauseToShowLanes(pausebeforelansesms * TASK_MILLISECOND, 1, &showLanesCallback, &ts, false);
Task tPauseBeforeStartLights(pausebeforestartlightsms * TASK_MILLISECOND, 1, &pauseBeforeStartLightsCallback, &ts, false);
Task tRaceCompleted(racecompletedintervalms * TASK_MILLISECOND, TASK_FOREVER, &raceCompletedCallback, &ts, false);
Task tShowVoltages(voltageDisplayIntervalms * TASK_MILLISECOND, TASK_FOREVER, &showVoltagesCallback, &ts, false);
Task tConfigureLaps(lapCountConfigIntervalms * TASK_MILLISECOND, TASK_FOREVER, &configureLapsCallback, &ts, false);

static void mqttCallback(char* topic, unsigned char* payload, unsigned int length){
  
    String msg = String((char*)payload).substring(0,length);
    #ifdef _DEBUG_
        Serial.println("Got message: " + msg + " to: " + String(topic));
    #endif
}

boolean connectMqtt(){
  boolean retval = false;
  if(pubSubClient.connected()) {
    retval = true;
  } else {
    #ifdef _DEBUG_
      Serial.print(millis());
      Serial.println(F(": Not connected to MQTT, trying to connect"));
    #endif      

    Serial.printf("[MQTT] Connecting to %s\n", mqttserverip);

    if (pubSubClient.connect(mqttclientname, mqttusername, mqttpasswd)) {
      pubSubClient.subscribe(mqttcommandtopic);
      retval = true;
      #ifdef _DEBUG_
        Serial.print(millis());
        Serial.println(F(": Connected and subscribed!"));
      #endif      
      Serial.printf("[MQTT] Connected to %s\n", mqttserverip);
    } else {
      // connection failed, retry happens from task scheduling if necessary
      // #ifdef _DEBUG_
        Serial.print(millis());
        Serial.println(F(": MQTT reconnect failed"));
      // #endif      
    }
  }
  return retval;
}

void wifiReconnectCallback() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Reconnecting to Wi-Fi...");
    // WiFi.begin(wifissidTo, wifipassTo);
    Serial.printf("[WiFi] Reconnecting to %s\n", wifissid);
    WiFi.reconnect();
  } 
  connectMqtt();
}

boolean sendMqttMsg(String message, String topic){
    boolean retval = false;
    #ifdef _DEBUG_
        Serial.println("Publishing: " + message + " to: " + topic);
    #endif
    if(pubSubClient.connected()){
        // See https://pubsubclient.knolleary.net/api#beginPublish
        int len = message.length();
        pubSubClient.beginPublish(topic.c_str(), len, true);
        pubSubClient.write((byte*)message.c_str(), len);
        retval = pubSubClient.endPublish();

        #ifdef _DEBUG_
            Serial.println("Publish ok with begin-write-end: " + String(retval));
        #endif

        //String msg = "Published: " + message + " to: " + topic;
        //pubSubClient.publish(mqttdebugtopic, msg.c_str());    
        
        #ifdef _DEBUG_
            Serial.println("Publish ok with begin-write-end: " + String(retval));
        #endif
    }  else {
      #ifdef _DEBUG_
          Serial.println("MQTT not connected!");
      #endif
    }
    return retval;
}

void networkSetupCallback() {
  static bool wifiStarted = false;
  static bool mqttConfigured = false;
  static bool runtimeTasksEnabled = false;

  if (!wifiStarted) {
    Serial.println("[WiFi] Starting connection");
    Serial.printf("[WiFi] Connecting to %s\n", wifissid);
    WiFi.begin(wifissid, wifipass);
    wifiStarted = true;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] Waiting for connection...");
    return;
  }

  if (!mqttConfigured) {
    Serial.printf("[WiFi] Connected to %s (%s)\n", wifissid, WiFi.localIP().toString().c_str());
    pubSubClient.setServer(mqttserverip, 1883);
    pubSubClient.setCallback(mqttCallback);
    mqttConfigured = true;
  }

  bool newlyConnected = false;
  if (!pubSubClient.connected()) {
    Serial.println("[MQTT] Connecting...");
    if (!connectMqtt()) {
      return;
    }
    newlyConnected = true;
  }

  if (newlyConnected) {
    String msg = "rclaptimer connected and subscribed. IP:" + WiFi.localIP().toString();
    pubSubClient.publish(mqttdebugtopic, msg.c_str());
    Serial.printf("[MQTT] Connected and subscribed (IP %s)\n", WiFi.localIP().toString().c_str());
  }

  if (!runtimeTasksEnabled) {
    tWifiReconnect.enable();
    tOtaPoll.enable();
    tPubsub.enable();
    runtimeTasksEnabled = true;
  }

  tNetworkSetup.disable();
}

void generateRandomStartDelay(){
  int delayms = random(startmindelayms, startmaxdelayms);
  tStartRace.setInterval(delayms * TASK_MILLISECOND);
  Serial.printf("[%lu][generateRandomStartDelay] Generated random start delay: %d ms\n", millis(), delayms);
}
void displayLaps() {
  lc.clearMatrix();
  lc.setChar(0, 6, 'L', false);
  lc.setChar(0, 5, 'A', false);
  lc.setChar(0, 4, 'P', false);
  lc.setDigit(0, 3, 5, false);
  lc.setChar(0, 2, ' ', false);

  uint8_t displayTarget = targetLapCount > 99U ? 99U : targetLapCount;
  uint8_t tens = displayTarget / 10U;
  uint8_t ones = displayTarget % 10U;

  if (displayTarget >= 10U) {
    lc.setDigit(0, 1, tens, false);
  } else {
    lc.setChar(0, 1, ' ', false);
  }
  lc.setDigit(0, 0, ones, false);
}

void displayReadyTxt(){
  lc.clearMatrix(); /* and clear the display */
  lc.setChar(0, 6, '-', false);
  lc.setChar(0, 5, 'A', false);
  lc.setChar(0, 4, 'E', false);
  lc.setChar(0, 3, 'A', false);
  lc.setChar(0, 2, 'd', false);
  lc.setDigit(0, 1, 4, false);
  lc.setChar(0, 0, '-', false);
}

void updateStartLightsCallback(){
  // Serial.printf("[updateStartLightsCallback] startlightson=%d\n", startlightson);
  if(startlightson<6){
    lc.setChar(0,7,' ',false); // Clear
    lc.setChar(0,6,' ',false); // Clear
    lc.setChar(0,5,' ',false); // Clear

    for(int i=0;i<5;i++){
      int column = 4 - i;
      if(i <= startlightson){
        lc.setDigit(0, column, 8, false); // Red light on
      } else {
        lc.setChar(0, column, ' ', false); // Remaining segments off
      }
    }

    startlightson++;

  } else {
    Serial.printf("[%lu][updateStartLightsCallback] All lights on. Now delay and turn off leds to start race\n", millis());
    tStartLightsUpdate.disable();
    tStartLightsUpdate.setIterations(7);
    generateRandomStartDelay(); // New random delay for race, sets tStartRace interval
    tStartRace.setIterations(1);
    tStartRace.enableDelayed();
  }
}

void startRaceCallback(){
  Serial.printf("[%lu][startRaceCallback] startlightson=%d\n", millis(), startlightson);
  startlightson=0;
  lc.clearMatrix();
  // resetLaneTracking();
  raceStartedMillis = millis();
  tRaceClockStartDelay.setIterations(1);
  tRaceClockStartDelay.enableDelayed();  // Delay before starting race clock display updates
}

void raceClockStartDelayCallback(){ // Start the race clock display updates after a delay
  Serial.printf("[%lu][raceClockStartDelayCallback]\n", millis());
  tUpdateRaceClockDisplay.enable();
  tStopInputsPoll.enable();
  tRaceClockStartDelay.disable();
  tRaceClockStartDelay.setIterations(1);
}

void updateRaceClockDisplayCallback(){
  unsigned long elapsedMillis = millis() - raceStartedMillis;
  // Now race is on - update the race clock display
  // Serial.printf("[updateRaceClockDisplayCallback] elapsedMillis=%lu\n", elapsedMillis);
  if(raceStartedMillis>0){

    unsigned long totalSeconds = elapsedMillis / 1000;
    unsigned long minutes = totalSeconds / 60;
    unsigned long seconds = totalSeconds % 60;
    unsigned long tenths = (elapsedMillis % 1000) / 100;

    // Display format MM:SS.t
    lc.setChar(0,7,' ',false); // Clear
    lc.setChar(0,6,' ',false); // Clear
    lc.setChar(0,5,' ',false); // Clear
    lc.setDigit(0, 4, minutes / 10, false); // Tens of minutes
    lc.setDigit(0, 3, minutes % 10, true); // Units of minutes
    lc.setDigit(0, 2, seconds / 10, false); // Tens of seconds
    lc.setDigit(0, 1, seconds % 10, true); // units of seconds
    lc.setDigit(0, 0, tenths, false); // tenths of seconds 
  }
}

void resetLaneTracking(){
  for (int i = 0; i < 4; ++i) {
    laneTimes[i] = 0;
    stopLowCounts[i] = 0;
    laneLastTriggerMillis[i] = 0;
  }
  updateLaneEnablement();
}

void handleStartButtonPress(){
  Serial.printf("[pollStartButtonCallback] Start button is LOW. Consecutive reads=%d\n", consecutiveStartButtonLowReads);
  if (consecutiveStartButtonLowReads < 255) {
    consecutiveStartButtonLowReads++;
  }
  if(!startButtonHandled){
    if (consecutiveStartButtonLowReads >= 2 && !tStartLightsUpdate.isEnabled()) {
      Serial.printf("[pollStartButtonCallback] Start button confirmed pressed after debouncing. lapCountConfigMode=%d voltageDisplayMode=%d\n", lapCountConfigMode ? 1 : 0, voltageDisplayMode ? 1 : 0);
      startButtonHandled = true;

      if(lapCountConfigMode){
        // Exit lap count configuration mode
        tConfigureLaps.disable();
        lapCountConfigMode = false;
        lapsConfigurable = false;
        
        // Store targetLapCount to non-volatile storage 
        prefs.begin("lapcount", false);           // "namespace", RW = false? (false = read/write)
        prefs.putUInt("lapcount", targetLapCount);    // save
        prefs.end();  
        displayReadyTxt();

        // Start race normally
        // resetRace();

        Serial.println("[pollButtonsCallback] Exiting lap count configuration mode.");
      } else if (voltageDisplayMode){
        // Exit voltage display mode
        voltageDisplayMode = false;
        tShowVoltages.disable();
        voltageDispLoopCount = 0;

        // Display Ready
        displayReadyTxt();

        Serial.println("[pollButtonsCallback] Exiting voltage display mode.");
      } else {    
        Serial.println("[pollButtonsCallback] Start button  pressed. Starting start light sequence.");
        // Start the start light sequence

        resetRace();
        // tStartLightsUpdate.enableDelayed();
        // tPauseBeforeStartLights.enableDelayed();
        tPauseToShowLanes.enableDelayed();
      }
    }
  } else {
    // Serial.println("[pollButtonsCallback] Start button press already handled, ignoring.");
  }
}

void handleUpButtonPress(){
  Serial.printf("[pollButtonsCallback] Up button is LOW. Consecutive reads=%d\n", consecutiveUpButtonLowReads);
  if (consecutiveUpButtonLowReads < 255) {
    consecutiveUpButtonLowReads++;
  }
  if(!upButtonHandled){
    if (consecutiveUpButtonLowReads >= 2 && lapsConfigurable) {
      upButtonHandled = true;
      lapCountConfigMode  = true;
      tPauseToShowLanes.disable();
      // Increase target lap count
      if (targetLapCount < MAX_LAP_COUNT) {
        targetLapCount++;
        displayLaps();
        Serial.printf("[pollButtonsCallback] Increased target lap count to %d\n", targetLapCount);
      }
    }
  }
}

void handleDownButtonPress(){
  Serial.printf("[pollButtonsCallback] Down button is LOW. Consecutive reads=%d\n", consecutiveDownButtonLowReads);
  if (consecutiveDownButtonLowReads < 255) {
    consecutiveDownButtonLowReads++;
  }
  if(!downButtonHandled){
    if (consecutiveDownButtonLowReads >= 2 && lapsConfigurable) {
      downButtonHandled = true;
      lapCountConfigMode  = true;
      tPauseToShowLanes.disable();
      // Decrease target lap count
      if (targetLapCount > 1) {
        targetLapCount--;
        displayLaps();
        Serial.printf("[pollButtonsCallback] Decreased target lap count to %d\n", targetLapCount);
      }
    }
  }
}

void pollButtonsCallback(){
  if (digitalRead(START_BUTTON_PIN) == LOW) {
    handleStartButtonPress();  
  } else if (digitalRead(UP_BUTTON_PIN) == LOW) {
    handleUpButtonPress();  
  } else if (digitalRead(DOWN_BUTTON_PIN) == LOW) {
    handleDownButtonPress();  
  } else {
    consecutiveStartButtonLowReads = 0;
    consecutiveUpButtonLowReads = 0;
    consecutiveDownButtonLowReads = 0;
    startButtonHandled = false;
    upButtonHandled = false;
    downButtonHandled = false;
  }
}


void resetRace(){
  Serial.println("[resetRace] Resetting race state.");

  raceResultsInitialized = false;
  raceResultCount = 0;
  raceResultIndex = 0;
  startlightson = 0;

  for (int i = 0; i < 4; ++i) {
    laneLapCounts[i] = 0;
    laneLastTriggerMillis[i] = 0;
    laneTimes[i] = 0;
    stopLowCounts[i] = 0; 
  }
  resetLaneTracking();
  tRaceClockStartDelay.disable();
  tRaceClockStartDelay.setIterations(1);
  tStopInputsPoll.disable();
  tStartRace.disable();
  tUpdateRaceClockDisplay.disable();
  tReEnableRaceClockDisplay.disable();

  tRaceCompleted.disable();

  displayLaps();
  lapsConfigurable = true;
}

void updateReferenceLevel(){
  uint16_t newLevel = CODED_REFERENCE_LEVEL_MV;

  uint16_t raw = analogRead(REFERENCE_LEVEL_PIN);
  uint16_t extLevel = static_cast<uint16_t>((static_cast<uint32_t>(raw) * 3300U) / 4095U);

  bool useExtRef = (digitalRead(USE_EXT_REF_JUMPER_PIN) == LOW);

  if (useExtRef) {
    newLevel = extLevel;
  }
  referenceLevelMilliVolts = newLevel;
  printf("[updateReferenceLevel] Reference level updated: %d mV external: %d external level: %d\n", referenceLevelMilliVolts, useExtRef, extLevel);
}

void pollStopInputsCallback(){
  if (raceStartedMillis == 0) {
    return;
  }

  for (int lane = 0; lane < 4; ++lane) {
    // Only check lanes that haven't recorded a time yet
    if (!laneEnabled[lane] || laneLapCounts[lane] >= targetLapCount) {
      continue;
    }

    uint16_t raw = analogRead(STOP_INPUT_PINS[lane]);
    uint16_t levelMv = static_cast<uint16_t>((static_cast<uint32_t>(raw) * 3300U) / 4095U);

    // Serial.printf("[pollStopInputsCallback] lane=%d levelMv=%d threshold=%d\n", lane, levelMv, threshold);

    if (levelMv > referenceLevelMilliVolts) {
      if (stopLowCounts[lane] < 255) {
        stopLowCounts[lane]++;
      }
      if (stopLowCounts[lane] >= 2) {
        unsigned long now = millis();
        if (laneLastTriggerMillis[lane] != 0U) {
          unsigned long elapsed = now - laneLastTriggerMillis[lane];
          if (elapsed < STOP_INPUT_TRIGGER_INTERVAL_MS) {
            stopLowCounts[lane] = 0;
            continue;
          }
        }

        unsigned long lapTime = now - raceStartedMillis;
        laneLastTriggerMillis[lane] = now;
        laneLapCounts[lane]++;
        Serial.printf("[pollStopInputsCallback] lane=%d lap=%u time=%lu ms level=%u\n", lane, laneLapCounts[lane], lapTime, levelMv);
        tUpdateRaceClockDisplay.disable();
        tReEnableRaceClockDisplay.setIterations(1);
        tReEnableRaceClockDisplay.enableDelayed();
        lc.setDigit(0, 7, lane+1, true); // Display lane number of lap time

        if (laneLapCounts[lane] >= targetLapCount) {
          lc.setChar(0, 6, 'F', false); // F for Finished
          laneTimes[lane] = lapTime;
          Serial.printf("[pollStopInputsCallback] lane=%d reached target laps (%u). Final time=%lu ms\n", lane, targetLapCount, lapTime);

          bool allFinished = true;
          for (int i = 0; i < 4; ++i) {
            if (laneEnabled[i] && laneLapCounts[i] < targetLapCount) {
              allFinished = false;
              break;
            }
          }

          if (allFinished) {
            Serial.println("[pollStopInputsCallback] All enabled lanes finished; scheduling race-complete display.");
            tUpdateRaceClockDisplay.disable();
            tReEnableRaceClockDisplay.disable();
            tStopInputsPoll.disable();
            raceResultsInitialized = false;
            raceResultIndex = 0;
            tRaceCompleted.setIterations(TASK_FOREVER);
            tRaceCompleted.enableDelayed();
          }
        } else {
          // Display lap count with formatting rules
          uint8_t displayLap = laneLapCounts[lane] > 99U ? 99U : laneLapCounts[lane];
          if (displayLap < 10U) {
            lc.setDigit(0, 6, displayLap, false);
            lc.setChar(0, 5, ' ', false);
          } else {
            uint8_t tens = displayLap / 10U;
            uint8_t ones = displayLap % 10U;
            lc.setDigit(0, 6, ones, false);
            lc.setDigit(0, 5, tens, true);
          }
        }

        stopLowCounts[lane] = 0;
      }
    } else {
      stopLowCounts[lane] = 0;
    }
  }
}

void showLanesCallback(){
  lapsConfigurable = false;
  tPauseToShowLanes.disable();
  tPauseToShowLanes.setIterations(1);
  lc.clearMatrix();

  lc.setChar(0, 7, 'L', false); // Clear disabled lanes
  lc.setChar(0, 6, 'A', false); // Clear disabled lanes
  lc.setChar(0, 5, 'H', false); // Clear disabled lanes
  lc.setChar(0, 4, 'E', false); // Clear disabled lanes

  for (int i = 0; i < 4; ++i) {
    if (laneEnabled[i]) {
      lc.setDigit(0, 3 - i, i + 1, false); // Show enabled lanes
    } else {
      lc.setChar(0, 3 - i, ' ', false); // Clear disabled lanes
    }
  }

  tPauseBeforeStartLights.enableDelayed();
}

void showVoltagesCallback(){
  
  lc.clearMatrix();

  if(voltageDispLoopCount == 0){
    updateReferenceLevel();

    lc.setChar(0, 7, 'U', false); // Input voltage
    lc.setChar(0, 6, 'E', false); // External reference


    lc.setDigit(0, 4, (referenceLevelMilliVolts / 1000) % 10, true); // Thousands, i.e. volts
    lc.setDigit(0, 3, (referenceLevelMilliVolts / 100) % 10, false); // Hundreds
    lc.setDigit(0, 2, (referenceLevelMilliVolts / 10) % 10, false); // Tens
    lc.setDigit(0, 1, referenceLevelMilliVolts % 10, false); // Units    
  } else if(voltageDispLoopCount > 0 && voltageDispLoopCount < 5){ 
    int lane = voltageDispLoopCount - 1;
    uint16_t raw = analogRead(STOP_INPUT_PINS[lane]);
    uint16_t levelMv = static_cast<uint16_t>((static_cast<uint32_t>(raw) * 3300U) / 4095U);

    lc.setChar(0, 7, 'U', false); // Input voltage
    lc.setDigit(0, 6, lane + 1, false); // Lane number

    lc.setDigit(0, 4, (levelMv / 1000) % 10, true); // Thousands, i.e. volts
    lc.setDigit(0, 3, (levelMv / 100) % 10, false); // Hundreds
    lc.setDigit(0, 2, (levelMv / 10) % 10, false); // Tens
    lc.setDigit(0, 1, levelMv % 10, false); // Units
  } else {
    // Count 5, show build number
    lc.clearMatrix();
    lc.setChar(0, 7, 'B', false);
    lc.setChar(0, 6, 'U', false);
    lc.setChar(0, 5, 'I', false);
    lc.setChar(0, 4, 'L', false);
    lc.setChar(0, 3, 'd', true);


    for (int idx = 0; idx < 3; ++idx) {
      lc.setDigit(0, idx, buildnro[2-idx], false);
    }
  }

  voltageDispLoopCount++;
  if(voltageDispLoopCount > 5){
    voltageDispLoopCount = 0;
  }  
}

void configureLapsCallback(){
}

void pauseBeforeStartLightsCallback(){
  lapsConfigurable = false;
  tPauseBeforeStartLights.disable();
  tPauseBeforeStartLights.setIterations(1);
  lc.clearMatrix();
  tStartLightsUpdate.enableDelayed();
}

void raceCompletedCallback(){
  Serial.printf("[%lu][raceCompletedCallback] Race completed callback fired.\n", millis());

  if (!raceResultsInitialized) {
    raceResultCount = 0;
    for (uint8_t lane = 0; lane < 4; ++lane) {
      if (laneEnabled[lane] && laneTimes[lane] > 0U) {
        raceResultOrder[raceResultCount++] = lane;
      }
    }

    if (raceResultCount == 0U) {
      Serial.println("[raceCompletedCallback] No finished lanes to display.");
      return;
    }

    // Already disabled when all lanes finished
    // tStopInputsPoll.disable();
    // tUpdateRaceClockDisplay.disable();

    for (uint8_t i = 0; i + 1 < raceResultCount; ++i) {
      uint8_t minIdx = i;
      for (uint8_t j = i + 1; j < raceResultCount; ++j) {
        if (laneTimes[raceResultOrder[j]] < laneTimes[raceResultOrder[minIdx]]) {
          minIdx = j;
        }
      }
      if (minIdx != i) {
        uint8_t tmp = raceResultOrder[i];
        raceResultOrder[i] = raceResultOrder[minIdx];
        raceResultOrder[minIdx] = tmp;
      }
    }

    raceResultIndex = 0;
    raceResultsInitialized = true;
  }

  // if (raceResultIndex >= raceResultCount) {
  //   Serial.println("[raceCompletedCallback] All results shown, disabling task.");
  //   raceResultsInitialized = false;
  //   tRaceCompleted.disable();
  //   return;
  // }

  uint8_t lane = raceResultOrder[raceResultIndex];
  unsigned long lapTime = laneTimes[lane];
  uint8_t rank = raceResultIndex + 1U;
  uint8_t rankDigit = rank > 9U ? 9U : rank;

  lc.clearMatrix();
  lc.setDigit(0, 7, rankDigit, true); // Rank with decimal point
  lc.setDigit(0, 6, lane + 1U, false); // Lane number (1-based)

  unsigned long totalSeconds = lapTime / 1000UL;
  unsigned long minutes = totalSeconds / 60UL;
  unsigned long seconds = totalSeconds % 60UL;
  unsigned long tenths = (lapTime % 1000UL) / 100UL;

  lc.setChar(0, 5, ' ', false);
  lc.setDigit(0, 4, static_cast<uint8_t>((minutes / 10UL) % 10UL), false);
  lc.setDigit(0, 3, static_cast<uint8_t>(minutes % 10UL), true);
  lc.setDigit(0, 2, static_cast<uint8_t>(seconds / 10UL), false);
  lc.setDigit(0, 1, static_cast<uint8_t>(seconds % 10UL), true);
  lc.setDigit(0, 0, static_cast<uint8_t>(tenths), false);

  raceResultIndex++;
  if (raceResultIndex >= raceResultCount) {
    raceResultIndex = 0;
  }

  // if (raceResultIndex < raceResultCount) {
  //   tRaceCompleted.setIterations(1);
  //   tRaceCompleted.enableDelayed();
  // } else {
  //   raceResultsInitialized = false;
  // }
}

void updateLaneEnablement(){
  updateReferenceLevel();
  for (int lane = 0; lane < 4; ++lane) {
    uint16_t raw = analogRead(STOP_INPUT_PINS[lane]);
    uint16_t levelMv = static_cast<uint16_t>((static_cast<uint32_t>(raw) * 3300U) / 4095U);
    bool laneActive = levelMv > SILENT_LANES_BELOW_MV_ON_RACE_START && levelMv < SILENT_LANES_ABOVE_MV_ON_RACE_START;
    laneEnabled[lane] = laneActive;
    Serial.printf("[updateLaneEnablement] lane=%d level=%u mV enabled=%d\n", lane, levelMv, laneEnabled[lane]);
    if (!laneEnabled[lane]) {
      laneTimes[lane] = 0;
      stopLowCounts[lane] = 0;
      laneLastTriggerMillis[lane] = 0;
    }
  }
}

void reEnableRaceClockDisplayCallback(){
  tUpdateRaceClockDisplay.enable();
  tReEnableRaceClockDisplay.disable();
}

void checkSetupButtons(){
  // Check if up or down buttons are pressed in startup
  // If either, display voltage cycle: E. <ext ref voltage in mV>, then <lane>.laneVoltages
  bool downPressed = (digitalRead(DOWN_BUTTON_PIN) == LOW);
  bool upPressed = (digitalRead(UP_BUTTON_PIN) == LOW);
  Serial.printf("checkSetupButtons: Checking setup buttons: DOWN=%d UP=%d\n", downPressed ? 1 : 0, upPressed ? 1 : 0);

  if(downPressed || upPressed){  
    // Start voltage display mode
    voltageDisplayMode = true;
    tShowVoltages.enable();

  }
  tButtonsPoll.enable();
}

void setup() {
  analogReadResolution(12); 
  Serial.begin(115200);
  delay(200);
  Serial.println("RC Lap timer starting up.");
  delay(1000);

  prefs.begin("lapcount", false);           // "namespace", RW = false? (false = read/write)
  targetLapCount = static_cast<uint8_t>(prefs.getUInt("lapcount", DEFAULT_LAP_COUNT));  // key, default
  prefs.end();  

  //Here a new LedController object is created without hardware SPI.
  lc=LedController<1,1>(DISP_DATA_IN,DISP_CLK,DISP_CS);
  lc.setIntensity(15); /* Set the brightness to a medium values 0..15 */ 

  lc.clearMatrix(); /* and clear the display */
  lc.setChar(0, 6, '-', false);
  lc.setChar(0, 5, 'H', false);
  lc.setChar(0, 4, 'E', false);
  lc.setChar(0, 3, 'L', false);
  lc.setChar(0, 2, 'L', false);
  lc.setDigit(0, 1, 0, false);
  lc.setChar(0, 0, '-', false);
  delay(2000);

  displayReadyTxt();

  pinMode(USE_EXT_REF_JUMPER_PIN, INPUT_PULLUP);
  pinMode(UP_BUTTON_PIN, INPUT_PULLUP);
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(DOWN_BUTTON_PIN, INPUT_PULLUP);
  pinMode(STOP_INPUT1_PIN, INPUT);
  analogSetPinAttenuation(STOP_INPUT1_PIN, ADC_11db); // allows ~0..3.3V range (approx)
  pinMode(STOP_INPUT2_PIN, INPUT);
  analogSetPinAttenuation(STOP_INPUT2_PIN, ADC_11db); // allows ~0..3.3V range (approx)
  pinMode(STOP_INPUT3_PIN, INPUT);
  analogSetPinAttenuation(STOP_INPUT3_PIN, ADC_11db); // allows ~0..3.3V range (approx)
  pinMode(STOP_INPUT4_PIN, INPUT);
  analogSetPinAttenuation(STOP_INPUT4_PIN, ADC_11db); // allows ~0..3.3V range (approx)
  pinMode(REFERENCE_LEVEL_PIN, INPUT);
  analogSetPinAttenuation(REFERENCE_LEVEL_PIN, ADC_11db); // allows ~0..3.3V range (approx)

  updateReferenceLevel();

  Serial.println("[SYS] System ready, waiting for network...");
  tNetworkSetup.enable();

  checkSetupButtons();

  // tButtonsPoll.enable();
  // tReferenceLevelUpdate.enable();
}



void pubsubLoopCallback(){
  // reconnect();

    #ifdef _VERBOSE_
      Serial.print(F("."));
      Serial.print(loopCount++);
    #endif      
  pubSubClient.loop();
}
void loop(){
   ts.execute();
}

