/*!
 * @file Bedside_Atom.ino
 * Bedside Atom V2
 * A WWVB syncronized tabletop clock
 *
 * Written by Zach Shiner, 2025
 * 
 * MIT License
 * See README.md and LICENSE.md for more information
 */

#include "ES100.h"
#include <Wire.h>
#include <EEPROM.h>
#include <TimeLib.h>              // https://github.com/PaulStoffregen/Time
#include "Adafruit_LEDBackpack.h" // https://github.com/adafruit/Adafruit_LED_Backpack
#include <Bounce2.h>              // https://github.com/thomasfredericks/Bounce2
#include <toneAC.h>               // https://bitbucket.org/teckel12/arduino-toneac/wiki/Home
#include "RTClib.h"               // https://github.com/adafruit/RTClib
#include "git-version.h"          // https://arduino.stackexchange.com/a/51488

// Compile debug/options
#define DEBUG ///< Print core functionality debug info
#define DEBUG_CLOCK ///< Print display-related debug info
#define DEBUG_CONTINUOUS ///< Don't stop decoding for debug
#define DISPLAY_SYNC_ON_DRIFT ///< Show "SYNC" only when the drift was large
//#define DISABLE_DISPLAY ///< Disable I2C display
//#define SYNC_LED ///< Keep builtin LED in sync with es100_EN
//#define DISABLE_PIEZO ///< Disable piezo buzzer alerts on system events
#define DISABLE_SYNC_PIEZO ///< Disable piezo buzzer when a good sync happens (annoying)
#define USE_RTC ///< Is an RTC onboard and do we want to use it?
#define DUAL_ANTENNA_SUPPORT /// Enable support for second antenna

// Initialize library objects
ES100 es100;
Adafruit_7segment matrix = Adafruit_7segment();
Bounce2::Button hourButton = Bounce2::Button();
Bounce2::Button minuteButton = Bounce2::Button();
Bounce2::Button DSTswitch = Bounce2::Button();
Bounce2::Button button_24hour = Bounce2::Button();
#ifdef USE_RTC
  RTC_DS1307 RTC;
#endif

// Pin Assignments
const uint8_t es100_IRQ = 7; // PE6
const uint8_t es100_EN = 0;  // PD2
const uint8_t clockButtonPin_Hour = 12;    // PD6
const uint8_t clockButtonPin_Minute = 4;   // PD4
const uint8_t clockSwitchPin_TZ0 = 5;      // PC6
const uint8_t clockSwitchPin_TZ1 = 8;      // PB4
const uint8_t clockSwitchPin_DST = 6;      // PD7
const uint8_t clockButtonPin_24HR = A1;    // PF6
const uint8_t clockButtonPin_ANT_SEL = A2; // PF5

// Constant data
const unsigned long baudrate = 115200;
const uint16_t alertTone_kHz = 4600; // small buzzer
// const uint16_t alertTone_kHz = 3900; // big buzzer
const unsigned int eepromAddress_useTwentyFourHourTime = 32; // arbitrary
const unsigned int eepromAddress_lastAntenna = 36;

// Variables for manipulating a time syncronization
volatile unsigned long atomicMillis = 0;
volatile uint8_t interruptCount = 0;
uint8_t lastInterruptCount = 0;
time_t lastGoodSyncTime = 0;
time_t lastSyncAttempt = 0;
unsigned long syncWatchdog = 0;
#ifdef USE_RTC
  bool rtcDetected = false;
#endif 

// Timestamps for faux multitasking
unsigned long secondsIndicatorMillis = 0;
unsigned long syncCheckIntervalMillis = 0;
unsigned long displayTimeMillis = 0;
unsigned long debugTimeMillis = 0;
unsigned long currentExecutionTime = 0;
unsigned long lastExecutionTime = 0;

// Switch interactions
uint8_t lastTZswitch = 0;
uint8_t heldLoops = 0;
const uint16_t holdThreshold = 800; //ms

// Cycle Times (seconds)
const unsigned long watchdogTimeout = SECS_PER_HOUR * 5;
const time_t staleTimeoutShort = SECS_PER_HOUR * 24;
const time_t staleTimeoutLong = SECS_PER_WEEK;
const int32_t driftThreshold = 59; 

// Flags to control reception
bool timeSyncInProgress = false;  // variable to determine if we are in receiving mode
bool triggerTimeSync = false;     // variable to trigger the reception
bool singleAntennaMode = false;    // use single or dual antenna mode
es100Antenna lastAntenna = ANT_1; // store which antenna was used last / which to start with

// ES100 Data Structures
ES100IRQstatus lastReadIRQStatus = {}; // read every interrupt
ES100Status0 lastReadStatus0 = {};     // read every interrupt
ES100Data validES100Data = {};         // only gets updated when rx complete & rx okay are both true

// Display status indicators
bool indicatorSecondsSeparator = true; // start illuminated
bool indicatorPM = false;
bool indicatorAL1 = false;
bool indicatorAL2 = false;
bool useTwentyFourHourTime = false; // true to use 24-hour clock

// Local time settings
int8_t timezone = 0;
int8_t UTCoffset = 0;
time_t localTime = 0;

// fixed day and night time brightness settings
const uint8_t nightlightStart = 9+12; //9pm
const uint8_t nightlightEnd = 7; // 7am
const uint8_t dayBright = 15;  // 0 to 15
const uint8_t nightBright = 5;  // 0 to 15

void atomic() {
  // Called procedure when we receive an interrupt from the ES100
  // Store milis of when the interrupt happened.  This is the second (time) boundary.
  // Full time frame reception happens in 134s, Tracking reception takes 24.5s
  atomicMillis = millis();
  interruptCount++;
  #ifdef DEBUG
    Serial.println("** INT ** ");
  #endif
}

void printES100DateTime(ES100DateTime dt) {
  Serial.print(F("Received UTC time = 20"));
  Serial.print(dt.year);
  Serial.print(":");
  Serial.print(dt.month);
  Serial.print(":");
  Serial.print(dt.day);
  Serial.print(" ");
  Serial.print(dt.hour);
  Serial.print(":");
  Serial.print(dt.minute);
  Serial.print(":");
  Serial.println(dt.second);
}

inline uint8_t decodeTZswitch() {
  uint8_t timeZoneSwitch; // position 0 through 3
  timeZoneSwitch = !digitalRead(clockSwitchPin_TZ0);
  timeZoneSwitch += !digitalRead(clockSwitchPin_TZ1) << 1;
  return timeZoneSwitch;
}

#ifdef USE_RTC
  time_t getNowRTC () {
    if(RTC.isrunning()) {
      time_t _now = RTC.now().unixtime();

      #ifdef DEBUG
        Serial.print(F("Current RTC time is: "));
        Serial.println(_now);
      #endif

      return _now;
    }
    else {
      #ifdef DEBUG
        Serial.println(F("RTC not yet set."));
      #endif

      return 0;
    }
  }
  
  // Store the name of getNowRTC to use later
  const getExternalTime syncProviderFunction = getNowRTC;
#endif

void adjustSystemTime (time_t _adjust) {
  time_t _new = now() + _adjust;
  
  #ifdef DEBUG
    Serial.print(F("Time adjustment: "));
    Serial.println(_adjust);
    Serial.println(F("Adjusting Time Library to: "));
    Serial.println(_new);
  #endif

  setSyncProvider(0);  // disable sync provider
  adjustTime(_adjust);

  #ifdef USE_RTC
    if (rtcDetected) {
      #ifdef DEBUG 
        Serial.println(F("Adjusting RTC to: "));
        Serial.println(_new);
      #endif

      RTC.adjust(DateTime(_new));
      setSyncProvider(syncProviderFunction); // restore sync provider
    }
  #endif
}

// Update time library with recieved time.  Return drift between system time and real time in seconds
int32_t updateTime(ES100DateTime dt) {
  time_t oldTime = now();
  
  // Align time update with the atomic offset
  // Calculate number of ms needed to get to a multiple of 1000ms away from atomicMillis
  long secondOffset = millis() - atomicMillis;
  delay(1000UL - (unsigned long)(secondOffset % 1000));

  // Calculate the whole number of second elapsed since the interrupt was called
  int secondAdjust = secondOffset / 1000;

  setSyncProvider(0);  // disable sync provider
  setTime(dt.hour,dt.minute,dt.second + 1 + secondAdjust,dt.day,dt.month,dt.year);

  Serial.print("secondOffset: ");
  Serial.println(secondOffset);

  // This is also a good place to align blinking seconds indicator to actual seconds
  secondsIndicatorMillis += (atomicMillis % 1000);  //Offset to next whole second

  time_t _now = now();
  #ifdef USE_RTC
    if (rtcDetected) {
      #ifdef DEBUG
        Serial.print(F("Set RTC to: "));
        Serial.println(_now);
      #endif

      RTC.adjust(DateTime(_now));
      setSyncProvider(syncProviderFunction); // restore sync provider
    }
  #endif

  return oldTime - _now;
}

void calculateUTCoffset(){
  #ifdef DEBUG
    Serial.println(F("Calculating UTC offset"));
  #endif

  timezone = -5 - decodeTZswitch();
  UTCoffset = timezone;

  if (DSTswitch.isPressed()) {
    // If DST begins today
    if (validES100Data.Status0.dstState == DSTbegin){
      if (hour() < 2 - timezone){
        UTCoffset = timezone;
      } else {
        UTCoffset = timezone + 1;
      }
    } else

    // If DST ends today
    if (validES100Data.Status0.dstState == DSTend){
      if (hour() < 2 - timezone - 1){
        UTCoffset = timezone + 1;
      } else {
        UTCoffset = timezone;
      }
    } else

    // If DST is in effect
    if (validES100Data.Status0.dstState == DSTactive){
      UTCoffset = timezone + 1;
    }
  }
}

void alertToneSingle(bool _background = false) {
  toneAC(alertTone_kHz, 10, 50, _background);
}

void alertToneDualDown(bool _background = false) {
  toneAC(alertTone_kHz, 10, 50, false);
  toneAC(alertTone_kHz - 1000, 10, 50, _background);
}

void alertToneDualUp(bool _background = false) {
  toneAC(alertTone_kHz - 1000, 10, 50, false);
  toneAC(alertTone_kHz, 10, 50, _background);
}

void setup() {
  Wire.begin();

  for (uint8_t i = 0; i < 4; i++) {
    digitalWrite(LED_BUILTIN, 1);
    delay(200);
    digitalWrite(LED_BUILTIN, 0);
    delay(200);
  }

  #ifndef DISABLE_DISPLAY
    // Begin display on i2c address 0x70
    matrix.begin(0x70);
    matrix.setBrightness(dayBright);

    // Turn on all addressable digits
    for (uint8_t i = 0; i < 5; i++) {
      matrix.writeDigitRaw(i, 0b11111111);
    }
    matrix.writeDisplay();
    alertToneSingle();
    delay(3000);

    #if defined(DEBUG) || defined(DEBUG_CLOCK)
      Serial.begin(baudrate);
    #endif

    // Encircle display startup animation
    const uint8_t animateDelay = 80;
    for (uint8_t i = 0; i < 6; i++) {
      matrix.writeDigitRaw(0, 0b00000001);
      matrix.writeDisplay();
      delay(animateDelay);
      matrix.clear();
      matrix.writeDigitRaw(1, 0b00000001);
      matrix.writeDisplay();
      delay(animateDelay);
      matrix.clear();
      matrix.writeDigitRaw(3, 0b00000001);
      matrix.writeDisplay();
      delay(animateDelay);
      matrix.clear();
      matrix.writeDigitRaw(4, 0b00000001);
      matrix.writeDisplay();
      delay(animateDelay/2);
      matrix.clear();
      matrix.writeDigitRaw(4, 0b00000010);
      matrix.writeDisplay();
      delay(animateDelay/2);
      matrix.clear();
      matrix.writeDigitRaw(4, 0b00000100);
      matrix.writeDisplay();
      delay(animateDelay);
      matrix.clear();
      matrix.writeDigitRaw(4, 0b00001000);
      matrix.writeDisplay();
      delay(animateDelay);
      matrix.clear();
      matrix.writeDigitRaw(3, 0b00001000);
      matrix.writeDisplay();
      delay(animateDelay);
      matrix.clear();
      matrix.writeDigitRaw(1, 0b00001000);
      matrix.writeDisplay();
      delay(animateDelay);
      matrix.clear();
      matrix.writeDigitRaw(0, 0b00001000);
      matrix.writeDisplay();
      delay(animateDelay);
      matrix.clear();
      matrix.writeDigitRaw(0, 0b00010000);
      matrix.writeDisplay();
      delay(animateDelay/2);
      matrix.clear();
      matrix.writeDigitRaw(0, 0b00100000);
      matrix.writeDisplay();
      delay(animateDelay/2);
      matrix.clear();
    }
    matrix.writeDisplay();
  #endif

  #if defined(DEBUG) || defined(DEBUG_CLOCK)
    while(!Serial && millis() < 5000) {
      // Wait for serial to connect
    }
    Serial.println(F("\r\n\r\n***  INIT  ***"));
    Serial.println("Version " GIT_VERSION);
  #endif

  // Initialize WWVB reciever
  es100.begin(es100_IRQ, es100_EN);
  attachInterrupt(digitalPinToInterrupt(es100_IRQ), atomic, FALLING);

  // Setup hardware buttons
  const uint16_t debounceInterval = 5;
  hourButton.attach(clockButtonPin_Hour, INPUT_PULLUP);
  minuteButton.attach(clockButtonPin_Minute, INPUT_PULLUP);
  DSTswitch.attach(clockSwitchPin_DST, INPUT_PULLUP);
  button_24hour.attach(clockButtonPin_24HR, INPUT_PULLUP);

  pinMode(clockSwitchPin_TZ0, INPUT_PULLUP);
  pinMode(clockSwitchPin_TZ1, INPUT_PULLUP);
  pinMode(clockButtonPin_ANT_SEL, INPUT_PULLUP);

  hourButton.interval(debounceInterval);
  minuteButton.interval(debounceInterval);
  DSTswitch.interval(debounceInterval);
  button_24hour.interval(debounceInterval);

  hourButton.setPressedState(LOW);
  minuteButton.setPressedState(LOW);
  DSTswitch.setPressedState(LOW);
  button_24hour.setPressedState(LOW);
  
  #ifdef DEBUG
    delay(1000);
    es100.enable();
    uint8_t deviceID = es100.getDeviceID();
    Serial.print("Device ID: 0x");
    Serial.print(deviceID, HEX);
    Serial.print(";\t");
    if (deviceID == 0x10) {
      Serial.println(F("Success: Device ID match"));
    } else {
      Serial.println(F("Error: Device ID mismatch"));
    }
    Serial.println(F("Finished setup"));
    
    lastInterruptCount = 0;
    interruptCount = 0;
  #endif

  #ifdef USE_RTC
    // Attempt to begin RTC - returns false if not present
    rtcDetected = RTC.begin();

    #ifdef DEBUG
      if (rtcDetected) {
        Serial.print(F("RTC Detected - "));

        if (RTC.isrunning()) {
          Serial.println(F("Already running"));
        } else {
          Serial.println(F("Not yet running"));
        }
      } else {
        Serial.println(F("RTC not present"));
      }
    #endif

    // Display out the RTC status
    #ifndef DISABLE_DISPLAY
      matrix.clear();
      matrix.println("rtc ");
      matrix.writeDigitNum(4, rtcDetected);
      matrix.writeDisplay();
      delay(1000);
      matrix.clear();
    #endif
  #endif

  #ifndef USE_RTC
    Serial.println(F("RTC support disabled"));
  #endif

  // Start with time set to Y2k
  adjustTime(SECS_YR_2000);
  setSyncInterval(300); // seconds

  // EEPROM state is unknown at startup.  Validate 24hr time data before moving on.
  uint8_t unknownEEPROMdata = EEPROM.read(eepromAddress_useTwentyFourHourTime);
  #ifdef DEBUG
    Serial.print(F("EEPROM stored 24hour: "));
    Serial.println(unknownEEPROMdata);
  #endif

  if (unknownEEPROMdata == 1) {
    useTwentyFourHourTime = true;
  }
  else if (unknownEEPROMdata == 0) {
    useTwentyFourHourTime = false;
  }
  else {
    EEPROM.write(eepromAddress_useTwentyFourHourTime, useTwentyFourHourTime);
    #ifdef DEBUG
        Serial.println(F("Wrote 24hr time false to EEPROM"));
    #endif
  }

  // repeat for antenna data
  #ifdef DUAL_ANTENNA_SUPPORT
    unknownEEPROMdata = EEPROM.read(eepromAddress_lastAntenna);
    #ifdef DEBUG
      Serial.print(F("EEPROM stored antenna: "));
      Serial.println(unknownEEPROMdata);
    #endif

    if (unknownEEPROMdata == ANT_1) {
      lastAntenna = ANT_1;
    }
    else if (unknownEEPROMdata == ANT_2) {
      lastAntenna = ANT_2;
    }
    else {
      EEPROM.write(eepromAddress_lastAntenna, ANT_1);
      #ifdef DEBUG
          Serial.println(F("Wrote ANT_1 to EEPROM."));
      #endif
    }

    singleAntennaMode = !digitalRead(clockButtonPin_ANT_SEL);
    #ifdef DEBUG
        Serial.print(F("Dual antennas supported. Jumper set to: "));
        if (singleAntennaMode) {
          Serial.println(F("Single"));
        } else {
          Serial.println(F("Dual"));
        }
    #endif
  #endif

  lastTZswitch = decodeTZswitch();

  alertToneDualDown();
}

void loop() {
  uint8_t timeoutCounter;  // limit how many times we retry talking to es100
  const uint8_t timeoutLimit = 5;
  
  // If we are not currently receiving a 1-minute frame and reception is triggered, enable ES100 and start reception
  if (!timeSyncInProgress && triggerTimeSync) {
    es100.enable();

    timeoutCounter = 0;
    while (es100.startRx(lastAntenna, singleAntennaMode) != EXIT_SUCCESS && timeoutCounter < timeoutLimit) {
      #ifdef DEBUG
        Serial.println(F("StartRx did not return EXIT_SUCCESS.  Retry in 5s..."));
      #endif
      delay(5000);
      timeoutCounter++;
    }

    #ifdef DEBUG
      if (timeoutCounter <= timeoutLimit) {
        Serial.print(F("StartRx: EXIT_SUCCESS; "));

        if (singleAntennaMode) {
          Serial.print(F("Single Antenna; "));
        } else {
          Serial.print(F("Dual Antenna; "));
        }
 
        if (lastAntenna == ANT_1) {
          Serial.println(F("Start ANT_1"));
        } else {
          Serial.println(F("Start ANT_2"));
        }
      } else {
        Serial.println(F("Critical failure.  StartRx failed"));
      }
    #endif
    
    lastSyncAttempt = now();
    syncWatchdog = millis();
    timeSyncInProgress = true;
    
    #ifndef DEBUG_CONTINUOUS
      triggerTimeSync = false;
    #endif

    /* Important to set the interrupt counter AFTER the startRx because the es100 
    * confirms that the rx has started by triggering the interrupt. 
    * We can't disable interrupts because the wire library will stop working
    * so we initialize the counters after we start so we can ignore the first false 
    * trigger
    */
    lastInterruptCount = 0;
    interruptCount = 0;
  }

  // If an interrupt was detected
  if (lastInterruptCount < interruptCount) {
    // Read registers once
    lastReadIRQStatus = es100.getIRQStatus();
    lastReadStatus0 = es100.getStatus0();

    #ifdef DEBUG
      Serial.print(F("Reading IRQ and Status0 registers. Received: "));
    #endif

    lastSyncAttempt = now();

    // If we received a valid decode
    if (lastReadIRQStatus.rxComplete && !lastReadIRQStatus.cycleComplete && lastReadStatus0.rxOk) { // IRQStatus = 0x01 & Status0 bit 0
      
      // Read remaining data from es100 and calculate time zone & DST offsets
      validES100Data.DateTimeUTC = es100.getUTCdateTime();
      validES100Data.Status0 = lastReadStatus0;

	    #ifdef DEBUG
        Serial.println(F("Successful Reception"));
        Serial.print(F("Atomic millis = "));
        Serial.println(atomicMillis);

        Serial.print(F("Time Library: "));
        Serial.print(year());
        Serial.print(":");
        Serial.print(month());
        Serial.print(":");
        Serial.print(day());
        Serial.print(" ");
        
        if(hour() <10)
          {
          Serial.print("0");
          }
        Serial.print(hour());
        Serial.print(":");
        if(minute() <10)
        {
          Serial.print("0");
        }
        Serial.print(minute());
        Serial.print(":");
        if(second() <10)
        {
          Serial.print("0");
        }
        Serial.println(second());

        // Serial Print out recieved time
        printES100DateTime(validES100Data.DateTimeUTC);
      #endif

      if(lastGoodSyncTime == 0){
        // Always chirp at the first time sync
        alertToneDualDown(true);
      } else {
        // Chirp at every time sync
        #ifndef DISABLE_PIEZO
          #ifndef DISABLE_SYNC_PIEZO
            alertToneDual(true);
          #endif
        #endif
      }

      int32_t drift = updateTime(validES100Data.DateTimeUTC);
      lastGoodSyncTime = now();
      lastSyncAttempt = lastGoodSyncTime;

      calculateUTCoffset();

      // Flag that we aren't recieving anymore.  Only happens if we get a good decode.
      timeSyncInProgress = false;

      #ifndef DEBUG_CONTINUOUS
        // Stop reception after a good decode
        timeoutCounter = 0;
        while (false && es100.stopRx() != EXIT_SUCCESS && timeoutCounter < timeoutLimit) {
          #ifdef DEBUG
            Serial.println(F("StopRx did not return EXIT_SUCCESS.  Retry in 5s..."));
          #endif
          delay(5000);
          timeoutCounter++;
        }
        #ifdef DEBUG
          if (timeoutCounter <= timeoutLimit) {
            Serial.println(F("StopRx: EXIT_SUCCESS; Disabling es100 after good Rx"));
          } else {
            Serial.println(F("Critical failure.  StopRx failed"));
          }
        #endif
        es100.disable();
      #endif

      #ifdef DEBUG
        Serial.print(F("status0.rxOk = 0b"));
        Serial.println(lastReadStatus0.rxOk, BIN);
        Serial.print(F("status0.antenna = 0b"));
        Serial.println(lastReadStatus0.antenna, BIN);
        Serial.print(F("status0.dstState = 0b"));
        Serial.println(lastReadStatus0.dstState, BIN);
        Serial.print(F("status0.tracking = 0b"));
        Serial.println(lastReadStatus0.tracking, BIN);
        Serial.print(F("Drift: "));
        Serial.println(drift);
      #endif

      lastAntenna = lastReadStatus0.antenna;
      EEPROM.update(eepromAddress_lastAntenna, lastAntenna);

      #ifndef DISABLE_DISPLAY
        #ifdef DISPLAY_SYNC_ON_DRIFT
        // Show "SYNC" only if time has drifted more than 1 minute
        if (abs(drift) > driftThreshold) {
        #endif
        #ifndef DISPLAY_SYNC_ON_DRIFT
        // always show sync on good update
        if (true) {
        #endif
          matrix.clear();
          matrix.println("SYNC");
          matrix.writeDisplay();
          delay(5000);
          matrix.clear();
        }
      #endif
    }
    else if (!lastReadIRQStatus.rxComplete && lastReadIRQStatus.cycleComplete){ // IRQStatus = 0x04
      #ifdef DEBUG
        Serial.println(F("Unsuccessful Reception"));
      #endif
    }
    else {
      #ifdef DEBUG
        Serial.println(F("Bad Data"));
      #endif
    }

    lastInterruptCount = interruptCount;
  }

  // Very rarely ES100 and MCU get out of sync with each other.  Let's reset the process every five hours of searching.
  if (timeSyncInProgress && (millis() - syncWatchdog) > (watchdogTimeout * 1000)) {
    
    timeoutCounter = 0;
    while (false && es100.stopRx() != EXIT_SUCCESS && timeoutCounter < timeoutLimit) {
      #ifdef DEBUG
        Serial.println(F("StopRx did not return EXIT_SUCCESS.  Retry in 5s..."));
      #endif
      delay(5000);
      timeoutCounter++;
    }
    #ifdef DEBUG
      if (timeoutCounter <= timeoutLimit) {
        Serial.println(F("StopRx: EXIT_SUCCESS; Disabling es100 after good Rx"));
      } else {
        Serial.println(F("Critical failure.  StopRx failed"));
      }
    #endif
    es100.disable();
    delay(1500);

    triggerTimeSync = true;
    timeSyncInProgress = false;
    syncWatchdog = millis();
  }

  // Coarse Update Schedule
  //   - Receive the current time on an schedule
  //   - Evaluate if DST is changing
  //   - Set stale indicators
  //   - Set display brightness on a schedule
  if (syncCheckIntervalMillis + 10000 < millis()) {

    // Are we currently recieving the time?
    if (!timeSyncInProgress) {

      // If we aren't already syncing (outer loop)
      // and the time isn't set, request a reception
      if(lastGoodSyncTime == 0){
        triggerTimeSync = true;
        #ifdef DEBUG
          Serial.println(F("Time not set, requesting sync"));
        #endif
      }

      // If we aren't already syncing (outer loop)
      // and the time is set (else)
      // and if we havn't had a successful sync this hour today
      else if(!(hour(lastGoodSyncTime) == hour() && day(lastGoodSyncTime) == day()))
      {
        triggerTimeSync = true;
        #ifdef DEBUG
          Serial.println(F("New hour, requesting sync"));
        #endif
      }
      else {
        // do nothing
        #ifdef DEBUG
          Serial.println(F("Sync not required"));
        #endif
      }
    }

    // If we aren't already syncing (outer loop)
    // and the time is set (else)
    // and it's daytime or we had a good sync this hour today
    else {
      // Already recieving the time, do nothing
      #ifdef DEBUG
        Serial.println(F("Time sync in progress..."));
      #endif
    }

    #ifndef DEBUG_CLOCK
      Serial.print(".");
    #endif

    if(lastGoodSyncTime != 0){
      calculateUTCoffset();

      // if we are in day mode (between end and start)
      if (nightlightEnd <= hour(localTime) && hour(localTime) < nightlightStart) { // local time calculated in the display update loop
        matrix.setBrightness(dayBright);

      // we are in night mode (between start and end)
      } else {
        matrix.setBrightness(nightBright);
      }

    }

    syncCheckIntervalMillis = millis();
  }
  

  // Every 500ms, change the state of various status LEDs
  if (secondsIndicatorMillis + 500 < millis()) {
    
    // Blink display separator every second
    indicatorSecondsSeparator = !indicatorSecondsSeparator;

    // Indicate if time is unset or exceeds short stale timeout
    if ((lastGoodSyncTime == 0) || (now() - lastGoodSyncTime) > staleTimeoutShort){
      indicatorAL1 = true;
      // Indicate if time exceeds long stale timeout
      if ((lastGoodSyncTime == 0) || (now() - lastGoodSyncTime) > staleTimeoutLong) {
        indicatorAL2 = true;
      }
    } 
    // Otherwise turn them off
    else {
      indicatorAL1 = false;
      indicatorAL2 = false;
    }

    secondsIndicatorMillis = millis();
  }

  hourButton.update();
  minuteButton.update();
  DSTswitch.update();
  button_24hour.update();

  // Only allow manual time changes if last sync is stale or never
  if (lastGoodSyncTime == 0 || (now() - lastGoodSyncTime) > staleTimeoutShort) {

    // Advance hour with a single button press less than the hold threshold
    if (hourButton.released() && hourButton.previousDuration() < (holdThreshold / 2)){
      #ifdef DEBUG
        Serial.println(F("HOUR Pressed\t"));
      #endif

      #ifndef DISABLE_PIEZO
        alertToneSingle();
      #endif

      adjustSystemTime(SECS_PER_HOUR);
      lastSyncAttempt = now();
    }

    // Advance minute with a single button press less than the hold threshold
    if (minuteButton.released() && minuteButton.previousDuration() < (holdThreshold / 2)){
      #ifdef DEBUG
        Serial.println(F("MINUTE Pressed\t"));
      #endif

      #ifndef DISABLE_PIEZO
        alertToneSingle();
      #endif

      adjustSystemTime(SECS_PER_MIN);
      lastSyncAttempt = now();
      
      // Undo rollover of minute
      if (minute(lastSyncAttempt) == 0) {
        adjustSystemTime(SECS_PER_DAY - SECS_PER_HOUR);
        lastSyncAttempt = now();
      }
    }

    // Advance minute with a button held longer than than the hold threshold
    if (minuteButton.isPressed() && !hourButton.isPressed() && minuteButton.currentDuration() > holdThreshold + (125 * heldLoops) && hourButton.currentDuration() > holdThreshold) {
      #ifdef DEBUG
        Serial.println(F("MINUTE Held\t"));
      #endif

      #ifndef DISABLE_PIEZO
        if (heldLoops < 1){
          alertToneSingle();
        }
      #endif

      adjustSystemTime(SECS_PER_MIN);
      heldLoops++;
      lastSyncAttempt = now();
      
      // Undo rollover of minute
      if (minute(lastSyncAttempt) == 0) {
        adjustSystemTime(SECS_PER_DAY - SECS_PER_HOUR);
        lastSyncAttempt = now();
      }
    }
    
    // Advance hour with a button held longer than than the hold threshold
    if (hourButton.isPressed() && !minuteButton.isPressed() && hourButton.currentDuration() > holdThreshold + (200 * heldLoops) && minuteButton.currentDuration() > holdThreshold) {
      #ifdef DEBUG
        Serial.println(F("HOUR Held\t"));
      #endif

      #ifndef DISABLE_PIEZO
        if (heldLoops < 1){
          alertToneSingle();
        }
      #endif

      adjustSystemTime(SECS_PER_HOUR);
      heldLoops++;
      lastSyncAttempt = now();
    }
  }

  // Show sync status if both adjust buttons are held
  if (minuteButton.isPressed() && hourButton.isPressed() && minuteButton.currentDuration() > 1000 && hourButton.currentDuration() > 1000 && heldLoops == 0) {
    #ifndef DISABLE_PIEZO
      alertToneDualDown();
    #endif
    
    #ifndef DISABLE_DISPLAY
      matrix.clear();
      matrix.println("LASt");
      matrix.writeDisplay();
      delay(1000);
      matrix.println("SYNC");
      matrix.writeDisplay();
      delay(1000);
      matrix.clear();

      time_t syncAge = now() - lastGoodSyncTime;

      // display "----" if we have not yet synced (or adjusted)
      if (lastGoodSyncTime == 0) {
        matrix.println("----"); 
        #ifdef DEBUG
            Serial.println(F("Not yet synced..."));
        #endif
      } else {

        // display in days if over 72 hours old
        if (elapsedDays(syncAge) > 2) {
          matrix.println(elapsedDays(syncAge));
          matrix.writeDisplay();
          delay(1500);
          matrix.println("DAYS");
        } else {

          // would have been nice if this macro was in TimeLib
          #define elapsedHours(_time_) ((_time_) / SECS_PER_HOUR)  // this is number of hours since Jan 1 1970

          // display in hours if over 1 hr hold
          if (elapsedHours(syncAge) > 0) {
            matrix.writeDigitNum(0, elapsedHours(syncAge) / 10);
            matrix.writeDigitNum(1, elapsedHours(syncAge) % 10);
            matrix.writeDigitAscii(3, 'h');
            matrix.writeDigitAscii(4, 'r');
          } else {

              // display in seconds because M can't be drawin on 7 segment display
              // the character S also looks like a 5 which is not ideal
              matrix.println(syncAge);
              matrix.writeDisplay();
              delay(1500);
              matrix.println(" SEC");
            }
          }
      }

      matrix.writeDisplay();
      delay(2000);
      matrix.clear();

      heldLoops++;
    #endif
  }

  if (!minuteButton.isPressed() && !hourButton.isPressed()) {
    heldLoops = 0;
  }

  // Toggle 24hr mode if button is pressed
  if (button_24hour.released()) {
    useTwentyFourHourTime = !useTwentyFourHourTime;

    #ifndef DISABLE_PIEZO
      if (useTwentyFourHourTime) {
        alertToneDualDown();
      } else {
        alertToneDualUp();
      }
    #endif

    #ifdef DEBUG
        Serial.println(F("24HR Pressed\t"));
    #endif

    EEPROM.update(eepromAddress_useTwentyFourHourTime, useTwentyFourHourTime);
  }

  //Send updates to the display driver
  if(displayTimeMillis + 50 < millis()){
    localTime = now();
    localTime += (time_t)UTCoffset * SECS_PER_HOUR;

    #ifndef DISABLE_DISPLAY

      if(useTwentyFourHourTime){
        matrix.writeDigitNum(0, hour(localTime) / 10);
        matrix.writeDigitNum(1, hour(localTime) % 10);
        indicatorPM = false;
      }
      else {
        if(hourFormat12(localTime) < 10){ // in 12-hour time first digit is 1 or blank
          matrix.writeDigitRaw(0, 0b00000000);
        } else {
          matrix.writeDigitNum(0, 1);
        }
        matrix.writeDigitNum(1, hourFormat12(localTime) % 10);
        indicatorPM = isPM(localTime);
      }

      matrix.writeDigitNum(3, minute(localTime) / 10);
      matrix.writeDigitNum(4, minute(localTime) % 10);

      uint8_t rawToWrite;
      rawToWrite =  0b00000001 * indicatorAL1;
      rawToWrite += 0b00000010 * indicatorAL2;
      rawToWrite += 0b00001100 * indicatorSecondsSeparator;
      rawToWrite += 0b00010000 * indicatorPM;
      matrix.writeDigitRaw(2, rawToWrite);
      
      matrix.writeDisplay();
    #endif

    #ifdef SYNC_LED
      // Turn onboard led on and off with the enable status
      digitalWrite(LED_BUILTIN, digitalRead(es100_EN));
    #endif

    displayTimeMillis = millis();
  }

  #ifdef DEBUG
    // Keep track of execution time
    currentExecutionTime = micros() - lastExecutionTime;
    lastExecutionTime = micros();
  #endif

  #ifdef DEBUG_CLOCK
    if(debugTimeMillis + 300 < millis()){
      if(indicatorPM){
        Serial.print("PM ");
      } else{
        Serial.print("AM ");
      }
      if(indicatorAL1){Serial.print("*");
      }else{Serial.print(" ");}
      if(indicatorAL2){Serial.print("*");
      }else{Serial.print(" ");}
      Serial.print("\t");
      Serial.print(hour());
      Serial.print(":");
      Serial.print(minute());
      Serial.print(":");
      Serial.print(second());
      Serial.print("\t");
      Serial.print("Last attempt: ");
      Serial.print(now() - lastSyncAttempt);
      Serial.print("s\t");
      Serial.print("Last sync: ");
      if(lastGoodSyncTime == 0){
        Serial.print("never\t");
      } else {
        Serial.print(now() - lastGoodSyncTime);
        Serial.print("s\t");
      }
      if(timeSyncInProgress){
        Serial.print("Watchdog: ");
        Serial.print(watchdogTimeout - ((millis() - syncWatchdog) / 1000));
        Serial.print("\t");
      }
      Serial.print("UTC Offset: ");
      Serial.print(UTCoffset);
      Serial.print("\tExec Time: ");
      Serial.print(currentExecutionTime / 1000);
      Serial.print(".");
      Serial.print(currentExecutionTime % 1000);
      if (currentExecutionTime % 1000 < 100) {
        Serial.print(0);
      }
      Serial.print("ms\tTZ: ");
      Serial.print(decodeTZswitch());

      Serial.print("\tTime Status :");
      switch (timeStatus()) {
        case timeNotSet:
          Serial.println("timeNotSet");
          break;
        case timeNeedsSync:
          Serial.println("timeNeedsSync");
          break;
        case timeSet:
          Serial.println("timeSet");
          break;
      }

      debugTimeMillis = millis();
    }
  #endif

  // Force a recalculate of UTC offset if time zone switches change
  uint8_t currentTZswitch = decodeTZswitch();
  if (lastTZswitch != currentTZswitch) {
    // delay 100ms, then re-read;  Shorting-style switch can cause misreads
    delay(100);

    #ifdef DEBUG
      currentTZswitch = decodeTZswitch();
      Serial.print(F("TZ Switch Changed! Now: "));
      Serial.println(currentTZswitch);
    #endif

    #ifndef DISABLE_PIEZO
      alertToneSingle();
    #endif

    #ifndef DISABLE_DISPLAY
      if(lastGoodSyncTime == 0) {
        matrix.clear();
        matrix.println(" -");
        int8_t tzSwitchState = 5 + currentTZswitch;
        matrix.writeDigitNum(3, tzSwitchState);
        matrix.writeDisplay();
        delay(1000);
        matrix.clear();
      }
    #endif

    if(lastGoodSyncTime != 0) {
      calculateUTCoffset();
    }

  }
  lastTZswitch = currentTZswitch;

  // Force a recalculate of UTC offset if DST switch changes
  if (DSTswitch.changed()) {
    #ifdef DEBUG
      Serial.println(F("DST Switch Changed!"));
    #endif

    if(lastGoodSyncTime != 0) {
      calculateUTCoffset();
    }

    #ifndef DISABLE_PIEZO
      alertToneSingle();
    #endif

    #ifndef DISABLE_DISPLAY
      matrix.clear();
      matrix.println("dSt ");
      matrix.writeDigitNum(4, DSTswitch.isPressed());
      matrix.writeDisplay();
      delay(1000);
      matrix.clear();
    #endif
  }
}
