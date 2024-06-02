#include "ES100.h"
#include <Wire.h>
#include <TimeLib.h>              // https://github.com/PaulStoffregen/Time
#include "Adafruit_LEDBackpack.h" // https://github.com/adafruit/Adafruit_LED_Backpack
#include <Bounce2.h>              // https://github.com/thomasfredericks/Bounce2

// Initialize library objects
ES100 es100;
Adafruit_7segment matrix = Adafruit_7segment();
Bounce2::Button hourButton = Bounce2::Button();
Bounce2::Button minuteButton = Bounce2::Button();
Bounce2::Button timeButton = Bounce2::Button();

#define DEBUG
#define DEBUG_CLOCK // Debug display related
//#define DEBUG_CONTINUOUS // Don't stop decoding for debug
//#define DISABLE_DISPLAY
const unsigned long baudrate = 115200; 

// Pin Assignments
const uint8_t es100_IRQ = 2;
const uint8_t es100_EN = LED_BUILTIN;
const uint8_t clockButtonHourPin = 7;
const uint8_t clockButtonMinutePin = 8;
const uint8_t clockButtonTimePin = 12;
const uint8_t clockButtonAlarmPin = 0;
const uint8_t clockSwitchAlarmOnPin = 0;
const uint8_t clockSwitchAlarm1Pin = 0;
const uint8_t clockSwitchAlarm2Pin = 0;

// Variables for manipulating a time syncronization
volatile unsigned long atomicMillis = 0;
volatile uint8_t interruptCount = 0;
uint8_t lastInterruptCount = 0;
time_t lastGoodSyncTime = 0;
time_t lastSyncAttempt = 0;

// Timestamps for faux multitasking
unsigned long secondsIndicatorMillis = 0;
unsigned long syncCheckIntervalMillis = 0;
unsigned long displayTimeMillis = 0;
unsigned long executionTime = 0;
unsigned long debugTimeMillis = 0;

// Flags to control reception
bool timeSyncInProgress = false;  // variable to determine if we are in receiving mode
bool triggerTimeSync = false;     // variable to trigger the reception

// ES100 Data Structures
ES100IRQstatus lastReadIRQStatus = {0}; // read every interrupt
ES100Status0 lastReadStatus0 = {0};     // read every interrupt
ES100Data validES100Data = {0};         // only gets updated when rx complete & rx okay are both true

// Display status indicators
bool indicatorSecondsSeparator = true; // start illuminated
bool indicatorPM = false;
bool indicatorAL1 = false;
bool indicatorAL2 = false;
const bool useTwentyFourHourTime = false; // true to use 24-hour clock

// Local time settings
const int8_t timezone = -5;   // America/New_York
int8_t UTCoffset = 0;
time_t localTime = 0;

// Useful constants
const uint16_t SECONDS_IN_HOUR = 3600;
const uint16_t SECONDS_IN_MINUTE = 60;


void atomic() {
  // Called procedure when we receive an interrupt from the ES100
  // Store milis of when the interrupt happened.  This is the second (time) boundary.
  // Full time frame reception happens in 134s, Tracking reception takes 24.5s
  atomicMillis = millis();
  interruptCount++;
  #ifdef DEBUG
    Serial.println("Interrupt Handler Called");
  #endif
}

void printES100DateTime(ES100DateTime dt) {
  Serial.print("received local time = 20");
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

void updateTime(ES100DateTime dt) {
  // Align time update with the atomic offset
  // Calculated number of ms needed to get to a multiple of 1000ms away from atomicMillis
  unsigned long secondOffset = millis() - atomicMillis;
  secondOffset -= 300; // my particular MCU takes about 300ms to think through this
  secondOffset %= 1000;
  delay(1000 - secondOffset);

  // Calculate the whole number of second elapsed since the interrupt was called
  unsigned long secondAdjust = secondOffset / 1000;

  setTime(dt.hour,dt.minute,dt.second + 1 + secondAdjust,dt.day,dt.month,dt.year);

  // This is also a good place to align blinking seconds indicator to actual seconds
  secondsIndicatorMillis += (atomicMillis % 1000);  //Offset to next whole second
}

void calculateUTCoffset(){
  // If DST begins today
  if (validES100Data.Status0.dstState == 0b10){
    if (hour() < 2 - timezone){
      UTCoffset = timezone;
    } else {
      UTCoffset = timezone + 1;
    }
  } else

  // If DST ends today
  if (validES100Data.Status0.dstState == 0b01){
    if (hour() < 2 - timezone - 1){
      UTCoffset = timezone + 1;
    } else {
      UTCoffset = timezone;
    }
  } else

  // If DST is in effect
  if (validES100Data.Status0.dstState == 0b11){
    UTCoffset = timezone + 1;
    
  } else
  { // aka 0b00: DST is not in effect
    UTCoffset = timezone;
  }
}

void setup() {
  Wire.begin();
  //pinMode(LED_BUILTIN, OUTPUT); // Use onboard LED for seconds
  
  #ifdef DEBUG
    Serial.begin(baudrate);
    Serial.println("");
    Serial.println("***  INIT  ***");
  #endif
  #ifndef DEBUG
    #ifdef DEBUG_CLOCK
      Serial.begin(baudrate);
      Serial.println("");
      Serial.println("***  INIT  ***");
    #endif
  #endif

  // Initialize WWVB reciever
  es100.begin(es100_IRQ, es100_EN);
  attachInterrupt(digitalPinToInterrupt(es100_IRQ), atomic, FALLING);

  // Setup button debouncing for hardware buttons
  const uint8_t debounceInterval = 5;
  hourButton.attach(clockButtonHourPin, INPUT_PULLUP);
  minuteButton.attach(clockButtonMinutePin, INPUT_PULLUP);
  timeButton.attach(clockButtonTimePin, INPUT_PULLUP);
  hourButton.interval(debounceInterval);
  minuteButton.interval(debounceInterval);
  timeButton.interval(debounceInterval);
  hourButton.setPressedState(LOW);
  minuteButton.setPressedState(LOW);
  timeButton.setPressedState(LOW);

  #ifdef DEBUG
    es100.enable();
    uint8_t deviceID = es100.getDeviceID();
    Serial.print("Device ID: 0x");
    Serial.println(deviceID, HEX);
    Serial.println("finished setup");
  #endif

  #ifndef DISABLE_DISPLAY
    // Begin display on i2c address 0x70
    matrix.begin(0x70);
    matrix.setBrightness(1); // 0 to 15
    matrix.println("BOOB");
    matrix.writeDisplay();
    delay(5000);
  #endif
}

void loop() {
  // If we are not currently receiving a 1-minute frame and reception is triggered, enable ES100 and start reception
  if (!timeSyncInProgress && triggerTimeSync) {
    es100.enable();

    #ifdef DEBUG
      Serial.print("Last Received on Antenna ");
      if(!validES100Data.Status0.antenna){
        Serial.println("1");
      } else {
        Serial.println("2");
      }
      Serial.println(es100.startRx(validES100Data.Status0.antenna));
    #endif
    #ifndef DEBUG
      // Use most recently known good antenna.  Use "valid" data
      es100.startRx(validES100Data.Status0.antenna);
    #endif
    
    lastSyncAttempt = now();
    timeSyncInProgress = true;
    triggerTimeSync = false;

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
      Serial.print("ES100 Interrupt received... ");
    #endif

    lastSyncAttempt = now();

    // If we received a valid decode
    if (lastReadIRQStatus.rxComplete && !lastReadIRQStatus.cycleComplete && lastReadStatus0.rxOk) { // IRQStatus = 0x01 & Status0 bit 0
      
      // Read remaining data from es100 and calculate time zone & DST offsets
      validES100Data.DateTimeUTC = es100.getUTCdateTime();
      validES100Data.Status0 = lastReadStatus0;

	    #ifdef DEBUG
        Serial.println("IRQ: Successful Reception");
        Serial.print("Atomic millis = ");
        Serial.println(atomicMillis);

        Serial.print("Time Library: ");
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

      updateTime(validES100Data.DateTimeUTC);
      lastGoodSyncTime = now();
      lastSyncAttempt = lastGoodSyncTime;

      calculateUTCoffset();

      // Flag that we aren't recieving anymore.  Only happens if we get a good decode.
      timeSyncInProgress = false;

      #ifndef DEBUG_CONTINUOUS
        #ifdef DEBUG
          Serial.println("Disabling es100 after good Rx");
        #endif

        // Stop reception after a good decode
        Serial.print("StopRx exit code: ");
        Serial.println(es100.stopRx());
        es100.disable();
      #endif

      #ifdef DEBUG
        Serial.print("status0.rxOk = 0b");
        Serial.println(lastReadStatus0.rxOk, BIN);
        Serial.print("status0.antenna = 0b");
        Serial.println(lastReadStatus0.antenna, BIN);
        Serial.print("status0.leapSecond = 0b");
        Serial.println(lastReadStatus0.leapSecond, BIN);
        Serial.print("status0.dstState = 0b");
        Serial.println(lastReadStatus0.dstState, BIN);
        Serial.print("status0.tracking = 0b");
        Serial.println(lastReadStatus0.tracking, BIN);
      #endif

      #ifndef DISABLE_DISPLAY
        matrix.println("SYNC");
        matrix.writeDisplay();
        delay(5000);
        matrix.clear();
      #endif
    }
    else if (!lastReadIRQStatus.rxComplete && lastReadIRQStatus.cycleComplete){ // IRQStatus = 0x04
      #ifdef DEBUG
        Serial.println("IRQ: Unsuccessful Reception");
      #endif
    }
    else {
      #ifdef DEBUG
        Serial.println("IRQ: Bad Data");
      #endif
    }

    lastInterruptCount = interruptCount;
  }


  // Receive the current time on an schedule
  // Also evaluate if DST is changing
  if (syncCheckIntervalMillis + 10000 < millis()) {

    // Are we currently recieving the time?
    if (!timeSyncInProgress) {

      // If we aren't already syncing (outer loop)
      // and the time isn't set, request a reception
      if(timeStatus() == timeNotSet){
        triggerTimeSync = true;
        #ifdef DEBUG
          Serial.println("Time not set, requesting sync");
        #endif
      }

      // If we aren't already syncing (outer loop)
      // and the time is set (else)
      // and if we havn't had a successful sync this hour today
      else if(!(hour(lastGoodSyncTime) == hour() && day(lastGoodSyncTime) == day()))
      {
        triggerTimeSync = true;
        #ifdef DEBUG
          Serial.println("New hour, requesting sync");
        #endif
      }
      else {
        // do nothing
        #ifdef DEBUG
          Serial.println("Sync not required");
        #endif
      }
    }

    // If we aren't already syncing (outer loop)
    // and the time is set (else)
    // and it's daytime or we had a good sync this hour today
    else {
      // Already recieving the time, do nothing
      #ifdef DEBUG
        Serial.print("Time sync in progress...");
      #endif
    }

    #ifndef DEBUG_CLOCK
      Serial.print(".");
    #endif

    if(timeStatus() != timeNotSet){
      calculateUTCoffset();
    }

    syncCheckIntervalMillis = millis();
  }
  

  // Every 500ms, change the state of various status LEDs
  if (secondsIndicatorMillis + 500 < millis()) {
    
    // Blink display separator every second
    indicatorSecondsSeparator = !indicatorSecondsSeparator;
    // Sync with onboard led for debug
    // digitalWrite(LED_BUILTIN, indicatorSecondsSeparator);

    // If time is unset or expired, twinkle the alarm lights
    if((timeStatus() == timeNotSet) || (now() - lastGoodSyncTime) > (time_t)SECONDS_IN_HOUR * 6){ //thinking 6 hours sounds right
      indicatorAL1 = !indicatorAL1;
      indicatorAL2 = !indicatorAL1;
    } 
    // Otherwise turn them off
    else {
      indicatorAL1 = false;
      indicatorAL2 = false;
    }

    secondsIndicatorMillis = millis();
  }


  //Send updates to the display driver
  if(displayTimeMillis + 50 < millis()){ // make this faster later
    localTime = now();
    localTime += (time_t)UTCoffset * SECONDS_IN_HOUR;

    indicatorPM = isPM(localTime);

    #ifndef DISABLE_DISPLAY
    if(useTwentyFourHourTime){
      matrix.writeDigitNum(0, hour(localTime) / 10);
      matrix.writeDigitNum(1, hour(localTime) % 10);
    }
    else {
      if(hourFormat12(localTime) < 10){ // in 12-hour time first digit is 1 or blank
        matrix.writeDigitRaw(0, B00000000);
      } else {
        matrix.writeDigitNum(0, 1);
      }
      matrix.writeDigitNum(1, hourFormat12(localTime) % 10, indicatorAL1);
    }
    // Position 2 is colon dots
    matrix.writeDigitNum(3, minute(localTime) / 10, indicatorAL2);
    matrix.writeDigitNum(4, minute(localTime) % 10, indicatorPM);
    matrix.drawColon(indicatorSecondsSeparator);
    matrix.writeDisplay();
    #endif

    displayTimeMillis = millis();
  }

  #ifdef DEBUG_CLOCK
    if(debugTimeMillis + 200 < millis()){
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
      Serial.print("UTC Offset: ");
      Serial.print(UTCoffset);
      Serial.print("\tExec Time: ");
      Serial.print((micros() - executionTime)/1000);
      Serial.print(".");
      Serial.print((micros() - executionTime)%1000);
      Serial.println("ms");

      debugTimeMillis = millis();
    }
  #endif


  hourButton.update();
  minuteButton.update();
  timeButton.update();

  // Advance time with button presses
  if(timeButton.isPressed() && hourButton.pressed()){
    #ifdef DEBUG
      Serial.println("HOUR Pressed\t");
    #endif
  }

  if(timeButton.isPressed() && minuteButton.pressed()){
    adjustTime(SECONDS_IN_MINUTE);
    #ifdef DEBUG
      Serial.println("MINUTE Pressed\t");
      #endif
  }

  #ifdef DEBUG
    // Keep track of execution time
    executionTime = micros();
  #endif
}
