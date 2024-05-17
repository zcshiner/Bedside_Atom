#include "ES100.h"
#include <Wire.h>
#include <TimeLib.h>

const uint8_t es100_IRQ = 2;
const uint8_t es100_EN = 4;
ES100 es100;

volatile unsigned long atomicMillis = 0;
unsigned long lastSyncMillis = 0;

volatile uint8_t interruptCnt = 0;
uint8_t lastinterruptCnt = 0;
unsigned long lastMillis = 0;

bool receiving = false;        // variable to determine if we are in receiving mode
bool trigger = true;           // variable to trigger the reception
bool continous = true;        // variable to tell the system to continously receive atomic time, if not it will happen every night at midnight
bool validdecode = false;      // variable to rapidly know if the system had a valid decode done lately

void atomic() {
  // Called procedure when we receive an interrupt from the ES100
  // Got a interrupt and store the currect millis for future use if we have a valid decode
  atomicMillis = millis();
  interruptCnt++;
}

void printUTCtime(ES100DateTime dt) {

  Serial.print("received UTC time = 20");
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
  setTime(dt.hour,dt.minute,dt.second,dt.day,dt.month,dt.year);
}

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println("");
  Serial.println("***  INIT  ***");

  es100.begin(es100_IRQ, es100_EN);

  /*  Time zone and DST setting:
   *  The value for es100.timezone can be positive or negative
   *  For example: es100.timezone = -5
   *  The value for es100.DSTenable can be true or false
   *  For example: es100.DSTenabled = true
   *  If es100.DSTenabled is false, then no DST correction is performed
   *  even if DST is in effect.
   */
  es100.timezone = -5;  //America/New_York
  es100.DSTenabled = false;


  attachInterrupt(digitalPinToInterrupt(es100_IRQ), atomic, FALLING);
}

void loop() {

  if (!receiving && trigger) {
    interruptCnt = 0;
    
    es100.enable();
    es100.startRx();
    
    receiving = true;
    trigger = false;

    /* Important to set the interrupt counter AFTER the startRx because the es100 
     * confirms that the rx has started by triggering the interrupt. 
     * We can't disable interrupts because the wire library will stop working
     * so we initialize the counters after we start so we can ignore the first false 
     * trigger
     */
    lastinterruptCnt = 0;
    interruptCnt = 0;
  }


  if (lastinterruptCnt < interruptCnt) {
    Serial.print("ES100 Interrupt received... ");
  
    if (es100.getIRQStatus() == 0x01 && es100.getRxOk() == 0x01) {
      validdecode = true;
      Serial.println("Valid decode");
      // Update lastSyncMillis for lcd display
      lastSyncMillis = millis();
      // We received a valid decode
      ES100DateTime d = es100.getDateTime();

      // Print out recieved time
      printUTCtime(d);

      // Get everything before disabling the chip.
      ES100Status0 status0 = es100.getStatus0();
      ES100NextDst nextDst = es100.getNextDst();

      updateTime(d);
      Serial.print("Time before TZ adjustment: ");
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

      receiving = false;
  
      if (!continous) {
        es100.stopRx();
        es100.disable();
      }
      else{
        trigger = true;
      }

      /* DEBUG */
      Serial.print("status0.rxOk = 0b");
      Serial.println(status0.rxOk, BIN);
      Serial.print("status0.antenna = 0b");
      Serial.println(status0.antenna, BIN);
      Serial.print("status0.leapSecond = 0b");
      Serial.println(status0.leapSecond, BIN);
      Serial.print("status0.dstState = 0b");
      Serial.println(status0.dstState, BIN);
      Serial.print("status0.tracking = 0b");
      Serial.println(status0.tracking, BIN);
      /* END DEBUG */
    }
    else {
      Serial.println("Invalid decode");
    }
    lastinterruptCnt = interruptCnt;
  }
 

  if (lastMillis + 1000 < millis()) {

    lastMillis = millis();
    Serial.print(".");
  }
}
