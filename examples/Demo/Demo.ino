/****
 *
 *   Demonstration sketch for the "Escapement" library. Version 1.0
 *
 *   Copyright 2014 by D. L. Ehnebuske 
 *   License terms: Creative Commons Attribution-ShareAlike 3.0 United States (CC BY-SA 3.0 US) 
 *                  See http://creativecommons.org/licenses/by-sa/3.0/us/ for specifics. 
 *
 *   Demonstration of the Escapement library. Very basic. See the library code for documentation and details.
 *
 ****/
 
#include <Escapement.h>                                // Import the header so we have access to the library
#include <Wire.h>                                      // Needed to fool the IDE into including this; it's needed by Escapement
#include <avr/eeprom.h>                                // Ditto

Escapement e;                                          // Instantiate a bendulum object that senses on A2 and
                                                       //   kicks on pin D12
/*
 *   Setup routine called once at power-on and at reset
 */
void setup() {
  Serial.begin(9600);                                  // Start the serial monitor
  Serial.println("Escapement Example v 1.0");          // Say who's talking on it
  e.enable();                                          // Start the Escapement
}

/*
 *   Loop routine called over and over so long as the Arduino is running
 */
void loop() {
  e.beat();                                            // Have the bendulum do one pass over the coil
  if (e.isTick()) {                                    // If it passed in the "tick" direction (as oppoed to "tock")
    switch (e.getRunMode()) {                          //   Do things based on the current "run mode"
      case COLDSTART:                                  //   When cold starting
        Serial.print(F("Cold started"));               //     Just say that's what we're doing
        break;
      case WARMSTART:                                  //   When settling in
        Serial.print(F("Settling. Count "));           //     Say that we're settling in and how far along we've gotten
        Serial.print(e.getCycleCounter());
        Serial.print(F(", delta "));                   //     Display the ratio of tick duration to tock duration
        Serial.print(e.getDelta(), 4);
        Serial.print(F(", current bpm "));             //       and the measured beats per minute
        Serial.print(e.getCurBpm(), 4);
        break;
      case SCALE:                                      //   When scaling for peak value
        Serial.print(F("Scaling. Count "));            //    Say that we're scaling and how far along we've gotten
        Serial.print(e.getCycleCounter());
        Serial.print(F(", delta "));                   //    Display the ratio of tick duration to tock duration
        Serial.print(e.getDelta(), 4);
        Serial.print(F(", current bpm "));             //      the measured beats per minute
        Serial.print(e.getCurBpm(), 4);
        Serial.print(F(", peak scaling "));            //      and the current peak scaling
        Serial.print(e.getPeakScale());
        break;
      case CALIBRATE:                                  //   When calibrating
        Serial.print(F("Calibrating. Count "));        //     Say we're calibrating, how much smoothing We've been
        Serial.print(e.getCycleCounter());             //       able to do so far, how symmetrical the "ticks" and
        Serial.print(F(", delta "));                   //       "tocks" are currently, how many beats per minute
        Serial.print(e.getDelta(), 4);                 //       on average we're seeing so far
        Serial.print(F(", average bpm "));
        Serial.print(e.getAvgBpm(), 4);
        Serial.print(F(", temp "));                    //       and the temperature
        Serial.print(e.getTemp());
        Serial.print(F(" C"));
        break;
      case CALFINISH:                                  //   When finished calibrating
        Serial.print(F("Finished calibrating. Temp: "));
        Serial.print(e.getTemp());
        Serial.print(F(" C, current bpm "));
        Serial.print(e.getCurBpm(), 4);
        Serial.print(F(", peak scaling "));
        Serial.print(e.getPeakScale());
        break;
      case RUN:                                        //   When running
        Serial.print(F("Running. Cal bpm "));          //     Say that we're running along normally and what the
        Serial.print(e.getAvgBpm(), 4);                //       calibrated beats per minute is and what the currently
        Serial.print(F(", current bpm "));             //       measured bpm is and the temperature reading
        Serial.print(e.getCurBpm(), 4);
        Serial.print(F(", temp "));
        Serial.print(e.getTemp());
        Serial.print(F(" C"));
        break;
    }
    Serial.println(F("."));
  }
}
