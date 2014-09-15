/****
 *
 *   Part of the "Escapement" library for Arduino. Version 0.20
 *
 *   Escapement.h Copyright 2014 by D. L. Ehnebuske 
 *   License terms: Creative Commons Attribution-ShareAlike 3.0 United States (CC BY-SA 3.0 US) 
 *                  See http://creativecommons.org/licenses/by-sa/3.0/us/ for specifics. 
 *
 *   See Escapement.cpp for description.
 *
 ****/
 
#ifndef Escapement_H
#define Escapement_H

#include <Wire.h>       // Use the Wire library to talk to the TMP102 temperature sensor: it's an I2C device
#include <avr/eeprom.h> // EEPROM read write library

#if ARDUINO >= 100
  #include <Arduino.h>  // Arduino 1.0
#else
  #include <WProgram.h> // Arduino 0022
#endif

// Compile-time options; uncomment to enable
// #define DEBUG

// Run mode constants
#define COLDSTART		(0)
#define WARMSTART		(1)
#define SCALE     		(2)
#define CALIBRATE		(3)
#define CALFINISH   	(4)
#define RUN				(5)

// Mode run length constants
#define TGT_SETTLE		(32)				// Number of beats to run SETTLE mode 
#define TGT_SCALE		(128)				// Number of cycles (ie., beats * 2) to run SCALE mode
#define TGT_SMOOTHING	(2048)				// Number of cycles to run CALIBRATE mode for a given temperature

// Bendulum sensing and and pushing constants
#define SETTLE_TIME 	(250)				// Time (ms) to delay to let things settle before looking for voltage spike
#define DELAY_TIME		(5)					// Time in ms by which to delay the start of the kick pulse
#define KICK_TIME		(20)				// Duration in ms of the kick pulse
#define INIT_PEAK		(10)				// Initial guess at peakScale during SCALE mode. (Must be less than the actual)
#define MAX_PEAK		(1)					// Scale the peaks (using eeprom.peakScale) so they're no bigger than this


// Other constants
#define ADDRESS_TMP102 (0x48)				// Wire address of the TMP102 temperature sensor
#define TEMP_MIN (13)						// Minimum temp we bother with (degrees C)
#define TEMP_MAX (38)                       // Maximum temp we bother with (degrees C)
#define TEMP_STEPS (50)						// Number of 0.5C steps we keep track of: 2 * (TEMP_MAX - TEMP_MIN)

// EEPROM data structure definition
struct settings_t {							// Structure of data stored in EEPROM
	int id;									// ID tag to know whether data (probably) belongs to this sketch
	int bias;								// Empirically determined correction factor for the real-time clock in 0.1 s/day
	int peakScale;							// Empirically determined scaling factor for peak induced voltage
	long deltaUspb;							// Speed adjustment factor, μs per beat
	bool compensated;						// Set to true if the Escapement is temperature compensated, else false
	long uspb[TEMP_STEPS];					// Empirically determined, temp-dependent, μs per beat.
	int curSmoothing[TEMP_STEPS];			// Current temp-dependent smoothing factor
};

#define SETTINGS_TAG (0xA1CF)               // If this is in eeprom.id, the contents of eeprom is (probably) ours

class Escapement {
private:
// Instance variables
	byte sensePin;							// Pin on which we sense the bendulum's passing
	byte kickPin;							// Pin on which we kick the bendulum as it passes
	int cycleCounter;						// Current cycle counter for SETTLING and SCALING modes
	settings_t eeprom;						// Contents of EEPROM -- our persistent parameters
	int curSmoothing[TEMP_STEPS];			// Current temp-dependent smoothing factor
	int tempIx;								// Temperature index (0 <= tempIx < TEMP_STEPS). -1 if none
	int lastTempIx;							// Temperature index last time we calculated it
	boolean tick;							// Whether currently awaiting a tick or a tock
	long tickAvg;							// Average duration of ticks (μs)
	long tockAvg;							// Average duration of tocks (μs)
	long tickPeriod;						// Duration of last tick (μs)
	long tockPeriod;						// Duration of last tock (μs)
	unsigned long lastTime;					// Clock time (μs) last time through beat()
	unsigned long timeBeforeLast;			// Clock time (μs) time before last time through beat()
	byte runMode;							// Run mode -- SETTLING, CALIBRATING or RUNNING
// Utility methods
	int getTempIx();						// Get the current temperature index
	boolean readEEPROM();					// Read EEPROM into instance variables
	void writeEEPROM();						// Write EEPROM from instance variables

public:
// Constructors
	Escapement(byte sensePin = A2, byte kickPin = 12);  // Escapement on specified sense and kick pins
// Operational methods
	void enable(byte initialMode = RUN);	// Do initialization of Escapement that needs to be done in startup()
	long beat();							// Do one beat (half a cycle) return  length of a beat in μs
	long cycle();							// Do one cycle (two beats) return length of a beat in μs
// Getters and setters
	int getCycleCounter();					// Get the number of cycles in the current mode (except RUNNING)
	int getBias();							// Get Arduino clock correction in tenths of a second per day
	void setBias(int factor);				// Set Arduino clock correction in tenths of a second per day
	int incrBias(int factor);				// Increment Arduino clock correction by factor tenths of a second per day
	int getPeakScale();						// Get the peak induced voltage scaling factor
	void setPeakScale(int scaleFactor);		// Set the peak induced voltage scaling factor
	float getTemp();						// Get the current temperature in C; -1 if none
	boolean isTick();						// True if the last beat was a "tick" false if it was a "tock"
	float getAvgBpm();						// Get the average beats per minute
	float getCurBpm();						// Get the current beats per minute
	float getDelta();						// Get the current ratio of tick length to tock length
	long getBeatDuration();					// Get the beat duration in μs
	long getBeatDelta();					// Get the beat duration delta in μs
	void setBeatDelta(long beatDelta);		// Set the beat duration delta in μs
	long incrBeatDelta(long incr);			// Increment beat duration delta so that clock runs faster by incr seconds per day
	int getRunMode();						// Get the current run mode -- SETTLING, CALIBRATING or RUNNING
	void setRunMode(byte mode);				// Set the run mode
};

#endif