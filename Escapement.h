/****
 *
 *   Part of the "Escapement" library for Arduino. Version 0.8
 *
 *   Escapement.h Copyright 2014-2015 by D. L. Ehnebuske 
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
#define CALSTART		(2)
#define CALIBRATE		(3)
#define CALFINISH   	(4)
#define RUN				(5)
#define CALRTC			(6)

// Mode run length constants
#define TGT_WARMUP		(1024)				// Number of beats to run in WARMSTART mode
#define TGT_SMOOTHING	(8192)				// Number of beats to run CALIBRATE mode for a given temperature

// Bendulum sensing and and pushing constants
#define SETTLE_TIME 	(250)				// Time to delay to let things settle before looking for voltage spike (ms)
#define N_SAMPLES		(35)				// Number of samples to average in reading the coil voltage (<= 64 so no o'flow)
#define DELAY_TIME		(1)					// Time by which to delay the start of the kick pulse (ms)
#define KICK_TIME		(9)					// Duration of the kick pulse (ms). Try 5-10 for a pendulum, 20-30 for a bendulum
#define NOISE_SIZE		(10)				// Assumed size of the noise in coil readings

// Other constants
#define ADDRESS_TMP102	(0x48)				// Wire address of the TMP102 temperature sensor
#define NO_TEMP			(0xfc80)			// Value of readTemp() when no temp reading available (=-56 degrees C)
#define NO_CAL			(-1)				// Value of getTempIx() when temperature is out of calibration temperature range
#define ABS_ZERO		(-273.15)			// Value of getTemp() when no temp reading available
#define TEMP_MIN		(18)				// Minimum temp we calibrate with (degrees C)
#define TEMP_STEPS		(16)				// Number of 0.5C steps we keep track of

// EEPROM data structure definition
struct settings_t {							// Structure of data stored in EEPROM
	unsigned int id;						// ID tag to know whether data (probably) belongs to this sketch
	int bias;								// Empirically determined correction factor for the real-time clock in 0.1 s/day
	long deltaUspb;							// Speed adjustment factor, μs per beat
	bool compensated;						// Set to true if the Escapement is temperature compensated, else false
	long uspb[TEMP_STEPS];					// Empirically determined, temp-dependent, μs per beat.
	int curSmoothing[TEMP_STEPS];			// Current temp-dependent smoothing factor
};

#define SETTINGS_TAG (0x3db3)               // If this is in eeprom.id, the contents of eeprom is (probably) ours

class Escapement {
private:
// Instance variables
	byte sensePin;							// Pin on which we sense the bendulum's passing
	byte kickPin;							// Pin on which we kick the bendulum as it passes
	int beatCounter;						// In WARMSTART mode, the number of beats since peakScale changed
	settings_t eeprom;						// Contents of EEPROM -- our persistent parameters
	int temp;								// Temperature (degrees C * 256)
	int lastTemp;							// Temperature (degrees C * 256) last time we calculated it
	boolean tick;							// Whether currently awaiting a tick or a tock
	long tickLength;						// Duration of last tick (μs)
	long tockLength;						// Duration of last tock (μs)
	unsigned long lastTime;					// Real time clock time (μs) last time through beat()
	unsigned long timeBeforeLast;			// Real time clock time (μs) time before last time through beat()
	long deltaT;							// Holds length of last beat (μs)
	int tempIx;								// Which "bucket" of temps we're dealing with currently
	byte runMode;							// Run mode -- SETTLING, CALIBRATING or RUNNING
// Utility methods
	int readTemp();							// Read TMP102, return temp in degrees C * 256 or NO_TEMP if unable to read
	int getTempIx(int t);					// Get the temperature index for temperature t, t in degrees C * 256
	boolean readEEPROM();					// Read EEPROM into instance variables
	void writeEEPROM();						// Write EEPROM from instance variables

public:
// Constructors
	Escapement(byte sensePin = A2, byte kickPin = 12);  // Escapement on specified sense and kick pins
// Operational methods
	void enable(byte initialMode = RUN);	// Do initialization of Escapement that needs to be done in sketch startup()
	long beat();							// Do one beat (half a cycle) return  length of a beat in μs
// Getters and setters
	String getSmoothing();					// Get the current smoothing information
	long getBias();							// Get Arduino clock correction in tenths of a second per day
	void setBias(long factor);				// Set Arduino clock correction in tenths of a second per day
	long incrBias(long factor);				// Increment Arduino clock correction by factor tenths of a second per day
	float getTemp();						// Get the current temperature in C; -1 if none
	boolean isTick();						// True if the last beat was a "tick" false if it was a "tock"
	boolean isTempComp();					// True if temperature compensated
	float getAvgBpm();						// Get the average beats per minute for the current temperature
	float getCurBpm();						// Get the current beats per minute as measured by the real-time clock
	float getBeatBpm();						// Get the duration last returned  by beat() converted to bpm
	float getDelta();						// Get the current ratio of tick length to tock length
	int getBeatCounter();					// Get the number of beats spent in WARMSTART mode
	long getBeatDuration();					// Get the beat duration in μs
	long getBeatDelta();					// Get the beat duration delta in μs
	void setBeatDelta(long beatDelta);		// Set the beat duration delta in μs
	long incrBeatDelta(long incr);			// Increment beat duration delta so that clock runs faster by incr seconds per day
	byte getRunMode();						// Get the current run mode -- SETTLING, CALIBRATING or RUNNING
	void setRunMode(byte mode);				// Set the run mode
};

#endif