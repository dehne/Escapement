/****
 *
 *   Part of the "Escapement" library for Arduino. Version 0.3
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
#define CALIBRATE		(2)
#define CALFINISH   	(3)
#define RUN				(4)
#define CALRTC			(5)

// Mode run length constants
#define TGT_SCALE		(512)				// Number of beats to run SCALE mode
#define TGT_SMOOTHING	(8192)				// Number of beats to run CALIBRATE mode for a given temperature

// Bendulum sensing and and pushing constants
#define SETTLE_TIME 	(250)				// Time to delay to let things settle before looking for voltage spike (ms)
#define DELAY_TIME		(5)					// Time by which to delay the start of the kick pulse (ms)
#define KICK_TIME		(10)				// Duration of the kick pulse (ms)
#define INIT_PEAK		(10)				// Initial guess at peakScale during SCALE mode. (Must be less than the actual)
#define MAX_PEAK		(1)					// Scale the peaks (using eeprom.peakScale) so they're no bigger than this


// Other constants
#define ADDRESS_TMP102	(0x48)				// Wire address of the TMP102 temperature sensor
#define NO_TEMP			(0xfc80)			// Value of readTemp() when no temp reading available (=-56 degrees C)
#define NO_CAL			(-1)				// Value of getTempIx() when tempwerature is out of calibration temperature range
#define ABS_ZERO		(-273.15)			// Value of getTemp() when no temp reading available
#define TEMP_MIN		(18)				// Minimum temp we calibrate with (degrees C)
#define TEMP_STEPS		(20)				// Number of 0.5C steps we keep track of: 2 * (TEMP_MAX - TEMP_MIN)

// EEPROM data structure definition
struct settings_t {							// Structure of data stored in EEPROM
	unsigned int id;						// ID tag to know whether data (probably) belongs to this sketch
	int bias;								// Empirically determined correction factor for the real-time clock in 0.1 s/day
	int peakScale;							// Empirically determined scaling factor for peak induced voltage
	long deltaUspb;							// Speed adjustment factor, μs per beat
	bool compensated;						// Set to true if the Escapement is temperature compensated, else false
	long uspb[TEMP_STEPS];					// Empirically determined, temp-dependent, μs per beat. <= 0 means 'not valid'
	int temp[TEMP_STEPS];					// Temp (degrees C * 256) at which uspb is valid.
};

#define SETTINGS_TAG (0x5346)               // If this is in eeprom.id, the contents of eeprom is (probably) ours

class Escapement {
private:
// Instance variables
	byte sensePin;							// Pin on which we sense the bendulum's passing
	byte kickPin;							// Pin on which we kick the bendulum as it passes
	int beatCounter;						// Counts beats during WARMSTART mode
	settings_t eeprom;						// Contents of EEPROM -- our persistent parameters
	int curSmoothing[TEMP_STEPS];			// Current temp-dependent smoothing factor
	int temp;								// Temperature (degrees C * 256)
	int lastTemp;							// Temperature (degrees C * 256) last time we calculated it
	float slope;							// Slope of least squares fit line
	float intercept;						// Intercept of least squares fit line
	boolean tick;							// Whether currently awaiting a tick or a tock
	long tickPeriod;						// Duration of last tick (μs)
	long tockPeriod;						// Duration of last tock (μs)
	unsigned long lastTime;					// Real time clock time (μs) last time through beat()
	unsigned long timeBeforeLast;			// Real time clock time (μs) time before last time through beat()
	long deltaT;							// Holds length of last beat (μs)
	int tempIx;								// Which "bucket" of temps we're dealing with currently
	byte runMode;							// Run mode -- SETTLING, CALIBRATING or RUNNING
// Utility methods
	void makeModel();						// Calculate least squares model from calibration data
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
	int getBeatCounter();					// Get the number of cycles in the current mode (except RUNNING)
	long getBias();							// Get Arduino clock correction in tenths of a second per day
	void setBias(long factor);				// Set Arduino clock correction in tenths of a second per day
	long incrBias(long factor);				// Increment Arduino clock correction by factor tenths of a second per day
	int getPeakScale();						// Get the peak induced voltage scaling factor
	void setPeakScale(int scaleFactor);		// Set the peak induced voltage scaling factor
	float getTemp();						// Get the current temperature in C; -1 if none
	boolean isTick();						// True if the last beat was a "tick" false if it was a "tock"
	boolean isTempComp();					// True if temperature compensated
	float getAvgBpm();						// Get the average beats per minute for the current temperature
	float getCurBpm();						// Get the current beats per minute as measured by the real-time clock
	float getBeatBpm();						// Get the duration last returned  by beat() converted to bpm
	float getDelta();						// Get the current ratio of tick length to tock length
	long getBeatDuration();					// Get the beat duration in μs
	long getBeatDelta();					// Get the beat duration delta in μs
	void setBeatDelta(long beatDelta);		// Set the beat duration delta in μs
	long incrBeatDelta(long incr);			// Increment beat duration delta so that clock runs faster by incr seconds per day
	byte getRunMode();						// Get the current run mode -- SETTLING, CALIBRATING or RUNNING
	void setRunMode(byte mode);				// Set the run mode
};

#endif