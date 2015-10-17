/****
 *
 *   Part of the "Escapement" library for Arduino. Version 0.6
 *
 *   Escapement.cpp Copyright 2014-2015 by D. L. Ehnebuske 
 *   License terms: Creative Commons Attribution-ShareAlike 3.0 United States (CC BY-SA 3.0 US) 
 *                  See http://creativecommons.org/licenses/by-sa/3.0/us/ for specifics. 
 *
 *   In pendulum clocks, the escapement is the mechanism that does two things. First, it drives the gear-works one 
 *   step forward for each tick or tock of the pendulum. Second, it gives the pendulum a tiny push with each tick or 
 *   tock to keep the pendulum going. The clock's gear-works are designed to move the clock's hands to display the 
 *   time. The gear ratios in the clock mechanism are such that the fixed duration of time that passes between a tick 
 *   and a tock (and between a tock and a tick) is exactly correct to have the clocks hands mark the passing of time.
 *
 *   This Escapement library implements the electronic equivalent of an mechanical escapement and gear-works for a 
 *   properly equipped pendulum or bendulum. (A "bendulum" is a long, slender, springy piece of metal, held 
 *   vertically, whose lower end is fixed and whose upper end has a weight that is free to move back and forth -- a 
 *   sort of upside-down pendulum.) By a "properly equipped" pendulum or bendulum I mean that it terminates in a 
 *   strong magnet that moves across a many-turn coil of fine wire positioned at the center of its swing. 
 *
 *   The basic idea behind the Escapement is that the motion of the magnet is detected as it passes over the coil by 
 *   the current the magnet induces. Just after the magnet passes, the Escapement object produces a pulse of current 
 *   through the coil. This induces a magnetic field in the coil. The field gives the magnet a small push, keeping the 
 *   pendulum or bendulum going. Each time this happens, the Escapement object returns the stored value for the number 
 *   of microseconds per beat of the pendulum or bendulum it is driving. The returned value can be used to drive a 
 *   time-of-day display.
 *
 *   Even carefully designed and built mechanical clocks don't keep perfect time. A major reason for this is that
 *   various properties of the materials from which they are built depend on temperature. For example, the length of 
 *   the pendulum in a pendulum clock changes slightly with temperature, slightly changing its period. Similarly, in 
 *   a clock that uses a balance wheel, temperature changes change the spring constant of the hairspring making it 
 *   slightly more or less springy, which slightly changes the balance wheel's ticking rate. Over the years, mechanical
 *   clock designers have invented many ways of compensating for such temperature-induced changes. 
 *
 *   The pendulum or bendulum the Escapement object drives is subject to the same sorts of temperature effects. To
 *   compensate for them, the Escapement object implements optional temperature compensation using the SparkFun TMP102
 *   temperature sensor.
 *
 *   O P E R A T I O N
 *   =================
 *
 *   Typically, an Escapement object is instantiated as a global in an Arduino sketch. It is then initialized using 
 *   the enable() method in the sketch's setup() function). Finally, the sketch's loop() function repeatedly invokes 
 *   the beat() method. When the magnet passes the coil, beat() returns the number of microseconds that have passed 
 *   since the magnet passed the last time. In this way, the Escapement can be used to drive a time-of-day clock 
 *   display. The Escapement object is able to turn pendulum or bendulum ticks and tocks into microseconds because it 
 *   adjusts to the period of whatever pendulum or bendulum it is driving. It does this through an automatic, 
 *   optionally temperature-compensated calibration process.
 *
 *   The beat() method has several operational modes that, together operate as a state machine to calibrate and 
 *   temperature-compensate the ticking pendulum or bendulum. It works like this.
 *
 *   When the Escapement is started by the enable() method, it attempts to read its persistent parameters from EEPROM.
 *   Depending on what it finds, the Escapement enters one of three run modes.
 *
 *   WARMSTART is entered if good calibration data is read and temperature sensing hasn't been added or removed from 
 *   the configuration since the data was written. With a WARMSTART, the Escapement runs using the corrected Arduino 
 *   clock as a time reference for TGT_WARMUP beats. It then transitions to RUN mode.
 *
 *   COLDSTART is entered if good calibration data can't be read or if it is forced by enable()'s initialMode parameter.
 *   In this mode all of the calibration data, including the Arduino clock's correction data is reset and then WARMSTART 
 *   mode is entered.
 *
 *   CALSTART is entered if good calibration data is read but temperature sensing has been added or removed. In this 
 *   mode, the calibration information, but not the Arduino clock's correction data is reset and then WARMSTART mode is 
 *   entered.
 *
 *   Once entered, RUN mode persists so long as the calibration for the current temperature has been completed. If it 
 *   has not, the Escapement object switches to CALIBRATE mode. During RUN mode, beat() returns the linear 
 *   interpolation of the two calibration points nearest the current temperature.
 *
 *   CALIBRATE mode lasts for TGT_SMOOTHING beats in each of TEMP_STEPS half-degree C "buckets" of temperature. 
 *   Buckets are indexed by the variable tempIX. The lowest temperature at which calibration is done is MIN_TMP. This 
 *   first bucket, like all the buckets runs for a half degree C. The highest temperature bucket starts at MIN_TEMP + 
 *   (TEMP_STEPS - 1) / 2. Calibration information is not collected for temperatures outside this range. If 
 *   temperature sensing is not available, temperature compensation cannot be done and only the first bucket is used.
 *
 *   If, during calibration, the temperature changes enough to fall into a different "bucket" before TGT_SMOOTHING 
 *   beats pass, the progress made in calibrating for the old temperature bucket is maintained in eeprom.uspb[] and 
 *   eeprom.curSmoothing[], and calibration at the new temperature bucket is started or, if partial calibration for at 
 *   that temperature bucket has been done earlier, resumed. During CALIBRATE mode, the duration the beats is measured 
 *   and used to adjust the calibration values for the tempIx and the tempIx + 1 buckets When calibration for either 
 *   the tempIx or  the tempIx + 1 buckets completes TGT_SMOOTHING beats, the Escapement object stores the contents of 
 *   eeprom -- Escapement's persistent parameters -- in the Arduino's EEPROM and switches to CALFINISH mode. During 
 *   CALIBRATE mode, the duration beat() returns is measured using the (corrected) Arduino real-time clock.
 *
 *   The CALFINISH mode does two things. First, it serves as notice to the using sketch that calibration for the 
 *   current temperature bucket is complete. And second, it uses the currently accumulated calibration information to
 *   least-squares fit model of the length of a beat as a function of temperature. The Escapement object then switches 
 *   to RUN mode. In CALFINISH mode, the duration beat() returns is measured using the (corrected) Arduino real-time 
 *   clock.
 *
 *   The net effect is that the Escapement object automatically characterizes the bendulum or pendulum it is driving 
 *   by determining the average duration of the beats at half-degree intervals as it encounters different temperatures. 
 *   Once the calibration is done for a given temperature range, it assumes the bendulum or pendulum behaves linearly 
 *   with temperature between calibration points.
 *
 *   This would work nearly perfectly except that, as hinted at above, the real-time clock in most Arduinos is stable 
 *   but not too accurate (it's a ceramic resonator, not a crystal). That is, real-time clock ticks are essentially 
 *   equal to one another in duration but their durations are not exactly the number of microseconds they should be. 
 *   To correct for this, we use a correction factor, eeprom.bias. The value of eeprom.bias is the number of tenths of 
 *   a second per day by which the real-time clock in the Arduino must be compensated in order for it to be accurate. 
 *   Positive eeprom.bias means the real-time clock's "microseconds" are shorter than real microseconds. Since the 
 *   real-time clock is the standard that's used for calibration, automatic calibration won't work well unless 
 *   eeprom.bias is set correctly. To help with setting eeprom.bias Escapement has one more mode: CALRTC.
 *
 *   In CALRTC mode, the duration beat() returns is the value measured by the (corrected) Arduino real-time clock. The  
 *   CALRTC mode persists until changed by setRunMode(). Because the value returned is the real-time-clock-measured 
 *   value, in this mode the Escapement is effectively driven by the real-time clock, not the pendulum or bendulum, 
 *   despite its ticking and tocking. The idea is to use the mode to adjust the real-time clock calibration (via the 
 *   setBias() and incrBias() methods) so that the clock driven by the Escapement keeps perfect time. Once the real-
 *   time clock is calibrated, entering CALIBRATE mode should produce a good automatic calibration.
 *
 ****/

#include "Escapement.h"

// Class Escapement

/*
 *
 * Constructors
 *
 */

// Escapement on specified sense and kick pins
Escapement::Escapement(byte sPin, byte kPin){
	sensePin = sPin;						// Pin on which we sense the bendulum's passing
	kickPin = kPin;							// Pin on which we kick the bendulum as it passes
}

/*
 *
 * Operational methods
 *
 */

// Enable the Escapement -- do the initialization that needs to be done in setup()
void Escapement::enable(byte initialMode) {
	analogReference(EXTERNAL);				// We have an external reference, a 47k+47k voltage divider between
											//   3.3V and ground
	pinMode(sensePin, INPUT);				// Set sense pin to INPUT since we read from it
	pinMode(kickPin, INPUT);				// Put the kick pin in INPUT (high impedance) mode so that the
											//   induced current doesn't flow to ground
	Wire.begin();							// Prep to talk to the TMP102 temperature sensor
	beatCounter = 1;						// Initialize beatCounter
	lastTemp = NO_TEMP;						// No lastTemp
	temp = readTemp();						//   but there is a temp
	tempIx = getTempIx(temp);
	tick = true;							// Whether currently awaiting a tick or a tock
	tickLength = tockLength = 0;			// Length of last tick and tock periods (μs)
	timeBeforeLast = lastTime = 0;			// Real time clock time (μs) last time through beat() (and time before that)
	deltaT = 0;								// Length of last beat (μs)

	if (initialMode != COLDSTART) {			// If forced cold start isn't requested
		if (readEEPROM()) {					//   Try getting info from EEPROM. If that works
			if ((temp != NO_TEMP) == eeprom.compensated) {
											//      If temp compensation mode matches
				setRunMode(WARMSTART);		//		  Start in WARMSTART mode
			} else {
				setRunMode(CALSTART);		//      Otherwise start in CALSTART mode
			}
		} else {							//   Else (invalid data in EEPROM)
			setRunMode(COLDSTART);			//     Cold start
		}
	} else {								//  Else (forced cold start)
		setRunMode(COLDSTART);				//    Cold start
	}
}
 
// Do one beat return length of a beat in μs
long Escapement::beat(){
	unsigned int currCoil = 0;					// The value read from coilPin. To get the value in volts, multiply by 
												//   AREF/1024, where AREF is the voltage on that pin. Here, AREF is set 
												//   by a 1:1 voltage divider between the 3.3V pin and Gnd, so each count
												//   is about 1.6 mV, más o menos. The exact value doesn't really matter 
												//   since we're looking for a peak above noise.
	unsigned int pastCoil = 0;					// The previous value of currCoil
	unsigned long topTime = 0;					// Clock time (μs) of passing of the magnet
	
	// watch for passing magnet
	delay(SETTLE_TIME);							// Wait for things to calm down
	do {										// Wait for the voltage to fall below the noise floor
		currCoil = analogRead(sensePin);
	} while (currCoil > NOISE_SIZE);
	currCoil /= NOISE_SIZE;
 	do {										// Wait for the magnet to pass over coil,
		pastCoil = currCoil;					//   Indicated by the voltage induced in the coil beginning to fall
		currCoil = analogRead(sensePin);
		for (int i = 1; i < N_SAMPLES; i++) {
			currCoil += analogRead(sensePin);
		}
		currCoil /= N_SAMPLES * NOISE_SIZE;
	} while (currCoil >= pastCoil);
	topTime= micros();							// Remember when magnet went by
	
	// Kick the magnet to keep it going
	pinMode(kickPin, OUTPUT);					// Prepare kick pin for output
	delay(DELAY_TIME);							// Wait desired time before pin turn-on
	digitalWrite(kickPin, HIGH);				// Turn kick pin on
	delay(KICK_TIME);							// Wait for duration of pulse
	digitalWrite(kickPin, LOW);					// Turn it off
	pinMode(kickPin, INPUT);					// Put kick pin in high impedance mode

	// Determine the length of time between beats in μs
	if (lastTime == 0) {						// if first time through
		lastTime = topTime;						//   Remember when we last saw the magnet go by
		return 0;								//   Return 0 -- no interval between beats yet!
	}
	deltaT = topTime - lastTime;				// Assume microseconds per beat will be whatever we measured for this beat
												// plus the (rounded) Arduino clock correction
	deltaT += ((eeprom.bias * deltaT) + 432000) / 864000;
	if (deltaT > 5000000) {						// If the measured beat is more than 5 seconds long
		return deltaT = 0;						//   it can't be real -- just ignore it and return
	}
	if (tick) {									//   If tick
		tickLength = deltaT;					//     Set tickLength to beat length
	} else {									//   else (tock)
		tockLength = deltaT;					//     Set tockLength to beat length
	}
	if (temp != NO_TEMP) {						// If running temperature compensated
		lastTemp = temp;						//   Remember old temp so we can tell if it changed
		temp = readTemp();						//   Update the temperature
		tempIx = getTempIx(temp);				//   And figure out which "bucket" of temperatures it's in
	}

	switch (runMode) {
		case COLDSTART:							// When cold starting
			setRunMode(WARMSTART);				//   eeprom.* has already been set to default so switch to CALSTART mode
			break;
		case WARMSTART:							// When warmstarting
			if (tick) {							//   If tick, remember tickLength
				tickLength = deltaT;
			} else {							//   Else (tock), remember tockLength
				tockLength = deltaT;
			}
			if (++beatCounter > TGT_WARMUP) {
				setRunMode(RUN);				//   If just did the warm-up switch from WARMSTART to RUN
			}
			break;
		case CALSTART:							// When starting a calibration,
			setRunMode(WARMSTART);				//   Begin by warming up to be sure everything is settled
			break;
		case CALIBRATE:							// When doing calibration
			if (tempIx == NO_CAL) break;		//   If not in calibration temp range, don't try doing it
			if (tempIx != getTempIx(lastTemp)) {
												//   If not the same temperature index as last time
				if (eeprom.curSmoothing[tempIx] > TGT_SMOOTHING) {
												//     If new temp index is already calibrated
					setRunMode(RUN);			//       Switch to RUN mode
					break;
				}
			}
			if (!eeprom.compensated) {			// If not temperature compensated
				eeprom.uspb[tempIx] += (deltaT - eeprom.uspb[tempIx]) / eeprom.curSmoothing[tempIx];
												//   Update running average
				if (++eeprom.curSmoothing[tempIx] > TGT_SMOOTHING) {
												//   If just reached a full smoothing interval
					writeEEPROM();				//     Make calibration parms persistent
					setRunMode(CALFINISH);		//     Switch to CALFINISH mode
				}
			} else {
/****
 *
 *			Temperature-compensated calibration model
 *
 *			Each of the tempIx buckets is our best guess for the beat duration at temperature t(tempIx) (in degrees 
 *			C * 256) where t(tempIx) = tempIx * 128 + TEMP_MIN * 256.
 *
 *			At each beat, the temperature and the beat duration are measured. In a Cartesian system where x is temp 
 *			and y is beat duration, the vertical distance between (temp, deltaT) and the line connecting the points 
 *			(t(tempIx), uspb[tempIx]) and (t(tempIx + 1), uspb[tempIx + 1]) is calculated as the error, e. The 
 *			quantity e is then apportioned between uspb[tempIx] and uspb[tempIx + 1] in proportion to the fraction of
 *			the distance, p, that temp is from t(tempIx) to t(tempIx + 1). The apportioned error is used to adjust the 
 *			running averages in uspb[tempIx] and uspb[tempIx + 1], thus updating the model. But we only attribute the
 *          error to an end point if the current temperature is not too far away from the temperature the end point
 *          represents.
 *
 ****/
	//							   <---------- slope: (y2 - y1) / (x2 - x1) ----------> * (temp - t(tempIx))
				float e = deltaT - ((eeprom.uspb[tempIx+1] - eeprom.uspb[tempIx])/128.0 *   (temp & 0x7f)    + eeprom.uspb[tempIx]);
				float p = (float)(temp & 127) / 128;
				if (eeprom.curSmoothing[tempIx] <= TGT_SMOOTHING) {
					if (eeprom.curSmoothing[tempIx] == 1) {
						eeprom.uspb[tempIx] = deltaT;		// Initial sample
						eeprom.curSmoothing[tempIx]++;
					} else if (p <= 0.6) {
						eeprom.uspb[tempIx] += e * (1 - p) / eeprom.curSmoothing[tempIx]++;
					}
				}
				if (eeprom.curSmoothing[tempIx + 1] <= TGT_SMOOTHING) {
					if (eeprom.curSmoothing[tempIx + 1] == 1) {
						eeprom.uspb[tempIx + 1] = deltaT;	// Initial sample
						eeprom.curSmoothing[tempIx + 1]++;
					} else if (p >= 0.4) {
						eeprom.uspb[tempIx + 1] += e * (1 - p) / eeprom.curSmoothing[tempIx + 1]++;
					}
				}
				
				if (eeprom.curSmoothing[tempIx] > TGT_SMOOTHING && eeprom.curSmoothing[tempIx + 1] > TGT_SMOOTHING) {
													//   If just reached a full smoothing interval for both 
					writeEEPROM();					//     Make calibration parms persistent
					setRunMode(CALFINISH);			//     Switch to CALFINISH mode
				}
			}
			break;
		case CALFINISH:							// When finished calibrating
			setRunMode(RUN);					//  Switch to RUN mode
			break;
		case RUN:								// When running
			if (!eeprom.compensated) {			//   If not temperature-compensated
				deltaT = eeprom.uspb[tempIx] + eeprom.deltaUspb;
												//     deltaT is the calibration value plus the manual correction, if any
				break;							//     Done!
			}
			// Temperature-compensated model
			if (tempIx != NO_CAL) {				//   If within calibrated temp range
				if (eeprom.curSmoothing[tempIx] <= TGT_SMOOTHING || eeprom.curSmoothing[tempIx + 1] <= TGT_SMOOTHING){
					setRunMode(CALIBRATE);		//     If should be calibrating,
				}								//       switch to CALIBRATE mode for next beat
//						 <--------- slope: (y2 - y1) / (x2 - x1) ----------> * (temp - t(tempIx))
				deltaT = (eeprom.uspb[tempIx+1] - eeprom.uspb[tempIx])/128.0 *   (temp & 0x7f) + 
				  eeprom.uspb[tempIx] + eeprom.deltaUspb;
												//     deltaT is the value from model plus the manual correction, if any
			}
			break;
		case CALRTC:							// When calibrating the Arduino real-time clock
			break;
	}
	tick = !tick;								// Switch whether a tick or a tock
	timeBeforeLast = lastTime;					// Update timeBeforeLast
	lastTime = topTime;							// Update lastTime
	return deltaT;								// Return calculated μs per beat
}

/*
 *
 * Getters, setters and incrementers
 *
 */

// Return the current smoothing information for the current temp.
String Escapement::getSmoothing() {
	if (eeprom.compensated) {
		return String(String(eeprom.curSmoothing[getTempIx(temp)]) + " -> " + String(eeprom.curSmoothing[getTempIx(temp) + 1]));
	} else {
		return String(eeprom.curSmoothing[getTempIx(temp)]);
	}
}
 
// Get, set or increment Arduino clock run rate correction in tenths of a second per day
long Escapement::getBias(){
	return eeprom.bias;
}
void Escapement::setBias(long factor){
	eeprom.bias = factor;
	writeEEPROM();								// Make it persistent
}
long Escapement::incrBias(long factor){
	eeprom.bias += factor;
	writeEEPROM();								// Make it persistent
	return eeprom.bias;
}

// Get the last temperature in degrees C; ABS_ZERO if none
float Escapement::getTemp() {
	if (temp == NO_TEMP) return ABS_ZERO;
	return temp / 256.0;
}

// Was the last beat a "tick" or a "tock"?
boolean Escapement::isTick() {
	return tick;
}

// Are we running temperature compensated?
boolean Escapement::isTempComp() {
	return eeprom.compensated;
}

// Get average beats per minute for the current temperature
float Escapement::getAvgBpm(){
	if (!eeprom.compensated) {
		return 60000000.0 / (eeprom.uspb[0] + eeprom.deltaUspb);
	}
	if (tempIx == NO_CAL || eeprom.uspb[tempIx + 1] == 0) return 60000000.0 / (eeprom.uspb[tempIx] + eeprom.deltaUspb);
	if (eeprom.uspb[tempIx] == 0) return 60000000.0 / (eeprom.uspb[tempIx + 1] + eeprom.deltaUspb);
	
	return 60000000.0 / ((eeprom.uspb[tempIx + 1] - eeprom.uspb[tempIx])/128.0 *   (temp & 0x7f) + 
				  eeprom.uspb[tempIx] + eeprom.deltaUspb);
}

// Get current beats per minute as measured by the (corrected) real-time clock
float Escapement::getCurBpm(){
	unsigned long diff;
	if (lastTime == 0 || timeBeforeLast == 0) return 0;
	diff = lastTime - timeBeforeLast;
	diff += ((eeprom.bias * diff) + 432000) / 864000;
	return 60000000.0 / diff;
}

// Get the value last returned by beat() converted to bpm
float Escapement::getBeatBpm() {
	return 60000000.0 / deltaT;
}

// Get the current ratio of tick length to tock length
float Escapement::getDelta(){
	if (tickLength == 0 || tockLength == 0) return 0;
	return (float)tickLength / tockLength;
}
// Get count of beats spent so far in WARMSTART mode
int Escapement::getBeatCounter(){
	return beatCounter;
}
// Get current beat duration in microseconds
long Escapement::getBeatDuration(){
	if (lastTime == 0 || timeBeforeLast == 0) return 0;
	return lastTime - timeBeforeLast + eeprom.deltaUspb;
}
// Get or set the beat delta in μs per beat or increment it in tenths of a second per day
long Escapement::getBeatDelta() {
	return eeprom.deltaUspb;
}
void Escapement::setBeatDelta(long beatDelta) {
	eeprom.deltaUspb = beatDelta;
	writeEEPROM();								// Make it persistent
}
long Escapement::incrBeatDelta(long incr) {
	eeprom.deltaUspb += round(deltaT * (incr / 864000.0));	
	writeEEPROM();								// Make it persistent
	return (eeprom.deltaUspb * 864000.0) / deltaT;
}

// Get/set the current run mode -- COLDSTART, WARMSTART, CALIBRATE, RUN or CALRTC
byte Escapement::getRunMode(){
	return runMode;
}
void Escapement::setRunMode(byte mode){
	switch (mode) {
		case COLDSTART:								//   Switch to cold starting mode
			eeprom.id = 0;							//     Say eeprom not written
			eeprom.bias = 0;						//     and rtc correction (tenths of a second per day) is zero
			break;
		case WARMSTART:								//   Switch to warm starting mode
			beatCounter = 1;						//     Reset cycle counter
			break;
		case CALSTART:								//   Switch to starting a new calibration run
			eeprom.compensated = temp != NO_TEMP;	//     Choose the calibration model: temp compensated or not
			eeprom.deltaUspb = 0;					//     Default the clock speed adjustment
			for (int i = 0; i < TEMP_STEPS; i++) {
				eeprom.uspb[i] = 0;					//     Wipe out old calibration info, if any
				eeprom.curSmoothing[i] = 1;
			}
			break;
		case CALIBRATE:							//   Switch to basic calibration mode
			break;
		case CALFINISH:							//   Switch to calibration finished mode
			break;
		case RUN:								//   Switch to RUN mode
			break;
		case CALRTC:							//   Switch to real-time clock calibration mode
			break;
	}
	runMode = mode;								//   Remember new mode
}

/*
 *
 * Private method to read the current temperature
 *
 */
int Escapement::readTemp() { 
 	if (Wire.requestFrom(ADDRESS_TMP102,2) == 2) {
		if (Wire.available() == 2) {
												// Get the temp (in C * 256) from the TMP102. The first read returns
												//   the most significant byte the second returns the least
												//   significant byte. The binary point is between them.
			return ((((byte)Wire.read()) << 8) | (byte)Wire.read());
#ifdef DEBUG
		} else {
			Serial.println("Wire.available failed");
#endif
		}
#ifdef DEBUG
	} else {
		Serial.println("Wire.requestFrom failed.");
#endif
	}
	return NO_TEMP;
}

/*
 *
 * Private method to convert a temp (degrees C * 256) into a temp index
 *
 * Returns 0 if not running temp compensated.
 * if temp is out of range we use for calibration returns NO_CAL
 * If temp is in range, the index returned is the one corresponding to the reading that is less than the temp
 * and the next higher index is always valid
 *
 */
int Escapement::getTempIx(int t) {
	if (!eeprom.compensated) return 0;			// If not temp compensated, index is always 0
	t = (t / 128)  - (2 * TEMP_MIN);			// Convert t to index
	if (t >= 0 && t < TEMP_STEPS - 1) {			// If within limits
		return t;								//   Return the index
	}
	return NO_CAL;								// If out of range index is NO_CAL
}

/*
 *
 * Private methods to read and write EEPROM
 *
 */

// Read EEPROM 
boolean Escapement::readEEPROM() {
	
	eeprom_read_block((void*)&eeprom, (const void*)0, sizeof(eeprom)); // Read from EEPROM
#ifdef DEBUG
	eeprom.id = 0; 								// Debug: Force eeprom.id to be wrong
#endif
	if (eeprom.id == SETTINGS_TAG) {			// If it looks like ours
		return true;							//  Say we read it okay
	} else {									// Otherwise
		eeprom.id = 0;							//   Default id to note that eeprom not read
		eeprom.bias = 0;						//   Default eeprom.bias (tenths of a second per day)
		eeprom.deltaUspb = 0;					//   Default eeprom.deltaUspb
		eeprom.compensated = true;
		for (int i = 0; i < TEMP_STEPS; i++) {	//   Default eeprom.uspb, eeprom.temp and eeprom.curSmoothing
			eeprom.uspb[i] = 0;
			eeprom.curSmoothing[i] = 1;
		}
		return false;
	}
}

// Write EEPROM
void Escapement::writeEEPROM() {
	eeprom.id = SETTINGS_TAG;					// Mark the EEPROM data structure as ours
	eeprom_write_block((const void*)&eeprom, (void*)0, sizeof(eeprom)); // Write it to EEPROM
}
