/****
 *
 *   Part of the "Escapement" library for Arduino. Version 0.87
 *
 *   Escapement.cpp Copyright 2014-2016 by D. L. Ehnebuske 
 *   License terms: Creative Commons Attribution-ShareAlike 3.0 United States (CC BY-SA 3.0 US) 
 *                  See http://creativecommons.org/licenses/by-sa/3.0/us/ for specifics. 
 *
 *   In pendulum clocks, the escapement is the mechanism that does two things. First, it drives the gear-works one 
 *   step forward for each tick or tock of the pendulum. Second, it gives the pendulum a tiny push with each tick and 
 *   each tock to keep it going. The clock's gear-works are designed to move the clock's hands to display the time. 
 *   The gear ratios in the clock mechanism are such that the fixed duration of time that passes between a tick 
 *   and a tock (and between a tock and a tick) is exactly correct to have the clocks hands mark the passing of time.
 *
 *   This Escapement library implements the electronic equivalent of a mechanical escapement and gear-works for a 
 *   properly equipped pendulum or bendulum. (A "bendulum" is a long, slender, springy piece of material, held 
 *   vertically, whose lower end is fixed and whose upper end has a weight that is free to move back and forth -- a 
 *   sort of upside-down pendulum.) By a "properly equipped" pendulum or bendulum I mean that one that terminates in
 *   a strong magnet that moves across a many-turn coil of fine wire positioned at the center of its swing. 
 *
 *   The basic idea behind the Escapement is that the motion of the magnet is detected as it passes over the coil by 
 *   the current the magnet induces. Just after the magnet passes, the Escapement object produces a pulse of current 
 *   through the coil. This induces a magnetic field in the coil. The field gives the magnet a small push, keeping the 
 *   pendulum or bendulum going. Each time this happens, the Escapement object returns the number of microseconds 
 *   that have passed since the last tick or tock. The successively returned values can be used to drive a time-of-day 
 *   display.
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
 *   the enable() method in the sketch's setup() function. Then, the sketch's loop() function repeatedly invokes 
 *   the beat() method. When the magnet passes the coil, beat() returns the number of microseconds that have passed 
 *   since the magnet passed the last time. In this way, the Escapement can be used to drive a time-of-day clock 
 *   display. The Escapement object is able to turn pendulum or bendulum ticks and tocks into microseconds because it 
 *   adjusts to the period of whatever pendulum or bendulum it is driving. It does this through an automatic, 
 *   optionally temperature-compensated, calibration process.
 *
 *   The beat() method has several internal operational modes that, together, operate as a state machine to calibrate 
 *   and temperature-compensate the ticking pendulum or bendulum. It works like this.
 *
 *   When the Escapement is started by the enable() method, it attempts to read its persistent parameters from EEPROM.
 *   Depending on what it finds, the Escapement enters one of three operational modes, COLDSTART, WARMSTART, or 
 *   CALIBRATE.
 *
 *   COLDSTART is entered if good calibration data can't be read or if it is forced by enable()'s initialMode parameter.
 *   In this mode all of the calibration data, including the Arduino clock's correction data is reset and then WARMSTART 
 *   mode is entered.
 *
 *   WARMSTART is entered if good calibration data is read and temperature sensing hasn't been added or removed from 
 *   the configuration since the data was written. With a WARMSTART, the Escapement runs using the corrected Arduino 
 *   clock as a time reference for TGT_WARMUP beats. It then transitions to MODEL mode.
 *
 *   CALIBRATE is entered if good calibration data is read but temperature sensing has been added or removed. In this 
 *   mode, the calibration information, but not the Arduino clock's correction data is reset and then WARMSTART mode is 
 *   entered. This mode may be used by the sketch to force a recalibration. In CALIBRATE mode, beat() returns is 
 *   measured using the (corrected) Arduino real-time clock.
 *
 *   MODEL uses the currently collected calibration information, if it exists, to create a least-squares fit model of 
 *   the length of a beat as a function of temperature. If temperature compensation is not being used, the model is 
 *   calculated as though we had information on only one temperature. If the information collected is insufficient to 
 *   create a model, COLLECT mode is entered to collect more data. Once the model is created, the Escapement object 
 *   switches to RUN mode. In MODEL mode, beat() returns the duration measured using the (corrected) Arduino 
 *   real-time clock.
 *
 *   RUN mode is used for normal operation. During RUN mode, beat() returns the beat length as calculated by the 
 *   linear least squares model defined during MODEL mode or, if the temperature is outside the model's range, the 
 *   value measured using the (corrected) Arduino real-time clock. If RUN mode detects that no model has been 
 *   calculated, it switches to MODEL mode. If it detects that the temperature is one for which we have not completed 
 *   data collection, it switches to COLLECT mode.
 *
 *   COLLECT mode collects information about the duration of TGT_SAMPLES beats for each of TEMP_STEPS half-degree C 
 *   "buckets" of temperature. Buckets are indexed by the variable tempIx. The 0th bucket is centered on TEMP_MIN. 
 *   This bucket, like all the buckets, runs for a half degree C. The highest temperature bucket is centered at 
 *   TEMP_MIN + (TEMP_STEPS - 1) / 2. Calibration information is not collected for temperatures outside this range. 
 *   If temperature sensing is not available, temperature compensation cannot be done so the temperature is assumed 
 *   to always be TEMP_MIN.
 *
 *   For each bucket there are two pieces of information, eeprom.uspb[] and eeprom.sampleCount[]. The first, 
 *   eeprom.uspb[] is the average beat duration (in microseconds) at the temperature of that bucket. The second, 
 *   eeprom.sampleCount[], is the number of samples that went into the average so far. A sample is collected if the 
 *   temperature is within 1/8 degree C of the center-temperature of the bucket when the beat takes place. Data 
 *   collection for a bucket consists of collecting TGT_SAMPLES samples for that bucket.
 *
 *   If, during collection, the temperature changes enough to fall into a different bucket before TGT_SAMPLES 
 *   samples are collected, the progress made in collecting samples for the old temperature bucket is maintained in 
 *   eeprom.uspb[] and eeprom.sampleCount[], and collecting at the new temperature bucket is started or resumed. 
 *   When TGT_SAMPLES samples have been taken for a bucket, the Escapement object stores the contents of the eeprom 
 *   structure -- Escapement's persistent parameters -- in the Arduino's EEPROM and switches to MODEL mode. During 
 *   COLLECT mode, beat() returns the duration measured using the (corrected) Arduino real-time clock.
 *
 *   The net effect of the COLLECT and MODEL modes is that the Escapement object automatically characterizes the 
 *   bendulum or pendulum it is driving by determining the average duration of beats at half-degree intervals as it 
 *   encounters different temperatures. It uses this information to calcualte a linear least-squares model of beat 
 *   duration as a function of temperature. It uses the model to calculate beat duration during RUN mode.
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
 *   CALRTC mode persists until changed by the Arduino sketch using setRunMode(). Because the value returned is the 
 *   real-time-clock-measured value, in this mode the Escapement is effectively driven by the real-time clock, not the 
 *   pendulum or bendulum, despite its ticking and tocking. The idea is to use the mode to adjust the real-time clock 
 *   calibration (via the setBias() and incrBias() methods) so that the clock driven by the Escapement keeps perfect 
 *   time. Once the real-time clock is calibrated, COLLECT mode should collect a good information about beat duration.
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
	temp = readTemp();						// Try reading the temp sensor
	slope = yIntercept = 0;					// There's no model yet
	tick = true;							// Whether currently awaiting a tick or a tock
	tickLength = tockLength = 0;			// Length of last tick and tock periods (μs)
	lastTime = 0;							// Real time clock time (μs) last time through beat()
	deltaT = 0;								// Length of last beat (μs)

	if (initialMode != COLDSTART) {			// If forced cold start isn't requested
		if (readEEPROM()) {					//   Try getting info from EEPROM. If that works
			if ((temp != NO_TEMP) == eeprom.compensated) {
											//      If temp compensation mode matches
				setRunMode(WARMSTART);		//		  Start in WARMSTART mode
			} else {
				setRunMode(CALIBRATE);		//      Otherwise start in CALIBRATE mode
			}
		} else {							//   Else (invalid data in EEPROM)
			setRunMode(COLDSTART);			//     Cold start
		}
	} else {								//  Else (forced cold start)
		setRunMode(COLDSTART);				//    Cold start
	}
	tempIx = getTempIx(temp);				// Set up tempIx based on the temp
}
 
// Do one beat return length of a beat in μs
long Escapement::beat(){
	unsigned int currCoil = 0;					// The value read from coilPin. To get the value in volts, multiply by 
												//   AREF/1024, where AREF is the voltage on that pin. Here, AREF is set 
												//   by a 1:1 voltage divider between the 3.3V pin and Gnd, so each count
												//   is about 1.6 mV, más o menos. The exact value doesn't really matter 
												//   since we're looking for a peak above noise.
	unsigned int pastCoil = 0;					// The previous value of currCoil
	
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
	lastTime = topTime;
	topTime = micros();							// Remember when magnet went by
	
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
	deltaT += ((eeprom.bias * deltaT) + 432000L) / 864000L;
	if (deltaT > 5000000) {						// If the measured beat is more than 5 seconds long
		return deltaT = 0;						//   it can't be real -- just ignore it and return
	}
	if (tick) {									//   If tick
		tickLength = deltaT;					//     Set tickLength to beat length
	} else {									//   else (tock)
		tockLength = deltaT;					//     Set tockLength to beat length
	}
	if (temp != NO_TEMP) {						// If temperature sensor is present
		temp = readTemp();						//   Update the temperature
		tempIx = getTempIx(temp);				//   And figure out which "bucket" of temperatures it's in
	}

	switch (runMode) {
		case COLDSTART:							// When cold starting
			setRunMode(WARMSTART);				//   eeprom.* has already been set to default so switch to WARMSTART mode
			break;
		case WARMSTART:							// When warmstarting
			if (++beatCounter > TGT_WARMUP) {	//   Let things tick along for TGT_WARMUP beats
				setRunMode(MODEL);				//   then switch to MODEL
			}
			break;
		case CALIBRATE:							// When starting a calibration,
			setRunMode(WARMSTART);				//   Begin by warming up to be sure everything is settled
			break;
		case COLLECT:							// When doing calibration
			if (tempIx == NO_CAL) break;		//   If outside temp range for which we do calibration, don't do it
			if (eeprom.sampleCount[tempIx] > TGT_SAMPLES) {
												//   If we already have all the data we need for this temp
				setRunMode(RUN);				//     Switch to RUN mode
				break;
			}
/****
 *
 *			Each of the tempIx buckets is the average beat duration at temperature t(tempIx) in degrees Celsius * 256
 *			where t(tempIx) = tempIx * 128 + TEMP_MIN * 256. If the current temperature corresponds to one of the buckets
 *			(i.e., it falls within 1/8 degree of one of the half-degree intervals), the running average is updated with 
 *			the current measured duration. If the current temperature falls somewhere else, nothing is done. If running
 *			uncompensated, temp is always TEMP_MIN * 256 and tempIx is always 0.
 *
 ****/

			if(abs(temp - ((tempIx << 7) + (TEMP_MIN << 8))) <= 32) {
												//   If current temp matches a tempIx bucket to within 1/8 degree C
				eeprom.uspb[tempIx] += (deltaT - eeprom.uspb[tempIx]) / eeprom.sampleCount[tempIx];
												//     Update running average
				if (++eeprom.sampleCount[tempIx] > TGT_SAMPLES) {
												//   If just reached a full smoothing interval
					writeEEPROM();				//     Make calibration parms persistent
					setRunMode(MODEL);			//     Switch to MODEL mode
				}
			}
			break;
		case MODEL:							// When finished calibrating
			{
				float xSum = 0.0;				//   Have a go at calculating the linear least squares for the data so far
				float ySum = 0.0;
				float xxSum = 0.0;
				float xySum = 0.0;
				int count = 0;
				for (int i = 0; i < TEMP_STEPS; i++) {
					if (eeprom.sampleCount[i] > TGT_SAMPLES) {
						float x = (((TEMP_MIN << 1) + i) << 7);
						float y = eeprom.uspb[i];
						count++;
						xSum += x;
						ySum += y;
						xxSum += x * x;
						xySum += x * y;
					}
				}
				if (count < 1) {				//   If not even one bucket is complete, continue collecting data
					setRunMode(COLLECT);
					break;
				} else {						//   Otherwise calculate linear lsq model parameters
					slope = count > 1 ? ((count * xySum - xSum * ySum) / (count * xxSum - xSum * xSum)) * 4096.0 : 0;
					yIntercept = (ySum - (slope / 4096.0) * xSum) / count;
					eeprom.speedAdj = 0;		//     and set the speed adjustment to 0 since it went with the old model (if any)
#ifdef DEBUG
					Serial.print("MODEL slope: ");
					Serial.print(slope);
					Serial.print(", yIntercept: ");
					Serial.println(yIntercept);
#endif
				}
			}
			setRunMode(RUN);					//  Switch to RUN mode
			break;
		case RUN:								// When running
			if (tempIx == NO_CAL) break;		//   If outside temp range, use rtc measured value
			if (yIntercept == 0) {				//   If the model hasn't been calculated
				setRunMode(MODEL);				//     Use rtc measured value and build the model at the next beat
				break;
			}
			if (eeprom.sampleCount[tempIx] <= TGT_SAMPLES) {
				setRunMode(COLLECT);			//   If not finished collecting data for this temp,
				break;							//     use rtc measured value and switch to COLLECT
			}
			deltaT = slope * temp / 4096L + yIntercept;
			deltaT += ((deltaT / 864L) * eeprom.speedAdj) / 1000L;
												//     i.e., deltaT * eeprom.speedAdj / 864000 without large intermediate results
												//     deltaT is what the model says it is + manual correction
			break;
		case CALRTC:							// When calibrating the Arduino real-time clock
			break;
	}
	tick = !tick;								// Switch whether a tick or a tock
	return deltaT;								// Return calculated μs per beat
}

/*
 *
 * Getters, setters and incrementers
 *
 */

// Return the current smoothing information for the current temp.
int Escapement::getSmoothing() {
	int ix = getTempIx(temp);
	return ix == NO_CAL ? 0 : eeprom.sampleCount[ix];
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

// Get beats per minute as modeled. If no model or outside of temperature range return 0.0
float Escapement::getBpmModel(){
	if (yIntercept == 0 || tempIx == NO_CAL) return 0.0;
	long dT = slope * temp / 4096L + yIntercept;
	dT += eeprom.speedAdj / 864000L;
	return 60000000.0 / dT;
}

// Get current beats per minute as measured by the (corrected) real-time clock
float Escapement::getBpmRTC(){
	unsigned long diff;
	if (lastTime == 0) return 0;
	diff = topTime - lastTime;
	diff += ((eeprom.bias * diff) + 432000) / 864000;
	return 60000000.0 / diff;
}

// Get the value last returned by beat() converted to bpm
float Escapement::getBpmBeat() {
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
// Get current beat duration as measured by the RTC (μs)
long Escapement::getBeatDuration(){
	if (lastTime == 0) return 0;
	return topTime - lastTime;
}
// Get, set or increment the manual speed adjustment in tenths of a second per day
long Escapement::getSpeedAdj() {
	return eeprom.speedAdj;
}
void Escapement::setSpeedAdj(long speedAdj) {
	eeprom.speedAdj = speedAdj;
	writeEEPROM();								// Make it persistent
}
long Escapement::incrSpeedAdj(long incr) {
	eeprom.speedAdj += incr;	
	writeEEPROM();								// Make it persistent
	return eeprom.speedAdj;						// Return new value
}
float Escapement::getM() {
	return float(slope)/4096.0;
}
long Escapement::getB() {
	return yIntercept;
}

// Get/set the current run mode -- COLDSTART, WARMSTART, COLLECT, RUN or CALRTC
byte Escapement::getRunMode(){
	return runMode;
}
void Escapement::setRunMode(byte mode){
	switch (mode) {
		case COLDSTART:								//   Switch to cold starting mode
			eeprom.id = 0;							//     Say eeprom not written,
			eeprom.bias = 0;						//     rtc correction (tenths of a second per day) is zero
			eeprom.compensated = temp != NO_TEMP;	//     and use whether we could read the temp to set compensated
			break;
		case WARMSTART:								//   Switch to warm starting mode
			beatCounter = 1;						//     Reset the beat counter
			break;
		case CALIBRATE:								//   Switch to starting a new calibration run
			eeprom.compensated = temp != NO_TEMP;	//     Choose the calibration model: temp compensated or not
			eeprom.speedAdj = 0;					//     Default the clock speed adjustment
			for (int i = 0; i < TEMP_STEPS; i++) {
				eeprom.uspb[i] = 0;					//     Wipe out old calibration info, if any
				eeprom.sampleCount[i] = 1;
			}
			slope = yIntercept = 0;					//     Do away with the old linear least squares model, too
			break;
		case COLLECT:								//   Switch to data collection mode
			break;
		case MODEL:									//   Switch to model creation mode
			break;
		case RUN:									//   Switch to normal running mode
			break;
		case CALRTC:								//   Switch to real-time clock calibration mode
			break;
	}
	runMode = mode;									//   Remember new mode
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
 * If temp is in range, the index returned is the one corresponding to the reading that is closest to the temp.
 * The next higher or lower index may not be valid
 *
 */
int Escapement::getTempIx(int t) {
	if (!eeprom.compensated) return 0;			// If not temp compensated, index is always 0
	t = ((t + 64) / 128)  - (2 * TEMP_MIN);		// Convert t to index
	if (t >= 0 && t < TEMP_STEPS) {				// If within limits
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
	if (eeprom.id == SETTINGS_TAG) {			// If it looks like ours
		return true;							//  Say we read it okay
	} else {									// Otherwise
		eeprom.id = 0;							//   Default id to note that eeprom not read
		eeprom.bias = 0;						//   Default RTC speed correction
		eeprom.speedAdj = 0;					//   Default manual speed adjustment
		eeprom.compensated = temp != NO_TEMP;	//   True iff sensor hardware existed at enable() time
		for (int i = 0; i < TEMP_STEPS; i++) {	//   Default eeprom.uspb, eeprom.temp and eeprom.sampleCount
			eeprom.uspb[i] = 0;
			eeprom.sampleCount[i] = 1;
		}
		return false;
	}
}

// Write EEPROM
void Escapement::writeEEPROM() {
	eeprom.id = SETTINGS_TAG;					// Mark the EEPROM data structure as ours
	eeprom_write_block((const void*)&eeprom, (void*)0, sizeof(eeprom)); // Write it to EEPROM
}
