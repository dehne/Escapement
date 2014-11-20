/****
 *
 *   Part of the "Escapement" library for Arduino. Version 0.3
 *
 *   Escapement.cpp Copyright 2014 by D. L. Ehnebuske 
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
 *   If this is successful and the data looks good, the mode is set to RUN, accomplishing a "hot start." If the read 
 *   is good, but there's not good calibration information, a "warm start" is assumed. A warm start uses the value of 
 *   Arduino real-time clock correction, eeprom.bias, but otherwise starts the calibration process from scratch by 
 *   entering WARMSTART mode. If neither of these applies, the Escapement does a "cold start" by entering COLDSTART 
 *   mode. A cold start sets the Arduino real-time clock to zero and then enters WARMSTART mode. Thus, a cold start 
 *   assumes that the Arduino real-time clock is accurate, which it probably isn't. But you can calibrate the Arduino
 *   real-time clock, as you'll see, below.
 *
 *   To cause the Escapement to ignore the persistent parameters in EEPROM and start from scratch, invoke the
 *   enable(COLDSTART) method instead of enable().
 *
 *   Cold starting only lasts one beat and is used to reset the real-time clock correction to zero and mark the eeprom 
 *   structure as having no valid content. Once that is done the Escapement enters WARMSTART mode. During COLDSTART 
 *   mode, the duration beat() returns is measured using the (corrected) Arduino real-time clock.
 *
 *   Except during hot start, the Escapement object quickly enters WARMSTART mode. It continues in  this mode for
 *   TGT_SCALE beats. During WARMSTART mode, the peak of the voltage spike induced in the coil by the passing magnet 
 *   is observed and the scaling factor required to detect it reliably is saved in eeprom.peakScale. During WARMSTART 
 *   mode, the duration beat() returns is measured using the (corrected) Arduino real-time clock.
 *
 *   With WARMSTART over, the Escapement object moves either CALRTC or to CALIBRATE mode, depending on whether the 
 *   Arduino's EEPROM has valid content. If it does not, we enter CALRTC mode to calibrate the Arduino real-time 
 *   clock. Otherwise, we transition to CALIBRATE mode.
 *
 *   CALIBRATE mode lasts for TGT_SMOOTHING beats in each of TEMP_STEPS half-degree C "buckets" of temperature. The 
 *   lowest temperature at which calibration is done is MIN_TMP. This first bucket, like all the buckets runs for a 
 *   half degree C. The highest temperature bucket starts at MIN_TEMP + (TEMP_STEPS - 1) / 2. Calibration information 
 *   is not collected for temperatures outside this range. If temperature readings are not available, temperature 
 *   compensation cannot be done and only the first bucket is used.
 *
 *   If, during calibration, the temperature changes enough to fall into a different "bucket" before TGT_SMOOTHING 
 *   beats pass, the progress made in calibrating for the old temperature bucket is maintained, and calibration at 
 *   the new temperature bucket is started or, if partial calibration for at that temperature bucket has been done 
 *   earlier, resumed. During CALIBRATE mode, the running average of the duration the beats is measured and saved 
 *   in eeprom.uspb[tempIx], where tempIx is the index corresponding to the bucket for the current temperature. 
 *   Similarly, the running average for the measured temperature is saved in eeprom.temp[tempIx]. When calibration 
 *   for the current temperature bucket completes TGT_SMOOTHING beats, the Escapement object stores the contents of 
 *   eeprom -- Escapement's persistent parameters -- in the Arduino's EEPROM and switches to CALFINISH mode. During 
 *   CALIBRATE mode, the duration beat() returns is measured using the (corrected) Arduino real-time clock.
 *
 *   The CALFINISH mode does two things. First, it serves as notice to the using sketch that calibration for the 
 *   current temperature bucket is complete. And second, it uses the currently accumulated calibration information to
 *   least-squares fit model of the length of a beat as a function of temperature. The Escapement object then switches 
 *   to RUN mode. In CALFINISH mode, the duration beat() returns is measured using the (corrected) Arduino real-time 
 *   clock.
 *
 *   RUN mode persists so long as the temperature stays within the the same bucket. If the temperature changes enough 
 *   to fall into a different temperature bucket, and calibration for that bucket new temperature has not been 
 *   completely calculated, the Escapement object switches to CALIBRATE mode. During RUN mode, beat() returns 
 *   slope * temp + intercept + eeprom.deltaUspb, where slope is the slope of the least squares 
 *   line fitted from the available calibration data and intercept is its intercept.
 *
 *   The net effect is that the Escapement object automatically characterizes the bendulum or pendulum it is driving,
 *   first by measuring the strength of the pulses the passing magnet induces in the coil. Second by measuring the 
 *   average duration of the beat and average temperature at half-degree intervals as it encounters them. Finally, 
 *   once the calibration is done for a given temperature range, it does a linear least squares fit to the available 
 *   calibration data. The Escapement then assumes the bendulum or pendulum behaves according to the linear model. As
 *   additional temperatures are encountered, it generates additional calibration data and updates its linear least-
 *   squares model.
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
	lastTemp = temp = readTemp();			// Initialize the temperature variables
	tempIx = getTempIx(temp);
	tick = true;							// Whether currently awaiting a tick or a tock
	tickPeriod = tockPeriod = 0;			// Length of last tick and tock periods (μs)
	timeBeforeLast = lastTime = 0;			// Real time clock time (μs) last time through beat() (and time before that)
	deltaT = 0;								// Length of last beat (μs)

	if (initialMode != COLDSTART) {			// If forced cold start isn't requested
		if (readEEPROM()) {					//   Try getting info from EEPROM. If that works
			if (eeprom.peakScale > INIT_PEAK && ((temp != NO_TEMP) == eeprom.compensated)) {
											//      If we have been through SCALE mode and temp compensation mode matches
				setRunMode(RUN);			//		  Start in RUN mode
			} else {
				setRunMode(WARMSTART);		//      Otherwise start in WARMSTART mode
			}
		} else {							//   Else (invalid data in EEPROM)
			setRunMode(COLDSTART);			//     cold start
		}
	} else {								//  Else
		setRunMode(COLDSTART);				//    Cold start
	}
}
 
// Do one beat return length of a beat in μs
long Escapement::beat(){
	int currCoil = 0;							// The value read from coilPin. The value read here, in volts, is 1024/AREF,
												//   where AREF is the voltage on that pin. AREF is set by a 1:1 voltage
												//   divider between the 3.3V pin and Gnd, so 1.65V. Más o menos. The exact 
												//   value doesn't really matter since we're looking for a spike above noise.
	int pastCoil = 0;							// The previous value of currCoil
	unsigned long topTime = 0;					// Clock time (μs) of passing of the magnet
	
	// watch for passing magnet
	delay(SETTLE_TIME);							// Wait for things to calm down
	do {										// Wait for the voltage to fall to zero
		currCoil = analogRead(sensePin);
	} while (currCoil > 0);
	while (currCoil >= pastCoil) {				// While the magnet hasn't passed over coil,
		pastCoil = currCoil;					//   loop waiting for the voltage induced in the coil to begin to fall
		currCoil = analogRead(sensePin) / eeprom.peakScale;
	}
	
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
	if (temp != NO_TEMP) {						// If running temperature compensated
		lastTemp = temp;						//   Remember old temp so we can tell if it changed
		temp = readTemp();						//   Update the temperature
		tempIx = getTempIx(temp);				//   And figure out which "bucket" of temperatures it's in
	}

	switch (runMode) {
		case COLDSTART:							// When cold starting
			setRunMode(WARMSTART);				//   Just switch to WARMSTART mode
			break;
		case WARMSTART:								// When scaling
			if (pastCoil > MAX_PEAK) {			//  If the peak was more than MAX_PEAK, increase the 
				eeprom.peakScale += 1;			//   scaling factor by one. We want 1 <= peak < MAX_PEAK
			}
			if (tick) {							//   If tick, remember tickPeriod
				tickPeriod = deltaT;
			} else {							//   Else (tock), remember tockPeriod
				tockPeriod = deltaT;
			}
			if (++beatCounter > TGT_SCALE) {
				setRunMode(CALIBRATE);			//   If just done scaling switch from WARMSTART to CALIBRATE
			}
			break;
		case CALIBRATE:							// When doing calibration
			if (tempIx != getTempIx(lastTemp)) {
												//   If not the same temperature index as last time
				if (curSmoothing[tempIx] > TGT_SMOOTHING) {
												//     If new temp index is already calibrated
					setRunMode(RUN);			//       Switch to RUN mode
					break;
				}
			}
			if (tick) {							//   If tick
				tickPeriod = topTime - lastTime;//     Calculate tick period and update average uspb
				tickPeriod += ((eeprom.bias * tickPeriod) + 432000) / 864000;
				eeprom.uspb[tempIx] += (tickPeriod - eeprom.uspb[tempIx]) / curSmoothing[tempIx];
			} else {							//   Else it's tock
				tockPeriod = topTime - lastTime;//     Calculate tock period and update average uspb
				tockPeriod += ((eeprom.bias * tockPeriod) + 432000) / 864000;
				eeprom.uspb[tempIx] += (tockPeriod - eeprom.uspb[tempIx]) / curSmoothing[tempIx];
			}
			if (eeprom.compensated) {			//   If temperature compensated, update temp average
				eeprom.temp[tempIx] += (temp - eeprom.temp[tempIx]) / curSmoothing[tempIx];
			}
			if (++curSmoothing[tempIx] > TGT_SMOOTHING) {
												//   If just reached a full smoothing interval
				writeEEPROM();					//     Make calibration parms persistent
				setRunMode(CALFINISH);			//     Switch to CALFINISH mode
			}
			break;
		case CALFINISH:							// When finished calibrating
			setRunMode(RUN);					//  Switch to RUN mode
			break;
		case RUN:								// When running
			if (tempIx != NO_CAL && curSmoothing[tempIx] <= TGT_SMOOTHING){
				setRunMode(CALIBRATE);			//   If should be calibrating due to temp change,
			}									//     switch to CALIBRATE mode
			deltaT = slope * temp + intercept + eeprom.deltaUspb;
			break;
		case CALRTC:							// When calibrating the Arduino real-time clock
			if (tick) {							//   If tick, remember tickPeriod
				tickPeriod = deltaT;
			} else {							//   Else (tock), remember tockPeriod
				tockPeriod = deltaT;
			}
//			break;
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

// Get the number of beats we've been in the current mode
int Escapement::getBeatCounter(){
	if (runMode == RUN) return -1;				// We don't count this since it could be huge
	if (runMode == CALIBRATE) return curSmoothing[getTempIx(temp)];
	return beatCounter;
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

// Get/set the value of eeprom.peakScale -- the scaling factor by which induced coil voltage readings is divided
int Escapement::getPeakScale() {
	return eeprom.peakScale;
}
void Escapement::setPeakScale(int scaleFactor) {
	eeprom.peakScale = scaleFactor;
	writeEEPROM();								// Make it persistent
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
	if (tempIx == NO_CAL || eeprom.uspb[tempIx] == 0) return 0;
	return 60000000.0 / (eeprom.uspb[tempIx] + eeprom.deltaUspb);
}

// Get current beats per minute as measured by the real-time clock
float Escapement::getCurBpm(){
	unsigned long diff;
	if (lastTime == 0 || timeBeforeLast == 0) return 0;
	diff = lastTime - timeBeforeLast;
	return 60000000.0 / (diff + eeprom.deltaUspb + ((eeprom.bias * diff) + 432000) / 864000);
}

// Get the value last returned by beat() converted to bpm
float Escapement::getBeatBpm() {
	return 60000000.0 / deltaT;
}

// Get the current ratio of tick length to tock length
float Escapement::getDelta(){
	if (tickPeriod == 0 || tockPeriod == 0) return 0;
	return (float)tickPeriod / tockPeriod;
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
		case COLDSTART:							//   Switch to cold starting mode
			eeprom.id = 0;						//     Say eeprom not written
			eeprom.bias = 0;					//     and rtc correction (tenths of a second per day) is zero
#ifdef DEBUG
			eeprom.bias = 794;					//     Debug: Put empirically determined bias here if you have one else 0
#endif
			break;
		case WARMSTART:							//   Switch to warm starting mode
			beatCounter = 1;					//     Reset cycle counter
			eeprom.compensated = temp != NO_TEMP;
			eeprom.peakScale = INIT_PEAK;		//     Default peak scaling value (adjusted during calibration)
			eeprom.deltaUspb = 0;				//     Default clock speed adjustment
			for (int i = 0; i < TEMP_STEPS; i++) {
				eeprom.uspb[i] = 0;				//     Wipe out old calibration info, if any
				eeprom.temp[i] = 0;
				curSmoothing[i] = 1;
			}									//     Default eeprom.deltaUspb and eeprom.deltaSmoothing
			break;
		case CALIBRATE:							//   Switch to basic calibration mode
			break;
		case CALFINISH:							//   Switch to calibration finished mode
			break;
		case RUN:								//   Switch to RUN mode
			makeModel();						//     Make the least squares model
			break;
		case CALRTC:							//   Switch to real-time clock calibration mode
			break;
	}
	runMode = mode;								//   Remember new mode
}

/*
 *
 * Private method to calculate least squares model from calibration data
 *
 */
void Escapement::makeModel() {
	int count = 0;
	float xSum = 0.0;
	float ySum = 0.0;
	float xxSum = 0.0;
	float xySum = 0.0;
	float t;
	float u;
	for (int i = 0; i < TEMP_STEPS; i++) {		// Calculate xSum, ySum, xxSum, xySum and count from calibration data
		if (curSmoothing[i] > TGT_SMOOTHING) {
			t = float (eeprom.temp[i]);
			u = float (eeprom.uspb[i]);
			xSum += t;
			ySum += u;
			xxSum += t * t;
			xySum += t * u;
			count++;
		}
	}
	if (count > 1) {							// If there are at least two calibration points, calculate least squares fit
		slope = (count * xySum - xSum * ySum) / (count * xxSum - xSum * xSum);
		intercept = (ySum - slope * xSum) / count;
	} else {									// Otherwise, for 1 point y = 0x + calibration point, for 0 points y = 0x + 0
		slope = 0.0;
		intercept = ySum;
	}
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
 */
int Escapement::getTempIx(int t) {
	if (!eeprom.compensated) return 0;			// If not temp compensated, index is always 0
	t = (t / 128)  - (2 * TEMP_MIN);			// Convert t to index
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
#ifdef DEBUG
	eeprom.id = 0; 								// Debug: Force eeprom.id to be wrong
#endif
	if (eeprom.id == SETTINGS_TAG) {			// If it looks like ours
		for (int i = 0; i < TEMP_STEPS; i++) {	//   Go through eeprom.uspb[]
			if (eeprom.uspb[i] <= 0) {			//   and set up eeprom.uspb, eeprom.temp and curSmoothing
				eeprom.uspb[i] = 0;				//   based on whether the calibration info is
				eeprom.temp[i] = 0;				//   valid. Positive values of eeprom.uspb are validly
				curSmoothing[i] = 1;			//   calibrated. Others need to be done from scratch
			} else {
				curSmoothing[i] = TGT_SMOOTHING + 1;
			}
		}
		return true;							//  Say we read it okay
	} else {									// Otherwise
		eeprom.id = 0;							//   Default id to note that eeprom not read
		eeprom.bias = 0;						//   Default eeprom.bias (tenths of a second per day)
		eeprom.peakScale = INIT_PEAK;			//   Default peak scaling value (adjusted during calibration)
		eeprom.deltaUspb = 0;					//   Default eeprom.deltaUspb
		eeprom.compensated = true;
		for (int i = 0; i < TEMP_STEPS; i++) {	//   Default eeprom.uspb, eeprom.temp and curSmoothing
			eeprom.uspb[i] = 0;
			eeprom.temp[i] = 0;
			curSmoothing[i] = 1;
		}
		return false;
	}
}

// Write EEPROM
void Escapement::writeEEPROM() {
	eeprom.id = SETTINGS_TAG;					// Mark the EEPROM data structure as ours
	for (int i = 0; i < TEMP_STEPS; i++) {		// Go through eeprom.uspb[]
		if (curSmoothing[i] <= TGT_SMOOTHING) { //  and mark the ones we partially calibrating by
			eeprom.uspb[i] = -eeprom.uspb[i];	//  negating them
		}
	}
	eeprom_write_block((const void*)&eeprom, (void*)0, sizeof(eeprom)); // Write it to EEPROM
	for (int i = 0; i < TEMP_STEPS; i++) {		// Go through eeprom.uspb[]
		if (curSmoothing[i] <= TGT_SMOOTHING) {	//  and restore the ones we negated
			eeprom.uspb[i] = -eeprom.uspb[i];
		}
	}
}
