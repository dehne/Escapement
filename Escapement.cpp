/****
 *
 *   Part of the "Escapement" library for Arduino. Version 0.22
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
 *   various properties of the material from which they are built depends on temperature. For example, the length of 
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
 *   temperature-compensated calibration process.
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
 *   assumes that the Arduino real-time clock is accurate, which it probably isn't.
 *
 *   To cause the Escapement to ignore the persistent parameters in EEPROM and start from scratch, invoke the
 *   enable(COLDSTART) method instead of enable().
 *
 *   Cold starting only lasts one beat and is used to reset the real-time clock correction to zero and mark the eeprom 
 *   having no valid content. Once that is done the Escapement enters WARMSTART mode.
 *
 *   Except during hot start, the Escapement object quickly enters WARMSTART mode. It continues in  this mode 
 *   TGT_SCALE beats. During WARMSTART mode, the peak of the voltage spike induced in the coil by the passing magnet 
 *   is observed and the scaling factor required to detect it reliably is saved in eeprom.peakScale. During WARMSTART 
 *   mode, the duration beat() returns is measured using the (corrected) Arduino real-time Clock.
 *
 *   With WARMSTART over, the Escapement object moves either CALRTC or to CALIBRATE mode, depending on whether the 
 *   Arduino's EEPROM has valid content. If it does not, we enter CALRTC mode to calibrate the Arduino real-time 
 *   clock. Otherwise, we transition to CALIBRATE mode.
 *
 *   CALIBRATE mode lasts for TGT_SMOOTHING cycles of equal temperature. If the temperature changes before 
 *   TGT_SMOOTHING cycles pass, whatever progress has been made on calibrating for the old temperature is maintained, 
 *   and calibration at the new temperature is started or, if partial calibration at that temperature has been done 
 *   earlier, resumed. During CALIBRATE mode, the average duration the beats is measured and saved in 
 *   eeprom.uspb[tempiX]. When calibration for the current temperature completes TGT_SMOOTHING cycles, the Escapement 
 *   object stores the contents of eeprom -- Escapement's persistent parameters -- in the Arduino's EEPROM and 
 *   switches to CALFINISH mode. During CALIBRATE mode, beat() returns eeprom.uspb[tempIx] + eeprom.deltaUspb.
 *
 *   The CALFINISH mode serves as notice to the using sketch that calibration for the current temperature is complete. 
 *   The Escapement object then switches to RUN mode. In CALFINISH mode, beat() returns eeprom.uspb[tempIx] + 
 *   eeprom.deltaUspb.
 *
 *   RUN mode persists so long as the temperature stays the same. If the temperature changes, and calibration for the  
 *   new temperature has not been completely calculated, the Escapement object switches to CALIBRATE mode. During RUN 
 *   mode, beat() returns eeprom.uspb[tempIx] + eeprom.deltaUspb.
 *
 *   The net effect is that the Escapement object automatically characterizes the bendulum or pendulum it is driving,
 *   first by measuring the strength of the pulses the passing magnet induces in the coil and second by measuring the 
 *   average duration of the beat at each temperatures encountered. Once the measurements are done for a given 
 *   temperature, the Escapement assumes the bendulum or pendulum is isochronous at that temperature.
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
 *   CALRTC mode persists until changed by setRunMode(). Because the value returned is the RTC-measured value, in this 
 *   mode the Escapement is effectively driven by the RTC, not the pendulum or bendulum, despite its ticking and 
 *   tocking. The idea is to use the mode to adjust the RTC calibration (via the setBias() and incrBias() methods) so 
 *   that the clock driven by the Escapement keeps perfect time. Once the real-time clock is calibrated, entering 
 *   CALIBRATE mode should produce a good automatic calibration.
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
	Wire.begin();							// Prep to talk to the TMP102
	beatCounter = 1;						// Initialize beatCounter

	if (initialMode != COLDSTART) {			// If forced cold start isn't requested
		if (readEEPROM()) {					//   Try getting info from EEPROM. If that works
			lastTempIx = tempIx = getTempIx();	
											//     Try getting the temperature
			if (eeprom.peakScale > INIT_PEAK && tempIx != -1) {
											//      If temp getting worked and we have been through SCALE mode
				setRunMode(RUN);			//        go right to RUN mode
			} else {
				setRunMode(WARMSTART);		//      Otherwise start in WARMSTART mode
			}
		} else {							//   Else (invalid data in EEPROM)
			setRunMode(COLDSTART);			//     cold start
		}
	} else {								//  Else
		setRunMode(COLDSTART);				//    Cold start
	}
	tick = true;							// Whether currently awaiting a tick or a tock
	tickPeriod = tockPeriod = 0;			// Length of last tick and tock periods (μs)
	timeBeforeLast = lastTime = 0;			// Clock time (μs) last time through beat() (and time before that)
}
 
// Do one beat return length of a beat in μs
long Escapement::beat(){
	int currCoil = 0;							// The value read from coilPin. The value read here, in volts, is 1024/AREF,
												//   where AREF is the voltage on that pin. AREF is set by a 1:1 voltage
												//   divider between the 3.3V pin and Gnd, so 1.65V. Más o menos. The exact 
												//   value doesn't really matter since we're looking for a spike above noise.
	int pastCoil = 0;							// The previous value of currCoil
	unsigned long topTime = 0;					// Clock time (μs) of passing of the magnet
	long deltaT = 0;							// Holds value to be returned
	
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
	
	lastTempIx = tempIx;						// Remember old tempIx so we can tell if temp changed
	tempIx = getTempIx();						// Update the temperature, if the sensor is attached

	switch (runMode) {
		case COLDSTART:							// When cold starting
			setRunMode(WARMSTART);				//   Just switch to WARMSTART mode
			deltaT = 0;							//   Say no time passed
			break;
		case WARMSTART:								// When scaling
			if (pastCoil > MAX_PEAK) {			//  If the peak was more than MAX_PEAK, increase the 
				eeprom.peakScale += 1;			//   scaling factor by one. We want 1 <= peak < MAX_PEAK
			}
			deltaT = topTime - lastTime;		//   Microseconds per beat is whatever we measured for this beat
												//   plus the (rounded) Arduino clock correction
			deltaT += ((eeprom.bias * deltaT) + 432000) / 864000;
			if (deltaT > 5000000) {				//   If the measured beat is more than 5 seconds long
				deltaT = 0;						//     it can't be real -- just ignore it
				break;
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
			if (tempIx != lastTempIx) {			//   If a new temperature reading
				if (curSmoothing[tempIx] > TGT_SMOOTHING) {
												//     If new temp is already calibrated
					setRunMode(RUN);			//       Switch to RUN mode
					break;
				}
			}
			if (tick) {							//   If tick
				tickPeriod = topTime - lastTime;//     Calculate tick period and update tick average
				tickPeriod += ((eeprom.bias * tickPeriod) + 432000) / 864000;
				eeprom.uspb[tempIx] += (tickPeriod - eeprom.uspb[tempIx]) / curSmoothing[tempIx];
			} else {							//   Else it's tock
				tockPeriod = topTime - lastTime;//     Calculate tock period and update tock average
				tockPeriod += ((eeprom.bias * tockPeriod) + 432000) / 864000;
				eeprom.uspb[tempIx] += (tockPeriod - eeprom.uspb[tempIx]) / curSmoothing[tempIx];
			}
			if (++curSmoothing[tempIx] > TGT_SMOOTHING) {
												//   If just reached a full smoothing interval
				writeEEPROM();					//     Make calibration parms persistent
				setRunMode(CALFINISH);			//     Switch to CALFINISH mode
			}
			deltaT = eeprom.uspb[tempIx] + eeprom.deltaUspb;
			break;
		case CALFINISH:							// When finished calibrating
			setRunMode(RUN);					//  Switch to RUN mode
			deltaT = eeprom.uspb[tempIx] + eeprom.deltaUspb;
			break;
		case RUN:								// When running
			if (curSmoothing[tempIx] < TGT_SMOOTHING){
				setRunMode(CALIBRATE);			//   Current temp's calibration isn't finished, switch to CALIBRATE mode
			}
			deltaT = eeprom.uspb[tempIx] + eeprom.deltaUspb;
			break;
		case CALRTC:							// When calibrating the Arduino real-time clock
			deltaT = topTime - lastTime;		//   deltaT is whatever we measured for this beat
												//   plus the (rounded) Arduino clock correction
			deltaT += ((eeprom.bias * deltaT) + 432000) / 864000;
			if (deltaT > 5000000) {				//   If the measured beat is more than 5 seconds long
				deltaT = 0;		//     it can't be real -- just ignore it
			}
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

// Get the number of cycles we've been in the current mode
int Escapement::getbeatCounter(){
	if (runMode == RUN) return -1;				// We don't count this since it could be huge
	if (runMode == CALIBRATE) return curSmoothing[tempIx];
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

// Get the last temperature in degrees C; -1 if none
float Escapement::getTemp() {
	if (tempIx == -1) return(-1);
	return(((tempIx + TEMP_MIN * 2)/2.0));
}

// Was the last beat a "tick" or a "tock"?
boolean Escapement::isTick() {
	return tick;
}

// Are we running emperature compensated?
boolean Escapement::isTempComp() {
	return eeprom.compensated;
}

// Get average beats per minute
float Escapement::getAvgBpm(){
	if (eeprom.uspb[tempIx] == 0) return 0;
	return 60000000.0 / (eeprom.uspb[tempIx] + eeprom.deltaUspb);
}

// Get current beats per minute
float Escapement::getCurBpm(){
	unsigned long diff;
	if (lastTime == 0 || timeBeforeLast == 0) return 0;
	diff = lastTime - timeBeforeLast;
	return 60000000.0 / (diff + eeprom.deltaUspb + ((eeprom.bias * diff) + 432000) / 864000);
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
	return (eeprom.deltaUspb * 864000.0) / eeprom.uspb[tempIx];
}
void Escapement::setBeatDelta(long beatDelta) {
	eeprom.deltaUspb = round(eeprom.uspb[tempIx] * (beatDelta / 864000.0));
	writeEEPROM();								// Make it persistent
}
long Escapement::incrBeatDelta(long incr) {
	eeprom.deltaUspb += round(eeprom.uspb[tempIx] * (incr / 864000.0));	
	writeEEPROM();								// Make it persistent
	return (eeprom.deltaUspb * 864000.0) / eeprom.uspb[tempIx];
}

// Get/set the current run mode -- COLDSTART, WARMSTART, CALIBRATE, RUN or CALRTC
byte Escapement::getRunMode(){
	return runMode;
}
void Escapement::setRunMode(byte mode){
	switch (mode) {
		case COLDSTART:						//   Switch to cold starting mode
			eeprom.id = 0;					//     Say eeprom not written
			eeprom.bias = 0;				//     and eeprom.bias (tenths of a second per day) is zero
#ifdef DEBUG
			eeprom.bias = 784;				//     Debug: Put empirically determined bias here if you have one
#endif
			break;
		case WARMSTART:						//   Switch to warm starting mode
			beatCounter = 1;				//     Reset cycle counter
			eeprom.compensated = true;		//     Assume we'll be doing temperature compensation and then test the assumption
			eeprom.compensated = !(getTempIx() == -1);
			lastTempIx = tempIx = getTempIx();	// Get the temp index
			eeprom.peakScale = INIT_PEAK;	//     Default peak scaling value (adjusted during calibration)
			eeprom.deltaUspb = 0;			//     Default clock speed adjustment
			for (int i = 0; i < TEMP_STEPS; i++) {
				eeprom.uspb[i] = 0;			//     Wipe out old calibration info, if any
				curSmoothing[i] = 1;
			}								//     Default eeprom.deltaUspb and eeprom.deltaSmoothing
			eeprom.peakScale = INIT_PEAK;	//     Reset eeprom.peakScale
			break;
		case CALIBRATE:						//   Switch to basic calibration mode
			break;
		case CALFINISH:						//   Switch to calibration finished mode
			break;
		case RUN:							//   Switch to RUN mode
			break;
		case CALRTC:						//   Switch to real-time clock calibration mode
			break;
	}
	runMode = mode;							//   Remember new mode
}

/*
 *
 * Private method to get the current temperature index
 *
 */
int Escapement::getTempIx() { 
	if (!eeprom.compensated) return 0;			// If no compensation, the answer's always 0
 	if (Wire.requestFrom(ADDRESS_TMP102,2) == 2) {
		if (Wire.available() == 2) {
												// Get the temp (in C) from the TMP102. The first read returns 
												//   the most significant byte the second returns the least 
												//   significant byte. The binary point is between them.
			int temp = ((((byte)Wire.read()) << 8) | (byte)Wire.read()) / 128;	// Temp in 0.5 C steps
			if (temp >= TEMP_MIN * 2 && temp <= TEMP_MAX * 2) {
				return temp - (2 * TEMP_MIN);
			}
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
	return -1;
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
			if (eeprom.uspb[i] <= 0) {			//   and set up eeprom.uspb and curSmoothing
				eeprom.uspb[i] = 0;				//   based on whether the calibration info is
				curSmoothing[i] = 1;			//   valid. Positive values are validly calibrated
			} else {							//   non-positive values need to be done from scratch
				curSmoothing[i] = TGT_SMOOTHING + 1;
			}
		}
		return true;							//  Say we read it okay
	} else {									// Otherwise
		eeprom.id = 0;							//   Default id to note that eeprom not written
		eeprom.bias = 0;						//   Default eeprom.bias (tenths of a second per day)
		eeprom.peakScale = INIT_PEAK;			//   Default peak scaling value (adjusted during calibration)
		for (int i = 0; i < TEMP_STEPS; i++) {	//   Default eeprom.uspb[] and curSmoothing[]
			eeprom.uspb[i] = 0; 
			curSmoothing[i] = 1;
		}
		return false;
	}
}

// Write EEPROM
void Escapement::writeEEPROM() {
	eeprom.id = SETTINGS_TAG;					// Mark the EEPROM data structure as ours
	for (int i = 0; i < TEMP_STEPS; i++) {		// Go through eeprom.uspb[]
		if (curSmoothing[i] <= TGT_SMOOTHING) { //  and mark the ones we're not done calibrating by
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
