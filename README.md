# The "Escapement" library for Arduino

Escapement Library Copyright 2014 - 2015 by D. L. Ehnebuske 
License terms: [Creative Commons Attribution-ShareAlike 3.0 United States (CC BY-SA 3.0 US)]
(http://creativecommons.org/licenses/by-sa/3.0/us/ "CC BY-SA 3.0 US")

## Introduction

In pendulum clocks, the escapement is the mechanism that does two things. First, it drives the gear-works one 
step forward for each tick or tock of the pendulum. Second, it gives the pendulum a tiny push with each tick or 
tock to keep the pendulum going. The clock's gear-works are designed to move the clock's hands to display the 
time. The gear ratios in the clock mechanism are such that the fixed duration of time that passes between a tick 
and a tock (and between a tock and a tick) is exactly correct to have the clocks hands mark the passing of time.

This Escapement library implements the electronic equivalent of an mechanical escapement and gear-works for a 
properly equipped pendulum or bendulum. (A "bendulum" is a long, slender, springy piece of metal, held 
vertically, whose lower end is fixed and whose upper end has a weight that is free to move back and forth -- a 
sort of upside-down pendulum.) By a "properly equipped" pendulum or bendulum I mean that it terminates in a 
strong magnet that moves across a many-turn coil of fine wire positioned at the center of its swing. 

The basic idea behind the Escapement is that the motion of the magnet is detected as it passes over the coil by 
the current the magnet induces. Just after the magnet passes, the Escapement object produces a pulse of current 
through the coil. This induces a magnetic field in the coil. The field gives the magnet a small push, keeping the 
pendulum or bendulum going. Each time this happens, the Escapement object returns the stored value for the number 
of microseconds per beat of the pendulum or bendulum it is driving. The returned value can be used to drive a 
time-of-day display.

Even carefully designed and built mechanical clocks don't keep perfect time. A major reason for this is that
various properties of the material from which they are built depends on temperature. For example, the length of 
the pendulum in a pendulum clock changes slightly with temperature, slightly changing its period. Similarly, in 
a clock that uses a balance wheel, temperature changes change the spring constant of the hairspring making it 
slightly more or less springy, which slightly changes the balance wheel's ticking rate. Over the years, mechanical
clock designers have invented many ways of compensating for such temperature-induced changes. 

The pendulum or bendulum the Escapement object drives is subject to the same sorts of temperature effects. To
compensate for them, the Escapement object implements optional temperature compensation using the SparkFun TMP102
temperature sensor.

## Operation

Typically, an Escapement object is instantiated as a global in an Arduino sketch. It is then initialized using 
the enable() method in the sketch's setup() function. Finally, the sketch's loop() function repeatedly invokes 
the beat() method. When the magnet passes the coil, beat() returns the number of microseconds that have passed 
since the magnet passed the last time. In this way, the Escapement can be used to drive a time-of-day clock 
display. The Escapement object is able to turn pendulum or bendulum ticks and tocks into microseconds because it 
adjusts to the period of whatever pendulum or bendulum it is driving. It does this through an automatic, 
temperature-compensated calibration process.

The beat() method has several operational modes that, together operate as a state machine to calibrate and 
temperature-compensate the ticking pendulum or bendulum. It works like this.

When the Escapement is started by the enable() method, it attempts to read its persistent parameters from EEPROM.
Depending on what it finds, the Escapement enters one of three run modes.

WARMSTART is entered if good calibration data is read and temperature sensing hasn't been added or removed from 
the configuration since the data was written. With a WARMSTART, the Escapement runs using the corrected Arduino 
clock as a time reference for TGT_WARMUP beats. It then transitions to RUN mode.

COLDSTART is entered if good calibration data can't be read or if it is forced by enable()'s initialMode parameter.
In this mode all of the calibration data, including the Arduino clock's correction data is reset and then WARMSTART 
mode is entered.

CALSTART is entered if good calibration data is read but temperature sensing has been added or removed. In this 
mode, the calibration information, but not the Arduino clock's correction data is reset and then WARMSTART mode is 
entered.

Once entered, RUN mode persists so long as the calibration for the current temperature has been completed. If it 
has not, the Escapement object switches to CALIBRATE mode. During RUN mode, beat() returns the linear 
interpolation of the two calibration points nearest the current temperature.

CALIBRATE mode lasts for TGT_SMOOTHING beats in each of TEMP_STEPS half-degree C "buckets" of temperature. 
Buckets are indexed by the variable tempIX. The lowest temperature at which calibration is done is MIN_TMP. This 
first bucket, like all the buckets runs for a half degree C. The highest temperature bucket starts at MIN_TEMP + 
(TEMP_STEPS - 1) / 2. Calibration information is not collected for temperatures outside this range. If 
temperature sensing is not available, temperature compensation cannot be done and only the first bucket is used.

If, during calibration, the temperature changes enough to fall into a different "bucket" before TGT_SMOOTHING 
beats pass, the progress made in calibrating for the old temperature bucket is maintained in eeprom.uspb[] and 
eeprom.curSmoothing[], and calibration at the new temperature bucket is started or, if partial calibration for at 
that temperature bucket has been done earlier, resumed. During CALIBRATE mode, the duration the beats is measured 
and used to adjust the calibration values for the tempIx and the tempIx + 1 buckets When calibration for either 
the tempIx or  the tempIx + 1 buckets completes TGT_SMOOTHING beats, the Escapement object stores the contents of 
eeprom -- Escapement's persistent parameters -- in the Arduino's EEPROM and switches to CALFINISH mode. During 
CALIBRATE mode, the duration beat() returns is measured using the (corrected) Arduino real-time clock.

The CALFINISH mode does two things. First, it serves as notice to the using sketch that calibration for the 
current temperature bucket is complete. And second, it uses the currently accumulated calibration information to
least-squares fit model of the length of a beat as a function of temperature. The Escapement object then switches 
to RUN mode. In CALFINISH mode, the duration beat() returns is measured using the (corrected) Arduino real-time 
clock.

The net effect is that the Escapement object automatically characterizes the bendulum or pendulum it is driving 
by determining the average duration of the beats at half-degree intervals as it encounters different temperatures. 
Once the calibration is done for a given temperature range, it assumes the bendulum or pendulum behaves linearly 
with temperature between calibration points.

This would work nearly perfectly except that, as hinted at above, the real-time clock in most Arduinos is stable 
but not too accurate (it's a ceramic resonator, not a crystal). That is, real-time clock ticks are essentially 
equal to one another in duration but their durations are not exactly the number of microseconds they should be. 
To correct for this, we use a correction factor, eeprom.bias. The value of eeprom.bias is the number of tenths of 
a second per day by which the real-time clock in the Arduino must be compensated in order for it to be accurate. 
Positive eeprom.bias means the real-time clock's "microseconds" are shorter than real microseconds. Since the 
real-time clock is the standard that's used for calibration, automatic calibration won't work well unless 
eeprom.bias is set correctly. To help with setting eeprom.bias Escapement has one more mode: CALRTC.

In CALRTC mode, the duration beat() returns is the value measured by the (corrected) Arduino real-time clock. The  
CALRTC mode persists until changed by setRunMode(). Because the value returned is the real-time-clock-measured 
value, in this mode the Escapement is effectively driven by the real-time clock, not the pendulum or bendulum, 
despite its ticking and tocking. The idea is to use the mode to adjust the real-time clock calibration (via the 
setBias() and incrBias() methods) so that the clock driven by the Escapement keeps perfect time. Once the real-
time clock is calibrated, entering CALIBRATE mode should produce a good automatic calibration.
