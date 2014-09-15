# The "Escapement" library for Arduino

Escapement Library Copyright 2014 by D. L. Ehnebuske 
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
If this is successful and the data looks good, the mode is set to RUN, accomplishing a "hot start." If the read 
is good, but there's not good calibration information, a "warm start" is assumed. A warm start uses the value of 
Arduino real-time clock correction, eeprom.bias, but otherwise starts the calibration process from scratch by 
entering WARMSTART mode. If neither of these applies, the Escapement does a "cold start" by entering COLDSTART 
mode. A cold start sets the Arduino real-time clock to zero and then enters WARMSTART mode. Thus, a cold start 
assumes that the Arduino real-time clock is accurate, which it probably isn't.

To cause the Escapement to ignore the persistent parameters in EEPROM and start from scratch, invoke the
enable(COLDSTART) method instead of enable().


Except during hot start, the Escapement object quickly enters WARMSTART mode. It continues in  this mode for 
TGT_SETTLE beats. The purpose of this mode is to let the bendulum or pendulum settle into a regular motion since 
its motion is typically disturbed at startup from having been given a start-up push by hand. While in WARMSTART 
mode, the duration returned is measured using the (corrected) Arduino real-time Clock.

Once it completes WARMSTART mode the Escapement object switches to SCALE mode for getTgtScale() cycles. (A cycle 
is two beats.) During SCALE mode, the peak voltage induced in the coil by the passing magnet is determined and 
saved in eeprom.peakScale. During SCALE mode, the duration beat() returns is measured using the (corrected) 
Arduino real-time Clock.

With SCALE over, the Escapement object moves to CALIBRATE mode, in which it remains for TGT_SMOOTHING 
additional cycles of equal temperature. If the temperature changes before TGT_SMOOTHING cycles pass, whatever 
progress has been made on calibrating for the old temperature is maintained, and calibration at the new 
temperature is started or, if partial calibration at that temperature has been done earlier, resumed. During 
CALIBRATE mode, the average duration of tick and tock beats is measured and saved in tickAvg, tockAvg and their 
average is saved in eeprom.uspb[tempiX]. When calibration for the current temperature completes TGT_SMOOTHING 
cycles, the Escapement object stores the contents of eeprom -- Escapement's persistent parameters -- in the 
Arduino's EEPROM and switches to CALFINISH mode. During CALIBRATE mode, beat() returns eeprom.uspb[tempIx].

The CALFINISH mode serves as notice to the using sketch that calibration for the current temperature is complete. 
The Escapement object then switches to RUN mode. In CALFINISH mode, beat() returns eeprom.uspb[tempix].

RUN mode persists so long as the temperature stays the same. If the temperature changes, and calibration for the  
new temperature has not been completely calculated, the Escapement object switches to CALIBRATE mode. During RUN 
mode, beat() returns eeprom.uspb[tempIx].

The net effect is that the Escapement object automatically characterizes the bendulum or pendulum it is driving,
first by measuring the strength of the pulses the passing magnet induces in the coil and second by measuring the 
average duration of the beat at each temperatures encountered. Once the measurements are done for a given 
temperature, the Escapement assumes the bendulum or pendulum is isochronous at that temperature.

This would work nearly perfectly except that, as hinted at above, the real-time clock in most Arduinos is stable 
but not too accurate (it's a ceramic resonator, not a crystal). That is, real-time clock ticks are essentially 
equal to one another in duration but their durations are not exactly the number of microseconds they should be. 
To correct for this, we use a correction factor, eeprom.bias. The value of eeprom.bias is the number of tenths of 
a second per day by which the real-time clock in the Arduino must be compensated in order for it to be accurate. 
Positive eeprom.bias means the real-time clock's "microseconds" are shorter than real microseconds. Since the 
real-time clock is the standard that's used for calibration, automatic calibration won't work well unless 
eeprom.bias is set correctly.
