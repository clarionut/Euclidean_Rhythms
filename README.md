# Euclidean Rhythms
![EuclideanRhythmsSml](https://github.com/user-attachments/assets/ccd983dd-b1db-48ab-b2c8-7528f476d753)

This repository contains details of my derivative of [Hagiwo's Euclidean Rhythms](https://note.com/solder_state/n/n433b32ea6dbc) synthesiser module.

In his notes about the module, Hagiwo comments that there is considerable latency due to the OLED display and also mentions instability in the program
possibly caused by memory issues. He partially addresses the latency issue by only updating the OLED on clock inputs, but this makes it difficult to
update the settings in the absence of a clock input.
During analysis of the code I found several things which would compound these issues, e.g.
- use of a relatively slow I2C OLED display. Swapped for an alternative using much faster hardware SPI
- using a graphics library which keeps an in-memory copy of the display. Changed to the less memory-intensive U8glib library
- the original code uses multiple loops over the channels, each processing a single aspect of the code. Restructured to carry out as many of these
operations as possible in a single loop
- use of the Arduino digitalWrite() function (intrinsically very slow) to output the triggers. Replaced by direct port operations to update all trigger
outputs much faster and simultaneously
- the rotary encoder originally used both interrupt pins of the Arduino with simple polling of the clock input, potentially leading to short clock pulses
being missed. The Encoder library can also be used in a single-interrupt mode allowing the clock input to use the other interrupt, with the interrupt
service routine directly generating the trigger outputs

These changes significantly improve the performance and allow the display to be updated on each clock and when the rotary encoder is used. The display
update is still relatively slow even using hardware SPI resulting in long & variable length trigger outputs. Also the OLED is always on even if the
module is not in use. I therefore added a screensaver which blanks the OLED after 30 seconds plus LEDs to monitor the trigger outputs. The OLED is
enabled again when the rotary encoder is used. This allows quite high clock speeds once the screensaver has kicked in.

The resulting code is smaller than the original allowing some extra functionality to be added:
- ability to 'Put' the current settings into the Arduino EEPROM and 'Get' them from EEPROM in a subsequent session. This version of the code starts up
with predefined default settings, but it would be easy to modify so that the stored settings are retrieved automatically at start up
- a probability control by potentiometer or input CV to vary the likelyhood of changes in Random mode. As in Hagiwo's original code the allowed ranges
of randomness are predefined in the code with the probability controls influencing how likely the changes are
- option for a gate input to re-zero all the output channels to the starting position. I didn't implement this hardware due to a lack of space on my
Eurorack panel but the code for it is present

There are some more details of the changes in the Arduino code comments. The hardware is similar to the original but uses an Arduino Pro Mini rather than
a Nano, has buffering on the outputs for protection and to give higher voltage triggers, and has additional circuitry for the probability controls &
manual step button.

Please note that I built this module using stripboard rather than a PCB. The KiCad files and Gerbers are for my proposed PCB layout, but bear in mind that
***I have not built the circuit using these files***. Please check carefully to ensure you're happy with them before use!
