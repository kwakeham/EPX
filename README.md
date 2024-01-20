# EPX Wireless controller codebase

This contains the code related to the EPX controller, a from the ground up EPS derailleur controller that supports both wired and wireless operation as well as new features such as multi gear. 

There are a lot of dead or dying EPS systems out there due to the lack of 14430 Cells that were used in the V2 and V3 Power units. These powerunits have legacy electronics and were not designed to be serviced (though better than Shimano). So no cells and not an easy repair (v2 ultra sonically welded, V3 glued plugs), and if you do V2 won't be 

So an open-ish design that uses the common nrf52832 and some newer tech could be the answer -- oh, also... Campagnolo derailleurs shift faster with more force than any SRAM or Shimano electronic. Couple that with some LiHV cells that are trading current for energy and these make everyone elses derailleurs look like amateur hour in a pro peloton.

Deraileurs are power pigs when moving and limited cell size means you can't just run everything all the time (cough cough arduino style). Care must be taken to keep power consumption down. Duty cycling the hall effect sensor, turning off motors and PID control when not needed, low power interrupt driven code, and more will be used.

I'm also not a huge fan of the direction Nordic is taking with their code for newer chips (nrf5340 and nrf54) with the NRF Connect SDK (NCS) that try desperately to force a specific embedded os down your throat. While efficient, it does conflict with some low level control in cases (but that could be my lack of experience with them). So this is still using the older NRF5 SDK that is in maintenance mode (which 99% of all products ACTUALLY use for now).

## BLE UART commands

| Ascii Command | Value 1 | Value 2 (if applicable) | Comment | Example |
| :------- | :----: | :---: | :--- | :--- |
| Computer | $1600 | 5 | ducks |
| "f" |  |  | Force save the position |
| "k" |  |  | Show gains |
| "p" | _float_  |  | Set proporitional (P) gain | "p18.1" will set proporitional to 18.1 |
| "i" | _float_  |  | Set integral (I) gain | "i9" will set integral to 9.0 |
| "d" | _float_  |  | Set derivative (D) gain | "d32.7" will set proporitional to 32.7 |
| "s" | _int_  |  | go to gear | "s5" goes to gear 5 directly from any gear|
| "s" | "+" or "-"  |  | increment gear | "s+" while in gear 5 will go to gear 6|
| "m" | "a" or "g"  |  | chose angle or gear mode | "ma" will switch to angle mode, while "mg" will go to gear mode|
| "t" | _int_  |  | if in angle mode go to taget angle | "t500" will go to angle 500, which is 1 turn (360 degree) + 240 degrees|


## To DO
- Add functional buttons, long and short press, need a flow tree for this
- Safe second flash file (smaller) for stuff that will need to be saved very often (rotations / gear / angle)  instead of the low update stuff (gear positions, gains, etc)
  - Add a DUMP and restore method for the big one
- Measure current in non debug, set baseline
- I2C Battery interface -- this is part of SBS but nobody wrote one for nrf5x?
  - Get current measurement from this
- LIS2DTW, accel for wakeup. Previous code but needs work.
- 