Bedside Atom
=============================

This is my take on an Arduino based clock that syncronizes to NIST's atomic time signal [WWVB](https://en.wikipedia.org/wiki/WWVB).

## Hardware
Universal Solder's [ES100-mod](https://universal-solder.ca/everset-es100-mod-wwvb-receiver/) is used to recieve and decode the phase-modulated broadcast from Fort Collins, Colorado.  

Time data is being displayed on a traditional clock-style quad 7-segment display.  I am using an [i2c controlled flavor](https://www.adafruit.com/product/878) from Adafruit.

## Theory of Operation
Every hour the reciever module is asked for an update.  That's sometimes tough here on the east coast, so It will keep trying until it's successful.  The system keeps track of the recieved UTC time and applys timezone / daylight savings time offsets as appropriate.  Local time is shown on the display.  If it's been over a few hours, additional LEDs will let us know. 
## License
Released under the MIT license, see `LICENSE`
