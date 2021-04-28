# RHF0M062, RHF0M0E5 Examples
 
This is a basic framework for using the Rising HF LoRaWan radio-modem and connecting a sensor
to a LoRaWan network like TTN.

We use a I2C FRAM device at address 0x50 for non-volatile memory. This device can be omitted
or replaces with an EEProm with minor changes.

DEVEUI is from an I2C 24AA025E64 chip at address 0x52 or can be set in the config.h file.
The SAMD-M0 processor has a 64bit UUID that could also be use for DEVEUI or the one that's 
pre loaded in the RHF modem.

This code base was developed and tested for USA TTN frequency band, but can be changed to other bands.

Tested with: RHF0M062-HF22 and RHF0M0E5-HF22, but should also work with RHF78-052LA-HF22
and other derivatives modules from Rising HF.

http://www.risinghf.com/home

This test code has 3 send routines,

1) Short status is used for error messages and is limited to 9 bytes, 
   Sent only when we start up and when we have an error... 
   (DR0 --> 11 bytes - 2 possible MAC bytes)

2) Status is sent infrequently with device information.. version, compile-time, etc...

3) Sensor data is sent, in the default case every hour...

We also has support for a periodic re-join.
 After moving from V2 to V3 on TTN, it became clear that the devices 
 need to re-join from time to time to support future moves or changes in 
 the nework or TTN.

This codebase also allows for a few download messages to be processed.

On startup, the code reads a data block stored in FRAM (if available) and compares
the compile date-time field and CRC to what is stored in FRAM to see if we have a 1st boot. 
If so we save a copy of the data block in FRAM for future use. If not we adjust the
boot count, save the boot cause.

USB port is turn off at boot to save power, this requires a double-tap of reset
switch to get back to the boot-loader.

There are many configuration option that you can set in config.h file including the 
LoRaWan keys, frequency tables, timming of send function etc...

The radio-modem has an EEProm that will keep most of its parameters stored after
a reboot... config.h has a define that that will diable re-setting parameters
in the modee if there are not needed.

debug1 and debug2 macros can be disabled to remove all debug output to the serial port, 
they allow us to use the more common "C" printf(...) function in our code instead of 
the Arduino Serial.print(...) functions.

This code was tested with proprietary hardware and with a RocketStream Mini-Ultra-Pro-LoRa
board.

~~~
TBD:
   Adding some logging to flash
   look at over-the-air code upgrade...
   Add support for EEProm option to replace FRAM
   An option to get DEVEUI from modem and or UUID from CPU
   Add more command to support Class B in modem library
   Documentation of modem library
   Add AT command to set various parameter from serial port

Libraries used by this project:
  LoRaWan-RFH0M062-mod
    This radio-modem library was developed by SEEED studios for the RHF78-052 device, I have
    made many change to support the newer radios, added error function and speed up timing 
    on messages to/from the radio.
  Stopwatch
  RTCZero-MOD
  FRAM_MB85RC_I2C
  Arduino_CRC32
~~~
  
