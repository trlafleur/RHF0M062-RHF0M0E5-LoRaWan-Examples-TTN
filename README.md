# RFH0M062
 RHF0M062-RHF0M0E5 Examples

This is a basic framework for using the Rising HF LoRaWan radio-modem and connecting it to a sensor.

We use a I2C FRAM device at address 0x50 for non-volatile memory. This device can be omitted
or replaces with an EEProm with minor changes.

DEVEUI is from an I2C 24AA025E64 chip at address 0x52 or can be set in the config.h file.
The SAMD-M0 processor has a 64bit UUID that could also be use for DEVEUI or the one that's 
pre loaded in the modem.

This code base was developed for USA TTN frequency band, but can be changed to other bands.

Tested with: RHF0M062-HF22 and RHF0M0E5-HF22, but should also work with RHF78-052A-HF22
and other derivatives modules from Rising HF.

http://www.risinghf.com/home

My test code has 3 send routines,

1) Short status is used for error messages and is limited to 9 bytes, 
   Sent only when we start up and when we have an error... 
   (DR0 --> 11 bytes - 2 possible MAC bytes)

2) Status is sent infrequently with device information.. version, compile-time, etc...

3) Sensor data is sent, in the default case every hour...

We also has support for a periodic re-join.
 After moving from V2 to V3 on TTN, it became clear that the devices 
 need to re-join from time to time to support future moves or changes in TTN.

This codebase also allows for a few download messages to be processed.

TBD:
   Adding some logging to flash
   look at over-the-air code upgrade...
   Add support for EEProm to replace FRAM
   Get DEVEUI from modem and or UUID from CPU


