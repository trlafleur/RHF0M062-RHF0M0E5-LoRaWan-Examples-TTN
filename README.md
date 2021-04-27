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




