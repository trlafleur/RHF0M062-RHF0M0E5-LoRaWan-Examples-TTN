/*
  LoRaWAN.cpp
  2013 Copyright (c) Seeed Technology Inc.  All right reserved.

  Author: Wayne Weng
  Date: 2016-10-17

  add rgb backlight fucnction @ 2013-10-15???
  
  The MIT License (MIT)

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.1  USA
 
  https://github.com/toddkrein/OTAA-LoRaWAN-Seeed
*/

/*
CHANGE LOG:
 *
 *  DATE         REV  DESCRIPTION
 *  -----------  ---  ----------------------------------------------------------
 *  29-Jul-2020 1.0   TRL - First test with RHF0M062
 *  27-AUG-2021 1.0a  TRL - added a few more commands
 *  15-JAN-2021 1.0b  TRL - added getDR, getTest, some Class B
 *  17-Jan-2021 1.0c  TRL - added getChannel, Get for RXWIN1, RXWIN2, sendCMD
 *  31-Jan-2021 1.0d  TRL - added some error code and delay1, buffer1 now using serial timeouts
 *  09-Feb-2021 1.0e  TTL - added some error messages and flags, auto-join, port on RX data
 *  19-Feb-2021 1.0f  TRL - added getmessage to most commands, _buffer is now 512 bytes, added SAVE to class command
 *  26-Feb-2021 1.0g  TRL - added AT+DR=SCHEME to setDataRate()  (It should have its oun command...)
 *  05-Apr-2021 1.0h  TRL - added test for modem busy after a command --> LinkBusyCheck()
 *  13-Apr-2021 1.0i  TRL - Added new error checking for Auto-Join
 
 *
 *  Notes:  1)  Tested with Arduino 1.8.13
 *          2)  Testing with RHF0M062 ver 3.5.13, 3.5.14 
 *          3)  added (char*) to sendCommand to fix compiler warning
 *          4)  added command for getDevEui, getDevAddr, getAppEui, getlw=len 
 *                  (some are not implemented in RHF0M062 ver 3.5.13)
 *          5)  added sendWakeUp() command to wake device from deep sleep..
 *          6)  added getSystemTime(), getRTC(), getRTCFull()
 *          7)  added getDR, getTest, setNetworkPublic
 *          8)  start of Class B commands
 *          9)  Serial1 is use for commands to radio
 *          10) Serial is use for debug messages if: _DEBUG_SERIAL_ is defined
 *          11) correct typo: Receice --> Receive, added Get for RXWIN1, RXWIN2
 *          12) delay1, buffer1 are using serial timeouts, work best if _DEBUG_SERIAL_ is NOT defined
 *          13) 
 *
 *
 *          
 *  Todo:   1) add addition new command as needed...
 *          2) finish adding class B commands
 *          3) 
 *          4) 
 *          5) 
 * 
 * TRL --> Tom Lafleur --> tom@lafleur.us
 */



#include "LoRaWan.h"

/* ************************************************************** */
// define a data structure for modem error-stat parameters we need
LoRaData  LoRaStats =
{
   .CurrentRSSI     =  0,
   .LastError       =  0,
   .CurrentSNR      =  0,
   .CurrentLINK     =  0,
   .GatewayCNT      =  0,
   .ErrorCNT        =  0,
   .LastErrorCode   =  0,
   .ACKFlag         =  false,
   .DoneFlag        =  false,
   .JoinFlag        =  false
};

 

extern LoRaData  LoRaStats;

LoRaWanClass::LoRaWanClass(void)
{
    memset(_buffer, 0, BUFFER_LENGTH_MAX);
}

void LoRaWanClass::init(void)
{
    SerialLoRa.begin(9600);
}

void LoRaWanClass::sendCMD(char* CMD)
{
    char cmd[64];
    
        while(SerialLoRa.available())SerialLoRa.read();     // flush serial stream
        
        memset(cmd, 0, 64);
        sprintf(cmd, "AT+%s\r\n", CMD);
        sendCommand(cmd);
#if _DEBUG_SERIAL_
        loraDebugPrint(DEFAULT_DEBUGTIME);
#endif
        delay1(DEFAULT_TIMEWAIT);
}


/*  ************************************************************** */
void LoRaWanClass::sendCMD(char* CMD, char *buffer, short length, unsigned int timeout)
{
    char cmd[64];
    
        while(SerialLoRa.available())SerialLoRa.read();
    
        memset(cmd, 0, 64);
        sprintf(cmd, "AT+%s\r\n", CMD);
        sendCommand(cmd);
#if _DEBUG_SERIAL_
        loraDebugPrint(DEFAULT_DEBUGTIME);
#endif
        //delay(DEFAULT_TIMEWAIT);
    if(buffer)
    {
        while(SerialLoRa.available())SerialLoRa.read();
        sendCommand( (char*) "AT+CH\r\n");
        readBuffer1(buffer, length, timeout);    
    }
}


/*  ************************************************************** */
bool LoRaWanClass::sendCMD(char* CMD, char* deviceRespond, char *buffer, short length, unsigned int timeout)
{
    char cmd[64];
    
        while(SerialLoRa.available())SerialLoRa.read();
        
        memset(cmd, 0, 64);
        sprintf(cmd, "AT%s\r\n", CMD);
        sendCommand(cmd);
        
    if(buffer)
    {
        readBuffer1(buffer, length, timeout);
        if(strstr(buffer, deviceRespond)) return true;
    }
    return false;  
}


/*  ************************************************************** */
void LoRaWanClass::getChannel(char *buffer, short length, unsigned int timeout)
{
    while(SerialLoRa.available())SerialLoRa.read();
    
    if(buffer)
    {
        while(SerialLoRa.available())SerialLoRa.read();
        sendCommand( (char*) "AT+CH\r\n");
        readBuffer1(buffer, length, timeout);    
    }
}


/*  ************************************************************** */
bool LoRaWanClass::getTest( unsigned int timeout)
{
    //char *ptr;
    
    while(SerialLoRa.available())SerialLoRa.read();     // flush serial stream
    
    sendCommand( (char*) "AT\r\n");
    
//    memset(_buffer, 0, BUFFER_LENGTH_MAX);
//    readBuffer1(_buffer, BUFFER_LENGTH_MAX, timeout);
    
    return waitForResponse( (char*) "+AT: OK\r\n", timeout);
    
//#if _DEBUG_SERIAL_    
//    Serial.print(_buffer);
//#endif  
//    getMessage();
//    ptr = strstr(_buffer, "+AT: OK");
//    if(ptr) return true;
//    
//    return false;
}  

/* ************************************************************** */
void LoRaWanClass::getVersion(char *buffer, short length, unsigned int timeout)
{
    while(SerialLoRa.available())SerialLoRa.read();
    
    if(buffer)
    {
        while(SerialLoRa.available())SerialLoRa.read();
        sendCommand( (char*) "AT+VER=?\r\n");
        readBuffer1(buffer, length, timeout);    
    }
}


/*  ************************************************************** */
void LoRaWanClass::getProtocalVersion(char *buffer, short length, unsigned int timeout)
{
    while(SerialLoRa.available())SerialLoRa.read();
    
    if(buffer)
    {
        while(SerialLoRa.available())SerialLoRa.read();
        sendCommand( (char*) "AT+LW=VER\r\n");
        readBuffer1(buffer, length, timeout);    
    }
}


/*  ************************************************************** */
void LoRaWanClass::getReceiveWindowFirst(char *buffer, short length, unsigned int timeout)
{
    while(SerialLoRa.available())SerialLoRa.read();
    
    if(buffer)
    {
        while(SerialLoRa.available())SerialLoRa.read();
        sendCommand( (char*) "AT+RXWIN1\r\n");
        readBuffer1(buffer, length, timeout);    
    }
}


/*  ************************************************************** */
void LoRaWanClass::getReceiveWindowSecond(char *buffer, short length, unsigned int timeout)
{
 
    while(SerialLoRa.available())SerialLoRa.read();
        
    if(buffer)
    {
        while(SerialLoRa.available())SerialLoRa.read();
        sendCommand( (char*) "AT+RXWIN2\r\n");
        readBuffer1(buffer, length, timeout);    
    }
}


/*  ************************************************************** */
void LoRaWanClass::getId(char *buffer, short length, unsigned int timeout)
{
    while(SerialLoRa.available())SerialLoRa.read();
        
    if(buffer)
    {
        while(SerialLoRa.available())SerialLoRa.read();
        sendCommand( (char*) "AT+ID=?\r\n");
        readBuffer1(buffer, length, timeout);    
    }
}


/*  ************************************************************** */
void LoRaWanClass::getDevEui(char *buffer, short length, unsigned int timeout)
{
    while(SerialLoRa.available())SerialLoRa.read();
    
    if(buffer)
    {
        while(SerialLoRa.available())SerialLoRa.read();
        sendCommand( (char*) "AT+ID=DevEui\r\n");
        readBuffer1(buffer, length, timeout);    
    }
}


/*  ************************************************************** */
void LoRaWanClass::getDevAddr(char *buffer, short length, unsigned int timeout)
{
    while(SerialLoRa.available())SerialLoRa.read();
    
    if(buffer)
    {
        while(SerialLoRa.available())SerialLoRa.read();
        sendCommand( (char*) "AT+ID=DevAddr\r\n");
        readBuffer1(buffer, length, timeout);    
    }
}


/*  ************************************************************** */
void LoRaWanClass::getAppEui(char *buffer, short length, unsigned int timeout)
{
     while(SerialLoRa.available())SerialLoRa.read();
    
    if(buffer)
    {
        while(SerialLoRa.available())SerialLoRa.read();
        sendCommand( (char*) "AT+ID=AppEui\r\n");
        readBuffer1(buffer, length, timeout);    
    }
}


/*  ************************************************************** */
void LoRaWanClass::getMaxLen(char *buffer, short length, unsigned int timeout)
{
    while(SerialLoRa.available())SerialLoRa.read();
    
    if(buffer)
    {
        while(SerialLoRa.available())SerialLoRa.read();
        sendCommand( (char*) "AT+LW=LEN\r\n");
        readBuffer1(buffer, length, timeout);    
    }
}


/*  ************************************************************** */
void LoRaWanClass::getSystemTime(char *buffer, short length, unsigned int timeout)
{
    while(SerialLoRa.available())SerialLoRa.read();
    
    if(buffer)
    {
        while(SerialLoRa.available())SerialLoRa.read();
        sendCommand( (char*) "AT+LW=DTR\r\n");
        readBuffer1(buffer, length, timeout);    
    }
}


/*  ************************************************************** */
void LoRaWanClass::getRTC(char *buffer, short length, unsigned int timeout)
{
    while(SerialLoRa.available())SerialLoRa.read();
    
    if(buffer)
    {
        while(SerialLoRa.available())SerialLoRa.read();
        sendCommand( (char*) "AT+RTC\r\n");
        readBuffer1(buffer, length, timeout);    
    }
}


/*  ************************************************************** */
void LoRaWanClass::getRTCFull(char *buffer, short length, unsigned int timeout)
{
    while(SerialLoRa.available())SerialLoRa.read();
    
    if(buffer)
    {
        while(SerialLoRa.available())SerialLoRa.read();
        sendCommand( (char*) "AT+RTC=FULL\r\n");
        readBuffer1(buffer, length, timeout);    
    }
}


/* ************************************************************** */
void LoRaWanClass::setRTC(char* RTCCMD)
{
    char cmd[32];
    
    while(SerialLoRa.available())SerialLoRa.read();     // flush serial stream
        
    memset(cmd, 0, 32);
    sprintf(cmd,  (char*) "AT+RTC=%s\r\n", RTCCMD);
    sendCommand(cmd);
    
#if _DEBUG_SERIAL_
    loraDebugPrint(DEFAULT_DEBUGTIME);
#endif
    delay1(DEFAULT_TIMEWAIT);
}


/* ************************************************************** */
void LoRaWanClass::setId(char *DevAddr, char *DevEUI, char *AppEUI)
{
    char cmd[64];
    
    if(DevAddr)
    {
        while(SerialLoRa.available())SerialLoRa.read();     // flush serial stream
        
        memset(cmd, 0, 64);
        sprintf(cmd, "AT+ID=DevAddr,\"%s\"\r\n", DevAddr);
        sendCommand(cmd);
        
#if _DEBUG_SERIAL_
        loraDebugPrint(DEFAULT_DEBUGTIME);
#endif
       delay1(DEFAULT_TIMEWAIT);
    }
    
    if(DevEUI)
    {
        while(SerialLoRa.available())SerialLoRa.read();     // flush serial stream
        
        memset(cmd, 0, 64);
        sprintf(cmd, "AT+ID=DevEui,\"%s\"\r\n", DevEUI);
        sendCommand(cmd);
        
#if _DEBUG_SERIAL_
        loraDebugPrint(DEFAULT_DEBUGTIME);
#endif
       delay1(DEFAULT_TIMEWAIT);
    }
    
    if(AppEUI)
    {
        while(SerialLoRa.available())SerialLoRa.read();     // flush serial stream
        
        memset(cmd, 0, 64);
        sprintf(cmd, "AT+ID=AppEui,\"%s\"\r\n", AppEUI);
        sendCommand(cmd);
        
#if _DEBUG_SERIAL_
        loraDebugPrint(DEFAULT_DEBUGTIME);
#endif
       delay1(DEFAULT_TIMEWAIT);
    }
}


/*  ************************************************************** */
void LoRaWanClass::setKey(char *NwkSKey, char *AppSKey, char *AppKey)
{
    char cmd[64];
    
    if(NwkSKey)
    {
        while(SerialLoRa.available())SerialLoRa.read();     // flush serial stream
        
        memset(cmd, 0, 64);
        sprintf(cmd, "AT+KEY=NWKSKEY,\"%s\"\r\n", NwkSKey);
        sendCommand(cmd);
        
#if _DEBUG_SERIAL_
        loraDebugPrint(DEFAULT_DEBUGTIME);
#endif
        delay1(DEFAULT_TIMEWAIT);
    }

    if(AppSKey)
    {
        while(SerialLoRa.available())SerialLoRa.read();     // flush serial stream
        
        memset(cmd, 0, 64);
        sprintf(cmd, "AT+KEY=APPSKEY,\"%s\"\r\n", AppSKey);
        sendCommand(cmd);
        
#if _DEBUG_SERIAL_
        loraDebugPrint(DEFAULT_DEBUGTIME);
#endif
        delay1(DEFAULT_TIMEWAIT);
    }

    if(AppKey)
    {
        while(SerialLoRa.available())SerialLoRa.read();     // flush serial stream 
               
        memset(cmd, 0, 64);
        sprintf(cmd, "AT+KEY= APPKEY,\"%s\"\r\n", AppKey);
        sendCommand(cmd);
        
#if _DEBUG_SERIAL_
        loraDebugPrint(DEFAULT_DEBUGTIME);
#endif
        delay1(DEFAULT_TIMEWAIT);
    }
}


/*  ************************************************************** */
void LoRaWanClass::getDR(char *buffer, short length, unsigned int timeout)
{
    if(buffer)
    {
        while(SerialLoRa.available())SerialLoRa.read();
        
        sendCommand( (char*) "AT+DR\r\n");
        readBuffer1(buffer, length, timeout);    
    }
}


/*  ************************************************************** */
void LoRaWanClass::setDataRate(_data_rate_t dataRate)
{
    char cmd[32];
    
    while(SerialLoRa.available())SerialLoRa.read();     // flush serial stream
        
    memset(cmd, 0, 32);
    sprintf(cmd, "AT+DR=%d\r\n", dataRate);
    sendCommand(cmd);
#if _DEBUG_SERIAL_
    loraDebugPrint(DEFAULT_DEBUGTIME);
#endif
    delay1(DEFAULT_TIMEWAIT);
}


void LoRaWanClass::setDataRate( _physical_type_t physicalType)
{
    char cmd[32];
    while(SerialLoRa.available())SerialLoRa.read();     // flush serial stream
    
    if(physicalType == EU434)sendCommand(               (char*) "AT+DR=EU433\r\n");
    else if(physicalType == EU868)sendCommand(          (char*) "AT+DR=EU868\r\n");
    else if(physicalType == US915)sendCommand(          (char*) "AT+DR=US915\r\n");
    else if(physicalType == US915HYBRID)sendCommand(    (char*) "AT+DR=US915HYBRID\r\n");
    else if(physicalType == AU915)sendCommand(          (char*) "AT+DR=AU915\r\n");
	else if(physicalType == AU915OLD)sendCommand(       (char*) "AT+DR=AU915OLD\r\n");
    else if(physicalType == CN470)sendCommand(          (char*) "AT+DR=CN470\r\n");
    else if(physicalType == CN779)sendCommand(          (char*) "AT+DR=CN779\r\n");
    else if(physicalType == AS923)sendCommand(          (char*) "AT+DR=AS923\r\n");
    else if(physicalType == KR920)sendCommand(          (char*) "AT+DR=KR920\r\n");
    else if(physicalType == IN865)sendCommand(          (char*) "AT+DR=IN865\r\n");
	else if(physicalType == IN865)sendCommand(          (char*) "AT+DR=SCHEME\r\n");
	    
#if _DEBUG_SERIAL_
    loraDebugPrint(DEFAULT_DEBUGTIME);
#endif
    delay1(DEFAULT_TIMEWAIT);
}


void LoRaWanClass::setDataRate(_data_rate_t dataRate, _physical_type_t physicalType)
{
    char cmd[32];
    while(SerialLoRa.available())SerialLoRa.read();     // flush serial stream
    
    if(physicalType == EU434)sendCommand(               (char*) "AT+DR=EU433\r\n");
    else if(physicalType == EU868)sendCommand(          (char*) "AT+DR=EU868\r\n");
    else if(physicalType == US915)sendCommand(          (char*) "AT+DR=US915\r\n");
    else if(physicalType == US915HYBRID)sendCommand(    (char*) "AT+DR=US915HYBRID\r\n");
    else if(physicalType == AU915)sendCommand(          (char*) "AT+DR=AU915\r\n");
	else if(physicalType == AU915OLD)sendCommand(       (char*) "AT+DR=AU915OLD\r\n");
    else if(physicalType == CN470)sendCommand(          (char*) "AT+DR=CN470\r\n");
    else if(physicalType == CN779)sendCommand(          (char*) "AT+DR=CN779\r\n");
    else if(physicalType == AS923)sendCommand(          (char*) "AT+DR=AS923\r\n");
    else if(physicalType == KR920)sendCommand(          (char*) "AT+DR=KR920\r\n");
    else if(physicalType == IN865)sendCommand(          (char*) "AT+DR=IN865\r\n");
	
#if _DEBUG_SERIAL_
    loraDebugPrint(DEFAULT_DEBUGTIME);
#endif
    delay1(DEFAULT_TIMEWAIT);
    
    while(SerialLoRa.available())SerialLoRa.read();     // flush serial stream
    memset(cmd, 0, 32);
    sprintf(cmd, "AT+DR=%d\r\n", dataRate);
    sendCommand(cmd);
    
#if _DEBUG_SERIAL_
    loraDebugPrint(DEFAULT_DEBUGTIME);
#endif
    delay1(DEFAULT_TIMEWAIT);
}


/*  ************************************************************** */
void LoRaWanClass::sendWakeUp(void)
{
    char cmd[32];
    
    while(SerialLoRa.available())SerialLoRa.read();     // flush serial stream
    
    memset(cmd, 0, 32);
    cmd[0] = 0xff;
    cmd[1] = 0xff;
    cmd[2] = 0xff;
    cmd[3] = 0xff;
    sprintf(cmd+4, "AT+VER\r\n");
    sendCommand(cmd);
    
#if _DEBUG_SERIAL_
    loraDebugPrint(DEFAULT_DEBUGTIME);
#endif
    delay1(DEFAULT_TIMEWAIT);
}


/*  ************************************************************** */    
bool LoRaWanClass::sendWakeUp(unsigned int timeout)
{
    char cmd[32];
    char *ptr;
    
    while(SerialLoRa.available())SerialLoRa.read();     // flush serial stream
          
    memset(cmd, 0, 32);                                 // clear CMD buffer
    cmd[0] = 0xff;                                      // send wakeup sequence
    cmd[1] = 0xff;
    cmd[2] = 0xff;
    cmd[3] = 0xff;
    sprintf(cmd+4, "AT+VER\r\n");                       // send a command and see if its awake
    sendCommand(cmd);
    
    memset(_buffer, 0, BUFFER_LENGTH_MAX);              // clear return RX buffer
    readBuffer1(_buffer, BUFFER_LENGTH_MAX, timeout);   // get any message from device
    
#if _DEBUG_SERIAL_    
    Serial.print(_buffer);
#endif  
    getMessage();
    ptr = strstr(_buffer, "+VER:");                     // check for correct respond
    if(ptr) return true;
    
    return false;
}


/*  ************************************************************** */
void LoRaWanClass::setPower(short power)
{
    char cmd[32];
    
    while(SerialLoRa.available())SerialLoRa.read();
        
    memset(cmd, 0, 32);
    sprintf(cmd, "AT+POWER=%d\r\n", power);
    sendCommand(cmd);
    
#if _DEBUG_SERIAL_
    loraDebugPrint(DEFAULT_DEBUGTIME);
#endif
    delay1(DEFAULT_TIMEWAIT);
}


/*  ************************************************************** */
void LoRaWanClass::setPort(unsigned char port)
{
    char cmd[32];
    
    while(SerialLoRa.available())SerialLoRa.read();
        
    memset(cmd, 0, 32);
    sprintf(cmd, "AT+PORT=%d\r\n", port);
    
    sendCommand(cmd);
#if _DEBUG_SERIAL_
    loraDebugPrint(DEFAULT_DEBUGTIME);
#endif
    delay1(DEFAULT_TIMEWAIT);
}


/*  ************************************************************** */
void LoRaWanClass::setAdaptiveDataRate(bool command)
{
    while(SerialLoRa.available())SerialLoRa.read();
    
    if(command)sendCommand( (char*) "AT+ADR=ON\r\n");
    else sendCommand(       (char*) "AT+ADR=OFF\r\n");
        
#if _DEBUG_SERIAL_
    loraDebugPrint(DEFAULT_DEBUGTIME);
#endif
    delay1(DEFAULT_TIMEWAIT);
}


/*  ************************************************************** */
void LoRaWanClass::setNetworkPublic(bool command)
{
    while(SerialLoRa.available())SerialLoRa.read();     // flush serial stream
        
    if(command)sendCommand( (char*) "AT+LW=NET, ON\r\n");
    else sendCommand(       (char*) "AT+LW=NET, OFF\r\n");
        
#if _DEBUG_SERIAL_
    loraDebugPrint(DEFAULT_DEBUGTIME);
#endif
    delay1(DEFAULT_TIMEWAIT);
}


/*  ************************************************************** */
void LoRaWanClass::setChannels(char* channel)
{
    char cmd[32];
    
    while(SerialLoRa.available())SerialLoRa.read();     // flush serial stream
        
    memset(cmd, 0, 32);
    sprintf(cmd,  (char*) "AT+CH=NUM %s\r\n", channel);
    sendCommand(cmd);
    
#if _DEBUG_SERIAL_
    loraDebugPrint(DEFAULT_DEBUGTIME);
#endif
    delay1(DEFAULT_TIMEWAIT);
}


/*  ************************************************************** */
void LoRaWanClass::setChannel(unsigned char channel, float frequency)
{
    char cmd[32];
    
    while(SerialLoRa.available())SerialLoRa.read();     // flush serial stream
        
    memset(cmd, 0, 32);
    sprintf(cmd,  (char*) "AT+CH=%d,%d.%d\r\n", channel, (short)frequency, short(frequency * 10) % 10);
    sendCommand(cmd);
    
#if _DEBUG_SERIAL_
    loraDebugPrint(DEFAULT_DEBUGTIME);
#endif
    delay1(DEFAULT_TIMEWAIT);
}


/*  ************************************************************** */
void LoRaWanClass::setChannel(unsigned char channel, float frequency, _data_rate_t dataRata)
{
    char cmd[32];
    
    while(SerialLoRa.available())SerialLoRa.read();     // flush serial stream
        
    memset(cmd, 0, 32);
    sprintf(cmd,  (char*) "AT+CH=%d,%d.%d,%d\r\n", channel, (short)frequency, short(frequency * 10) % 10, dataRata);
    sendCommand(cmd);
    
#if _DEBUG_SERIAL_
    loraDebugPrint(DEFAULT_DEBUGTIME);
#endif
    delay1(DEFAULT_TIMEWAIT);
}


/* ************************************************************** */
void LoRaWanClass::setChannel(unsigned char channel, float frequency, _data_rate_t dataRataMin, _data_rate_t dataRataMax)
{
    char cmd[32];
    
    while(SerialLoRa.available())SerialLoRa.read();     // flush serial stream
        
    memset(cmd, 0, 32);
    sprintf(cmd, (char*)  "AT+CH=%d,%d.%d,%d,%d\r\n", channel, (short)frequency, short(frequency * 10) % 10, dataRataMin, dataRataMax);
    sendCommand(cmd);
    
#if _DEBUG_SERIAL_
    loraDebugPrint(DEFAULT_DEBUGTIME);
#endif
    delay1(DEFAULT_TIMEWAIT);
}


/*  ************************************************************** */
bool LoRaWanClass::LinkCheckReq(unsigned int timeout)
{
    while(SerialLoRa.available())SerialLoRa.read();
  
    sendCommand( (char*) "AT+MSG\r\n");
    
    bool result =  waitForResponse( (char*) "Done\r\n", timeout);
    getMessage();
    return result;
    
//    memset(_buffer, 0, BUFFER_LENGTH_MAX);
//    readBuffer(_buffer, BUFFER_LENGTH_MAX, timeout);
//#if _DEBUG_SERIAL_    
//    Serial.print(_buffer);
//#endif
//    getMessage();    
//    if(strstr(_buffer, "Done"))return true;
//    return false;
}


/*  ************************************************************** */
bool LoRaWanClass::LinkCheckReq1(unsigned int timeout)
{
    while(SerialLoRa.available())SerialLoRa.read();
  
    sendCommand( (char*) "AT+CMSGHEX\r\n");
    
    bool result = waitForResponse( (char*) "Done\r\n", timeout);
    getMessage();
    return result;
    
//    memset(_buffer, 0, BUFFER_LENGTH_MAX);
//    readBuffer(_buffer, BUFFER_LENGTH_MAX, timeout);
//#if _DEBUG_SERIAL_    
//    Serial.print(_buffer);
//#endif
//    getMessage();    
//    if(strstr(_buffer, "Done"))return true;
//    return false;
}


/*  ************************************************************** */
bool LoRaWanClass::transferPacket(char *buffer, unsigned int timeout)
{
    unsigned char length = strlen(buffer);
    
    while(SerialLoRa.available())SerialLoRa.read();
    
    sendCommand( (char*) "AT+MSG=\"");
    for(unsigned char i = 0; i < length; i ++)SerialLoRa.write(buffer[i]);
    sendCommand( (char*) "\"\r\n");
    
    bool result = waitForResponse( (char*) "Done\r\n", timeout);
    getMessage();
    return result;
    
//    memset(_buffer, 0, BUFFER_LENGTH_MAX);
//    readBuffer(_buffer, BUFFER_LENGTH_MAX, timeout);
//#if _DEBUG_SERIAL_    
//    Serial.print(_buffer);
//#endif
//    getMessage();    
//    if(strstr(_buffer, "+MSG: Done"))return true;
//    return false;
}


/*  ************************************************************** */
bool LoRaWanClass::transferPacket(unsigned char *buffer, unsigned char length, unsigned int timeout)
{
    char temp[2] = {0};
    
    while(SerialLoRa.available()) SerialLoRa.read();
    
    sendCommand( (char*) "AT+MSGHEX=\"");
    for(unsigned char i = 0; i < length; i ++)
    {
        sprintf(temp,"%02x", buffer[i]);
        SerialLoRa.write(temp); 
    }
    sendCommand( (char*) "\"\r\n");
    
    bool result = waitForResponse( (char*) "Done\r\n", timeout);
    getMessage();
    return result;
        
//    memset(_buffer, 0, BUFFER_LENGTH_MAX);
//    readBuffer(_buffer, BUFFER_LENGTH_MAX, timeout);
//#if _DEBUG_SERIAL_    
//    Serial.print(_buffer);
//#endif
//    getMessage();    
//    if(strstr(_buffer, "+MSGHEX: Done"))return true;
//    return false;
}


/*  ************************************************************** */
bool LoRaWanClass::transferPacketWithConfirmed(char *buffer, unsigned int timeout)
{
    unsigned char length = strlen(buffer);
    
    while(SerialLoRa.available())SerialLoRa.read();
    
    sendCommand( (char*) "AT+CMSG=\"");
    for(unsigned char i = 0; i < length; i ++)SerialLoRa.write(buffer[i]);
    sendCommand( (char*) "\"\r\n");
    
    bool result = waitForResponse( (char*) "Done\r\n", timeout);
    getMessage();
    return result;    
        
//    memset(_buffer, 0, BUFFER_LENGTH_MAX);
//    readBuffer(_buffer, BUFFER_LENGTH_MAX, timeout);
//#if _DEBUG_SERIAL_    
//    Serial.print(_buffer);
//#endif
//    getMessage();      
//    if(strstr(_buffer, "ACK Received"))return true;
//    return false;
}


/*  ************************************************************** */
bool LoRaWanClass::transferPacketWithConfirmed(unsigned char *buffer, unsigned char length, unsigned int timeout)
{
    char temp[2] = {0};
    
    while(SerialLoRa.available())SerialLoRa.read();
    
    sendCommand( (char*) "AT+CMSGHEX=\"");
    for(unsigned char i = 0; i < length; i ++)
    {
        sprintf(temp,"%02x", buffer[i]);
        SerialLoRa.write(temp); 
    }
    sendCommand( (char*) "\"\r\n");
    
    bool result = waitForResponse( (char*) "Done\r\n", timeout);
    getMessage();
    return result;
//    
//#if _DEBUG_SERIAL_    
//    Serial.print(_buffer);
//#endif      
//    memset(_buffer, 0, BUFFER_LENGTH_MAX);
//    readBuffer(_buffer, BUFFER_LENGTH_MAX, timeout);
//    getMessage();
//    if(strstr(_buffer, "ACK Received"))return true;
//    return false;
}


/*  ************************************************************** */
short LoRaWanClass::receivePacket(char *buffer, short length, short *rssi)
{
    char *ptr;
    short number = 0;
    
    ptr = strstr(_buffer, "RSSI ");
    if(ptr)*rssi = atoi(ptr + 5);
    else *rssi = -255;
    
    ptr = strstr(_buffer, "RX: \"");
    if(ptr)
    {        
        ptr += 5;
        
        uint8_t bitStep = 0;
        if(*(ptr + 2) == ' ')bitStep = 3;   // Firmware version 2.0.10
        else bitStep = 2;                   // Firmware version 2.1.15
        
        for(short i = 0; ; i ++)
        {
            char temp[2] = {0};
            unsigned char tmp, result = 0;
            
            temp[0] = *(ptr + i * bitStep);
            temp[1] = *(ptr + i * bitStep + 1);
           
            for(unsigned char j = 0; j < 2; j ++)
            {
                if((temp[j] >= '0') && (temp[j] <= '9'))
                tmp = temp[j] - '0';
                else if((temp[j] >= 'A') && (temp[j] <= 'F'))
                tmp = temp[j] - 'A' + 10;
                else if((temp[j] >= 'a') && (temp[j] <= 'f'))
                tmp = temp[j] - 'a' + 10;

                result = result * 16 + tmp;
            }
            
            if(i < length)buffer[i] = result;

            if(*(ptr + (i + 1) * bitStep) == '\"' && *(ptr + (i + 1) * bitStep + 1) == '\r' && *(ptr + (i + 1) * bitStep + 2) == '\n')
            {
                number = i + 1;
                break;
            }
        }        
    }
       
    memset(_buffer, 0, BUFFER_LENGTH_MAX);
    
    return number;
}

/*  ************************************************************** */
short LoRaWanClass::receivePacket(char *buffer, short length, short *rssi, short *port)
{
    char *ptr;
    short number = 0;
    
    ptr = strstr(_buffer, "RSSI ");
    if(ptr) *rssi = atoi(ptr + 5);
    else *rssi = -255;
    
    ptr = strstr(_buffer, "PORT: ");
    if(ptr) *port = atoi(ptr + 6);
        
    ptr = strstr(_buffer, "RX: \"");
    if(ptr)
    {        
        ptr += 5;
        
        uint8_t bitStep = 0;
        if(*(ptr + 2) == ' ')bitStep = 3;   // Firmware version 2.0.10
        else bitStep = 2;                   // Firmware version 2.1.15
        
        for(short i = 0; ; i ++)
        {
            char temp[2] = {0};
            unsigned char tmp, result = 0;
            
            temp[0] = *(ptr + i * bitStep);
            temp[1] = *(ptr + i * bitStep + 1);
           
            for(unsigned char j = 0; j < 2; j ++)
            {
                if((temp[j] >= '0') && (temp[j] <= '9'))
                tmp = temp[j] - '0';
                else if((temp[j] >= 'A') && (temp[j] <= 'F'))
                tmp = temp[j] - 'A' + 10;
                else if((temp[j] >= 'a') && (temp[j] <= 'f'))
                tmp = temp[j] - 'a' + 10;

                result = result * 16 + tmp;
            }
            
            if(i < length)buffer[i] = result;

            if(*(ptr + (i + 1) * bitStep) == '\"' && *(ptr + (i + 1) * bitStep + 1) == '\r' && *(ptr + (i + 1) * bitStep + 2) == '\n')
            {
                number = i + 1;
                break;
            }
        }        
    }
       
    memset(_buffer, 0, BUFFER_LENGTH_MAX);
    
    return number;
}

/*  ************************************************************** */
bool LoRaWanClass::transferProprietaryPacket(char *buffer, unsigned int timeout)
{
    unsigned char length = strlen(buffer);
    
    while(SerialLoRa.available())SerialLoRa.read();
    
    sendCommand( (char*) "AT+PMSG=\"");
    for(unsigned char i = 0; i < length; i ++)SerialLoRa.write(buffer[i]);
    sendCommand( (char*) "\"\r\n");
    
    bool result = waitForResponse( (char*) "Done\r\n", timeout);
    getMessage();
    return result;
    
//    memset(_buffer, 0, BUFFER_LENGTH_MAX);
//    readBuffer(_buffer, BUFFER_LENGTH_MAX, timeout);
//#if _DEBUG_SERIAL_    
//    Serial.print(_buffer);
//#endif    
//    getMessage();
//    if(strstr(_buffer, "+PMSG: Done"))return true;
//    return false;
}


/*  ************************************************************** */
bool LoRaWanClass::transferProprietaryPacket(unsigned char *buffer, unsigned char length, unsigned int timeout)
{
    char temp[2] = {0};
    
    while(SerialLoRa.available())SerialLoRa.read();
    
    sendCommand( (char*) "AT+PMSGHEX=\"");
    for(unsigned char i = 0; i < length; i ++)
    {
        sprintf(temp,"%02x", buffer[i]);
        SerialLoRa.write(temp); 
    }
    sendCommand( (char*) "\"\r\n");
    
    bool result = waitForResponse( (char*) "Done\r\n", timeout);
    getMessage();
    return result;
    
//    memset(_buffer, 0, BUFFER_LENGTH_MAX);
//    readBuffer(_buffer, BUFFER_LENGTH_MAX, timeout);
//#if _DEBUG_SERIAL_    
//    Serial.print(_buffer);
//#endif    
//    getMessage();
//    if(strstr(_buffer, "+PMSGHEX: Done"))return true;
//    return false;
}

 /* ************************************************************** */       
void LoRaWanClass::setUnconfirmedMessageRepeatTime(unsigned char time)
{
    char cmd[32];
    
    while(SerialLoRa.available())SerialLoRa.read();     // flush serial stream
        
    if(time > 15) time = 15;
    else if(time == 0) time = 1;
    
    memset(cmd, 0, 32);
    sprintf(cmd, "AT+REPT=%d\r\n", time);
    sendCommand(cmd);
    
#if _DEBUG_SERIAL_
    loraDebugPrint(DEFAULT_DEBUGTIME);
#endif
    delay1(DEFAULT_TIMEWAIT);
}


/*  ************************************************************** */
void LoRaWanClass::setConfirmedMessageRetryTime(unsigned char time)
{
    char cmd[32];
    
    while(SerialLoRa.available())SerialLoRa.read();     // flush serial stream
        
    if(time > 15) time = 15;
    else if(time == 0) time = 1;
    
    memset(cmd, 0, 32);
    sprintf(cmd, "AT+RETRY=%d\r\n", time);
    sendCommand(cmd);
    
#if _DEBUG_SERIAL_
    loraDebugPrint(DEFAULT_DEBUGTIME);
#endif
    delay1(DEFAULT_TIMEWAIT);    
}


/*  ************************************************************** */
void LoRaWanClass::setReceiveWindowFirst(bool command)
{
    while(SerialLoRa.available())SerialLoRa.read();     // flush serial stream
       
    if(command)sendCommand( (char*) "AT+RXWIN1=ON\r\n");
    else sendCommand(       (char*) "AT+RXWIN1=OFF\r\n");
        
#if _DEBUG_SERIAL_
    loraDebugPrint(DEFAULT_DEBUGTIME);
#endif
    delay1(DEFAULT_TIMEWAIT);
}


/*  ************************************************************** */
void LoRaWanClass::setReceiveWindowFirst(unsigned char channel, float frequency)
{
    char cmd[32];
    
    while(SerialLoRa.available())SerialLoRa.read();     // flush serial stream
        
    memset(cmd, 0, 32);
    sprintf(cmd, "AT+RXWIN1=%d,%d.%d\r\n", channel, (short)frequency, short(frequency * 10) % 10);
    sendCommand(cmd);
    
#if _DEBUG_SERIAL_
    loraDebugPrint(DEFAULT_DEBUGTIME);
#endif
    delay1(DEFAULT_TIMEWAIT);
}


/*  ************************************************************** */
void LoRaWanClass::setReceiveWindowSecond(float frequency, _data_rate_t dataRate)
{
    char cmd[32];
    
    while(SerialLoRa.available())SerialLoRa.read();     // flush serial stream
       
    memset(cmd, 0, 32);
    sprintf(cmd, "AT+RXWIN2=%d.%d,%d\r\n", (short)frequency, short(frequency * 10) % 10, dataRate);
    sendCommand(cmd);
    
#if _DEBUG_SERIAL_
    loraDebugPrint(DEFAULT_DEBUGTIME);
#endif
    delay1(DEFAULT_TIMEWAIT);
}


/*  ************************************************************** */
void LoRaWanClass::setReceiveWindowSecond(float frequency, _spreading_factor_t spreadingFactor, _band_width_t bandwidth)
{
    char cmd[32];
    
    while(SerialLoRa.available())SerialLoRa.read();     // flush serial stream
        
    memset(cmd, 0, 32);
    sprintf(cmd, "AT+RXWIN2=%d.%d,%d,%d\r\n", (short)frequency, short(frequency * 10) % 10, spreadingFactor, bandwidth);
    sendCommand(cmd);
    
#if _DEBUG_SERIAL_
    loraDebugPrint(DEFAULT_DEBUGTIME);
#endif
    delay1(DEFAULT_TIMEWAIT);
}


/*  ************************************************************** */
void LoRaWanClass::setDutyCycle(bool command)
{
    while(SerialLoRa.available())SerialLoRa.read();     // flush serial stream
    
    if(command)sendCommand( (char*) "AT+LW=DC, ON\r\n");
    else sendCommand(       (char*) "AT+LW=DC, OFF\r\n");
          
#if _DEBUG_SERIAL_
    loraDebugPrint(DEFAULT_DEBUGTIME);
#endif
    delay1(DEFAULT_TIMEWAIT);
}


/*  ************************************************************** */
void LoRaWanClass::setJoinDutyCycle(bool command)
{
    while(SerialLoRa.available())SerialLoRa.read();     // flush serial stream
    
    if(command)sendCommand( (char*) "AT+LW=JDC,ON\r\n");
    else sendCommand(       (char*) "AT+LW=JDC,OFF\r\n"); 
         
#if _DEBUG_SERIAL_
    loraDebugPrint(DEFAULT_DEBUGTIME);
#endif
    delay1(DEFAULT_TIMEWAIT);
}


/*  ************************************************************** */
void LoRaWanClass::setReceiveWindowDelay(_window_delay_t command, unsigned short _delay)
{
    char cmd[32];
    
    while(SerialLoRa.available())SerialLoRa.read();     // flush serial stream
        
    memset(cmd, 0, 32);
    if(command == RECEIVE_DELAY1) sprintf(cmd,          "AT+DELAY=RX1,%d\r\n", _delay);
    else if(command == RECEIVE_DELAY2) sprintf(cmd,     "AT+DELAY=RX2,%d\r\n", _delay);
    else if(command == JOIN_ACCEPT_DELAY1) sprintf(cmd, "AT+DELAY=JRX1,%d\r\n", _delay);
    else if(command == JOIN_ACCEPT_DELAY2) sprintf(cmd, "AT+DELAY=JRX2,%d\r\n", _delay); 
    sendCommand(cmd);
    
#if _DEBUG_SERIAL_
    loraDebugPrint(DEFAULT_DEBUGTIME);
#endif
    delay1(DEFAULT_TIMEWAIT);
}


/*  ************************************************************** */
void LoRaWanClass::setClassType(_class_type_t type)
{
    while(SerialLoRa.available())SerialLoRa.read();     // flush serial stream
    
    if(type == CLASS_A)     sendCommand( (char*) "AT+CLASS=A,SAVE\r\n");
    else if(type == CLASS_B)sendCommand( (char*) "AT+CLASS=B,SAVE\r\n");   
    else if(type == CLASS_C)sendCommand( (char*) "AT+CLASS=C,SAVE\r\n");
#if _DEBUG_SERIAL_
    loraDebugPrint(DEFAULT_DEBUGTIME);
#endif
    delay1(DEFAULT_TIMEWAIT);
}


/*  ************************************************************** */
void LoRaWanClass::getClass(char *buffer, short length, unsigned int timeout)
{
    while(SerialLoRa.available())SerialLoRa.read();
    
    if(buffer)
    {
        while(SerialLoRa.available())SerialLoRa.read();
        sendCommand( (char*) "AT+CLASS\r\n");
        readBuffer1(buffer, length, timeout);    
    }
}


/*  ************************************************************** */
void LoRaWanClass::setDeviceMode(_device_mode_t mode)
{
    while(SerialLoRa.available())SerialLoRa.read();     // flush serial stream
    
    if(mode == LWABP)           sendCommand((char*) "AT+MODE=LWABP\r\n");
    else if(mode == LWOTAA)     sendCommand((char*) "AT+MODE=LWOTAA\r\n");
#if _DEBUG_SERIAL_
    loraDebugPrint(DEFAULT_DEBUGTIME);
#endif
    delay1(DEFAULT_TIMEWAIT);
}


/*  ************************************************************** */
bool LoRaWanClass::setOTAAJoin(_otaa_join_cmd_t command, unsigned int timeout)  
{
    bool result = false;
    while(SerialLoRa.available())SerialLoRa.read();     // flush serial stream
    
    if(command == JOIN)         sendCommand((char*) "AT+JOIN\r\n");
    else if(command == FORCE)   sendCommand((char*) "AT+JOIN=FORCE\r\n"); 
 
    result =  waitForJoinResponse(timeout);
    getMessage();
    return  result; 
}


/*  ************************************************************** */
bool LoRaWanClass::setOTAAJoinAuto(unsigned int min_period, unsigned int max_period, unsigned int step)  
{
    char cmd[64];
    bool result = false;
    
    while(SerialLoRa.available())SerialLoRa.read();     // flush serial stream
    
    memset(cmd, 0, 64);
    sprintf(cmd, (char*)  "AT+JOIN=AUTO, %u, %u, %u\r\n", min_period, max_period, step);
    sendCommand(cmd);
 
    //result = waitForResponse( (char*) "Network joined\r\n", max_period);
    result = waitForJoinResponse(max_period);
    getMessage();
    return result;
    
 
}

/*  ************************************************************** */
bool LoRaWanClass::LinkBusyCheck()
    {
        return LoRaStats.BusyFlag;
    }
    
    
 /*  ************************************************************** */
short LoRaWanClass::LinkErrorCheck()
    {
        return LoRaStats.LastError;

    }


// enum _error_msg_t           { OK, DONE, BUSY, ERROR, JOIN_NETWORK_FIRST, NET_JOIN, NO_FREE_CH, NO_BAND, DR_ERROR, LENGTH_ERROR, WAIT_ACK, FPENDING, JOIN_FAILED};                 
//                               0   1     2     3      4                   5         6           7        8         9             10        11        12       
/* ************************************************************** */
void LoRaWanClass::getMessage (void)
{
        char *ptr;
        
      // lets reset error/status  messages
        LoRaStats.CurrentRSSI   =  0;
        LoRaStats.LastError     =  0;
        LoRaStats.CurrentSNR    =  0.0;
        LoRaStats.CurrentLINK   =  0;
        LoRaStats.GatewayCNT    =  0;
        LoRaStats.LastErrorCode =  0;
        LoRaStats.ACKFlag       =  false;
        LoRaStats.DoneFlag      =  false;               
        LoRaStats.JoinFlag      =  false;
        LoRaStats.BusyFlag      =  false;
        

        if (ptr = strstr(_buffer, "ERROR("))         
            {
                LoRaStats.LastErrorCode = atoi(ptr + 6);
                LoRaStats.LastError = ERROR;
            }
         if (ptr = strstr(_buffer, "RSSI"))         
            {
                LoRaStats.CurrentRSSI = atoi(ptr + 4);
            }
        if (ptr = strstr(_buffer, "SNR"))          
            {
                LoRaStats.CurrentSNR = atof(ptr + 3);
            }
        if (ptr = strstr(_buffer, "Link"))
            {
               
               LoRaStats.CurrentLINK = atoi(ptr + 4);
               if (ptr = strstr(_buffer, ","))
               { 
                LoRaStats.GatewayCNT = atoi(ptr + 1);
               }
            }     
        if      (strstr(_buffer, "busy"))             { LoRaStats.LastError = BUSY; LoRaStats.BusyFlag = true; }
            
        else if (strstr(_buffer, "first"))              LoRaStats.LastError = JOIN_NETWORK_FIRST;
        else if (strstr(_buffer, "free"))               LoRaStats.LastError = NO_FREE_CH;
        else if (strstr(_buffer, "band"))               LoRaStats.LastError = NO_BAND;
        else if (strstr(_buffer, "DR error"))           LoRaStats.LastError = DR_ERROR;
        else if (strstr(_buffer, "Length"))             LoRaStats.LastError = LENGTH_ERROR;

        else if (strstr(_buffer, "Wait ACK"))           LoRaStats.LastError = WAIT_ACK;
        else if (strstr(_buffer, "FPENDING"))           LoRaStats.LastError = FPENDING;
        else if (strstr(_buffer, "Join failed"))        LoRaStats.LastError = JOIN_FAILED;
            
        if (strstr(_buffer, "Done"))                    LoRaStats.DoneFlag  = true;
        if (strstr(_buffer, "ACK Received"))            LoRaStats.ACKFlag   = true;
        if (strstr(_buffer, "Network joined"))          LoRaStats.JoinFlag  = true;                                                     
} 
          


/*  ************************************************************** */
void LoRaWanClass::setDeviceLowPower(void)
{
    while(SerialLoRa.available())SerialLoRa.read();
    
    sendCommand( (char*) "AT+LOWPOWER\r\n");
#if _DEBUG_SERIAL_
    loraDebugPrint(DEFAULT_DEBUGTIME);
#endif
    delay1(DEFAULT_TIMEWAIT);
}


/*  ************************************************************** */  
void LoRaWanClass::setDeviceReset(void)
{  
    while(SerialLoRa.available())SerialLoRa.read();
    
    sendCommand( (char*) "AT+RESET\r\n");   
#if _DEBUG_SERIAL_
    loraDebugPrint(DEFAULT_DEBUGTIME);
#endif
    delay1(DEFAULT_TIMEWAIT);
}


/*  ************************************************************** */
bool LoRaWanClass::setDeviceReset(unsigned int timeout)
{
    char *ptr;  
    
    while(SerialLoRa.available())SerialLoRa.read();
    
    sendCommand( (char*) "AT+RESET\r\n");   
    
    memset(_buffer, 0, BUFFER_LENGTH_MAX);
    readBuffer1(_buffer, BUFFER_LENGTH_MAX, timeout);
#if _DEBUG_SERIAL_    
    Serial.print(_buffer);
#endif  
    getMessage();
    ptr = strstr(_buffer, "+RESET: OK");
    if(ptr) return true;
    
    return false;
}


/*  ************************************************************** */
void LoRaWanClass::setDeviceDefault(void)
{
        
    while(SerialLoRa.available())SerialLoRa.read();
    
    sendCommand( (char*) "AT+FDEFAULT=RISINGHF\r\n");
#if _DEBUG_SERIAL_
    loraDebugPrint(DEFAULT_DEBUGTIME);
#endif
    delay1(DEFAULT_TIMEWAIT);
}


/*  ************************************************************** */    
bool LoRaWanClass::setDeviceDefault(unsigned int timeout)
{
    char *ptr;
        
    while(SerialLoRa.available())SerialLoRa.read();
    
    sendCommand( (char*) "AT+FDEFAULT=RISINGHF\r\n");
    
    memset(_buffer, 0, BUFFER_LENGTH_MAX);
    readBuffer1(_buffer, BUFFER_LENGTH_MAX, timeout);
#if _DEBUG_SERIAL_    
    Serial.print(_buffer);
#endif  
    getMessage();
    ptr = strstr(_buffer, "+FDEFAULT: OK");
    if(ptr) return true;
    
    return false;
}


/*  ************************************************************** */
void LoRaWanClass::initP2PMode(unsigned short frequency, _spreading_factor_t spreadingFactor, _band_width_t bandwidth, 
                                unsigned char txPreamble, unsigned char rxPreamble, short power)
{
    char cmd[64] = {0,};
    
    while(SerialLoRa.available())SerialLoRa.read();     // flush serial stream
    
    sprintf(cmd, "AT+TEST=RFCFG,%d,%d,%d,%d,%d,%d\r\n", frequency, spreadingFactor, bandwidth, txPreamble, rxPreamble, power);
       
    sendCommand( (char*) "AT+MODE=TEST\r\n");
    delay(DEFAULT_TIMEWAIT);
    
    sendCommand(cmd);
    delay(DEFAULT_TIMEWAIT);
    
    sendCommand( (char*) "AT+TEST=RXLRPKT\r\n");
    delay1(DEFAULT_TIMEWAIT);
}


/*  ************************************************************** */
bool LoRaWanClass::transferPacketP2PMode(char *buffer, unsigned int timeout)
{
    unsigned char length = strlen(buffer);
    
    while(SerialLoRa.available())SerialLoRa.read();
    
    
    sendCommand( (char*) "AT+TEST=TXLRSTR,\"");
    for(unsigned char i = 0; i < length; i ++)SerialLoRa.write(buffer[i]);
    sendCommand( (char*) "\"\r\n");
    
    memset(_buffer, 0, BUFFER_LENGTH_MAX);
    readBuffer(_buffer, BUFFER_LENGTH_MAX, timeout);

    if(strstr(_buffer, "+TEST: TX DONE"))return true;
    return false;
    
    //return waitForResponse( (char*) "+TEST: TX DONE\n", timeout);
}


/*  ************************************************************** */
bool LoRaWanClass::transferPacketP2PMode(unsigned char *buffer, unsigned char length, unsigned int timeout)
{
    char temp[2] = {0};
    
    while(SerialLoRa.available())SerialLoRa.read();
    
    sendCommand( (char*) "AT+TEST=TXLRPKT,\"");
    for(unsigned char i = 0; i < length; i ++)
    {
        sprintf(temp,"%02x", buffer[i]);
        SerialLoRa.write(temp);    
    }
    sendCommand( (char*) "\"\r\n");
    
    memset(_buffer, 0, BUFFER_LENGTH_MAX);
    readBuffer(_buffer, BUFFER_LENGTH_MAX, timeout);
    
    if(strstr(_buffer, "+TEST: TX DONE"))return true;
    return false;
    
    //return waitForResponse( (char*) "+TEST: TX DONE\n", timeout);
}


/*  ************************************************************** */
short LoRaWanClass::receivePacketP2PMode(unsigned char *buffer, short length, short *rssi, unsigned int timeout)
{
    char *ptr;
    short number;
    
    while(SerialLoRa.available())SerialLoRa.read();
    
    sendCommandAndWaitForResponse( (char*) "AT+TEST=RXLRPKT\r\n",  (char*) "+TEST: RXLRPKT", 2);
    
    while(SerialLoRa.available())SerialLoRa.read();
    memset(_buffer, 0, BUFFER_LENGTH_MAX);
    readBuffer(_buffer, BUFFER_LENGTH_MAX, timeout);
    
    ptr = strstr(_buffer, "LEN");
    if(ptr)number = atoi(ptr + 4);
    else number = 0;
    
    if(number <= 0)return 0;
    
    ptr = strstr(_buffer, "RSSI:");
    if(ptr)*rssi = atoi(ptr + 5);
    else *rssi = -255;
    
    ptr = strstr(_buffer, "RX \"");
    if(ptr)
    {
        ptr += 4;
        
        uint8_t bitStep = 0;
        if(*(ptr + 2) == ' ')bitStep = 3;   // Firmware version 2.0.10
        else bitStep = 2;                   // Firmware version 2.1.15
        
        for(short i = 0; i < number; i ++)
        {
            char temp[2] = {0};
            unsigned char tmp, result = 0;
            
            temp[0] = *(ptr + i * bitStep);
            temp[1] = *(ptr + i * bitStep + 1);
           
            for(unsigned char j = 0; j < 2; j ++)
            {
                if((temp[j] >= '0') && (temp[j] <= '9'))
                tmp = temp[j] - '0';
                else if((temp[j] >= 'A') && (temp[j] <= 'F'))
                tmp = temp[j] - 'A' + 10;
                else if((temp[j] >= 'a') && (temp[j] <= 'f'))
                tmp = temp[j] - 'a' + 10;

                result = result * 16 + tmp;
            }
            
            if(i < length)buffer[i] = result;
        }
    }
    
    memset(_buffer, 0, BUFFER_LENGTH_MAX);
    
    return number;
}


/*  ************************************************************** */
short LoRaWanClass::getBatteryVoltage(void)
{
    int battery = 0;
    
    return battery;
}


/*  ************************************************************** */
void LoRaWanClass::loraDebug(void)
{
    if(Serial.available())SerialLoRa.write(Serial.read());
    if(SerialLoRa.available())Serial.write(SerialLoRa.read());
}

#if _DEBUG_SERIAL_
/* ************************************************************** */
void LoRaWanClass::loraDebugPrint(unsigned int timeout)
{
    unsigned long timerStart, timerEnd;

    timerStart = millis();
    
    while(1)
    {
        while(SerialLoRa.available()) Serial.write(SerialLoRa.read());  
        
        timerEnd = millis();
        if( (timerEnd - timerStart) > (1000 * timeout)) break;
    }
}
#endif

/* ************************************************************** */
void LoRaWanClass::sendCommand(char *command)
{
    SerialLoRa.print(command);
}


/*  ************************************************************** */
void LoRaWanClass::delay1(unsigned long timeout)
{
    unsigned long timerStart, timerEnd;
    unsigned long SerialtimerStart = 0;
    unsigned long SerialtimerEnd;
    
    timerStart = millis();
    while(1)
    {
       while( SerialLoRa.available() )
       {
            char c = SerialLoRa.read();             // flush char from stream
            SerialtimerStart = millis();            // restart serial timer
       }  
      
       if (SerialtimerStart > 0 )                   // we need to wait for first char from serial stream befor starting timer
       {
            SerialtimerEnd = millis();              // check if we waited long enought for a char from serial stream 
            if( (SerialtimerEnd - SerialtimerStart) > SERIAL_TIMEOUT)
            { 
                //Serial1.println("Got a serial break\n");
                break;                              // is so, we are done here...
            } 
        }        
        timerEnd = millis();
        if( (timerEnd - timerStart) >  timeout) break;
    }
}


/* ************************************************************** */
short LoRaWanClass::readBuffer1(char *buffer, short length, unsigned int timeout)
{
    short i = 0;
    unsigned long timerStart, timerEnd;
    unsigned long SerialtimerStart = 0, SerialtimerEnd;
    
    timerStart = millis();

    while(1)
    {
        if(i < length)
        {
            while( SerialLoRa.available() )
            {
                char c = SerialLoRa.read();  
                buffer[i ++] = c;
                SerialtimerStart = millis();            // restart serial timer
            }  
      
            if (SerialtimerStart > 0 )                  // we need to wait for first char from serial stream befor starting timer
            {
                SerialtimerEnd = millis();              // check if we waited long enought for a char from serial stream 
                if( (SerialtimerEnd - SerialtimerStart) > SERIAL_TIMEOUT)
                { 
                    break;                              // is so, we are done here...
                } 
            }
        }        
        timerEnd = millis();
        if( (timerEnd - timerStart) > (1000 * timeout)) break;
    }
    
    return i;
}


/* ************************************************************** */
short LoRaWanClass::readBuffer(char *buffer, short length, unsigned int timeout)
{
    short i = 0;
    unsigned long timerStart, timerEnd;

    timerStart = millis();

    while(1)
    {
        if(i < length)
        {
            while(SerialLoRa.available())
            {
                char c = SerialLoRa.read();  
                buffer[i ++] = c;
            }  
        }
        
        timerEnd = millis();
        if( (timerEnd - timerStart) > (1000 * timeout)) break;
    }
    
    return i;
}



/* ************************************************************** */
bool LoRaWanClass::waitForJoinResponse(unsigned int timeout)
{  
    const char response1[] = "Network joined\r\n";
    const char response2[] = "Join failed\r\n";
    const char response3[] = "Joined already\r\n";
    const char response4[] = "busy\r\n";
    const char response5[] = "DR error\r\n";
    
    short len1 = strlen(response1);
    short len2 = strlen(response2);
    short len3 = strlen(response3);
    short len4 = strlen(response4);
    short len5 = strlen(response5);
               
    short sum1 = 0;
    short sum2 = 0;
    short sum3 = 0;
    short sum4 = 0; 
    short sum5 = 0; 
    
    short i = 0;
      
    unsigned long timerStart,timerEnd;
    
    timerStart = millis();
    
    memset(_buffer, 0, BUFFER_LENGTH_MAX);
  
    while(1)
    {      
        if(SerialLoRa.available())
        {
            char c = SerialLoRa.read();
            _buffer[i ++] = c;
            
#if _DEBUG_SERIAL_
            Serial.print (c);
#endif
            sum1 = (c == response1[sum1]) ? sum1 + 1 : 0;
            if(sum1 == len1) 
                { 
                 /* Serial.println("Join"); */ 
                 LoRaStats.JoinFlag  = true; 
                 return true; 
                }                 // we have joined
               
            sum2 = (c == response2[sum2]) ? sum2 + 1 : 0;
            if(sum2 == len2) 
                {
                 /* Serial.println("Not-Join"); */ 
                 LoRaStats.LastError = JOIN_FAILED; 
                 return false; 
                 }            // we are not joined
                
            sum3 = (c == response3[sum3]) ? sum3 + 1 : 0;
            if(sum3 == len3) 
                { 
                 /* Serial.println("Join already");*/ 
                 LoRaStats.JoinFlag  = true;  
                 return true; 
                 }        // we have already joined
                
            sum4 = (c == response4[sum4]) ? sum4 + 1 : 0;
            if(sum4 == len4) 
                {
                 /* Serial.println("Modem is Busy"); */  
                 LoRaStats.LastError = BUSY; 
                 LoRaStats.BusyFlag = true; 
                 return false; 
                }      // we are busy
                
            sum5 = (c == response4[sum5]) ? sum5 + 1 : 0;
            if(sum5 == len5) 
                { 
                 /* Serial.println("DR Error"); */  
                 LoRaStats.LastError = DR_ERROR; 
                 return false; 
                }      // DR Error
        }
        
        timerEnd = millis();
 
        if( (timerEnd - timerStart) > (1000 * timeout) ) { /* Serial.println("Time-Out"); */  return false;}    // timer has expired  
    }

    return false;        // we will never get here
}


/* ************************************************************** */
bool LoRaWanClass::waitForResponse(char* response, unsigned int timeout)
{
    const char response1[] = "Done\r\n";
    const char response2[] = "busy\r\n";
    
    short len  = strlen(response);
    short len1 = strlen(response1);
    short len2 = strlen(response2);
    
    short sum  = 0;
    short sum1 = 0;
    short sum2 = 0;
        
    short i = 0;
        
    unsigned long timerStart,timerEnd;   
    
    timerStart = millis();
    
    memset(_buffer, 0, BUFFER_LENGTH_MAX);
    
    while(1)
    {
        if(SerialLoRa.available())
        {
            char c = SerialLoRa.read();
            _buffer[i ++] = c;

#if _DEBUG_SERIAL_
            Serial.print (c);
#endif
            sum = (c == response[sum]) ? sum + 1 : 0;
            if(sum == len) { /* Serial.println("Got respond"); */ ; return true; }
                
            sum1 = (c == response1[sum1]) ? sum1 + 1 : 0;
            if(sum1 == len1) { /* Serial.println("Got Done respond"); */  return false; }
                
            sum2 = (c == response2[sum2]) ? sum2 + 1 : 0;
            if(sum2 == len2) { /* Serial.println("Modem is Busy"); */  return false; }      // we are busy
        }
        
        timerEnd = millis();
        if( (timerEnd - timerStart) > (1000 * timeout))  {  /* Serial.println("Time-Out"); */  return false;}
    }

    return false;       // we will never get here
}


/* ************************************************************** */
bool LoRaWanClass::sendCommandAndWaitForResponse(char* command, char *response, unsigned int timeout)
{
    sendCommand(command);
    
    bool result = waitForResponse(response, timeout);
    getMessage();
    return result;
}


/*  ************************************************************** */
LoRaWanClass lora;

/* ************************************************************** */
/* ************************************************************** */
/* ************************************************************** */
