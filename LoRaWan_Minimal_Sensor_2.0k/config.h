


#ifndef CONFIG_h
#define CONFIG_h


//#define ModemSetup              // This will setup radio-modem, and set settings in the devices EEProm.
#define EnableRadio             // Will enable radio modem to allow sending of messages...
//#define UseModemDEVEUI          // if define, will use DEVEUI from modem
#define Confirm                 // Use confirm on data messages
#define TTN                     // if we are using TTN
#define TTNV3                   // if we are using TTN-Ver 3
#define MyTest                  // Changes report times for testing only...
#define UningInterrupts         // if using interrupts
//#define MinDataRate     DR1     // Set minimum data rate for this project
//#define LoRaClassC              // if not define, we will assume Class A
//#define UseDelay                 // Will use delay() instead of sleep... (for testing only...)
#define Battery_Installed       // If we have a battery --> we most likley have solar
//#define ExternalPower           // if we have external power
//#define ATECC608A               // Microchip ATECC608A/B Crypto engine
//#define RHF0M062
//#define RHF0M0E5
#define MiniUltraProLoRa        // Rocket Scream Mini Ultra Pro with RHF0M062 radio


//  Enable debug output to serial port on port 1, un-define this will stop printing
#define MY_DEBUG1               // used in this program, level 1 debug messages
#define MY_DEBUG2               // used in this program, level 2 debug messages

#define ON      1               // LED states
#define OFF     0
#define Toggle  2


// Standard function
#include  <Wire.h>
#include  <RTCZero.h>             // https://github.com/arduino-libraries/RTCZero
#include  <FRAM_MB85RC_I2C.h>     // https://github.com/sosandroid/FRAM_MB85RC_I2C                               
#include  <Arduino_CRC32.h>       // https://github.com/arduino-libraries/Arduino_CRC32
#include  <stdarg.h>
#include  <string.h>
#include  <stdio.h>
#include  <math.h>

// My functions
#include  <arduino.h>
#include  "Utilities_M0.h"
#include  <LoRaWan.h>
#include  <Stopwatch.h>
#include  "Sensor.h"


// projects externals...
extern void setErrorCode(uint8_t);
extern bool checkJoinStatus();


// projects defines...
#define MCP_24AA025E64_I2C_ADDRESS 0x52    // I2C address of Microchip 24AA02E64 or 24AA025E64 EUI64 chip 
#define MCP_24AA025E64_MAC_ADDRESS 0xF8    // Location in memory of unique deveui 64 bits


// define flash usage..
#define   MasterBootBase       0x0000       // location in FRAM for storage of MasterBootBlock
#define   MasterBootBase1      0x0080       // alt location in FRAM for storage of MasterBootBlock


// Globals that can be adjusted via command...
#ifdef MyTest    
// Note: setting status-TX to be less that Data-TX, status will be send at Data-TX interval
#define   TX_Interval          60                        // in seconds, 1800 = 30min, 3600 = 1hr
#define   TX_StatusInterval    240                       // in seconds, 3600 = 1hr, 86400 = 24hr, time before sending status
#define   ClassC_Sleep         120
#define   TimeToReJoin         86400                     // if we need to re-join every 1 days = 86400,  5 days = 432000 seconds, 7 days = 604800
#define   MAX_POWER            8                         // set max TX power level 8 = +14dbm

#else
#define   TX_Interval          1800                      // in seconds, 1800 = 30min, 3600 = 1hr, 600 = 10min, 900 = 15min
#define   TX_StatusInterval    3600                      // in seconds, 3600 = 1hr, 86400 = 24hr, time before sending status
#define   ClassC_Sleep         60
#define   TimeToReJoin         86400                     // if we need to re-join, every 1 days = 86400, 5 days = 432000 seconds, 7 days = 604800
#define   MAX_POWER            5                         // set max TX power level 5 = +20dbm

#endif    // end of MyTest

// define projects I/O pins
#ifdef MiniUltraProLoRa
#define   BattPin               A5                        // Battery Voltage on pin A5.  1meg/1meg divider = 1/2,  6.6v max
#define   MyLED                 13
#define   RadioReset            7                         // Low will reset radio
// #else
#define   BattPin               A5                        // Battery Voltage on pin A5.  1meg/1meg divider = 1/2,  6.6v max 
#define   MyLED                 13
#define   RadioReset            2                         // Low will reset radio
#endif

// Project related TTN data ports
#define DataPort              21

#define DataMesgType          0x00                        //  sensor Data
#define StatusMesgType        0x01
#define ShortStatusMesgType   0x02


// My error codes:
#define errMsgLength              1                       // if message length is to short...
#define errUnableToJoin           2                       // unable to join network
#define errStatusMsgNotSent       3
#define errShortStatusMsgNotSent  4
#define errDataMsgNotSent         5
#define errRebootedCPU_Radio      6
#define errRebootedCPU_CMD        7
#define errCheckStatusFailed      8
#define errRadioNotResponding     9
#define errMsgNotSent             10
#define errResetPacketErrorCnt    11
#define errSleepTime              12
#define errNoFRAM                 13
#define errRadioBusy1             14
#define errRadioBusy2             15
#define errRadioSetupFailed       16
#define ackRXMessage              17


#define MaxPacketErrors           128                    // Max number of packet send errors, prior to a re-join...


#define NIBBLE_TO_HEX_CHAR(i) ((i <= 9) ? ('0' + i) : ('A' - 10 + i))
#define HIGH_NIBBLE(i) ((i >> 4) & 0x0F)
#define LOW_NIBBLE(i) (i & 0x0F)

// version of "hex to bin" macro that supports both lower and upper case
#define HEX_CHAR_TO_NIBBLE(c) ((c >= 'a') ? (c - 'a' + 0x0A) : ((c >= 'A') ? (c - 'A' + 0x0A) : (c - '0')))
#define HEX_PAIR_TO_BYTE(h, l) ((HEX_CHAR_TO_NIBBLE(h) << 4) + HEX_CHAR_TO_NIBBLE(l))

// enum _error_msg_t              { OK, DONE, BUSY, ERROR, JOIN_NETWORK_FIRST, NET_JOIN, NO_FREE_CH, NO_BAND, DR_ERROR, LENGTH_ERROR, WAIT_ACK, FPENDING, JOIN_FAILED};                 
//                                  0   1     2     3      4                   5         6           7        8         9             10        11        12       
const char ErrorCodes [13] [20] = {"None", "Done", "Busy", "Error", "Join Network First", "Joined Networked", "No Free Channels", "No Band",
                                   "Data Rate Error", "Length Error", "Wait Ack", "FPENDING Flag Set", "Join Failed"};

 
/* *****************************************************************************
  TTN Frequency plan in USA    

Uplink:

    903.9 - SF7BW125 to SF10BW125
    904.1 - SF7BW125 to SF10BW125
    904.3 - SF7BW125 to SF10BW125
    904.5 - SF7BW125 to SF10BW125
    904.7 - SF7BW125 to SF10BW125
    904.9 - SF7BW125 to SF10BW125
    905.1 - SF7BW125 to SF10BW125
    905.3 - SF7BW125 to SF10BW125
    904.6 - SF8BW500

Downlink:

    923.3 - SF7BW500 to SF12BW500 (RX1)
    923.9 - SF7BW500 to SF12BW500 (RX1)
    924.5 - SF7BW500 to SF12BW500 (RX1)
    925.1 - SF7BW500 to SF12BW500 (RX1)
    925.7 - SF7BW500 to SF12BW500 (RX1)
    926.3 - SF7BW500 to SF12BW500 (RX1)
    926.9 - SF7BW500 to SF12BW500 (RX1)
    927.5 - SF7BW500 to SF12BW500 (RX1)
    923.3 - SF12BW500 (RX2)

*****************************************************************************
Type    DataRate    Configuration   BitRate   Bytes   | TxPower Configuration
                                                      |
US915   0           SF10/125 kHz    980       11      | 0       30dBm
        1           SF9 /125 kHz    1760      53      | 1       28dBm
        2           SF8 /125 kHz    3125      125     | 2       26dBm
        3           SF7 /125 kHz    5470      242     | 3       24dBm
        4           SF8 /500 kHz    12500     242     | 4       22dBm   <--- Max for RHF0M062?
        5:7         RFU                       NU      | 5       20dBm
        8           SF12/500 kHz    980       53      | 6       18dBm
        9           SF11/500 kHz    1760      129     | 7       16dBm
        10          SF10/500 kHz    3900      242     | 8       14dBm
        11          SF9 /500 kHz    7000      242     | 9       12dBm
        12          SF8 /500 kHz    12500     242     | 10      10dBm
        13          SF7 /500 kHz    21900     242     | 11:15   RFU
        14:15       RFU                       NU      | 
***************************************************************************** */

// The min uplink data rate for all countries               // plans is DR0-DR3
#define UPLINK_DATA_RATE_MIN      DR1                       // Change to DR1 because we have moore that 11 bytes (DR0) to send
#define UPLINK_DATA_RATE_MAX_US   DR3                       // United States max data rate for 125Khz BW uplink channels = DR3
#define UPLINK_DATA_RATE_500kHz   DR4

#define DOWNLINK_DATA_RATE_US     DR8                       // US Receive Window Data Rate =  DR8
#define US_RX_DR                  DR8

#define FREQ_RX_WNDW_SCND_US      923.3                     // TTN

#define MyMinDataSize             34                        // Min packet size for this project

// LoRaWan channel defines for USA
// 125 khz channels
const float US_channels_Block [8][8] = { 
  {902.3, 902.5, 902.7, 902.9, 903.1, 903.3, 903.5, 903.7},   // rx1 923.3     Frequency Block 1
  {903.9, 904.1, 904.3, 904.5, 904.7, 904.9, 905.1, 905.3},   // rx1 923.9     Frequency Block 2 --> TTN?? on 923.3??
  {905.5, 905.7, 905.9, 906.1, 906.3, 906.5, 906.7, 906.9},   // rx1 924.5     Frequency Block 3
  {907.1, 907.3, 907.5, 907.7, 907.9, 908.1, 908.3, 908.5},   // rx1 925.1     Frequency Block 4 (note: Z-Wave in this band)
  {908.7, 908.9, 909.1, 909.3, 909.5, 909.7, 909.9, 910.1},   // rx1 925.7     Frequency Block 5
  {910.3, 910.5, 910.7, 910.9, 911.1, 911.3, 911.5, 911.7},   // rx1 926.3     Frequency Block 6
  {911.9, 912.1, 912.3, 912.5, 912.7, 912.9, 913.1, 913.3},   // rx1 926.9     Frequency Block 7
  {913.5, 913.7, 913.9, 914.1, 914.3, 914.5, 914.7, 914.9}    // rx1 927.5     Frequency Block 8
};
// 500 kHz uplink channels
const float US_channels_Block9  [8] =  {903.0, 904.6, 906.2, 907.8, 909.4, 911.0, 912.6, 914.2};   // 500kHz channels Frequency Block 9
// 500kHz downlink channels
const float US_channels_Block10 [8] =  {923.3, 923.9, 924.5, 925.1, 925.7, 926.3, 926.9, 927.5};   // rx1 channel     Frequency Block 10

#define DEFAULT_RESPONSE_TIMEOUT      6               // this is the time to wait for modem resposnse on most command
#define DEFAULT_RESPONSE_TIMEOUT_TX   30              // Radio-Modem is set for x retrys on comfirm message, average respond from TTN is 5.2 sec
#define DEFAULT_Modem_Confirm_Retrys  2               // modem will automatic retry this many time to re-send a confirms messsages
#define DEFAULT_Modem_Retrys          3               // modem will automatic retry this many time to re-send un-confirm messages

// Set DEVEUI and Keys for LoRaWan... This allowes for TTN V2 and V3 as well as private network keys 
// OTAA and ABP supported...
#ifdef TTN          // TTN public network -->  (TTN recommend that we use OTAA and not ABP)

  #ifdef TTNV3      // TTN Ver 3
  // OTAA
  #define APP_KEY   "0CB09305C08D0A609E0930AE00B0A606"  // this is our end-to-end encryption key, used to derive NWK_S_KEY, APP_S_KEY
  #define DEV_EUI   "0000000000000000"                  // this define who we are, if DEV_EUI is all zero, we will get from EUI64-EEPROM
  #define APP_EUI   "70B3D57EF0001E41"                  // this select the application to use at TTN application server


  #else             // TTN Ver 2
  #define APP_KEY   "0CB09305C08D0A609E0930AE00B0A604"  // this is our end-to-end encryption key, used to derive NWK_S_KEY, APP_S_KEY
  #define DEV_EUI   "0000000000000000"                  // this define who we are, if DEV_EUI is zero, we will get from EUI64-EEPROM
  #define APP_EUI   "70B3D57EF0001E40"                  // this select the application to use at TTN application server

  #endif  // end of #ifdef TTNV3

                                                      // NWK_S_KEY, APP_S_KEY, and DEV_ADDR are generated by server in OTAA mode
// ABP                                                // for ABP mode we also need to supply these
#define NWK_S_KEY "Your Network Session Key"          // Network Session Key, used between Node and Network Server
#define APP_S_KEY "Your App Session Key"              // Application Sesion Key, used between Node and Applacation Server
#define DEV_ADDR  "Your Device Address"               // Network Device ID

#else               // private network

// OTAA
#define APP_KEY   "0CB09305C08D0A609E0930AE00B0A605"
#define DEV_EUI   "0000000000000000"                  // if DEV_EUI is zero, we will get from EUI64-EEPROM
#define APP_EUI   "70B3D57EF0001E40"

// ABP
#define NWK_S_KEY "Your Network Session Key"
#define APP_S_KEY "Your App Session Key"
#define DEV_ADDR  "Your Device Address"

#endif    // end of #ifdef TTN


#endif    // end of config.h

/* ************************************************************* */
/* ************************************************************* */
