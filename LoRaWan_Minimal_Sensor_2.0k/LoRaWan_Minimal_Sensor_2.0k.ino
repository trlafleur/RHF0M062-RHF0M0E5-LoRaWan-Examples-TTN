/*******************************************************************************

  This is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

  tom@lafleur.us

 * *****************************************************************************

    CHANGE LOG:

      DATE         REV  DESCRIPTION
      -----------  ---  ----------------------------------------------------------
      01-Aug-2020  1.0  TRL - First Build using RHF0M062 LoRa radio    
      29-Apr-2021  2.0k TRL - Code clean up, fix error in checkMsgLength, changed some delay's to sleep function
                              Short Message ACK for RX Messages, Radio DEVEUI
      

      Notes:  1)  Tested with Arduino 1.8.13, 2.0.0b5, Compiler Optimize setting: -O2
              2)  Tested using a Feather M0 varient.h file to use serial5 
              3)  RocketStream MiniUltraProLoRa board with RHF0M062 Ver 3.5.14
              4)  RX Message format, 1st byte is messsage type ID
              5)  You need to run once with ModemSetup undefine to load parameters in modem
              6)  
              7)  
              8)  
              9)  Because we disable USB to save power, we need to double click reset button to get into boot-mode
              10) Requires LoRaWan library LoRaWan-RFH0M062-mod 05-Apr-2021 1.0h or later
              11) We tend to use FRAM on all of our project, but EEPROM can also be used with minor changes
              12) SAMD18 CPU has a 128bit unique serial number, but looking at the bit, on a number of chip,
                    many of the bits are the same, so to build a unique DEVEUI, we will use the 4th byte
                    then the 1st byte to build the DEVEUI. This looks to be random enought. (TBD)
              13) The RHF0M062 radio has a DEVEUI that we can also read after a device factory default 
              14)               


      Todo:   1)  
              2)  
              3) 
              4)
              5)  Need to set unused pins to best state for slepping at low power
              6)  Need some more modem error checking?
              7)  Allow use of EEPROM instead of FRAM
              8)  local Tag for logging, TAG = __FUNCTION__
              9)  

   Reference's:

     
        https://github.com/things-nyc/LowPowerLoRaBoards/blob/master/src/feather_m0_lora.cpp            
        https://lowpowerlab.com/forum/moteino-m0/moteino-m0-versus-standard-moteino-r6-current-draw/
        https://lowpowerlab.com/guide/moteino/moteinom0/


   _____________________________________________________________________________
   USA data rate (DR5-->DR7 not used) (Note: MAC commands take one or two bytes)

    DR    SF/BW       Max Data Bytes
    ----------------------------------
    DR0   SF10/125    11
    DR1   SF9/125     53    <--- we need this as minimum data rate in this project
    DR2   SF8/125     126
    DR3   SF7/125     242
    DR4   SF8/500     242
    DR8   SF12/500    53
    DR9   SF11/500    129
    DR10  SF10/500    242
    DD11  SF9/500     242
    DR12  SF8/500     242
    DR13  SF7/500     242



    This will define a FRAM map for this project
    Function          Size        Base address      Note:
    _____________________________________________________________________________

    MasterBootBase     128          0x0000          // Master boot block
    MasterBootBase1    128          0x0080          // copy of Master boot block, saved but not used


 *******************************************************************************/
//#define MinDataRate DR1
#include "config.h"
#include "Utilities_M0.h"

/* ************************************************************* */
#if   defined RHF0M062
#define SKETCHNAME      "LoRaWan Sensor, RHF0M062-Radio"
#elif defined RHF0M0E5
#define SKETCHNAME      "LoRaWan Sensor, RHF0M0E5-Radio"
#elif defined MiniUltraProLoRa
#define SKETCHNAME      "LoRaWan MiniUltraProLoRa with RHF0M062-Radio"
#else
#warning No radio defined...
#endif    // if defined RHF0M062

#define SKETCHVERSION   "2.0k"                          // format must be 4 bytes long... "2.0i"

#ifndef ADAFRUIT_FEATHER_M0
#error  Requires a Feather M0 varient.h file.....       // We are using Serial-5 for RHF0M062 Radio communication
                                                        // Serial-2 for debug messages, USB serial is disabled to save power
#endif

// Creating object for FRAM chip
FRAM_MB85RC_I2C FRAM;

// Creating object for RTC
RTCZero rtc;

// Creating object for CRC32
Arduino_CRC32 crc32;

// Creating object for stopwatch timer
Stopwatch sw;                       // The timing starts here by default
  
// this will allow floating point printing to work... odd but it works?
String output;


// Functions forward declarations
void      setup();
void      loop();
uint16_t  GetVbat(int VbatPin);
void      doSleep(uint32_t millis);
void      RevBytes(unsigned char *b, size_t c);
void      getDevEui(char *buf);
bool      get_E64_deveui(char *pdeveui);
void      printKey(const char *name, char *key, uint8_t len, bool lsb);
void      saveEUI (const char *name, const uint8_t *key, uint8_t len, bool lsb);
void      saveEUI4(const char *name, const uint8_t *key, uint8_t len, bool lsb);
void      SetLED (uint8_t led, uint8_t state);
void      saveFRAM(void);
void      readFRAM(void);
bool      sendStatusMessage(void);
bool      checkJoinStatus(void);
void      resetIOPins(void);
void      setIOPins(void);
bool      sentShortStatusMessage(void);
void      setErrorCode (uint8_t error);
uint8_t   getCurrentDR(void);
bool      checkRadio(void);
void      rebootCPU(void);
bool      JoinNetwork(void);
bool      ForceJoinNetwork(void);
void      GoToSleep(uint32_t sTime);
void      setAllChannels(void);
void      setMyChannels(const float* channels);
uint8_t   getMaxLength(void);
void      clearAllChannels(void);
void      processRxMessage(void);
void      displayErrorMessages(void);
bool      resetRadio(void);
void      getMinMaxSendTime (uint32_t sendTime);
uint32_t  getTX_Interval (void);
void      setACKCode (uint8_t error);

// Global variables....
uint32_t   nextRunTime        = 0;              // these need to be uint32_t as rtc.getEpoch() can return a large number...
uint32_t   nextStatRunTime    = 0;
uint32_t   nextReJoinTime     = 0;

bool       firtsBootFlag      = false;
bool       errorFlag          = true;           // true will force sending short status message on first boot..
bool       FRAMFlag           = false;          // true if we have a working FRAM device
uint16_t   uint16_vbat;                         // last Vbat reading * 100, in 16bit format 3.3v = 330
float      VbatCal            = 0.0;

char          MyDevEUI        [18];             // this is our real DEVEUI (EUI64) hex string, it is set at run time
char          buf             [18];             // we need room for 16 hex nibbles and a 0x00 string terminator
unsigned char data            [54] = {0};       // buffer for the LoRaWAN data packet to be sent
char          buffer          [1024];           // buffer for text messages received from the LoRaWAN module for display

uint8_t   maxTXLength         = 0;              // this is the current max TX packet size based on our current DR, SF and BW
uint8_t   currentDR           = 0;              // this is the last Data Rate (DRx) use by the modem

uint32_t  PacketSent          = 0;              // total packets sent
float     errorRate           = 0.0;            // packet lost rate, ie: if we are unable to send


 // debug only
uint8_t busyCount1 = 0;                        // debug only
uint8_t busyCount2 = 0;

uint16_t maxSendTime = 0;
uint16_t minSendTime = 0xffff;
uint16_t lastSendTime = 0;

// local Tag for logging
const char* TAG = "";                           // debug function tag for logging <-- TBD

/* ************************************************************** */
// define a data structure for storage in FRAM of parameters we use (~97 bytes)
typedef struct MYDATA_t
{
  char        CompileDate[22];              // Current compile data, max string size is 21 char + 0x00
  uint32_t    tx_interval;                  // Data transmit time in seconds
  uint32_t    tx_stat_interval;             // Status transmit time in seconds
  uint32_t    re_join_interval;             // Time to re-join network
  uint8_t     maxPower;                     // Max TX power
  uint16_t    bootCount;                    // Boot count for this device
  uint8_t     errorCount;                   // Error Count
  char        MyAppEUI[18];                 // Current AppEUI 16 + 2
  char        MyAppKey[34];                 // Current AppKey 32 + 2
  uint8_t     MyError;                      // Last Error message for this device
  uint8_t     UnableToSendCnt;              // Number of time we unable to send a message
  uint32_t    MyCRC32;                      // CRC32, need to be last entry as we will store CRC32 here on boot
} MyData;

// define a struct joining MYDATA_t to an array of bytes to be stored
union MYDATAFRAM_t
{
  MyData  BootDataStruct;
  uint8_t FRAMArray[sizeof(MYDATA_t)];
};

union MYDATAFRAM_t  MasterBootBlockCopy;                   // data to read from FRAM memory at boot
union MYDATAFRAM_t  MasterBootBlockCopy1;                  // data to read from FRAM memory at boot copy1

// pre-set data struct with defaults at 1st boot
union MYDATAFRAM_t  MasterBootBlock =                      // this contains all varables that we need to save in the FRAM at 1st boot
{
  {
    {.CompileDate     =  __DATE__ ", " __TIME__ },         // this is needed by a GCC bug --> {} on char/strings
    .tx_interval      =  TX_Interval,                      // 60
    .tx_stat_interval =  TX_StatusInterval,                // 120
    .re_join_interval =  TimeToReJoin,                     // once a week for now   
    .maxPower         =  MAX_POWER,                        //
    .bootCount        =  0x00,
    .errorCount       =  0x00,
    {.MyAppEUI        =  APP_EUI },
    {.MyAppKey        =  APP_KEY },
    .MyError          =  0x00,
    .UnableToSendCnt  =  0x00,                              // range 0 -> 250
    .MyCRC32          =  0x0000                             // must be last entry in the structure
  }
};


/* ************************************************************** */
/* ************************* Setup ****************************** */
/* ************************************************************** */
void setup(void)
{
  // local Tag for logging
  TAG = __FUNCTION__;

  delay(2000);                        // required to allow time for re-programming if using sleep-mode
  Serial1.begin(115200);              // debug port Serial-2 ie: -->  Serial1 in Arduino

  debug1("\n>>Started: %s\n>>Ver: %s\n", SKETCHNAME, SKETCHVERSION);
  debug1(">>%s\n\n", MasterBootBlock.BootDataStruct.CompileDate);

#ifdef TTNV3
  debug1("\n>>TTN Version 3\n");
#else
  debug1("\n>>TTN Version 2\n");
#endif

#ifdef UningInterrupts
  // Configure the CPU regulator to run in normal mode when CPU is in standby mode
  // Otherwise it defaults is in low power mode and can only supply 50 uA, this is needed for interrupt's to function
  SYSCTRL->VREG.bit.RUNSTDBY = 1;

  // Enable the DFLL48M clock in standby mode, this will allow interrupt's
  SYSCTRL->DFLLCTRL.bit.RUNSTDBY = 1;
#endif

  USBDevice.detach();                                   // to save power, detatch USB connection
  USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE;           // Disable USB

  Wire.begin();                       // I2C connections
  FRAM.begin();                       // FRAM storage
  rtc.begin(true);                    // Initialize the real time clock (RTC)

  // display and save last reboot cause...
  if (PM->RCAUSE.reg == PM_RCAUSE_SYST)  {
    debug1(">>Reset requested by system\n");   // 0x40   64
  }
  if (PM->RCAUSE.reg == PM_RCAUSE_WDT)   {
    debug1(">>Reset requested by Watchdog\n"); // 0x20   32
  }
  if (PM->RCAUSE.reg == PM_RCAUSE_EXT)   {
    debug1(">>External reset requested\n");    // 0x10   16
  }
  if (PM->RCAUSE.reg == PM_RCAUSE_BOD33) {
    debug1(">>Reset brown out 3.3V\n");        // 0x04   4
  }
  if (PM->RCAUSE.reg == PM_RCAUSE_BOD12) {
    debug1(">>Reset brown out 1.2v\n");        // 0x02   2
  }
  if (PM->RCAUSE.reg == PM_RCAUSE_POR)   {
    debug1(">>Normal power on reset\n");       // 0x01   1
  }

  debug1(">>Free Ram: %u bytes\n\n", FreeRam() );

  // Check to see if we have rebooted. We check the compile date stored in FRAM, and compare it to the date from this compile,
  //   if they are the same, we have a reboot, we also check for a valid CRC32 of full data block.

  // let Compute and save CRC32 of MasterBootBlock
  MasterBootBlock.BootDataStruct.MyCRC32 = crc32.calc( MasterBootBlock.FRAMArray, sizeof (( MasterBootBlock.FRAMArray) - 4));  // Compute CRC32

  debug1(">>Reading from MasterBootBlockCopy in FRAM\n");
      
  if ( FRAM.checkDevice() == 7)                         // lets check that we have a FRAM
  {
  debug1("****--> ERROR, FRAM not detected!!\n");
    FRAMFlag = false;
    setErrorCode (errNoFRAM);
    memcpy ( &MasterBootBlockCopy.FRAMArray , &MasterBootBlock.FRAMArray , sizeof ( MasterBootBlock.FRAMArray) );   // save working copy if no FRAM
  }  
  else
  {
  FRAMFlag = true;
  FRAM.readArray ( MasterBootBase, sizeof ( MasterBootBlockCopy.FRAMArray) , MasterBootBlockCopy.FRAMArray);

   //debug1("Size: %u\n", sizeof ( MasterBootBlockCopy.FRAMArray));
   //hexdump1(MasterBootBlockCopy.FRAMArray, 64);

  // if CRC32 is bad or compile date's are not a match, we assume this is a 1st boot, 
  //   We check current compile date with what stored in FRAM and CRC32 of data
  if ( (strcmp (MasterBootBlockCopy.BootDataStruct.CompileDate, MasterBootBlock.BootDataStruct.CompileDate)) || \
        (!( MasterBootBlock.BootDataStruct.MyCRC32 == MasterBootBlockCopy.BootDataStruct.MyCRC32) ) )
    {
      debug1(">>This is 1st boot, Saving MasterBootBlock to FRAM\n");                                                // we did not compare
      firtsBootFlag = true;

      FRAM.eraseDevice();                                                                                            // lets clear the FRAM
      FRAM.writeArray ( MasterBootBase,   sizeof ( MasterBootBlock.FRAMArray), MasterBootBlock.FRAMArray);           // not a re-boot, so save original MasteBootBlock
      FRAM.writeArray ( MasterBootBase1,  sizeof ( MasterBootBlock.FRAMArray), MasterBootBlock.FRAMArray);           // save alt MasteBootBlock, as it was at 1st boot
      FRAM.readArray  ( MasterBootBase,   sizeof ( MasterBootBlockCopy.FRAMArray), MasterBootBlockCopy.FRAMArray);   // active copy with 1st boot data plus any updates
      FRAM.readArray  ( MasterBootBase1,  sizeof ( MasterBootBlockCopy1.FRAMArray), MasterBootBlockCopy1.FRAMArray); // copy of data at 1st boot
    }
    else
    {
      //  Adjust boot-count and save it
      MasterBootBlockCopy.BootDataStruct.bootCount++;                   // bump boot count and save it in fram
      saveFRAM();
      debug1("*>Compile string Matches, so we have rebooted, BootCnt: %u\n", MasterBootBlockCopy.BootDataStruct.bootCount );
      firtsBootFlag = false;
    }
      // for now, lets clear this to get good data between boots
      MasterBootBlockCopy.BootDataStruct.UnableToSendCnt = 0;
      saveFRAM();
  }
  
  setIOPins();                        // setup any I/O pin that we might need...

  // Set up ATD and reference, for ATD to use:
  //    options --> AR_DEFAULT, AR_INTERNAL, AR_EXTERNAL, AR_INTERNAL1V0, AR_INTERNAL1V65, AR_INTERNAL2V23
  analogReference (AR_DEFAULT);       // AR_DEFAULT is set to VDD -> +3.3v, AR_EXTERNAL is set to external pin on chip
  analogReadResolution (12);          // We want 12 bits, 0 --> 4095

  // Start sensor
  Sensor_init ();                     // this will set up our sensor's

  // let get a valid DEVEUI for use as our ID
  //   if hard coded DEVEUI in config.h is != 0, we will use it, otherwise we will use EUI64 from EEprom
  // TBD --> option to use DEVEUI from CPU or from RHF0M062

  getDevEui(buf);
  strcpy(MyDevEUI, buf);
  debug1 (">>MyDevEUI: %s\n", MyDevEUI);

  debug1("\n");

//  this is the end point of a goto, we get here if we had issue with radio-modem setup, we will try radio setup again
RestartSetup: 

  /* ************************************************************** */
#ifdef EnableRadio

    pinMode(RadioReset, OUTPUT);                             // force a clean reset on the radio...
    digitalWrite(RadioReset, LOW);                           // toggle radio re-set pin
    delay(250);
    digitalWrite(RadioReset, HIGH);
    delay(1000);

  // set up LoRa radio modem library
  lora.init();                                                // Initialize the LoRaWAN radio library

  if (checkRadio())
  {
    memset(buffer, 0, 256);                                   //  We call getVersion, because after a reset the LoRaWAN module can be
    lora.getVersion(buffer, 256, DEFAULT_RESPONSE_TIMEOUT);   //   in sleep mode, then the first call only wakes it up and will not respond.
    debug1("%s\n", buffer);

    memset(buffer, 0, 256);
    lora.getProtocalVersion(buffer, 256, DEFAULT_RESPONSE_TIMEOUT);  // get current LoRaWAN protocal version from device...
    debug1("%s\n", buffer);
  }  
  else
  {
    debug1("**Check radio failed in setup\n");
    setErrorCode (errRadioSetupFailed);
    goto RestartSetup;                                         // let try again...
  }

/* ************************************************************** */
// we only need to set up the modem once as it's saves the information in it's EEprom
#ifdef ModemSetup                      

if (checkRadio() == true)
{
  debug1("\n**** Modem Setup Enable ****\n");

  if (lora.setDeviceDefault(DEFAULT_RESPONSE_TIMEOUT))          // start at a default base line
  { 
    debug1(">>Modem Default OK\n");             
  }
  else
  {
    debug1("**Modem Default failed\n"); 
    setErrorCode (errRadioSetupFailed);
    goto RestartSetup;         
  }

// lets set ID's in modem
#ifdef UseModemDEVEUI
// Using DEVEUI from modem
    lora.setId  (NULL, NULL, (char*) APP_EUI);                    // Set DevAddr, DevEUI,  AppEUI as needed
    lora.setKey (NULL, NULL, (char*) APP_KEY);                    // Set NetSKEY, AppSKEY, AppKey
    debug1(">>Setting DEVEUI from modem\n");
    memset(buffer, 0, 64);    
    lora.getDevEui(buffer, 64, DEFAULT_RESPONSE_TIMEOUT + 20);
    debug1("%s\n", buffer);
#else
// let get a valid DEVEUI for use as our device ID
//   if hard coded DEVEUI in config.h is != 0, we will use it, otherwise we will use EUI64 from EEprom
    getDevEui(buf);
    strcpy( MyDevEUI, buf);
    lora.setId  (NULL, (char*) MyDevEUI, (char*) APP_EUI);        // Set DevAddr, DevEUI,  AppEUI as needed
    lora.setKey (NULL, NULL, (char*) APP_KEY);                    // Set NetSKEY, AppSKEY, AppKey
#endif  // #ifdef UseModemDEVEUI
  
  lora.setDataRate(US915);                                      // this command is slow in modem!  
  delay(250);
  
#ifdef  TTN
  setMyChannels(8, 2, US_channels_Block);        // set channel starting at 8, in freq block 2 for TTN
  memset(buffer, 0, 1024);
  lora.sendCMD( (char*) "CH=NUM 8-15,65", buffer, 1024, DEFAULT_RESPONSE_TIMEOUT);
  lora.setNetworkPublic(true);
  
#else
  setMyChannels(0, 1, US_channels_Block);        // We will use frequency block 1 on our private network
  memset(buffer, 0, 1024);
  lora.sendCMD( (char*) "CH=NUM 0-7,64", buffer, 1024, DEFAULT_RESPONSE_TIMEOUT);
  lora.setNetworkPublic(false);
#endif

  lora.setDeviceMode(LWOTAA);                     // we are only using OTAA

#ifdef TTNV3
  memset(buffer, 0, 256);
  lora.sendCMD( (char*) "DELAY=RX2, 5000", buffer, 1024, DEFAULT_RESPONSE_TIMEOUT);     // TTN V3 Window is now 5 sec
  memset(buffer, 0, 256);
  lora.sendCMD( (char*) "DELAY=RX1, 5000", buffer, 1024, DEFAULT_RESPONSE_TIMEOUT);
#endif

  lora.setConfirmedMessageRetryTime   (DEFAULT_Modem_Confirm_Retrys);   // Set retry count's
  lora.setUnconfirmedMessageRepeatTime(DEFAULT_Modem_Retrys);           // Set retry count's

  lora.setAdaptiveDataRate(true);                               // Set Adaptive Data Rate

  lora.setPower(MasterBootBlockCopy.BootDataStruct.maxPower);   // Set power

  lora.setPort(DataPort);                                       // all data packets are sent to network on a this port for decoding

//      memset(buffer, 0, 1024);
//      lora.getChannel(buffer, 1024, DEFAULT_RESPONSE_TIMEOUT+30);
//      //debug1("%s\n",buffer);                                // Arduino with debug1 is ONLY printing the 1st 77 char of this string???
//      Serial1.println(buffer);

#ifdef LoRaClassC                                               // Set Class type, this is now save in modems EEProm with ver 3.5.14
  debug1(">>Setting LoRa Class C\n");
  lora.setClassType(CLASS_C);
#else
  debug1("Setting LoRa Class A\n");
  lora.setClassType(CLASS_A);
#endif
}     // if (checkradio())

#else         // else of ModemSetup

  if (checkRadio() == false)
  {
    debug1("**Check radio fail in radio setup\n");
    setErrorCode (errRadioSetupFailed);    
    goto RestartSetup;                                                // Let try again
  }
#endif    // end of ModemSetup

  if (checkMsgLength(MyMinDataSize) >= MyMinDataSize);  
    {debug1(">>Message length is ok...\n");}
  
  // lora.sendCMD( (char*) "LOG=ON", buffer, 1024, DEFAULT_RESPONSE_TIMEOUT);
  
  debug1("\n*>Checking join status...\n");

  setLED (MyLED, ON);                                                // turn on TX LED...
    checkJoinStatus();                                              // check joining status and re-join if needed...
  setLED (MyLED, OFF);


#else
  debug1(">>Radio is NOT enabled\n");
#endif      // end of EnableRadio

#ifdef MyTest
 debug1(">>Using short TX interval: %u Sec\n", TX_Interval);
#endif   

#ifdef UseDelay
 debug1(">>Using delay for timing\n");
#endif   

#ifdef Confirm
  debug1(">>Using Confirmed on data messages\n");      
#endif

 // pre-set next periodic re-join time
  nextReJoinTime  = rtc.getEpoch() + MasterBootBlockCopy.BootDataStruct.re_join_interval;

  debug1(">>Setup Complete...\n\n");
}
/* ******************* End of Setup ***************************** */
/* ************************************************************** */


/* ************************************************************** */
uint8_t getErrorCnt (void)
{
  // local Tag for logging
  TAG = __FUNCTION__;
  
  return MasterBootBlockCopy.BootDataStruct.UnableToSendCnt;  // error count
}


/* ************************************************************** */
uint32_t getTX_Interval (void)
{
  // local Tag for logging
  TAG = __FUNCTION__;
  
  return MasterBootBlockCopy.BootDataStruct.tx_interval;  // error count
}


/* ************************************************************** */
// this is for debug only...
void getMinMaxSendTime (uint32_t sendTime)
{
  // local Tag for logging
  TAG = __FUNCTION__;

  maxSendTime = (uint16_t) max( maxSendTime, sendTime);
  minSendTime = (uint16_t) min( minSendTime, sendTime);
  debug1("***Send Time Min: %u, Max: %u\n", minSendTime, maxSendTime);
}


/* ************************************************************** */
uint8_t checkMsgLength(uint8_t MyMsgLength)
{
  // local Tag for logging
  TAG = __FUNCTION__;

  uint8_t CurrentLength = getMaxLength();             // get current length from modem
  //debug1("**MyMsgLength: %u, CurrentLength: %u\n", MyMsgLength, CurrentLength);

  if (MyMsgLength > CurrentLength)                   // check to make sure we have enought room for message
  {
    debug1("*>Current length is too small...\n");
    debug1("**We Need: %u, Have: %u\n", MyMsgLength, CurrentLength);    // let re adjust data rate

    if ((MyMsgLength > 11)  && (MyMsgLength <= 53))
    {
      debug1("*>Setting DR1\n");
      lora.setDataRate(DR1);
      return getMaxLength();
    }
    else if ((MyMsgLength > 53)  && (MyMsgLength <= 126))
    {
      debug1("*>Setting DR2\n");
      lora.setDataRate(DR2);
      return getMaxLength();
    }
    else if ((MyMsgLength > 126) && (MyMsgLength <= 242))
    {
      debug1("*>Setting DR3\n");
      lora.setDataRate(DR3);
      return getMaxLength();
    }
    else if (MyMsgLength > 242)
    {
      debug1("*>Message length too large\n");
      setErrorCode (errMsgLength);
      return 0;
    }
  }
  return getMaxLength();
}


/* ************************************************************** */
uint8_t getMaxLength(void)
{
  // local Tag for logging
  TAG = __FUNCTION__;

  memset(buffer, 0, 256);
  lora.getMaxLen(buffer, 256, DEFAULT_RESPONSE_TIMEOUT);            // get current max length from modem...
  //debug1("%s\n", buffer);

  char *ptr = strstr(buffer, "LEN, ");
  if (ptr)    maxTXLength = atoi(ptr + 5);
  else        maxTXLength = 0;

  return maxTXLength;
}


/* ************************************************************** */
// ChIndex = starting channel number 0-63, ChBlock is channel block,1-8, Channels is array of frequencies
void setMyChannels( unsigned char ChIndex,  unsigned char ChBlock, const float channels[][8])
{
  // local Tag for logging
  TAG = __FUNCTION__;

  uint8_t i;
  
  if ( !( (ChBlock >= 1) && (ChBlock <= 8)  ) )     ChBlock = 2;  // if out of range set to USA TTN
  if ( !( (ChIndex >= 0) && (ChIndex <= 63) ) )     ChIndex = 8;  // if out of range set to USA TTN

  // now we can set 125kHz as one block of channels
  for (i = (0 + ChIndex); i < (8 + ChIndex); i++)
  {
    if (channels[ChBlock - 1][i - ChIndex] != 0)
    {
      //debug1("i: %u, Ch: %u, Block: %u, Freq: %7.2f\n", i, i-ChIndex, ChBlock-1, channels[ChBlock-1][i-ChIndex]);
      lora.setChannel(i, channels[ChBlock-1][i - ChIndex], UPLINK_DATA_RATE_MIN, UPLINK_DATA_RATE_MAX_US);
    }
  }
  i = 64 + (ChIndex / 8);

  // lets set a 500kHz channel
  lora.setChannel(i, US_channels_Block9 [i - 64], UPLINK_DATA_RATE_500kHz);
  
  // lets set an RX1 window
  //lora.setReceiveWindowFirst(0, US_channels_Block10 [i-64]);
}


/* ************************************************************** */
void clearAllChannels(void)
{
  // local Tag for logging
  TAG = __FUNCTION__;

  lora.setChannels ((char*) ", 0" );     // enable only one channel
  lora.setChannel  (0, 0.0);             // now delete it
}


/* ************************************************************** */
void setAllChannels(void)
{
  // local Tag for logging
  TAG = __FUNCTION__;

  setMyChannels(0,  1, US_channels_Block);
  setMyChannels(8,  2, US_channels_Block);    // <-- TTN
  setMyChannels(16, 3, US_channels_Block);
  setMyChannels(24, 4, US_channels_Block);
  setMyChannels(32, 5, US_channels_Block);
  setMyChannels(40, 6, US_channels_Block);
  setMyChannels(48, 7, US_channels_Block);
  setMyChannels(56, 8, US_channels_Block);
}


/* ************************************************************** */
// return Vbat * 100  ie: 3.3v * 100 = 330
uint16_t GetVbat(int VbatPin)
{
  // local Tag for logging
  TAG = __FUNCTION__;
  
#ifdef Battery_Installed
#define VbatSamples 10

  uint32_t vbat = analogRead (VbatPin) ;     // this is a junk read to clear ATD
  delay(50);

  vbat = 0;                                  // we will take multiple reading to get a stable count
  for (int i = 0; i < VbatSamples; i++)
  {
    vbat += analogRead(VbatPin);
    delay (2);
  }
  vbat /= VbatSamples;

  float Vsys =  (((float) vbat) *  0.001611722 ) + VbatCal;   // read the battery voltage, 12bits = 0 -> 4095, divider is 2/1, so max = 6.6v
                                                              // 6.6v / 4095 = 0.001611722
  debug1(">>Vbat: %7.3f V\n", Vsys);
  if (Vsys > 6.5) (Vsys = 6.5);             // lets do a bounds check...  Vbat should never be much over 4.3v in this system
  if (Vsys < 0)   (Vsys = 0);
  Vsys = Vsys + 0.005;                      // lets rounnd up if needed
  uint16_t VBAT = (uint16_t) (Vsys * 100);  // convert float to integer * 100  (3.34V --> 334)
  return VBAT;
#elif
  return 0;       // if no battery, set to zero
#endif
}


/* ************************************************************** */
void setIOPins(void)
{
  // local Tag for logging
  TAG = __FUNCTION__;

  // Setup Digital Pins
  //    for(int i = 0; i < 26; i ++)              // Set all pins to HIGH to save power (reduces the
  //    {                                         // current draw during deep sleep by around 0.7mA).
  //        if (i!=13) {                          // Don't switch on the onboard user LED (pin 13).
  //          pinMode(i, OUTPUT);
  //          digitalWrite(i, HIGH);
  //        }
  //    }

  //  
  //  pinMode(1,OUTPUT);
  //  pinMode(2,OUTPUT);
  //  pinMode(3,OUTPUT);
  //  pinMode(4,OUTPUT);
  //  pinMode(5,OUTPUT);
  //  pinMode(6,OUTPUT);
  //  pinMode(7,OUTPUT);
  //  pinMode(8,OUTPUT);
  //  pinMode(9,OUTPUT);
  //  pinMode(10,OUTPUT);
  //  pinMode(11,OUTPUT);
  //  pinMode(12,OUTPUT);
  //  pinMode(13,OUTPUT);
  //  pinMode(A0,OUTPUT);
  //  pinMode(A1,OUTPUT);
  //  pinMode(A2,OUTPUT);
  //  pinMode(A3,OUTPUT);

}


/* ************************************************************** */
void resetIOPins(void)
{
  // local Tag for logging
  TAG = __FUNCTION__;

}


/* ************************************************************** */
// Process an in-comming message to any port
// Message format:  1st byte = message ID
//                  2nd byte --> message
//    0x00 = Set Max Power
//    0x01 = Set APPEUI 8 bytes         <--- not yet implemented
//    0x02 = Set APPKEY 16 bytes        <--- not yet implemented
//    0x03 = Set Tx Data Message interval
//    0x04 = Set TX Status Message interval
//    0x05 = Set soil type
//    0x06 = Reset error counters   code = 0xaf
//    0x07 = Force a re-boot        code = 0x5f        
//    0x08 = Force Re-Join          code = 0xaf
//    0x09 = Set periodic re_join_interval
//    0x0A = 
//

char  MyCode  = 0xaf;            // special code's!
char  MyCode1 = 0x5f;
/* ************************************************************** */

void processRxMessage()
{
  // local Tag for logging
  TAG = __FUNCTION__;

  short length;
  short rssi;
  short port;
  
  memset(buffer, 0, 256);                                 // clear RX buffer
  length = lora.receivePacket(buffer, 256, &rssi, &port); // see if we got an RX messsage??

  if (length)                                             // Yes, we have a message
  {
    debug1(">>RX message -> Length is: %d\n", length);
    debug1(">>Port: %u, RSSI is: %d\n", port, rssi);
    debug1(">>Data is: ");

    for (unsigned char i = 0; i < length; i ++)
    {
      debug1("%02x ", buffer[i]);
    }
    debug1("\n");

    switch (buffer[0])
    {
      /* ***************** */
      case 0x00:                        // set max TX power
        {
          if ( length == 2)
          {
            if ( (buffer[1] >= 4)  &&  (buffer[1] <= 10) )                                      // set range for TX power
            {
              debug1(">>Message 0, Set TX power: %u\n", buffer[1]);
              MasterBootBlockCopy.BootDataStruct.maxPower = buffer[1];
              lora.setPower(MasterBootBlockCopy.BootDataStruct.maxPower);                      // if we are here, radio must be enabled...
              saveFRAM();
            }
          }
          break;
        }

      /* ***************** */
      case 0x01:                        // set new AppEUI
        {
          if ( length == 9)
          {
            //btox( MasterBootBlockCopy.BootDataStruct.MyAppEUI,  &buffer[1], 16);                 // convert to a hex string for modem...
            debug1(">>Message 1, Set APPEUI: %s\n", MasterBootBlockCopy.BootDataStruct.MyAppEUI);  // for now will not use this...............
            saveFRAM();
          }
          break;
        }

      /* ***************** */
      case 0x02:                        // set New AppKEY
        {
          if ( length == 17)
          {
            //btox( MasterBootBlockCopy.BootDataStruct.MyAppKEY,  &buffer[1], 32);                 // convert to a hex string for modem...
            debug1(">>Message 2, Set APPKEY: %s\n", MasterBootBlockCopy.BootDataStruct.MyAppKey);  // for now will not use this...............
            saveFRAM();
          }
          break;
        }

      /* ***************** */
      case 0x03:                        // set TX Interval
        {
          if ( length == 5)
          {
            uint32_t newTX = ((buffer[1] << 24) + (buffer[2] << 16) + (buffer[3] << 8) + buffer[4]);    // 4 bytes for a 32 bit word
            debug1(">>Message 3, Set Data TX Interval: %u\n", newTX);                                   // 10 min = 600  = 0x00000258
                                                                                                        // 1hr    = 3600 = 0x00000e10
            if ((newTX >= 10) && (newTX <= 86400))                      // set bounds for now at 10 sec and 1 day (86,400 sec)
            {
              MasterBootBlockCopy.BootDataStruct.tx_interval  = newTX;
              saveFRAM();
            }
            else {
              debug1(">>Message 3, Set TX Interval out of range: %u\n", newTX);
            }
          }
          break;
        }

      /* ***************** */
      case 0x04:                        // Set Status TX Interval, Status is only send when its tiem and we are awake from data-send
        {                               //   Note: setting status-TX to be less that Data-TX, status will be send at Data-TX interval
          if ( length == 5)
          {
            uint32_t newTX = ((buffer[1] << 24) + (buffer[2] << 16) + (buffer[3] << 8) + buffer[4]);    // 4 bytes for a 32 bit word
            debug1(">>Message 4, Set Status TX Interval: %u\n", newTX);

            if ((newTX >= 30) && (newTX <= 86400))                      // set bounds for now at 30 sec and 1 day (86,400 sec)
            {
              MasterBootBlockCopy.BootDataStruct.tx_stat_interval  = newTX;
              saveFRAM();
            }
            else {
              debug1(">>Message 4, Set Status TX Interval out of range: %u\n", newTX);
            }
          }
          break;
        }

      /* ***************** */
      case 0x05:                        // extra...
        {
          if ( length == 2)
          {
            debug1(">>Message 5, : %u\n", buffer[1]);
          }
          break;
        }

      /* ***************** */
      case 0x06:                        // reset error counters
        {
          if ( length == 2)
          {
            debug1(">>Message 6, Reset Error Counters: %u\n", buffer[1]);
            if ( buffer[1] == MyCode)
            {
              MasterBootBlockCopy.BootDataStruct.errorCount       = 0;
              MasterBootBlockCopy.BootDataStruct.UnableToSendCnt  = 0;
              MasterBootBlockCopy.BootDataStruct.bootCount        = 0;
              MasterBootBlockCopy.BootDataStruct.MyError          = 0;

              maxSendTime   = 0;
              minSendTime   = 0xffff;
              lastSendTime  = 0;
              busyCount1    = 0;
              busyCount2    = 0;

              saveFRAM();
            }
            else 
            {
              debug1(">>Message 6, Reset Error Counters Code is invalid: %u\n", buffer[1]);
            }
          }
          break;
        }

      /* ***************** */
      case 0x07:                        // Re-Boot CPU
        {
          if ( length == 2)
          {
            debug1(">>Message 7, Re-Booting CPU %u\n", buffer[1]);
            if ( buffer[1] == MyCode1)
            {
              MasterBootBlockCopy.BootDataStruct.errorCount++;
              setErrorCode (errRebootedCPU_CMD);
              rebootCPU();
            }
            else 
            {
              debug1(">>Message 7, Reboot Code is invalid: %u\n", buffer[1]);
            }
          }
          break;
        }

      /* ***************** */
      case 0x08:                        // Force a re-join of network
        {
         if ( length == 2)
          {
            debug1(">>Message 8, ReJoin Network %u\n", buffer[1]);
            if ( buffer[1] == MyCode)
            {
              checkJoinStatus(); 
            }
            else 
            {
              debug1(">>Message 8, Reboot Code is invalid: %u\n", buffer[1]);
            }
          }
          break;
        }
        
      /* ***************** */
      case 0x09:                        // Set periodic re_join_interval
        {
          if ( length == 5)
          {
            uint32_t newTX = ((buffer[1] << 24) + (buffer[2] << 16) + (buffer[3] << 8) + buffer[4]);    // 4 bytes for a 32 bit word
            debug1(">>Message 9, Set Data Re-Join Interval: %u\n", newTX);

            if ((newTX >= 600) && (newTX <= 604800))                      // set bounds for now at 10 min, (600 sec) and 1 week (604,800 sec)
            {
              MasterBootBlockCopy.BootDataStruct.re_join_interval  = newTX;
              saveFRAM();
            }
            else 
            {
              debug1(">>Message 9, Set TX periodic Interval out of range: %u\n", newTX);
            }
          }
          break;
        }
                
      /* ***************** */
      case 0x0A:
        {
          debug1(">>Message 10\n");
          break;
        }
        
      /* ***************** */
      default:
        {
          debug1(">>Default Message Received ID: 0x%02x\n", buffer[0]);
          break;
        }

    }   // end of switch (buffer[0])
    setACKCode (ackRXMessage);                          // we will send a short message on next TX time to ACK receiving a message
    debug1(">>RX Message ACK...\n");
  }   // end of if(length)
}   // end of processMessage()


/* ************************************************************** */
void saveFRAM(void)
{
  // local Tag for logging
  TAG = __FUNCTION__;

  if( FRAMFlag == true)
  {  
    MasterBootBlockCopy.BootDataStruct.MyCRC32 = \
      crc32.calc ( MasterBootBlockCopy.FRAMArray, sizeof (( MasterBootBlockCopy.FRAMArray) - 4));  // Compute CRC32
    FRAM.writeArray ( MasterBootBase, sizeof ( MasterBootBlockCopy.FRAMArray) , MasterBootBlockCopy.FRAMArray);
  }  
}


/* ************************************************************** */
void readFRAM(void)
{
  // local Tag for logging
  TAG = __FUNCTION__;

  if( FRAMFlag == true)
  {  
  // do we need this??
  }  
}


/* ************************************************************** */
// this will display error message from the modem after some command
void displayErrorMessages(void)
{
  // local Tag for logging
  TAG = __FUNCTION__;

  debug1("\n>>RSSI: %d, SNR: %7.2f\n", LoRaStats.CurrentRSSI, LoRaStats.CurrentSNR);
  debug1(">>Last Error: %s, Error Code: %d, Error Count: %u\n", ErrorCodes[LoRaStats.LastError], LoRaStats.LastErrorCode, LoRaStats.ErrorCNT);
  debug1(">>Link: %d, Gateway Count: %d\n", LoRaStats.CurrentLINK  , LoRaStats.GatewayCNT );
  
  debug1(">>ACK flag:  %s \n",   LoRaStats.ACKFlag  ? "true" : "false" );
  debug1(">>JOIN flag: %s \n",   LoRaStats.JoinFlag ? "true" : "false" );
  debug1(">>Done flag: %s \n",   LoRaStats.DoneFlag ? "true" : "false" );
  debug1(">>Busy flag: %s \n\n", LoRaStats.BusyFlag ? "true" : "false" );
}


/* ************************************************************** */
/* This message is 11 bytes or less (DR0 data rate), we use it to send errors messages
    1 byte mesg type, 2 bytes vbat, 2 bytes boot count, 1 byte boot cause, 1 byte last erorr code,
    1 byte current DR, 1 byte error count.

    MAC messages may take up a 1 or 2 bytes at times, so we will keep this at 9 bytes

    We want to keep short status message well under DR0 limit of 11 (9) bytes
*/
bool sentShortStatusMessage(void)
{
  // local Tag for logging
  TAG = __FUNCTION__;

  //debug2("*>In Sending Short Status message\n");

  int packetSize = 9;                                           // set current packed size
  bool result = 0;

  if (checkMsgLength(packetSize) >= packetSize)
  {
    debug2(">>Message length is ok...\n");
  }
  else
  {
    debug2("**Error Message length...\n");                      // error if here
    setErrorCode (errMsgLength);
  }

  data[0] = ShortStatusMesgType;                                // message format

  // Vbat is stored in data[1] and data [2]
  uint16_vbat = GetVbat(BattPin);                               // read the battery voltage...
  data[1] = (char) (uint16_vbat >> 8);
  data[2] = (char) (uint16_vbat & 0x00FF);
  data[3] = (char) (MasterBootBlockCopy.BootDataStruct.bootCount & 0x00FF);
  data[4] = (char) PM->RCAUSE.reg;                                        // send reboot cause
  data[5] = (char) MasterBootBlockCopy.BootDataStruct.MyError;            // Last error
  data[6] = (char) getCurrentDR();                                        // current Data Rate
  data[7] = (char) MasterBootBlockCopy.BootDataStruct.errorCount;         // current error count
  data[8] = (char) MasterBootBlockCopy.BootDataStruct.UnableToSendCnt;    // unable to send a packet count
  
#ifdef EnableRadio
  debug2("\n*>Sending Short Status message\n");

  if (checkRadio()) 
  {                                                                      
  sw.startNewTimer();                                                   // Restart timer
  setLED (MyLED, ON);                                                    // set our TX led
    result = lora.transferPacketWithConfirmed(data, packetSize, DEFAULT_RESPONSE_TIMEOUT_TX);     // send message
    if (lora.LinkBusyCheck()) 
    {
      busyCount1++;
      setErrorCode (errRadioBusy1);   
      debug1("** Modem is Busy......\n");
      GoToSleep(rtc.getEpoch() + 5);
//    delay(5000);

      result = lora.transferPacketWithConfirmed(data, packetSize, DEFAULT_RESPONSE_TIMEOUT_TX);    // send message again
      displayErrorMessages(); 
      if (lora.LinkBusyCheck()) 
      {
        busyCount2++;
        setErrorCode (errRadioBusy2); 
      }           
    }
  setLED (MyLED, OFF);
  lastSendTime = sw.getElapsedMillis();
  getMinMaxSendTime ( lastSendTime );
  debug1("***Send Time in MS: %u\n", lastSendTime);
  } 
#endif    // end of EnableRadio
  return result;
}


/* ************************************************************** */
/* Status Message 36 bytes, We send this infrequently just to keep us upto date
   1 byte, message type, 2 bytes Vbat xx.xx, 2 bytes BootCnt xxxx, 1 bytes BootCause xx, 1 byte last error,
   1 byte error count, 4 bytes Version String, 21 bytes Compile String
   test data --> 01 01 4C 00 01 02 32 31 2E 30 41 75 67 20 32 31 20 32 30 32 30 2C 20 31 34 3A 32 36 3A 32 36 00
*/
bool sendStatusMessage(void)
{
  // local Tag for logging
  TAG = __FUNCTION__;

  int packetSize = 36;
  bool result = false;

  //debug2("\n*>In Sending Status Message\n");

  if (checkMsgLength(packetSize) >= packetSize)
  {
    debug2(">>Message length ok...\n");
  }
  else
  {
    debug2("**Error Message length...\n");                      // error if here
    setErrorCode (errMsgLength);
  }

  data[0] = StatusMesgType;                                    // message format

  // Vbat is stored in data[1] and data [2]
  uint16_vbat = GetVbat(BattPin);                              // read the battery voltage...
  data[1]  = (char)  (uint16_vbat >> 8);
  data[2]  = (char)  (uint16_vbat & 0x00FF);
  data[3]  = (char)  (MasterBootBlockCopy.BootDataStruct.bootCount & 0x00FF);
  data[4]  = (char)  PM->RCAUSE.reg;                                        // send reboot cause
  data[5]  = (char)  MasterBootBlockCopy.BootDataStruct.MyError;
  data[6]  = (char)  getCurrentDR();                                        // current Data Rate
  data[7]  = (char)  MasterBootBlockCopy.BootDataStruct.errorCount;
  data[8]  = (char)  MasterBootBlockCopy.BootDataStruct.UnableToSendCnt;    // unable to send a packet count
  data[9]  = (char)  SKETCHVERSION[0];                                      // send SKETCHVERSION,  only send first 4 bytes --> "1.6a"
  data[10] = (char)  SKETCHVERSION[1];
  data[11] = (char)  SKETCHVERSION[2];
  data[12] = (char)  SKETCHVERSION[3];
  memcpy(&data[13],  &MasterBootBlock.BootDataStruct.CompileDate, sizeof(MasterBootBlock.BootDataStruct.CompileDate) - 1); // send compile date/time, 21 bytes
  data[35] = (char)  0x00;   // not used yet
  
#ifdef EnableRadio
  debug2("\n*>Sending Status message\n");

  if (checkRadio())                                         // check to see if we have a radio
  {
  sw.startNewTimer();    
  setLED (MyLED, ON);                                        // Restart timer
    result = lora.transferPacketWithConfirmed (data, packetSize, DEFAULT_RESPONSE_TIMEOUT_TX);     // send message

    if (lora.LinkBusyCheck())  
    {
      busyCount1++;
      setErrorCode (errRadioBusy1);   
      debug1("** Modem is Busy......\n");
      GoToSleep(rtc.getEpoch() + 5);
//    delay(5000);

      result = lora.transferPacketWithConfirmed (data, packetSize, DEFAULT_RESPONSE_TIMEOUT_TX);     // send message again
      displayErrorMessages();
      if (lora.LinkBusyCheck()) 
      {
        busyCount2++; 
        setErrorCode (errRadioBusy2);
      }           
    }
  setLED (MyLED, OFF);
  lastSendTime = sw.getElapsedMillis();
  getMinMaxSendTime ( lastSendTime );
  debug1("***Send Time in MS: %u\n", lastSendTime);
  }
#endif    // EnableRadio

  return result;
}


/* ************************************************************** */
uint8_t getCurrentDR(void)
{
  // local Tag for logging
  TAG = __FUNCTION__;

#ifdef EnableRadio 
  memset(buffer, 0, 256);
  lora.getDR(buffer, 256, DEFAULT_RESPONSE_TIMEOUT);            // get current data rate from modem...
  //debug2(">>DR: %s\n",buffer);
  char *ptr = strstr(buffer, "+DR: DR");                  

  debug2(">>Current DR: %u\n", atoi(ptr + 7) ); 
  if (ptr) { currentDR = atoi(ptr + 7); }
  else { currentDR = 0; }                                       // error code for DR
  return currentDR;
#else
   return 12;
#endif
}


/* ************************************************************** */
void setErrorCode (uint8_t error)
{
  // local Tag for logging
  TAG = __FUNCTION__;
  
  MasterBootBlockCopy.BootDataStruct.MyError = error;            // add new error message code
  MasterBootBlockCopy.BootDataStruct.errorCount++;               // bump error counter and save it in fram
  errorFlag = true;
  saveFRAM();
}

/* ************************************************************** */
void setACKCode (uint8_t error)
{
  // local Tag for logging
  TAG = __FUNCTION__;
  
  MasterBootBlockCopy.BootDataStruct.MyError = error;            // Ack message code 
  errorFlag = true;
  saveFRAM();
}


/* ************************************************************** */
bool ForceJoinNetwork(void)
{
  // local Tag for logging
  TAG = __FUNCTION__;

  bool result = false;
  
// see if we are joined, if not start a join cycle
  result = lora.setOTAAJoin(JOIN, DEFAULT_RESPONSE_TIMEOUT + 20);  
  if (result) return true; 
  
for (uint8_t i = 1; i < 24 ; i++)                                 // we will attempt to join 24 times...
  {
    debug1("\n*>Joining... Attemp: %d\n", i);
    result = lora.setOTAAJoinAuto(30, 1800, 1);
    
    if (result)
    {
      debug1(">>We have join...\n");
      return true;
    }

    displayErrorMessages();
    if (LoRaStats.BusyFlag) 
    { 
      debug1("**Modem is busy...\n");
      busyCount1++;
    }

    if ( LoRaStats.LastError == DR_ERROR)
    {
      lora.setDataRate(DR1);
    }

    debug1("**Waiting to join again... %d\n", i);
    // we should sleep here.... 
    GoToSleep(rtc.getEpoch() + 20);        // we will sleep for 20 sec...  
  }   // end of for (...)
  return false; 
}


// set backoff times
#define backOffTime1   1800               // 30 min 
#define backOffTime2   3600               // 1 hr
#define backOffTime3   7200               // 2 hr 
#define backOffTime4   21600              // 6 hr
#define backOffTime5   43200              // 12 hr
#define backOffTime6   86400              // 1 day
/* ************************************************************** */
bool checkJoinStatus(void)
{
  // local Tag for logging
  TAG = __FUNCTION__;
  
   // debug2(">>In Check Join Status...\n");
  
  // let attempt to join network with current parameters...
  if ( ForceJoinNetwork() )  return true;        // all ok, we are joined

  // if we are unable to join, let try to force a join with a backoff adjustment
  debug1("*>Starting Backoff Timers...\n");

  GoToSleep(rtc.getEpoch() + backOffTime1);
  if ( ForceJoinNetwork())  return true;

  GoToSleep(rtc.getEpoch() + backOffTime2);
  if ( ForceJoinNetwork())  return true;

  GoToSleep(rtc.getEpoch() + backOffTime3);
  if ( ForceJoinNetwork())  return true;

  GoToSleep(rtc.getEpoch() + backOffTime4);
  if ( ForceJoinNetwork())  return true;

  GoToSleep(rtc.getEpoch() + backOffTime5);
  if ( ForceJoinNetwork())  return true;

  GoToSleep(rtc.getEpoch() + backOffTime6);
  if ( ForceJoinNetwork())  return true;

  // if we are here, we were unable to join network after all backoff's...
  debug1("**Unable to join network after backoff's...\n");
  setErrorCode (errUnableToJoin);
  
      rebootCPU();        // we should reboot at this point   ************* We may have a serious error, or network is un-avilable ***********

  return false;
}


/* ************************************************************** */
void rebootCPU(void)
{
  // local Tag for logging
  TAG = __FUNCTION__;
  
  lora.setDeviceReset();
  setErrorCode (errRebootedCPU_Radio);
  MasterBootBlockCopy.BootDataStruct.errorCount++;
  saveFRAM();

  debug1("\n**** Rebooting CPU ****\n");
  delay (500);
  
        NVIC_SystemReset();      // MAJOR! --> processor software reset of CPU --> ************* We must have a serious error ***********
}

/* ************************************************************** */
// Reset radio
bool resetRadio(void)
{
  // local Tag for logging
  TAG = __FUNCTION__;
  
  if (lora.setDeviceReset(DEFAULT_RESPONSE_TIMEOUT + 10))      // try to reset the radio
  {
    debug1(">>Reset OK\n");                                    // if we did reset the radio

    if (checkJoinStatus())                                     // try to re join..
    {
      return true;  
    }   
  }                                                                              
  debug1("**Reset Failed\n"); 
  return false; 
}


/* ************************************************************** */
// See if radio is responding
bool testRadio(void)
{
  // local Tag for logging
  TAG = __FUNCTION__;

  for (uint8_t i = 0; i < 16 ; i++)
  {                                                             //  We call getTest() because after a reset the LoRaWAN module can be sleeping
    if (lora.getTest(DEFAULT_RESPONSE_TIMEOUT) )
    {
      debug2(">>Check Radio OK\n");
      return true;
    }
    else
    {
      delay(100);
    }     
  } 
  return false; 
}


/* ************************************************************** */
// See if radio is responding, if not we should reboot CPU
bool checkRadio(void)
{
  // local Tag for logging
  TAG = __FUNCTION__;

  debug1("*>Checking Radio...\n");
  
  if(testRadio())    return true;
  
  debug1("\n**ERROR No responds from radio\n");
  setErrorCode (errRadioNotResponding);
  
  if (resetRadio())                      // try to reset radio
  {
    if (testRadio()) return true;        // see if radio is responding
  }

  // if not, let try a hardware reset of radio
    pinMode(RadioReset, OUTPUT);         // force a clean hardware reset on the radio...
    digitalWrite(RadioReset, LOW);       // toggle radio re-set pin
    delay(250);
    digitalWrite(RadioReset, HIGH);
    delay(1000);
    
  if (testRadio())    return true;      // see if radio is responding

  // if not, let try a hardware reset CPU
    debug1("\n**ERROR still no responds from radio, re-booting CPU\n");
    rebootCPU();                          // it's time to re-boot CPU
   return false;                                 
}


/* ************************************************************** */
/* ************************ LOOP ******************************** */
/* ************************************************************** */
void loop(void)
{
  // local Tag for logging
  TAG = __FUNCTION__;

  bool result;

//   getEpoch() is preset with a time offset of 946684800 sec's
//   Saturday, 1 January 2000 00:00:00 = 946684800

#ifdef EnableRadio 

//  memset(buffer, 0, 256);
//  lora.getRTCFull(buffer, 256, DEFAULT_RESPONSE_TIMEOUT);
//  debug1("\n%s\n", buffer);
//
//  memset(buffer, 0, 1024);
//  lora.getChannel(buffer, 1024, DEFAULT_RESPONSE_TIMEOUT+30);
//  //debug1("%s\n",buffer);                                // Arduino is ONLY printing the 1st 77 char of this string...
//  Serial1.println(buffer);


/* ************************************************************** */
// check for too many un-sent packets, if so, we will try to re-join...
  if  (MasterBootBlockCopy.BootDataStruct.UnableToSendCnt >= MaxPacketErrors)        // see if we have too many packet send errors, if so, we should do a network re-join... 
  {
    MasterBootBlockCopy.BootDataStruct.UnableToSendCnt = 0;
    setErrorCode (errResetPacketErrorCnt);
    checkJoinStatus();                                                               // lets try to re join...                                                
  }


/* ************************************************************** */
  // check if we had an error, if so,  send a short status message
  if  (errorFlag)
  {  
      result = sentShortStatusMessage();                                             // if we had an error try to a send short message
      PacketSent++;

    if (result)
    {
      debug1(">>Sending Short Status was OK, so lets check for RX Packet\n");
      errorFlag = false;
      processRxMessage();
    }
    else
    {
      MasterBootBlockCopy.BootDataStruct.UnableToSendCnt++;
      errorFlag = true; 
      setErrorCode(errShortStatusMsgNotSent);
      debug1("**We were unable to send Short Status Packet, Packet Errors: %u\n", MasterBootBlockCopy.BootDataStruct.UnableToSendCnt);
      displayErrorMessages();
    }
  }


/* ************************************************************** */
// check if it's time to do a periodic re-join
  if ( rtc.getEpoch() >= nextReJoinTime ) 
  {
      nextReJoinTime = rtc.getEpoch() + MasterBootBlockCopy.BootDataStruct.re_join_interval;     // set next re-join interval
      debug1(">>Time to Re-Joing the network\n");
      checkJoinStatus();       
  }


/* ************************************************************** */
  // check if its time to  send a status message
  if ( rtc.getEpoch() >= nextStatRunTime )                              // will send on first boot or at nextStatRunTime...
  {
      //memset(buffer, 0, 256);
      //lora.getSystemTime(buffer, 1024, DEFAULT_RESPONSE_TIMEOUT);     // Request time from server
      //debug1("%s\n", buffer);
      
      nextStatRunTime = rtc.getEpoch() + MasterBootBlockCopy.BootDataStruct.tx_stat_interval;     // set status TX interval for next TX
      result =  sendStatusMessage();                                    // sent status messsage
      PacketSent++; 
          
    if (result)
    {
      debug1(">>Sending Status was OK, so lets check for RX Packet\n");
      errorFlag = false;
      processRxMessage();
    }
    else
    {
      MasterBootBlockCopy.BootDataStruct.UnableToSendCnt++;
      errorFlag = true;
      setErrorCode(errStatusMsgNotSent); 
      debug1("**We were unable to send Status Packet, Packet Errors: %u\n", MasterBootBlockCopy.BootDataStruct.UnableToSendCnt);       // if here, message was not sent
      displayErrorMessages();
    }
  }
#endif    // EnableRadio 


/* ************************************************************** */
// check if its time to send a sensor data message
  if ( rtc.getEpoch() >= nextRunTime)               // check if its time to send sensor data message
  {
        nextRunTime = (rtc.getEpoch() +  MasterBootBlockCopy.BootDataStruct.tx_interval);            // get current time, add delay = nextRunTime in sec --> 300 = 5 mins
        debug1("\n>>Sending Sensor Data\n");
        result = Sensor_send (data, 0 );            // run sensor code to start TX
        PacketSent++;

#ifdef EnableRadio       
    if (result)
    {
      debug1(">>Sending Sensor Data Message was OK, so lets check for RX Packet\n");
      errorFlag = false; 
      processRxMessage();
    }
    else
    {
      MasterBootBlockCopy.BootDataStruct.UnableToSendCnt++;
      errorFlag = true;
      setErrorCode(errDataMsgNotSent);
      debug1("**We were unable to send Sensor Data Packet, Packet Errors: %u\n", MasterBootBlockCopy.BootDataStruct.UnableToSendCnt);
      displayErrorMessages();
      // we need a re-try/re-join function here....
    }
#endif    // EnableRadio

  }

#ifdef EnableRadio 
// lets check packet error counter... 
  errorRate = abs( ((((float) PacketSent - (float) MasterBootBlockCopy.BootDataStruct.UnableToSendCnt) / (float) PacketSent) - 1 ) * 100);  
  debug1(">>Packets: %u, Packet Errors: %u, Lost Rate: %7.2f%%\n", PacketSent , MasterBootBlockCopy.BootDataStruct.UnableToSendCnt, errorRate);
  debug1(">>Radio Busy1: %u, Busy2: %u\n", busyCount1, busyCount2 );
#endif  // EnableRadio

#ifdef  LoRaClassC
  // If were here, we are in Class C, we will take a short nap, radio staying active, minimal power saving
  GoToSleep(rtc.getEpoch() + ClassC_Sleep);
  processRxMessage();

#else       // we are in class A if we are here

#ifdef EnableRadio
  lora.setDeviceLowPower();               // set the LoRaWAN module into sleep mode
#endif

  // let start shutting down our tasks and sensors
  Sensor_sleep();                         // save any data needed here prior to sleep
  resetIOPins();                          // put I/O pins in low power state

      GoToSleep(nextRunTime);             // it's time for sleep....

  setIOPins();                            // reset I/O pins for normal operation...
  Sensor_wakeup();

#endif  // end of LoRaClassC

}   // end of loop

/* ************************************************************** */
/* ****************** End of LOOP ******************************* */
/* ************************************************************** */



/* ************************************************************** */
void GoToSleep(uint32_t sTime)
{
  // local Tag for logging
  TAG = __FUNCTION__;
  
  debug1("*>Current Time is:      %lu\n",  rtc.getEpoch());
  debug1("*>Going to sleep until: %lu\n\n", sTime);

GoToSleep:

  doSleep(sTime);                        // sleep now....

  if (rtc.getEpoch() < sTime )          // check to see if cause of wakeup was an external interrupt, if so, go back to sleep again
  {
   // debug2( "We were in ISR, so going back to sleep\n");
    goto GoToSleep;                     // we were in ISR, so go back to sleep for for now...
  }
  debug1(">>Wakeing up at:        %lu\n\n",  rtc.getEpoch());

#ifdef EnableRadio
  checkRadio();                         // check to make sure radio is alive after wake up
#endif

}


/* ************************************************************** */
void doSleep(uint32_t sTime)
{
  // local Tag for logging
  TAG = __FUNCTION__;
  
  if ( rtc.getEpoch() < sTime)              // we need to make sure new sleep time is pass current time
  {
#ifdef UseDelay
    while ( rtc.getEpoch() < sTime ) 
    {
      delay(10);                            // this will fake sleeping for testing...
    }
#else
    rtc.setAlarmEpoch(sTime);               // set a new alarm time in seconds
    rtc.enableAlarm(rtc.MATCH_HHMMSS);      // currently only checking for HMS, not days or years
    rtc.standbyMode();                      // bring CPU into deep sleep mode (until woken up by the RTC alarm or interrupt)
#endif

  }
  else    // if we are here, sleep time is in the past --> this is an error...
  {
    debug1("***** ERROR --> New sleep time is in the past *****\n\n");
    setErrorCode (errSleepTime);
  }
}


/* ****************************************************************************** */
// convert's a byte array to a hex-string array
// need to have room in string array for and ending 0x00 this is added
// n = size of new hex string
void btox(char *xp,  char *bb, int n)
{
  // local Tag for logging
  TAG = __FUNCTION__;

  const char xx[] = "0123456789ABCDEF";
  xp[n] = 0x00;                           // terminate the string
  while (--n >= 0) xp[n] = xx[(bb[n >> 1] >> ((1 - (n & 1)) << 2)) & 0xF];
}


/* ****************************************************************************** */
// Check for hard-coded DEVEUI, if all zero, then look for a 24AA025E64 chip (or ATECC608A)
// we need to add code here if we have a ATECC608A
//  also for Modem DEVEUI or CPU UUI
void getDevEui(char *buf)
{
  // local Tag for logging
  TAG = __FUNCTION__;

  if (DEV_EUI == "0000000000000000")          // check if DEV_EUI in keys.h is zero
  {

#if  MCP_24AA025E64_I2C_ADDRESS
    // Get from MCP 24AA025E64 hardware DEVEUI (override default settings if found)
    char Mybuf[10];
    if (get_E64_deveui(Mybuf))               // get DEVEUI from MCP 24AA025E64
    {
      btox( buf,  Mybuf, 16);
      //debug1 (">>MyDevEUI: %s\n", buf);
      return;
    }
#elif  ATECC608A

#else
#error !DEVEUI is not defined
#endif

  }   // end if (...)

  else
  {
    // using hard-coded DEVEUI from Keys's file...
    strcpy( (char*) buf, DEV_EUI);
    //debug1 (">MyDevEUI: %s\n", buf);
    debug1 (">>Using Hard-Coded DEVEUI\n");
    return;
  }
  debug1 ("**No DEVEUI defined\n");
}


/* ****************************************************************************** */
// Read DEVEUI from Microchip 24AA025E64 2Kb serial EEprom, if present
bool get_E64_deveui(char *pdeveui)
{
  // local Tag for logging
  TAG = __FUNCTION__;

#ifdef MCP_24AA025E64_I2C_ADDRESS

  Wire.setClock(400000);                                // Init this just in case, no more than 400KHz
  Wire.beginTransmission(MCP_24AA025E64_I2C_ADDRESS);   // I2C address of EEPROM
  Wire.write(MCP_24AA025E64_MAC_ADDRESS);               // address of EUI64  
  uint8_t i2c_ret = Wire.endTransmission();

  // check if device is avilable on i2c bus
  if (i2c_ret == 0)
  {
    char deveui[32] = "";
    uint8_t data;

    Wire.beginTransmission(MCP_24AA025E64_I2C_ADDRESS);
    Wire.write(MCP_24AA025E64_MAC_ADDRESS);
    Wire.endTransmission();

    Wire.requestFrom(MCP_24AA025E64_I2C_ADDRESS, 8);
    while (Wire.available())
    {
      data = Wire.read();
      *pdeveui++ = data;
    }
    debug1 (">>Using EUI64 from EEPROM\n");
    return true;
  }
  else
#endif // MCP 24AA025E64
    debug1 ("Could not find an EUI64 EEPROM\n");
  return false;
}


/* ****************************************************************************** */
// Display a EUI key string in hex
void printKey(const char *name, char *key, uint8_t len, bool lsb)
{
  // local Tag for logging
  TAG = __FUNCTION__;

  const char *p;
  char keystring[ (len + strlen (name) + 1) ] = "";         // must be large enought for key string and name
  char keybyte[3];

  for (uint8_t i = 0; i < len ; i++)
  {
    p = lsb ? (key + len - 1) - i : key + i;
    sprintf(keybyte, "%02X", *p);
    strncat(keystring, keybyte, 2);
  }
  debug2("%s: %s\n", name, keystring);
}


/* ********************* The Very End *************************** */
/* ************************************************************** */
