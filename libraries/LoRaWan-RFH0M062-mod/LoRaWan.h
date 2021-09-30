/*
  LoRaWAN.h
  2013 Copyright (c) Seeed Technology Inc.  All right reserved.

  Author: Wayne Weng
  Date: 2016-10-17

  add rgb backlight fucnction @ 2013-10-15
  
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
*/

#ifndef _LORAWAN_H_
#define _LORAWAN_H_


#include <Arduino.h>

#define Serial              Serial1          // see notes
#define SerialLoRa          Serial5

#define _DEBUG_SERIAL_      1
#define DEFAULT_TIMEOUT     5       // second
#define DEFAULT_TIMEWAIT    300     // millisecond
#define DEFAULT_DEBUGTIME   2       // second
#define SERIAL_TIMEOUT      100     // millisecond, 1 char at 9600baud is about 1ms

//#define BATTERY_POWER_PIN    A4
//#define CHARGE_STATUS_PIN    A5

#define BUFFER_LENGTH_MAX    1024


enum _class_type_t          { CLASS_A   = 0, CLASS_B, CLASS_C };
enum _physical_type_t       { EU434     = 0, EU868, US915, US915HYBRID, AU915, AU915OLD, CN470, CN779, AS923, KR920, IN865 };
enum _device_mode_t         { LWABP     = 0, LWOTAA, TEST };
enum _otaa_join_cmd_t       { JOIN      = 0, FORCE };
enum _window_delay_t        { RECEIVE_DELAY1 = 0, RECEIVE_DELAY2, JOIN_ACCEPT_DELAY1, JOIN_ACCEPT_DELAY2 };
enum _band_width_t          { BW125     = 125, BW250 = 250, BW500 = 500 };
enum _spreading_factor_t    { SF12      = 12, SF11 = 11, SF10 = 10, SF9 = 9, SF8 = 8, SF7 = 7 };
enum _data_rate_t           { DR0       = 0, DR1, DR2, DR3, DR4, DR5, DR6, DR7, DR8, DR9, DR10, DR11, DR12, DR13, DR14, DR15 };
enum _error_msg_t           { OK, DONE, BUSY, ERROR, JOIN_NETWORK_FIRST, NET_JOIN, NO_FREE_CH, NO_BAND, DR_ERROR, LENGTH_ERROR, WAIT_ACK, FPENDING, JOIN_FAILED};               
   
                    
/* ************************************************************** */
// define a data structure for modem stat parameters we need
typedef struct LORADATA_t 
{
    int16_t     CurrentRSSI;        // Current RSSI from last Rx
    char        LastError;          // Last Error from radio                
    float       CurrentSNR;         // Current DNR from last Rx
    uint8_t     CurrentLINK;        // Current Link Status
    uint8_t     GatewayCNT;         // Current Gateway Count from Link message
    uint8_t     ErrorCNT;           // Error Count
    int8_t      LastErrorCode;      // Error Message from modem
    bool        ACKFlag;            // true if we received an ACK on last message
    bool        DoneFlag;           // true if we received a Done
    bool        JoinFlag;           // true if we go a join from network
    bool        BusyFlag;           // True if Modem is busy 
} LoRaData; 

extern LoRaData  LoRaStats;



/*****************************************************************
Type    DataRate    Configuration   BitRate| TxPower Configuration 
EU434   0           SF12/125 kHz    250    | 0       10dBm
        1           SF11/125 kHz    440    | 1       7 dBm
        2           SF10/125 kHz    980    | 2       4 dBm
        3           SF9 /125 kHz    1760   | 3       1 dBm
        4           SF8 /125 kHz    3125   | 4       -2dBm
        5           SF7 /125 kHz    5470   | 5       -5dBm
        6           SF7 /250 kHz    11000  | 6:15    RFU
        7           FSK:50 kbps     50000  | 
        8:15        RFU                    |     
******************************************************************
Type    DataRate    Configuration   BitRate| TxPower Configuration 
EU868   0           SF12/125 kHz    250    | 0       20dBm
        1           SF11/125 kHz    440    | 1       14dBm
        2           SF10/125 kHz    980    | 2       11dBm
        3           SF9 /125 kHz    1760   | 3       8 dBm
        4           SF8 /125 kHz    3125   | 4       5 dBm
        5           SF7 /125 kHz    5470   | 5       2 dBm
        6           SF7 /250 kHz    11000  | 6:15    RFU
        7           FSK:50 kbps     50000  | 
        8:15        RFU                    | 
******************************************************************
Type    DataRate    Configuration   BitRate| TxPower Configuration
US915   0           SF10/125 kHz    980    | 0       30dBm
        1           SF9 /125 kHz    1760   | 1       28dBm
        2           SF8 /125 kHz    3125   | 2       26dBm
        3           SF7 /125 kHz    5470   | 3       24dBm
        4           SF8 /500 kHz    12500  | 4       22dBm
        5:7         RFU                    | 5       20dBm
        8           SF12/500 kHz    980    | 6       18dBm
        9           SF11/500 kHz    1760   | 7       16dBm
        10          SF10/500 kHz    3900   | 8       14dBm
        11          SF9 /500 kHz    7000   | 9       12dBm
        12          SF8 /500 kHz    12500  | 10      10dBm
        13          SF7 /500 kHz    21900  | 11:15   RFU
        14:15       RFU                    | 
*******************************************************************/


class LoRaWanClass
{
    public:
    
        LoRaWanClass(void);
        
        /**
         *  \brief Initialize the conmunication interface
         *  
         *  \return Return null
         */
        void init(void);

         /**
         *  \brief Send a command to device
         *  
         *  \return Return null
         */
        void sendCMD(char* CMD);
            
         /**
         *  \brief Send a command to device
         *  
         *  param [in] *buffer The output data cache
         *  \param [in] length The length of data cache
         *  \param [in] timeout The over time of read
         *  
         *  \return Return null
         */
        void sendCMD(char* CMD, char *buffer, short length, unsigned int timeout);
        
        /**
         *  \brief Send a command to device, check for respond
         *  
         *  param [in] *buffer The output data cache
         *  \param [in] length The length of data cache
         *  \param [in] timeout The over time of read
         *  
         *  \return Return null
         */
        bool sendCMD(char* CMD, char* deviceRespond, char *buffer, short length, unsigned int timeout);
         
         /**
         *  \brief Get Channels from device
         *  
         *  \param [in] *buffer The output data cache
         *  \param [in] length The length of data cache
         *  \param [in] timeout The over time of read
         *  
         *  \return Return null.
         */        
        void getChannel(char *buffer, short length, unsigned int timeout = DEFAULT_TIMEOUT);
        
         /**
         *  \brief Test command from device
         *  
         *  \param [in] *buffer The output data cache
         *  \param [in] length The length of data cache
         *  \param [in] timeout The over time of read
         *  
         *  \return Return true if radio is responding.
         */
        bool getTest( unsigned int timeout = DEFAULT_TIMEOUT);
         
         /**
         *  \brief Read the version from device
         *  
         *  \param [in] *buffer The output data cache
         *  \param [in] length The length of data cache
         *  \param [in] timeout The over time of read
         *  
         *  \return Return null.
         */
        void getVersion(char *buffer, short length, unsigned int timeout = DEFAULT_TIMEOUT);

       /**
         *  \brief Read the protocal version from device
         *  
         *  \param [in] *buffer The output data cache
         *  \param [in] length The length of data cache
         *  \param [in] timeout The over time of read
         *  
         *  \return Return null.
         */
        void getProtocalVersion(char *buffer, short length, unsigned int timeout = DEFAULT_TIMEOUT);
                
        /**
         *  \brief Read the ID from device
         *  
         *  \param [in] *buffer The output data cache
         *  \param [in] length The length of data cache
         *  \param [in] timeout The over time of read
         *  
         *  \return Return null.
         */  
        void getId(char *buffer, short length, unsigned int timeout = DEFAULT_TIMEOUT);
        
        /**
         *  \brief Read the DevEui from device (not working at this time...TRL)
         *  
         *  \param [in] *buffer The output data cache
         *  \param [in] length The length of data cache
         *  \param [in] timeout The over time of read
         *  
         *  \return Return null.
         */                 
        void getDevEui(char *buffer, short length, unsigned int timeout = DEFAULT_TIMEOUT);
        
        /**
         *  \brief Read the DevADDR from device (not working at this time...TRL)
         *  
         *  \param [in] *buffer The output data cache
         *  \param [in] length The length of data cache
         *  \param [in] timeout The over time of read
         *  
         *  \return Return null.
         */                
        void getDevAddr(char *buffer, short length, unsigned int timeout = DEFAULT_TIMEOUT);
        
        /**
         *  \brief Read the AppEui from device (not working at this time...TRL)
         *  
         *  \param [in] *buffer The output data cache
         *  \param [in] length The length of data cache
         *  \param [in] timeout The over time of read
         *  
         *  \return Return null.
         */               
        void getAppEui(char *buffer, short length, unsigned int timeout = DEFAULT_TIMEOUT);
        
        /**
         *  \brief Read the max payload length from device
         *  
         *  \param [in] *buffer The output data cache
         *  \param [in] length The length of data cache
         *  \param [in] timeout The over time of read
         *  
         *  \return Return null.
         */        
        void getMaxLen(char *buffer, short length, unsigned int timeout = DEFAULT_TIMEOUT);
        
         /**
         *  \brief Request system time from the network
         *  
         *  \param [in] *buffer The output data cache
         *  \param [in] length The length of data cache
         *  \param [in] timeout The over time of read
         *  
         *  \return Return null.
         */        
        void getSystemTime(char *buffer, short length, unsigned int timeout = DEFAULT_TIMEOUT);
        
         /**
         *  \brief Request system time from the network
         *  
         *  \param [in] *buffer The output data cache
         *  \param [in] length The length of data cache
         *  \param [in] timeout The over time of read
         *  
         *  \return Return null.
         */        
        void getRTC(char *buffer, short length, unsigned int timeout = DEFAULT_TIMEOUT);
        
         /**
         *  \brief Request system time from the network
         *  
         *  \param [in] *buffer The output data cache
         *  \param [in] length The length of data cache
         *  \param [in] timeout The over time of read
         *  
         *  \return Return null.
         */        
        void getRTCFull(char *buffer, short length, unsigned int timeout = DEFAULT_TIMEOUT);
        
         /**
         *  \brief Set system time 
         *  
         *  \param [in] RTC a string
         *  
         *  \return Return null.
         */        
        void setRTC(char* RTCCMD);       
        
        
         /**
         *  \brief Request RXWIN1 from the network
         *  
         *  \param [in] *buffer The output data cache
         *  \param [in] length The length of data cache
         *  \param [in] timeout The over time of read
         *  
         *  \return Return null.
         */        
         void getReceiveWindowFirst(char *buffer, short length, unsigned int timeout = DEFAULT_TIMEOUT);
        
         /**
         *  \brief Request RXWIN2 from the network
         *  
         *  \param [in] *buffer The output data cache
         *  \param [in] length The length of data cache
         *  \param [in] timeout The over time of read
         *  
         *  \return Return null.
         */
         void getReceiveWindowSecond(char *buffer, short length, unsigned int timeout = DEFAULT_TIMEOUT);
        
        /* Sets... */  
        /**
         *  \brief Set the ID
         *  
         *  \param [in] *DevAddr The end-device address
         *  \param [in] *DevEUI The end-device identifier
         *  \param [in] *AppEUI The application identifier
         *  
         *  \return Return null.
         */        
        void setId(char *DevAddr, char *DevEUI, char *AppEUI);
        
        /**
         *  \brief Set the key
         *  
         *  \param [in] *NwkSKey The network session key
         *  \param [in] *AppSKey The application session key
         *  \param [in] *AppKey The Application key
         *  
         *  \return Return null.
         */       
        void setKey(char *NwkSKey, char *AppSKey, char *AppKey);
        
           /**
         *  \brief Request current Data Rate from Modem
         *  
         *  \param [in] *buffer The output data cache
         *  \param [in] length The length of data cache
         *  \param [in] timeout The over time of read
         *  
         *  \return Return null.
         */        
        void getDR(char *buffer, short length, unsigned int timeout = DEFAULT_TIMEOUT);      
        
        /**
         *  \brief Set the data rate
         *  
         *  \param [in] dataRate The date rate of encoding
         *  
         *  \return Return null.
         */                
        void setDataRate(_data_rate_t dataRate ); 
        
              /**
         *  \brief Set the data rate type
         *  
         *  \param [in] physicalType The type of ISM
         *  
         *  \return Return null.
         */                
        void setDataRate( _physical_type_t physicalType ); 
        
        /**
         *  \brief Set the data rate
         *  
         *  \param [in] dataRate The date rate of encoding
         *  \param [in] physicalType The type of ISM
         *  
         *  \return Return null.
         */                
        void setDataRate(_data_rate_t dataRate , _physical_type_t physicalType ); 
       
        /**
         *  \brief ON/OFF adaptive data rate mode
         *  
         *  \param [in] command The date rate of encoding
         *  
         *  \return Return null.
         */        
        void setAdaptiveDataRate(bool command);
        
                /**
         *  \brief ON/OFF public/private network
         *  
         *  \param [in] command The true = public
         *  
         *  \return Return null.
         */        
        void setNetworkPublic(bool command);
         
        /**
         *  \brief send a wakeup command, 4x 0xff and AT+VER\r\d
         *  
         *  \param [in] void
         *  
         *  \return Return null.
         */       
        void sendWakeUp(void);
 
        /**
         *  \brief send a wakeup command, 4x 0xff and AT+VER\r\d
         *  
         *  \param [in] void
         *  
         *  \return Return true if awake.
         */       
        bool sendWakeUp(unsigned int timeout);
 
        
        /**
         *  \brief Set the output power
         *  
         *  \param [in] power The output power value
         *  
         *  \return Return null.
         */        
        void setPower(short power); 
              
        /**
         *  \brief Set the port number
         *  
         *  \param [in] port The port number, range from 1 to 255
         *  
         *  \return Return null.
         */        
        void setPort(unsigned char port);
       
         /**
         *  \brief Set the channel parameter
         *  
         *  \param [in] channel a string
         *  
         *  \return Return null.
         */       
        void setChannels(char* channel);
        
        /**
         *  \brief Set the channel parameter
         *  
         *  \param [in] channel The channel number, range from 0 to 71
         *  \param [in] frequency The frequency value
         *  
         *  \return Return null.
         */       
        void setChannel(unsigned char channel, float frequency);
       
        /**
         *  \brief Set the channel parameter
         *  
         *  \param [in] channel The channel number, range from 0 to 71
         *  \param [in] frequency The frequency value. Set frequecy zero to disable one channel
         *  \param [in] dataRata The date rate of channel
         *  
         *  \return Return null.
         */     
        void setChannel(unsigned char channel, float frequency, _data_rate_t dataRata);
        
        /**
         *  \brief Set the channel parameter
         *  
         *  \param [in] channel The channel number, range from 0 to 71
         *  \param [in] frequency The frequency value
         *  \param [in] dataRataMin The minimum date rate of channel
         *  \param [in] dataRataMax The maximum date rate of channel
         *  
         *  \return Return null.
         */        
        void setChannel(unsigned char channel, float frequency, _data_rate_t dataRataMin, _data_rate_t dataRataMax);
    
         /**
         *  \Link Check Transfer the data
         *  
         *  \param [in] timeout The over time of transfer
         *  
         *  \return Return bool. Tree : transfer done, false : transfer failed
         */        
        bool LinkCheckReq(unsigned int timeout = DEFAULT_TIMEOUT);
        
                /**
         *  \Link Check Transfer the data
         *  
         *  \param [in] timeout The over time of transfer
         *  
         *  \return Return bool. Tree : transfer done, false : transfer failed
         */        
        bool LinkCheckReq1(unsigned int timeout = DEFAULT_TIMEOUT);
        
        
         /**
         *  \brief Transfer the data
         *  
         *  \param [in] *buffer The transfer data cache
         *  \param [in] timeout The over time of transfer
         *  
         *  \return Return bool. Truee : transfer done, false : transfer failed
         */        
        bool transferPacket(char *buffer, unsigned int timeout = DEFAULT_TIMEOUT);
        
        /**
         *  \brief Transfer the data
         *  
         *  \param [in] *buffer The transfer data cache
         *  \param [in] length The length of data cache
         *  \param [in] timeout The over time of transfer
         *  
         *  \return Return bool. True : transfer done, false : transfer failed
         */
        bool transferPacket(unsigned char *buffer, unsigned char length, unsigned int timeout = DEFAULT_TIMEOUT);
        
        /**
         *  \brief Transfer the packet data
         *  
         *  \param [in] *buffer The transfer data cache
         *  \param [in] timeout The over time of transfer
         *  
         *  \return Return bool. True : Confirmed ACK, false : Confirmed NOT ACK
         */
        bool transferPacketWithConfirmed(char *buffer, unsigned int timeout = DEFAULT_TIMEOUT);
        
        /**
         *  \brief Transfer the data
         *  
         *  \param [in] *buffer The transfer data cache
         *  \param [in] length The length of data cache
         *  \param [in] timeout The over time of transfer
         *  
         *  \return Return bool. True : Confirmed ACK, false : Confirmed NOT ACK
         */
         
        bool transferPacketWithConfirmed(unsigned char *buffer, unsigned char length, unsigned int timeout = DEFAULT_TIMEOUT);

        /**
         *  \brief Receive the data
         *  
         *  \param [in] *buffer The receive data cache
         *  \param [in] length The length of data cache
         *  \param [in] *rssi The RSSI cache
         *  
         *  \return Return Receive data number
         */
        short receivePacket(char *buffer, short length, short *rssi);
        
         /**
         *  \brief Receive the data
         *  
         *  \param [in] *buffer The receive data cache
         *  \param [in] length The length of data cache
         *  \param [in] *rssi The RSSI cache
         *  \param [in] *port of the RX data
         *  
         *  
         *  \return Return Receive data number
         */
        short receivePacket(char *buffer, short length, short *rssi, short *port);

        
        /**
         *  \brief Transfer the proprietary data
         *  
         *  \param [in] *buffer The transfer data cache
         *  \param [in] timeout The over time of transfer
         *  
         *  \return Return bool. True : transfer done, false : transfer failed
         */
        bool transferProprietaryPacket(char *buffer, unsigned int timeout = DEFAULT_TIMEOUT);
 
        /**
         *  \brief Transfer the proprietary data
         *  
         *  \param [in] *buffer The transfer data cache
         *  \param [in] length The length of data cache
         *  \param [in] timeout The over time of transfer
         *  
         *  \return Return bool. True : transfer done, false : transfer failed
         */       
        bool transferProprietaryPacket(unsigned char *buffer, unsigned char length, unsigned int timeout = DEFAULT_TIMEOUT);
        
        /**
         *  \brief Set device mode
         *  
         *  \param [in] mode The mode of device
         *  
         *  \return Return null
         */
        void setDeviceMode(_device_mode_t mode);
        
        /**
         *  \brief Set device join a network
         *  
         *  \param [in] command The type of join
         *  \param [in] timeout The over time of join
         *  
         *  \return Return bool. True : join OK, false : join NOT OK
         */
        bool setOTAAJoin(_otaa_join_cmd_t command, unsigned int timeout = DEFAULT_TIMEOUT);
 
        /**
         *  \brief Set device join a network
         *  
         *  \param [in] min_period time of join
         *  \param [in] max_period time of join
         *  \param [in] step time of join
         *  \param [in] timeout = max_period
         *  
         *  \return Return bool. True : join OK, false : join NOT OK
         */       
        bool setOTAAJoinAuto(unsigned int min_period, unsigned int max_period, unsigned int step);
        
        /**
         *  \brief Set message unconfirmed repeat time
         *  
         *  \param [in] time The repeat time, range from 1 to 15
         *  
         *  \return Return null
         */
        void setUnconfirmedMessageRepeatTime(unsigned char time);
        
        /**
         *  \brief Set message retry times time
         *  
         *  \param [in] time The retry time, range from 0 to 254
         *  
         *  \return Return null
         */
        void setConfirmedMessageRetryTime(unsigned char time);
        
        /**
         *  \brief ON/OFF receive window 1
         *  
         *  \param [in] command The true : ON, false OFF
         *  
         *  \return Return null
         */
        void setReceiveWindowFirst(bool command);
        
        /**
         *  \brief Set Receive window 1 channel mapping
         *  
         *  \param [in] channel The channel number, range from 0 to 71
         *  \param [in] frequency The frequency value of channel
         *  
         *  \return Return null
         */
        void setReceiveWindowFirst(unsigned char channel, float frequency);
        
        /**
         *  \brief Set Receive window 2 channel mapping
         *  
         *  \param [in] frequency The frequency value of channel
         *  \param [in] dataRate The date rate value
         *  
         *  \return Return null
         */
        void setReceiveWindowSecond(float frequency, _data_rate_t dataRate);
        
        /**
         *  \brief Set Receive window 2 channel mapping
         *  
         *  \param [in] frequency The frequency value of channel
         *  \param [in] spreadingFactor The spreading factor value
         *  \param [in] bandwidth The band width value
         *  
         *  \return Return null
         */
        void setReceiveWindowSecond(float frequency, _spreading_factor_t spreadingFactor, _band_width_t bandwidth);
        
        /**
         *  \brief ON/OFF duty cycle limitation
         *  
         *  \param [in] command The true : ON, false OFF
         *  
         *  \return Return null
         */
        void setDutyCycle(bool command);
        
        /**
         *  \brief ON/OFF join duty cycle limitation
         *  
         *  \param [in] command The true : ON, false OFF
         *  
         *  \return Return null
         */
        void setJoinDutyCycle(bool command);
        
        /**
         *  \brief Set Receive window delay
         *  
         *  \param [in] command The delay type
         *  \param [in] _delay The delay value(millisecond)
         *  
         *  \return Return null
         */
        void setReceiveWindowDelay(_window_delay_t command, unsigned short _delay);
        
        /**
         *  \brief Set LoRaWAN class type
         *  
         *  \param [in] type The class type
         *  
         *  \return Return null
         */
        void setClassType(_class_type_t type);
        
               /**
         *  \brief Read the current class from device
         *  
         *  \param [in] *buffer The output data cache
         *  \param [in] length The length of data cache
         *  \param [in] timeout The over time of read
         *  
         *  \return Return null.
         */  
        void getClass(char *buffer, short length, unsigned int timeout);
                       
        /**
         *  \brief Set device into low power mode
         *  
         *  \return Return null
         */
        void setDeviceLowPower(void);
        
        /**
         *  \brief Reset device
         *  
         *  \return Return null
         */
        void setDeviceReset(void);
        
               /**
         *  \brief Reset device
         *  
         *  \return Return true if ok
         */
        bool setDeviceReset(unsigned int timeout);
        
        /**
         *  \brief Setup device default
         *  
         *  \return Return null
         */
        void setDeviceDefault(void);
        
               /**
         *  \brief Setup device default
         *  
         *  \return Return true if OK
         */
        bool setDeviceDefault(unsigned int timeout);
        
        /**
         *  \brief Initialize device into P2P mode
         * 
         *  \param [in] frequency The ISM frequency value
         *  \param [in] spreadingFactor The spreading factor value
         *  \param [in] bandwidth The band width value
         *  \param [in] txPreamble The transfer packet preamble number
         *  \param [in] rxPreamble The receive packet preamble number
         *  \param [in] power The output power
         *  
         *  \return Return null
         */
        void initP2PMode(unsigned short frequency = 433, _spreading_factor_t spreadingFactor = SF12, _band_width_t bandwidth = BW125, 
                         unsigned char txPreamble = 8, unsigned char rxPreamble = 8, short power = 20); 
        
        /**
         *  \brief Transfer the data
         *  
         *  \param [in] *buffer The transfer data cache
         *  
         *  \return Return bool. True : transfer done, false : transfer failed
         */
        bool transferPacketP2PMode(char *buffer, unsigned int timeout = DEFAULT_TIMEOUT); 
        
        /**
         *  \brief Transfer the data
         *  
         *  \param [in] *buffer The transfer data cache
         *  \param [in] length The length of data cache
         *  
         *  \return Return bool. True : transfer done, false : transfer failed
         */
        bool transferPacketP2PMode(unsigned char *buffer, unsigned char length, unsigned int timeout = DEFAULT_TIMEOUT);
        
        /**
         *  \brief Receive the data
         *  
         *  \param [in] *buffer The receive data cache
         *  \param [in] length The length of data cache
         *  \param [in] *rssi The RSSI cache
         *  \param [in] timeout The over time of receive
         *  
         *  \return Return Receive data number
         */
        short receivePacketP2PMode(unsigned char *buffer, short length, short *rssi, unsigned int timeout = DEFAULT_TIMEOUT);
        
        
        
        /*  ************************************************************** */
        bool  LinkBusyCheck();
        
        short LinkErrorCheck();
        
        /**
         *  \brief LoRaWAN raw data
         *  
         *  \return Return null
         */
        void loraDebug(void);
#if _DEBUG_SERIAL_
        void loraDebugPrint(unsigned int timeout);  
#endif  

        /**
         *  \brief Read battery voltage
         *  
         *  \return Return battery voltage
         */
        short getBatteryVoltage(void);
        
        
    private:
        void    sendCommand(char *command);
        short   readBuffer (char* buffer, short length, unsigned int timeout = DEFAULT_TIMEOUT);                // readbuffer,  wait utill timeout
        short   readBuffer1(char* buffer, short length, unsigned int timeout = DEFAULT_TIMEOUT);                // readbuffer1, wait utill timeout or serial timeout
        bool    waitForResponse(char* response, unsigned int timeout = DEFAULT_TIMEOUT);
        bool    sendCommandAndWaitForResponse(char* command, char *response, unsigned int timeout = DEFAULT_TIMEOUT);
        void    getMessage (void); 
        void    delay1(unsigned long);                                                                           // delay until timeout or serial timeout
        bool    waitForJoinResponse(unsigned int timeout);   
        
        char    _buffer[BUFFER_LENGTH_MAX];

};


 extern LoRaWanClass lora;


#endif
