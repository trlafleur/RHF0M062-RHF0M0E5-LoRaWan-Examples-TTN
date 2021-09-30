/*
 * 
 *  test sensor...
 * 
 */


#include "Sensor.h"

extern uint16_t   uint16_vbat;
extern uint8_t    currentDR; 
extern uint8_t    errorFlag;
extern            Stopwatch sw;
extern uint8_t    checkMsgLength(uint8_t MyMsgLength);
extern float      errorRate;
extern uint16_t   GetVbat(int VbatPin);


/* ************************************************************* */
void Sensor_init (void)
{
    // local Tag for logging
  const char* TAG = __FUNCTION__;

}

/* ************************************************************* */
bool Sensor_send (unsigned char *data, uint32_t time)
{

  // local Tag for logging
  const char* TAG = __FUNCTION__;
  int packetSize = 22;                                      // set current packed size
  
// we will send data in big-endian format
  if (checkMsgLength(packetSize) >= packetSize)
  {
    debug1(">>Message length ok...\n");
  }
  else
  {
    debug1("**Error Message length...\n");                      // error if here
    setErrorCode (errMsgLength);
  }

    data[0] = DataMesgType;
    
    // Vbat is stored in data[1] and data [2]
    uint16_vbat = GetVbat(BattPin);                        // read the battery voltage...
    data[1] = (char) (uint16_vbat >> 8);
    data[2] = (char) (uint16_vbat & 0x00FF);

    data [3] = (char) 3;                   
    data [4] = (char) 4;               
    data [5] = (char) 5;
    data [6] = (char) 6;
    data [7] = (char) 7;
    data [8] = (char) 8;
    data [9] = (char) 9;
    data[10] = (char) 10;
    data[11] = (char) 11;

    uint16_t TX_IntervalMin = TX_Interval / 60;
    data[12] = (char) (TX_IntervalMin >> 8);           // TX Interval
    data[13] = (char) (TX_IntervalMin & 0x00FF);
    
    data[14] = (char) 14;
    data[15] = (char) 15;

    data[16] = (char) 16;           
    data[17] = (char) 17;
    data[18] = (char) 18;
    data[19] = (char) 19;

    data[20] = (char) currentDR;                     // current Data Rate
    data[21] = (char) abs (errorRate + .5);          // current packed lost in %, converted to char

    //hexdump1(data, packetSize);

#ifdef EnableRadio     
    setLED (MyLED,ON);
    sw.startNewTimer();                             // Restart timer
      bool result = lora.transferPacket(data, packetSize, DEFAULT_RESPONSE_TIMEOUT + 10);  // send the data packet (21 bytes) with a default timeout
    debug1("***Time in MS: %u\n",sw.getElapsedMillis());
    setLED (MyLED,OFF);
    return result;
#endif
    return true;
}
  

/* ************************************************************* */
void Sensor_sleep (void)
{
    // local Tag for logging
  const char* TAG = __FUNCTION__;


}


/* ************************************************************* */
void Sensor_wakeup (void)
{
    // local Tag for logging
  const char* TAG = __FUNCTION__;



}


/* ************************************************************* */
void Sensor_SaveState (void)
{
    // local Tag for logging
  const char* TAG = __FUNCTION__;



}


/* ************************************************************* */
void Sensor_RestoreState (void)
{
    // local Tag for logging
  const char* TAG = __FUNCTION__;
  

}


/* ******************* The End ********************************* */
