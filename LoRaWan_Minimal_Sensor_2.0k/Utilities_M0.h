/*
 * 
 * 
 * 
 * 
 * 
 */

#ifndef UTILITIES_M0_h
#define UTILITIES_M0_h


#include "config.h"
extern  RTCZero rtc;


/* ************************************************************** */
// we use a macro debugx to print, this allows us to turn off printing of messages
//    on non debug code....
/* These are use for local debug of code */
#ifdef MY_DEBUG1
//#define debug1(x, ...) Serial1.printf("[%s] " x, TAG, ##__VA_ARGS__)
#define debug1(x,...) Serial1.printf(x, ##__VA_ARGS__)
//#define debug1(x,...) Serial1.printf("%u: "##x, rtc.getEpoch(), ##__VA_ARGS__)
#else
#define debug1(x,...)
#endif

#ifdef MY_DEBUG2
#define debug2(x,...) Serial1.printf(x, ##__VA_ARGS__)
#else
#define debug2(x,...)
#endif


// forward declarations
void setLED (uint8_t led, uint8_t state);
void saveTime(void);
void getTime(void);
void hexdump  (unsigned char *buffer, unsigned long index, unsigned long width);
void hexdump1 (const void *addr, const uint32_t len);
uint32_t  FreeRam ();

#endif
