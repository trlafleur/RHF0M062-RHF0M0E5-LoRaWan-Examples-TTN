/*
 * 
 * 
 * 
 */

#include "config.h"
#include "Utilities_M0.h"

uint64_t CurrentTime;
uint64_t NewTime;


/* ************************************************************* */
// Get current free ram that's is available
extern "C" char *sbrk(int i);
     
uint32_t FreeRam () 
{
  char stack_dummy = 0;
  return &stack_dummy - sbrk(0);
}


/* ************************************************************* */
// 0 = off, 1 = on, 2 = toggle state
void setLED (uint8_t led, uint8_t state)   // void setLED (int MyLED,int state)
{
  // local Tag for logging
  const char* TAG = __FUNCTION__;
  
  //if (( led >= 0) || ( led <= 3))
    {
      if ( state == 0) digitalWrite(led, 0);  return;            // turn off LED
      if ( state == 1) digitalWrite(led, 1);  return;            // turn on LED
      if ( state == 2) digitalWrite(led, !digitalRead(led));     // toggle it
    }
}


/* ************************************************************** */
// this will save the current time...
void saveTime(void)
{
  CurrentTime = millis();
}


/* ************************************************************** */
// this will print the elapse time...
void getTime(void)
{
  // local Tag for logging
  const char* TAG = __FUNCTION__;
  
  NewTime = millis();
  debug1("***Total Time: %u ms\n", (uint32_t) (NewTime - CurrentTime) );
}


/* ************************************************************************************** */
/* A function to print a buffer in hex format
 *
 *  hexdump (buffer, length, 16);
 */
#include <stdio.h>
#include <stdlib.h>

void hexdump(unsigned char *buffer, unsigned long index, unsigned long width)
{
    unsigned long i;
    for (i=0; i<index; i++)
    {
      debug1(PSTR("%02x "), buffer[i]); 
    }
    
    for (unsigned long spacer=index;spacer<width;spacer++)
      debug1(PSTR("  ")); 
      debug1(PSTR(": "));
      for (i=0; i<index; i++)
      {
        if (buffer[i] < 32) debug1(PSTR("."));
        else debug1(PSTR("%c"), buffer[i]);
      }
    debug1(PSTR("\n")); 
}


/* ************************************************************************************** */
void hexdump1 ( const void *addr, const uint32_t len) 
{
    int i;
    unsigned char buff[17];
    const unsigned char * pc = (const unsigned char *)addr;

    // Length checks.

    if (len == 0) {
        debug1("  ZERO LENGTH\n");
        return;
    }
    else if (len < 0) {
        debug1("  NEGATIVE LENGTH: %d\n", len);
        return;
    }

    debug1("\n");
    // Process every byte in the data.

    for (i = 0; i < len; i++) 
    {
        // Multiple of 16 means new line (with line offset).

        if ((i % 16) == 0) 
        {
            // Don't print ASCII buffer for the "zeroth" line.

            if (i != 0)
                debug1 ("  %s\n", buff);

            // Output the offset.

            debug1 ("  %04x ", i);
        }

        // Now the hex code for the specific character.
        debug1 (" %02x", pc[i]);

        // Add to buffer a printable ASCII character for later.

        if ((pc[i] < 0x20) || (pc[i] > 0x7e)) // isprint() may be better.
            buff[i % 16] = '.';
        else
            buff[i % 16] = pc[i];
        buff[(i % 16) + 1] = '\0';
    }

    // Pad out last line if not exactly 16 characters.

    while ((i % 16) != 0) 
    {
        debug1 ("   ");
        i++;
    }

    // And print the final ASCII buffer.

    debug1 ("  %s\n", buff);
}
/* ************************************************************************************** */
/* ************************************************************************************** */
