/*
 * 
 * 
 * 
 */


#ifndef Sensor_Moisture_h
#define Sensor_Moisture_h

#include "config.h"

void Sensor_init          (void);
bool Sensor_send          (unsigned char *data, uint32_t time); 
void Sensor_sleep         (void); 
void Sensor_wakeup        (void);
void Sensor_SaveState     (void); 
void Sensor_RetoreState   (void);

#endif
