#ifndef __SCHEDULER_H
#define __SCHEDULER_H

#include "stm32f4xx_hal.h"

#define INIT_STATE                   0
#define STANDBY_STATE                1
#define RUNNING_STATE                2  


void SystemStateUpdate(void);
void HighFlightmode_control_select(void);

void Control_Scheduler(void);
void RCMODE_Update(void);
static void LandDetector(void);
void set_System_State(uint8_t state);
uint8_t get_System_State(void);
void MagCail_Trig(void);

#endif 

