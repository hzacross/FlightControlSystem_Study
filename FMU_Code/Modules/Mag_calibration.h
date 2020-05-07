#ifndef __MAG_CALIBRATION_H
#define __MAG_CALIBRATION_H

#include "stm32f4xx_hal.h"
#include "math.h"

void Mag_Calibration_update(void);
void Mag_Calibration_Instrction(void);
uint8_t get_Mag_Calibration_Instrction_Step(void);
void Mag_Calibration_Operation(void);
void Mag_calibration_init(void);
void Mag_calibration_init_XY(void);
void Mag_calibration_init_Z(void);
void Traverse_magvalue_XY(void);
void Cali_parameter_caculation_XY(void);
uint8_t isCali_XYFactor_Check_Succeed(void);
void Traverse_magvalue_Z(void);
void Cali_parameter_caculation_Z(void);
uint8_t isCali_XZFactor_Check_Succeed(void);
void Save_Cali_parameter(void);
void Reset_Mag_calibration(void);
void Get_calibration_offset(void);

extern float magxoffset,magyoffset,magzoffset;
	
#endif 

