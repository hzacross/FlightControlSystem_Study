#ifndef __FILTER_H
#define __FILTER_H

#include "stm32f4xx_hal.h"
#include "math.h"

typedef struct
{
	 float           fc;
	 float           a1;
	 float           a2;
	 float           b0; 
	 float           b1;
	 float           b2;
	 float           y_1;        
	 float           y_2;       
}IIR_coeff_Typedef;

typedef struct
{
	float state;
	float RC;
	float dT;
} P_LowPassFilter1p;

float LowPassFilter1p(P_LowPassFilter1p *filter, float input, unsigned short f_cut, float dT);
void  cal_iir_coeff(IIR_coeff_Typedef *coeff,float fs, float fc);
float get_iir_output(IIR_coeff_Typedef* coeff,float sample);

#endif
