/*
  ******************************************************************************
  * @file    filter.c
  * @author  across 
  * @version v1.0
  * @date    2020-02-27
  * @brief   Ò»½×µÍÍ¨ÂË²¨Æ÷£¬¶ş½×µÍÍ¨ÂË²¨Æ÷
 ******************************************************************************
*/

#include "filter.h"
#include "md_struct.h"


// LowPassFilter1p
float LowPassFilter1p(P_LowPassFilter1p *filter, float input, unsigned short f_cut, float dT)
{
    // Pre calculate and store RC
    if (!filter->RC) {
        filter->RC = 1.0f / ( 2.0f * M_PI_F * f_cut );
    }

    filter->dT = dT;    
    filter->state = filter->state + dT / (filter->RC + dT) * (input - filter->state);
    return filter->state;
}


// LowPassFilter2p
void cal_iir_coeff(IIR_coeff_Typedef *coeff,float fs, float fc)
{		
    float fr =0;
    float ohm =0;
    float c =0;
		
    fr= fs/fc;
    ohm=tanf(M_PI_F/fr);
    c=1.0f+2.0f*cosf(M_PI_F/4.0f)*ohm + ohm*ohm;
		
    coeff->fc = fc;

		coeff->b0 = ohm*ohm/c;
		coeff->b1 = 2.0f*coeff->b0;
		coeff->b2 = coeff->b0;
		coeff->a1 = 2.0f*(ohm*ohm-1.0f)/c;
		coeff->a2 = (1.0f-2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;

}

float get_iir_output(IIR_coeff_Typedef* coeff,float sample)
{
    if (coeff->fc <= 0.0f) {
        return sample;
    }
    else
    {
        float y_0 = sample - coeff->y_1 * coeff->a1 - coeff->y_2 * coeff->a2;
				
        float output = y_0 * coeff->b0 + coeff->y_1 * coeff->b1 + coeff->y_2 * coeff->b2;
				
        coeff->y_2 = coeff->y_1;
        coeff->y_1 = y_0;

        return output;
    }	
}

