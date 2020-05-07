/*******************************************************************************
* 文件名称：md_rc.h
*
* 摘    要：遥控器相关C文件
*
* 当前版本：
* 作    者：ACROSS
* 日    期：2018/05/15
* 编译环境：keil5
*
* 历史信息：
*******************************************************************************/
#include "md_rc.h"
#include "md_rc_ppm.h"
#include "md_rc_sbus.h"
#include "md_config.h"



uint16_t rc_pwm_in[RC_INPUT_CHANNELS];
__IO ITStatus rc_newdata_flag   = RESET;
void RC_Input_Init(void)
{
	/*根据遥控器的不同进行初始化*/
#ifdef RC_INPUT_PPM
	PPM_Init();
#else
	SBUS_Init();
#endif
}

void RC_Input_Loop(void)
{
	/*根据遥控器的不同获取数据*/
#ifdef RC_INPUT_PPM
	PPM_GetData(rc_pwm_in,RC_INPUT_CHANNELS);
#else
	SBUS_GetData(rc_pwm_in,RC_INPUT_CHANNELS);
#endif
}
