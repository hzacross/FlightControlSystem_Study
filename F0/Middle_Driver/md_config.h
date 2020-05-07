
/*******************************************************************************
* 文件名称：md_config.h
*
* 摘    要：中间驱动层的一些配置
*
* 当前版本：
* 作    者：ACROSS
* 日    期：2018/05/15
* 编译环境：keil5
*
* 历史信息：
*******************************************************************************/
#ifndef __MD_CONFIG_H
#define __MD_CONFIG_H

/*
遥控器配置信息,两者选其一
*/
//#define RC_INPUT_PPM  /*使用PPM信号作为遥控器输入*/
#define RC_INPUT_SBUS /*使用SBUS信号作为遥控器输入*/

#if defined (RC_INPUT_PPM) && defined (RC_INPUT_SBUS)
#error "These two macros (RC_INPUT_PPM,RC_INPUT_SBUS) cannot be defined at the same time!"
#endif

#if !defined (RC_INPUT_PPM) && !defined (RC_INPUT_SBUS)
#error "Please define a macro to decide which type of RC that be used!"
#endif

#endif






