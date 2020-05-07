项目说明：
基于PX4开源飞控V2版本硬件，以stm32 cubemx为驱动开发方式，重构飞行控制系统软件。

FMU_Code 该目录下代码为V2硬件的F4芯片代码；
F0       该目录为协处理器代码，针对国内版本的硬件，主芯片型号为F100系列；
F1       该目录为协处理器代码，主芯片型号为F103系列；

软件功能：
基本的状态估计算法以及控制算法，能够完成定点悬停功能；

教程视频在线观看地址：
飞控算法基础之PID控制 
https://www.bilibili.com/video/BV1ft411a7Kw
飞控算法基础之卡尔曼滤波
https://www.bilibili.com/video/BV11b411p7E8


百度网盘资料下载地址：
链接：https://pan.baidu.com/s/1g-yXVzcrvSq37rC_R4QXmQ 
提取码：ioye

淘宝购买链接：
https://item.taobao.com/item.htm?spm=a230r.1.14.264.44247806auR97T&id=595296421400&ns=1&abbucket=18#detail

2020.05.07
init version 
测试了AHRS数据正常，未验证状态估计数据；
