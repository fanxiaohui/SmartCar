#ifndef __SPEED_H
#define __SPEED_H

#include "include.h"
#include "common.h"
#include "PID.h"

#define PI   3.14
#define Small_gear  1
#define Big_gear    2
#define Wheel       3     
#define Speed_Test(x)    (((x/512)*2*PI*Wheel)/Big_gear)

//定义速度采集状态
typedef enum
{
	Speed_NOTINIT = 0,
	Speed_FINISH,             //速度采集完毕
	Speed_FAIL,               //速度采集失败(采集行数少了)
	Speed_GATHER,             //速度采集中
	Speed_START,              //开始速度采集
	Speed_STOP,               //禁止速度采集
}Speed_STATUS_e;


void Speed_Init(void);       //编码器测速初始化
void Speed_Get(void);        //速度采集，1s采集一次
void PIT0_IRQHandler(void);  //PIT0中断函数

void RS540_Init(void);
void RS540_Turn(uint16 Duty);

float RS540_PID(float speed, float actualSpeed);

#endif
