#ifndef __PID_H
#define __PID_H

#include "include.h"
#include "common.h"

struct _pid 
{
	uint16 SetSpeed;           //定义设定值
	int16 err;                 //定义偏差值
	int16 err_next;            //定义上一个偏差值
	int16 err_last;            //定义最上前的偏差值
	float Kp, Ki, Kd;          //定义比例、积分、微分
	float increment;           //定义增量
};

#endif
