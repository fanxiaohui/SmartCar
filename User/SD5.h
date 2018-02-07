#ifndef __SD5_H
#define __SD5_H

#include "include.h"
#include "common.h"


#define	SD5_Normal  0           //正常
#define	SD5_Left    1             //最左
#define	SD5_Right   2            //最右
                   //舵机占空比状态

void SD5_Init(void);
void SD5_Turn(uint16 Duty);
float SD5_PID(int16 actual_err);
void  SD5_Contral(int16 average);

#endif
