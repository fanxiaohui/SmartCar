#ifndef __SD5_H
#define __SD5_H

#include "include.h"
#include "common.h"
#include "Image_Process.h"  //图像处理头文件，需要其内部路况标识

/************************************************************************/
/*                           舵机状态                                   */
#define	SD5_Normal  0             //正常
#define	SD5_Left    1             //最左
#define	SD5_Right   2             //最右
/************************************************************************/

void SD5_Init(void);
void SD5_Turn(uint16 Duty);
float SD5_PID(int16 actual_err);
void  Get_SD5_PID(void);
void  SD5_Contral(int16 average);

#endif
