#ifndef __IMAGE_PROCESS_H
#define __IMAGE_PROCESS_H

#include "common.h"
#include "include.h"
/************************************************************************/
/*                             路况标识                                 */
extern     uint8      cross_flag;              //十字标志
extern     uint8      S_flag;                  //连续S弯标志
extern     uint8      L_Sharp_turn_flag;       //左传急弯标志
extern     uint8      R_Sharp_turn_flag;       //右传急弯标志
extern     uint8      Big_Turn_flag;           //大弯缓弯标志
extern     uint8      L_Loop_flag;             //左侧环岛标志
extern     uint8      R_Loop_flag;             //右侧环岛标志
extern     uint8      Barrier_flag;            //障碍标志
extern     uint8      Stop_flag;               //停止标志
/************************************************************************/

/************************************************************************/
/*                      外部调用变量及数组                              */
extern uint8 img[CAMERA_H][CAMERA_W];           //定义存储转化后的图像的二维数组
extern uint8 imgbuff[CAMERA_SIZE];              //定义存储接收原始图像的一维数组
/************************************************************************/

/************************************************************************/
/*                      内部输出变量及数组                              */
extern uint8  zhongxian[CAMERA_H][CAMERA_W];        //定义储存中线的数组
extern uint8  bianyan[CAMERA_H];                    //定义储存边沿信息数据
/************************************************************************/

void Get_MiddleLine(uint8 midline1_H, uint8 midline2_H, uint8 midline3_H);  //获取中线,提取前瞻行 
void Get_Road_flag(void);    //获取路面特殊路况标志

void Flag_Init(void);
void Buzzer_Init(void);
void Buzzer_NO(void);      //响一声


#endif
