#ifndef __PORT_H
#define __PORT_H

/*包含在common,h 第417行*/

#include "MK60_port.h"

#define abs(x)         (x>0)?(x):(-x)

/*************************板子外设所用管脚定义****************************/
//printf的串口
#define        UART_printf_port_Tx                 UART2
#define        UART_printf_baud                    9600

#define        UART_BlueTooth                      UART2
#define        UART_BlueTooth_Baud                 9600

//液晶接口
#define        LCD_WR                              PTC9
#define        LCD_RD                              PTC10
#define        LCD_CS                              PTC11
#define        LCD_RS                              PTC12
#define        LCD_RST                             PTC13 
#define        LCD_Date_Out                        PTC_B0_OUT
#define        LCD_Date_IN                         PTC_B0_IN
#define        LCD_D1                              PTC0          //液晶D1口对应PC0
//PC0~7是数据线

/*摄像头接口*/
#define        OV7725_Date_IN                      PTB_B0_IN      //PB0-7
#define        OV7725_SCCB_SDA                     PTA25
#define        OV7725_SCCB_SCL                     PTA26
#define        OV7725_PCLK                         PTA27
#define        OV7725_Href                         PTA28
#define        OV7725_VSYNC                        PTA29
  //摄像头属性设置
#define        OV7725_DMA_CH                       DMA_CH0
#define        OV7725_W                            160
#define        OV7725_H                            60
#define        OV7725_ImgSize                      (OV7725_EAGLE_W * OV7725_EAGLE_H)/8   //解压缩后的图像大小

//舵机
#define       SD5_FTM                              FTM1
#define       SD5_FTM_CH                           FTM_CH0       //PA12
#define       SD5_FTM_Freq                         (300)
#define       SD5_Duty_Middle                      (463)         //舵机中值
#define       SD5_Right_max                        (596)         //舵机左转极值
#define       SD5_Left_max                         (329)         //舵机右转极值

//编码器
#define       ECM512_FTM                           FTM2              
#define       ECM512_FTM_A                         FTM2_QDPHA_PIN    //PA10
#define       EMC512_FTM_B                         FTM2_QDPHB_PIN    //PA11
#define       EMC512_PIT                           PIT0              //编码器定时中断

//电机
#define       RS540_FTM                            FTM0
#define       RS540_FTM_1                          FTM_CH4           //PD4
#define       RS540_FTM_2                          FTM_CH5           //PD5
#define       RS540_ferq                           (20000)           //20k


/************************************************************************/

#endif
