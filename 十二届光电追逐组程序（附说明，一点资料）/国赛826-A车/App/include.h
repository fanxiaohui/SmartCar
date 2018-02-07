#ifndef __INCLUDE_H__
#define __INCLUDE_H__

#include  "common.h"            //常用类型声明和宏定义（内部已定义MK60_conf.h和PORT_cfg.h）

/*
 * Include 用户自定义的头文件
 */

/***********************************************************************************************************************************/

#include  "MK60_wdog.h"         //看门狗驱动函数
#include  "MK60_gpio.h"         //IO口操作
#include  "MK60_uart.h"         //串口
#include  "MK60_SysTick.h"      //滴答定时器函数
#include  "MK60_lptmr.h"        //低功耗定时器(延时)
#include  "MK60_i2c.h"          //I2C
#include  "MK60_spi.h"          //SPI
#include  "MK60_ftm.h"          //FTM
#include  "MK60_pit.h"          //PIT
#include  "MK60_rtc.h"          //RTC
#include  "MK60_adc.h"          //ADC
#include  "MK60_dac.h"          //DAC
#include  "MK60_dma.h"          //DMA
#include  "MK60_FLASH.h"        //FLASH
#include  "MK60_can.h"          //CAN
#include  "MK60_sdhc.h"         //SDHC
#include  "MK60_usb.h"          //usb

/***********************************************************************************************************************************/

#include  "VCAN_LED.H"          //LED
#include  "VCAN_KEY.H"          //KEY
#include  "VCAN_MMA7455.h"      //三轴加速度MMA7455
#include  "VCAN_NRF24L0.h"      //无线模块NRF24L01+
#include  "VCAN_RTC_count.h"    //RTC 时间转换
#include  "VCAN_camera.h"       //摄像头总头文件
#include  "VCAN_LCD.h"          //液晶总头文件
#include  "ff.h"                //FatFs
#include  "VCAN_TSL1401.h"      //线性CCD
#include  "VCAN_key_event.h"    //按键消息处理
#include  "VCAN_NRF24L0_MSG.h"  //无线模块消息处理
#include  "VCAN_computer.h"     //多功能调试助手
#include  "VCAN_BMP.h"          //BMP
#include  "VCAN_menu.h"         //山外液晶屏菜单

/***********************************************************************************************************************************/

#include  "vcan_img2sd.h"       //存储图像到sd卡一个文件
#include  "vcan_sd_app.h"       //SD卡应用（显示sd看上图片固件）

#include  "Vcan_touch.h"        //触摸驱动

/******************************************用户自定义文件（按app文件格式写入）******************************************************/

  /***控制算法程序***/
#include  "motor_control.h"             //电机控制算法
#include  "servo_control.h"             //舵机控制算法
#include  "distance_control.h"          //双车控距

  /***图像处理程序***/
#include  "edge_extract.h"              //边沿提取算法
#include  "middle_extract.h"            //中线提取算法
#include  "fit_curves.h"                //逆透视拟合中线函数库
#include  "image_binaryzation.h"        //图像二值化算法
#include  "image_rectification.h"       //图像矫正算法
#include  "podao.h"

  /***其他功能函数库***/
#include  "font.h"                      //文字库
#include  "car_skill.h"                 //车动作

  /***屏幕 FLASH***/
#include  "user.h"   
#include  "key.h"     
#include  "user_v_set_filter.h"
#include  "user_flash.h"

  /***系统MK60函数***/
#include  "ftm_function.h"              //FTM功能函数（包括  电机PWM波输出  舵机PWM波输出  编码器正交解码输入）
#include  "system_set.h"                //MK60中断优先级配置，及小车状态函数
//#include  "MK60_conf.h"     //山外K60平台的配置头文件（已在common.h文件内配置）
//#include  "MK60_it.h"       //山外K60 平台中断服务函数
//#include  "PORT_cfg.h"      //管脚复用配置（已在common.h文件内配置）

  /***系统外设函数***/
#include  "camera.h"                    //摄像头函数库
#include  "LCD.h"                       //液晶屏函数库
#include  "LCD_menu.h"                  //液晶屏菜单程序
#include  "SEEKFREE_MT9V032.h"          //总钻风配置程序
#include  "SEEKFREE_IIC.h"              //陀螺仪配置程序
#include  "SEEKFREE_MPU6050.h"          //陀螺仪配置程序
#include  "ultrasonic.h"                //总钻风配置程序
#include  "correspond.h"

  /***外部应用函数***/
#include  "DLib_Product_string.h"       //快速拷贝函数




/***********************************************************************************************************************************/




#endif








