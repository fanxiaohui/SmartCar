/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,山外科技
 *     All rights reserved.
 *     技术讨论：山外论坛 http://www.vcan123.com
 *
 *     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留山外科技的版权声明。
 *
 * @file       main.c
 * @brief      山外K60 平台主程序
 * @author     山外科技
 * @version    v5.0
 * @date       2013-08-28
 */

#include "common.h"
#include "include.h"
#include "OV7725.h"
#include "SD5.h"
#include "Speed.h"
#include "data_handle.h"
#include "Image_Process.h"

/*提取中线的三行*/
extern uint8 midline1_L,midline2_L,midline3_L;

extern uint8 img[CAMERA_H][CAMERA_W];                      //定义存储转化后的图像的二维数组
extern uint8 imgbuff[CAMERA_SIZE];                         //定义存储接收原始图像的一维数组

extern uint8  SD5_flag;     //舵机状态
extern int16  Speed_val;    //编码器采集脉冲数

 
/*!
 *  @brief      main函数
 *  @since      v5.3
 *  @note       山外摄像头 LCD 测试实验
 */

 void  main(void)
{
       OV7725_Init();
       LCD_init();
       SD5_Init();
       SD5_PID_Init();
       RS540_Init();
       Buzzer_Init();
       Speed_Init();
 //      Flag_Init();

       Site_t site = { 0, 0 };                              //显示图像左上角位置
       Size_t imgsize = { CAMERA_W, CAMERA_H };              //图像大小
       Size_t size = { LCD_W, LCD_H };                      //显示区域图像大小

       float average;
       int SD5_Duty;
       uint16 RS540_Duty = 185;

    while(1)
    {
       SD5_flag=SD5_Normal;
       RS540_Turn(RS540_Duty);
	   Get_img();                      //获取图像
	   Speed_Get();        //速度采集，50ms采集一次  改：磨轮胎，30秒反转一次
       
	   Get_MiddleLine(28, 31, 33);
	 //  Get_Road_flag();
	   SD5_Turn(SD5_Duty);
	
		/************************************************************************/
		/* 以上为提取中线代码     以下为偏差计算                                */
		/************************************************************************/
		average = (midline1_L*0.4 + midline2_L*0.3 + midline3_L*0.3) - CAMERA_W / 2;
		if (abs(average) > 10)
			average = average*3.5;
		else if (abs(average) < 1)
			average = average*0;
                else
			average = average*2.3;
		  SD5_Contral(average);
                  
	  LCD_Img_gray_Z(site, size,(uint8*) img, imgsize);
	/****************************************************************************/
	
    }
}

 /*
 for(i=0;i<CAMERA_H;i++)
			 for(j=0;j<CAMERA_W;j++)
			   zhongxian[i][j]=255;


	 for(i=CAMERA_H-1;i>CAMERA_H-4;i--)              //提取前三行中线
	  {
		  for(j=CAMERA_W/2;j<CAMERA_W;j++)
		  {
			if(img[i][j]==0xff&&img[i][j+1]==0x00)
			  {
				   left_back=j;
				   left_flag=1;
				  break;
			  }
			  else
			  {
				left_flag=0;
				left_back=CAMERA_W-10;
			  }
		  }
		  for(j=CAMERA_W/2;j>0;j--)
		  {
			  if(img[i][j]==0xff&&img[i][j-1]==0x00)
				  {
					right_back=j;
					right_flag=1;
					break;
				  }
			  else
			  {
				right_flag=0;
				right_back=10;
			  }
		  }
//          if(right_flag==1&&left_flag==1)
//          {
		  n=(right_back+left_back)/2;
		   img[i][n]=0;
		   zhongxian[i][n]=0;
		  //}
	   }

		for(i=CAMERA_H-4;i>0;i--)              //提取中线
	   {
		  for(j=left_back-10;j<left_back+10;j++)
		  {
			if(img[i][j]==0xff&&img[i][j+1]==0x00)
			  {
				   left_back=j;
				   left_flag=1;
				  break;
			  }
			  else left_flag=0;
		  }
		  for(j=right_back+10;j>right_back-10;j--)
		  {
			  if(img[i][j]==0xff&&img[i][j-1]==0x00)
				  {
					right_back=j;
					right_flag=1;
					break;
				  }
			  else right_flag=0;
		  }

		  n=(right_back+left_back)/2;

		  if(i==midline1_H)
		  {
			midline1_L=n;
//            if(left_flag==0&&right_flag==1)
//              SD5_flag1=0;
//            if(right_flag==0&&left_flag==1)
//              SD5_flag1=1;
		  }
		  if(i==midline2_H)
		  {
			midline2_L=n;
//            if(left_flag==0&&right_flag==1)
//              SD5_flag2=0;
//            if(right_flag==0&&left_flag==1)
//              SD5_flag2=1;
		  }
		  if(i==midline3_H)
			midline3_L=n;


//             if(SD5_flag1==0&&SD5_flag2==0)    //左边沿检测不到
//               SD5_flag= SD5_Left;          //就向左打死
//             if(SD5_flag1==1&&SD5_flag2==1)    //右边沿检测不到
//               SD5_flag= SD5_Right;          //就向右打死

		   img[i][n]=0;
		   zhongxian[i][n]=0;

	   }
 */




