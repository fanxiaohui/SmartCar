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

/***********************************************************************************************************************************/

uint32 run_time=0;
uint32 time0=0;
uint32 stoptime=0;
uint32 send_error=0;
uint8 zhengpao=0;
/***********************************************************************************************************************************/

void PORTB_IRQHandler(void);    //场中断接收中断服务函数
void DMA0_IRQHandler(void);     //DMA采集完成中断服务函数

void PIT0_IRQHandler();          //舵机输出中断服务函数
void PIT1_IRQHandler();          //电机输出中断服务函数
void PIT3_IRQHandler();

void PORTD_IRQHandler(void);    //超声波中断服务函数

void PORTC_IRQHandler();         //无线通信

/***********************************************************************************************************************************/

void  main(void)
{
      /*配置单片机外设系统初始化*/
      DisableInterrupts;         //禁止全部中断
      user_init();
      /*配置中断服务函数*/
      if( DIP7)
      {
        user_menu();   //按键操作系统
      }
      car_camera_init();             //摄像头初始化
      DisableInterrupts;         //禁止全部中断
      set_vector_handler(PORTD_VECTORn ,PORTD_IRQHandler);    //设置PORTE的中断复位函数为 PORTE_IRQHandler（中断复位函数为 超声波接收函数）
      set_vector_handler(PORTB_VECTORn,PORTB_IRQHandler);      //设置PORTB的中断复位函数为 PORTB_IRQHandler（中断复位函数为 场中断信号接收函数）
      set_vector_handler(PORTC_VECTORn,PORTC_IRQHandler); 
      set_vector_handler(DMA0_VECTORn ,DMA0_IRQHandler);       //设置DMA0的中断复位函数为 DMA0_IRQHandler（中断复位函数为 DMA信号接收函数）
      set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);       //设置PIT0的中断复位函数为 PIT0_IRQHandler（中断复位函数为 舵机控制函数）
      set_vector_handler(PIT1_VECTORn ,PIT1_IRQHandler);       //设置PIT0的中断复位函数为 PIT0_IRQHandler（中断复位函数为 电机控制函数）
      set_vector_handler(PIT3_VECTORn ,PIT3_IRQHandler);       //设置PIT0的中断复位函数为 PIT0_IRQHandler（中断复位函数为 电机控制函数）
      
      /*配置拨码开关*/
      
//      if(!DIP6)
//      {
//            erase_flash();
//      }
      if(DIP4)
      {
            double_car_flag=0;
      }
      else
      {
            double_car_flag=1;
      }
      if(DIP5)
      {
            zhengpao=1;
      }
      else
      {
            zhengpao=0;
      }
      
      loop_LOR_flag1=round(other1_a);//环道方向,1为向右
      loop_LOR_flag2=round(other1_b);
      loop_LOR_flag3=round(other1_c);
      
      dir_overtake_po_1=0;//坡道超车方向（1为前车，靠右停）
      dir_overtake_po_2=0;
      po_cnt_S=1;//1，第一个坡道超，   2，第二个坡道超
      
      dir_overtake_start=round(other1_d);
      
      STOP_OUT_able=round(other5_a);
      STOP_PATTERN_able=round(other5_b);
      
      
      /*执行循环外操作*/
      
      bar_range();    //获取障碍物搜索范围数组
      bar_range_loop() ;
      get_weight();   //获取中心线权值数组
      tan_angle();    //获取tan角度对照表
      
      barrier_able=1; //使能搜索障碍物
      loop_able=1;
      
      overtake_zhi_sum=round(cnt_a);
      overtake_ring_sum=round(cnt_b);
      overtake_po_sum=round(cnt_c);
      overtake_start_sum=round(cnt_d);
      overtake_cross_sum=round(cnt_e);
      
      run_time=0;     //程序执行时间数值清0
      qibu_flag=1;
      overtake_cross_stop_over=1;
      overtake_po_stop_over=1;
      stop_wait_over=1;
      if(loop_l_r_flag)
      {
            END_LINE_TEMP_delay_time2=30;
      }
      else
      {
            END_LINE_TEMP_delay_time2=60;
      }
      if(!DIP2)
      {
            speed_yun_flag=1;
      }
      /*小车运行主循环*/
      
      while( 1 )
      {
            switch( car_status )
            {
            case CAR_WAIT:
                  {
                        car_wait();     //小车等待状态（延时，使能中断，进入行驶模式）
                  }
                  break;
            case CAR_RUN:
                  if( DIP0==0 )
                  {
                        if( mt9v032_finish_flag )         //当摄像头采集完一场图像
                        {
                              //          lptmr_time_start_us();      //开始计时(ns)
                              uint16 copy;                //消除黑边，使用内存移动函数，左边右边各消除一列
                              for( copy=0 ; copy<120 ; copy++ )     //大约200US
                              {
                                    memmove(&img[copy][0],&image_h[copy][0],188);  //快速拷贝摄像头采集的图像
                              }
                              mt9v032_finish_flag=0;      //摄像头采集完成标志位置  0
                              barrel_rectification();     //桶形失真矫正并获取处理用图像
                              img_binary();               //图像二值化处理
                              tingche();
                              ring_dir();
                              edge_extract();             //边沿提取
                              get_inv_edge();             //获取逆透视后边沿坐标
                              midline_extract();          //中线提取
                              car_control();
                              podao_jc();
                              servo_control();            //舵机控制
                              motor_control();            //电机控制
                              distance_control();
                              //          run_time=lptmr_time_get_us();        //获取计时时间
                        }
                  }
                  else
                  {
                        if( DIP0==1 )
                        {
                              if( mt9v032_finish_flag )         //当摄像头采集完一场图像
                              {
                                    //  lptmr_time_start_us();      //开始计时(ns) 
                                    
                                    uint16 copy;                //消除黑边，使用内存移动函数，左边右边各消除一列
                                    for( copy=0 ; copy<120 ; copy++ )     //大约200US
                                    {
                                          memmove(&img[copy][0],&image_h[copy][0],188);  //快速拷贝摄像头采集的图像
                                    }
                                    mt9v032_finish_flag=0;      //摄像头采集完成标志位置  0
                                    if(DIP1)
                                    {
                                          barrel_rectification();     //桶形失真矫正并获取处理用图像
                                          //normal_extract();
                                          img_binary();               //图像二值化处理
                                          ring_dir();
                                          edge_extract();             //边沿提取
                                          get_inv_edge();             //获取逆透视后边沿坐标
                                          midline_extract();          //中线提取
                                          car_control();
                                          podao_jc();
                                          servo_control();            //舵机控制
                                          motor_control();            //电机控制
                                          distance_control();
                                          
                                          LCD_CAMERA_erzhiSHOW();     //显示图像  
                                          if(!DIP3)
                                          {
                                                seekfree_sendimg();
                                          }
                                          
                                        
                                          
                                          site.x=25;
                                          site.y=10;
                                          LCD_num_BC(site, (uint32)car_flag,1, BLUE, RED);
                                          site.y=25;
                                          LCD_num_BC(site, (uint32)overtake_ring_flag,1, BLUE, RED);
                                          site.y=40;
                                          LCD_num_BC(site, (uint32)overtake_po_flag,1, BLUE, RED);
                                          site.y=55;
                                          LCD_num_BC(site, (uint32)overtake_cross_flag,1, BLUE, RED);
                                          
                                          site.x=45;
                                         
                                          site.y=55;
                                          LCD_num_BC(site, (uint32)LOOP_TEMP,1, BLUE, RED);
                                          site.y=70;
                                          LCD_num_BC(site, (uint32)LOOP_IN,1, BLUE, RED);
                                          
                                          site.x=65;
                                         
                                          site.y=55;
                                          LCD_num_BC(site, (uint32)img_threshold,3, BLUE, RED);
                                          site.y=70;
                                          LCD_num_BC(site, (uint32)fabsf(CAR_position),2, BLUE, RED);
                                          
                                          site.x=80;
                                          site.y=10;
                                          LCD_num_BC(site, (uint32)(fabsf(out_row[0])),3, BLUE, RED);
                                          site.y=25;
                                          LCD_num_BC(site, (uint32)(loop_able),3, BLUE, RED);
                                          ;            
                                          
                                 
                                    }
                                    else
                                    {
                                          if(DIP2)
                                          {
                                                LCD_CAMERA_show();
                                                if(!DIP3)
                                                {
                                                      seekfree_sendimg();
                                                }
                                          }  
                                          else
                                          {
                                                LCD_CAMERA_BARREL_show();
                                                if(!DIP3)
                                                {
                                                      seekfree_sendimg();
                                                }
                                          }
                                    }
                                    //A_send_B1();
                              }
                        }
                  }
                  break;
            case CAR_STOPP:
                  {
                        //                        disable_irq (PIT1_IRQn);                 //关闭电机定时中断
                        //                        ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0);
                        //                        ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0);
                        //                        ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0);
                        //                        ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0);
                              if( mt9v032_finish_flag )         //当摄像头采集完一场图像
                              {
                                    uint16 copy;                //消除黑边，使用内存移动函数，左边右边各消除一列
                                    for(copy=0; copy<120; copy++)     //大约200US
                                    {
                                          memmove(&img[copy][0],&image_h[copy][0],188);  //快速拷贝摄像头采集的图像
                                    }
                                    mt9v032_finish_flag=0;      //摄像头采集完成标志位置  0
                                    barrel_rectification();     //桶形失真矫正并获取处理用图像
                                    img_binary();               //图像二值化处理
                                    //tingche2();
                                    ring_dir();
                                    
                                    edge_extract();             //边沿提取
                                    get_inv_edge();             //获取逆透视后边沿坐标
                                    midline_extract();          //中线提取
                                    car_control();
                                   // podao_jc();
                                    servo_control();            //舵机控制
                                    motor_control();            //电机控制
                                    distance_control();
                                    if(QAQ_STOP)
                                    {
                                          LCD_CAMERA_erzhiSHOW();
                                    }
                                    
                              }
                        }
                  break;
            default:
                  break;
            }
            
            if( END_LINE_TEMP==1 )
            {
                  if(( end_line_row>=50 || end_line_row==-1 ) &&(abs(speedout_val)>=4*10000))
                  {
                        if(!double_car_flag)
                        {
                              //END_LINE_TEMP_delay_flag2=1;
                              car_status = CAR_STOPP;
                              END_LINE_TEMP=0;
                              END_LINE_TEMP_1=0;
                              end_line_row=-1;     //终点线所在行置  -1
                        }
                        else
                        {
                              if(!speedout_val5_flag)
                              {
                                    speedout_val5_flag=1;
                                    speedout_val5=speedout_val;
                              }
                        }
                  }
            }
            if(speedout_val5_flag && (((abs(speedout_val-speedout_val5)>=1*10000) && !car_flag) || ((abs(speedout_val-speedout_val5)>=0*10000) && car_flag)))
            {
                  //END_LINE_TEMP_delay_flag2=1;
                  car_status = CAR_STOPP;
                  END_LINE_TEMP=0;
                  END_LINE_TEMP_1=0;
                  end_line_row=-1;     //终点线所在行置  -1
            }
      }
}

/***********************************************************************************************************************************/

void PORTB_IRQHandler(void)     //场中断接收中断服务函数
{
      PORTB_ISFR = (1<<16);      //使能DMA
      DMA_EN(DMA_CH0); 
      VSYNC();
      
}

/***********************************************************************************************************************************/

void DMA0_IRQHandler(void)      //DMA采集完成中断服务函数
{
      DMA_IRQ_CLEAN(DMA_CH0);
      row_finished();
}

/***********************************************************************************************************************************/

void PIT0_IRQHandler()           //舵机输出中断服务函数
{
      if(servo_enable)
      {
            servo_output();        //必须先初始化以后，才能输出PWM波
      }
      
      PIT_Flag_Clear(PIT0);  //清中断标志位
}

/***********************************************************************************************************************************/

void PIT1_IRQHandler()           //电机输出中断服务函数
{
      speedout_count=ftm_quad_get(FTM2);  //获取  FTM2 交解码 的脉冲数
      ftm_quad_clean(FTM2);                 //复位 FTM2 正交解码 的脉冲数
      speedout_val+=speedout_count;
      if(motor_enable)
      {
            motor_control_pit();        //电机增量式PID控制
            motor_output();             //必须先初始化以后，才能输出PWM波
      }
      
      PIT_Flag_Clear(PIT1);       //清中断标志位
}

/***********************************************************************************************************************************/

void PIT3_IRQHandler()
{
      
      //----------定时停车-----------//
      if(!DIP3)
      {
            if(stoptime>=(50*10))
                  car_stop_flag=1;
            
            else
                  stoptime++;
      }
      else
      {
            if(stoptime!=0)
            {
                  car_stop_flag=0;
                  stoptime=0;
            }
      }
      
      
      
      //--------定时三秒清除避障--------//
      if(barrier_able_flag)
      {
            if(barrier_time>=BARRIER_TIME1)
            {
                  barrier_time=0;
                  barrier_able_flag=0;
                  barrier_able=0;
                  barrier_ing_flag=0;
            }
            else
                  barrier_time++;
      }
      else
            barrier_time=0;
      
      
      //----------超声波标志-----------//
      if(chaoshengbo_delay_cnt1== CHAOSHENGBODELAY_TIME1)
      {
            chaoshengbo_flag=0;
      }
      else
            chaoshengbo_delay_cnt1++;
      
      
      //----------发车通信时间限制-------//
      if(send_start_flag)
      {
            if(send_error>=25)
            {
                  send_error_flag=1;
                  send_error=0;
            }
            else
            {
                  send_error++;
                  send_error_flag=0;
            }
      }
      else
      {
            send_error=0;
            send_error_flag=0;
      }
      
      //-----------蜂鸣器-----------//
      if(Bee_flag)
      {
            if(Bee_time>=2)
            {
                  Bee_time=0;
                  Bee_flag=0;
            }
            else
            {
                  Bee_time++;
            }
      }
      
      
      PIT_Flag_Clear(PIT3);       //清中断标志位
}

/***********************************************************************************************************************************/

void PORTD_IRQHandler(void)     //超声波采集中断服务函数
{
      PORT_FUNC(D,7,chaoshengbo);
}

/***********************************************************************************************************************************/

void PORTC_IRQHandler()//无线通信
{
      uint8  n;    //引脚号
      uint32 flag;
      flag = PORTC_ISFR;
      PORTC_ISFR  = ~0;                                   //清中断标志位
      n = 10;
      if(flag & (1 << n))                                 //PTE27触发中断
      {
            nrf_handler();
      }
}

/***********************************************************************************************************************************/



