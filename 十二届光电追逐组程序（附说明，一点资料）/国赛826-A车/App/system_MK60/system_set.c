#include "include.h"
#include "common.h"

/***********************************************************************************************************************************/

uint8 car_flag=0,car_start_flag=1;                      //前后车标志位
uint8  double_car_flag=0;            //双车
uint8 send_error_flag=0,send_start_flag=0;
CAR_STATUS car_status=CAR_WAIT;
//uint8 car_run=0;
//uint8 run_time=0;
//float OutData[4];
uint8 servo_enable,motor_enable;
/***********************************************************************************************************************************/

void car_init();   //中断优先级配置函数
void car_wait();

/***********************************************************************************************************************************/

void car_init()    //中断优先级配置函数
{
      NVIC_SetPriority(PORTB_IRQn,0);   //场中断接收中断服务函数 
      NVIC_SetPriority(DMA0_IRQn,1);    //DMA采集完成中断服务函数
      NVIC_SetPriority(PORTC_IRQn,2);   //超声波中断服务函数
      NVIC_SetPriority(PORTD_IRQn,3); //无线通信
      NVIC_SetPriority(PIT0_IRQn,4);    //舵机输出中断服务函数
      NVIC_SetPriority(PIT1_IRQn,5);    //电机输出中断服务函数
      NVIC_SetPriority(PIT3_IRQn,6);    //电机输出中断服务函数
      ftm_quad_init(FTM2);       //正交初始化
      pit_init_ms(PIT0,20);      //初始化PIT0，定时时间为： 20ms，用于控制舵机输出
      pit_init_ms(PIT1,20);      //初始化PIT1，定时时间为： 20ms，用于控制电机输出
      pit_init_ms(PIT3,20);   //定时
}

/***********************************************************************************************************************************/

void car_wait()
{
      enable_irq (PORTB_IRQn);   //使能PORTB中断（场中断信号接收中断）
      enable_irq (PORTD_IRQn);   //使能PORTB中断（场中断信号接收中断）
      enable_irq (PORTC_IRQn);
      enable_irq (PIT3_IRQn);    //使能PIT3中断（标志位定时中断）
      enable_irq (PIT0_IRQn);    //使能PIT0中断（舵机控制输出定时中断）
      enable_irq (PIT1_IRQn);    //使能PIT1中断（电机控制输出定时中断）
      EnableInterrupts;          //使能全部中断
      
      DELAY_MS(1000);
      car_status=CAR_RUN;        //小车状态改为行驶
      car_start_flag=1;
      if(!double_car_flag)
      {
            servo_enable=1;
            motor_enable=1;
            car_start_flag=0;
      }
      else
      {
            car_judge();
      }
}

/***********************************************************************************************************************************/
void car_judge()
{
      //----------------------确认前后车-----------------------//
      while(car_start_flag)
      {//用超声波不好使，所以用起跑线和障碍物
            if(1 || mt9v032_finish_flag )         //当摄像头采集完一场图像
            {
//                  uint16 copy;                //消除黑边，使用内存移动函数，左边右边各消除一列
//                  for( copy=0 ; copy<120 ; copy++ )     //大约200US
//                  {
//                        memmove(&img[copy][0],&image_h[copy][0],188);  //快速拷贝摄像头采集的图像
//                  }
//                  mt9v032_finish_flag=0;      //摄像头采集完成标志位置  0
//                  barrel_rectification();     //桶形失真矫正并获取处理用图像
//                  img_binary();               //图像二值化处理
//                  edge_extract();             //边沿提取
//                  if(END_LINE_TEMP==1)
//                  {
//                        if(QianChe_extract()==1)
//                        {
//                              car_flag=1;
//                        }
//                        else
//                        {
//                              car_flag=0;
//                        }
//                  }
//                  else
//                  {
//                        if((L_barrier_flag && (L_barrier_down_row>15)) || (R_barrier_flag && (R_barrier_down_row>15)))
//                        {
//                              car_flag=1;
//                        }
//                        else
//                        {
//                              car_flag=0;
//                        }
//                  }//以后出错直接改双车识别就行了，以上
                  if(chaoshengbo_flag && car_distance<=300)
                  {
                        car_flag=1;
                  }
                  else if(!chaoshengbo_flag)
                  {
                        car_flag=0;
                  }
                  //----------------------通讯确认前后车--------------------//
                  qipao_tongxin();
                  
            }
            
            if(round(other1_f)!=0)//用标志位手动控制
            {
                  if(round(other1_f)==1)
                  {
                        car_flag=0;
                  }
                  else
                  {
                        car_flag=1;
                  }
                  car_start_flag=0;
            }
      }
      
      if(car_flag)
      {
            gpio_set (PORT_CSB, 0);//关闭发送超声波
            barrier_able=0; //使能搜索障碍物
      }
      else
      {
            barrier_able=1; //使能搜索障碍物
            Bee_flag=1;
      }
      overtake_start_flag=1;//初始化为起跑模式（发车模式完成直接进入起跑模式）
      send_start_flag=0;
      servo_enable=1;
      motor_enable=1;
}

uint8 QianChe_extract()
{
      int16 x,y,z;
      double aa;
      Site_xy1 xy1;
      uint8 QianChe_see_flag=0;
      if( end_line_row!=-1 )
      {
            aa=inv_distance[end_line_row]-40;
            xy1=get_invinv_img(aa , 0 );
      }
      else
      {
            xy1.x=ROW_END;
      }
      for(x=ROW_END;x>xy1.x ;x--)
      {
            for(y=left_edge[x];y<=right_edge[x]-5;y++)
            {
                  if(y<=(COL_END-3) && img_data[x][y]==DOT_W && img_data[x][y+1]==DOT_W && img_data[x][y+2]==DOT_B && img_data[x][y+3]==DOT_B)
                  {
                        QianChe_see_flag=1;
                        break;
                  }
            }
            if(QianChe_see_flag==1)
                  break;
      }
      if(QianChe_see_flag==1)
      {
            return 1;
      }
      else
      {
            return 0;
      }
}

void qipao_tongxin()
{
      if(!car_flag)
      { 
            while(!nrf_send_finish_flag)
            {
                  send_start_flag=1;
                  A_send_B0();
                  if(send_error_flag && !nrf_send_finish_flag)
                        break;
            }
            send_start_flag=0;
            send_error=0;
            send_error_flag=0;
            while(!nrf_read_finish_flag && nrf_send_finish_flag)
            {
                  send_start_flag=1;
                  A_read_B0();
                  if(send_error_flag && !nrf_read_finish_flag)
                        break;
            }
            send_start_flag=0;
            send_error=0;
            send_error_flag=0;
            nrf_send_finish_flag=0;
            nrf_read_finish_flag=0;
      }
      else
      {
            while(!nrf_read_finish_flag)
            {
                  send_start_flag=1;
                  B_read_A0();
                  if(send_error_flag)
                        break;
            }
            send_start_flag=0;
            send_error=0;
            send_error_flag=0;
            nrf_send_finish_flag=0;
            nrf_read_finish_flag=0;
      }
}