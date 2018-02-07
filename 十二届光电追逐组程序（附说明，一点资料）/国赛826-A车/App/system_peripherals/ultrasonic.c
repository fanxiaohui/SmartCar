#include "include.h"
#include "common.h"

/***********************************************************************************************************************************/
uint8 chaoshengbo_flag=0;
int8 open=0;
int32 car_distance;      //距离
int32 car_distance0[10];
int32 car_distance_temp; //距离
uint32 timevar;           //定时器计数值，转换成时间
int16 chaoshengbo_delay_cnt1=0;
/***********************************************************************************************************************************/

void ultrasonic_init();      //超声波初始化函数
void chaoshengbo(void);

/***********************************************************************************************************************************/

void ultrasonic_init()       //超声波初始化函数
{
  port_init(PTD7,ALT1|IRQ_EITHER| PF |PULLUP);
//  port_init(PTD6,ALT1|PULLUP);
//  gpio_init(PTD6,GPO,1);
  port_init(PORT_CSB,ALT1|PULLUP);
  gpio_init(PORT_CSB,GPO,1);
}

/***********************************************************************************************************************************/

void chaoshengbo(void)
{     
  //      chaoshengbo_flag=0;
  if(PTD7_IN==1&&open==0)      //高电平读取
  {
    pit_time_start(PIT2);
    open=1;
  }
  else if(PTD7_IN==0&&open==1)
  {
    uint32 time=pit_time_get(PIT2);
    if(time != ~0)       //没超时
    {
      timevar =1000*time/bus_clk_khz;                 //时间为微妙
      //car_distance=1500;
      car_distance_temp = timevar * 340  / 1000;      //距离为毫米
      if(car_distance_temp>=50 && car_distance_temp<=3000)
      {                 
        car_distance = car_distance_temp;
        chaoshengbo_delay_cnt1=0;
        chaoshengbo_flag=1;
      }
    }
    pit_close(PIT2);
    open=0;
  }
}

/***********************************************************************************************************************************/



