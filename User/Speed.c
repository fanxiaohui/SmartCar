#include "Speed.h"
#include "Image_Process.h"
#include "PID.h"

volatile Speed_STATUS_e      Speed_flag = Speed_FINISH;   //图像状态
struct _pid   Speed_pid;                                  //速度pid


int16    Speed_val = 10;                                       //编码器采集到的速度脉冲数
int16    RS540_Duty;
uint16   Tire = 1;

/*
*  @brief      速度采集初始化函数(编码器初始化)
*  @since      v5.0
*/
void Speed_Init(void)
{
	ftm_quad_init(ECM512_FTM);
	pit_init_ms(EMC512_PIT, 1000);                          //初始化PIT0，定时时间为：50ms
	set_vector_handler(PIT0_VECTORn, PIT0_IRQHandler);      //设置PIT0的中断复位函数为 PIT0_IRQHandler
//	enable_irq(PIT0_IRQn);                                  //使能PIT0中断

}

/*
*  @brief      速度采集函数
*  @since      v5.0
*/
void Speed_Get(void)
{
	Speed_flag = Speed_START;
	enable_irq(PIT0_IRQn);                                  //使能PIT0中断
/*	while (Speed_flag != Speed_FINISH)                      //等待速度采集完成
	{
		if (Speed_flag == Speed_FAIL)                      //若速度采集失败
		{
			PIT_Flag_Clear(EMC512_PIT);                    //清中断标志位
			enable_irq(PIT0_IRQn);                         //使能PIT0中断，开始再次采集
		}
	}
*/	
}

/*
*  @brief      PIT0中断服务函数
*  @since      v5.0
*/
void PIT0_IRQHandler(void)
{
	Speed_val = ftm_quad_get(ECM512_FTM);
	ftm_quad_clean(ECM512_FTM);
	PIT_Flag_Clear(EMC512_PIT);       //清中断标志位
    printf("\t\r\nSpeed_val = %d\n",Speed_val);
  
	/*磨轮胎，30秒反转一次*/
	/*if (Tire)
	{
		ftm_pwm_duty(RS540_FTM, RS540_FTM_1, 120);
		ftm_pwm_duty(RS540_FTM, RS540_FTM_2, 0);
		Tire = 0;
	}
	else
	{
		ftm_pwm_duty(RS540_FTM, RS540_FTM_1, 0);
		ftm_pwm_duty(RS540_FTM, RS540_FTM_2, 120);
		Tire = 1;
	}*/



	//Buzzer_NO();

/*	if (Speed_flag == Speed_START)
	{
		Speed_flag = Speed_GATHER;              //速度采集中
		Speed_val = ftm_quad_get(ECM512_FTM);
		Speed_flag = Speed_FINISH;
		ftm_quad_clean(ECM512_FTM);
		PIT_Flag_Clear(EMC512_PIT);       //清中断标志位
	}
	else
	{
		Speed_flag = Speed_FAIL;
		PIT_Flag_Clear(EMC512_PIT);       //清中断标志位

	}
*/
}

/*
*  @brief      速度脉冲LCD显示函数
*  @since      v5.0
*/
void Speed_Show(int16 speed)
{
	Site_t  site1 = {10 , 10};
	if (speed > 0)
	{
             LCD_num_BC(site1,speed,9, BLUE,RED);
	} 
	else
	{
		LCD_num_BC(site1,123,9, BLUE,RED);
		Site_t  site2 = { 30 , 30 };
		LCD_num_BC(site2,ABS(speed),9, BLUE,RED);
	}
}


void RS540_Init()
{
	ftm_pwm_init(RS540_FTM, RS540_FTM_1, RS540_ferq, 0);
	ftm_pwm_init(RS540_FTM, RS540_FTM_2, RS540_ferq, 0);
}

void RS540_Turn(uint16 Duty)
{
	ftm_pwm_duty(RS540_FTM, RS540_FTM_1, Duty);
}

float RS540_PID(float speed,float actualSpeed)
{
	uint8 index;
	Speed_pid.SetSpeed = speed;
	Speed_pid.Kp = 0.5;
	Speed_pid.Ki = 0.35;
	Speed_pid.Kd = 0.1;

	Speed_pid.err = Speed_pid.SetSpeed - actualSpeed;
	/*
	if (abs(pid.err) > 200)           //变积分过程
	{
		index = 0.0;
	}
	else if (abs(pid.err) < 180) 
	{
		index = 1.0;
		pid.integral += pid.err;
	}
	else 
	{
		index = (200 - abs(pid.err)) / 20;
		pid.integral += pid.err;
	}*/	
	

	float incrementSpeed = Speed_pid.Kp*(Speed_pid.err - Speed_pid.err_next) + index*Speed_pid.Ki*Speed_pid.err + Speed_pid.Kd*(Speed_pid.err - 2 * Speed_pid.err_next + Speed_pid.err_last);

	Speed_pid.err_last = Speed_pid.err_next;
	Speed_pid.err_next = Speed_pid.err;

	return incrementSpeed;
}
