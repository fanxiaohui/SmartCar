#include "Speed.h"

volatile Speed_STATUS_e      Speed_flag = Speed_FINISH;   //图像状态

int16    Speed_val;                                       //编码器采集到的速度脉冲数
int16    RS540_Duty;


/*
*  @brief      速度采集初始化函数(编码器初始化)
*  @since      v5.0
*/
void Speed_Init(void)
{
	ftm_quad_init(ECM512_FTM);

	pit_init_ms(EMC512_PIT, 1000);                          //初始化PIT0，定时时间为：1000ms
	set_vector_handler(PIT0_VECTORn, PIT0_IRQHandler);      //设置PIT0的中断复位函数为 PIT0_IRQHandler
//	enable_irq(PIT0_IRQn);                                  //使能PIT0中断

}

void Speed_Get(void)
{
	Speed_flag = Speed_START;
	enable_irq(PIT0_IRQn);                                  //使能PIT0中断
	while (Speed_flag != Speed_FINISH)                      //等待速度采集完成
	{
		if (Speed_flag == Speed_FAIL)                      //若速度采集失败
		{
			PIT_Flag_Clear(EMC512_PIT);                    //清中断标志位
			enable_irq(PIT0_IRQn);                         //使能PIT0中断，开始再次采集
		}
	}
}

/*
*  @brief      PIT0中断服务函数
*  @since      v5.0
*/
void PIT0_IRQHandler(void)
{
	if (Speed_flag == Speed_START)
	{
		Speed_flag = Speed_GATHER;              //速度采集中
		Speed_val = ftm_quad_get(EMC512_PIT);
		Speed_flag = Speed_FINISH;
		PIT_Flag_Clear(EMC512_PIT);       //清中断标志位
	}
	else
	{
		Speed_flag = Speed_FAIL;
		PIT_Flag_Clear(EMC512_PIT);       //清中断标志位

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
/*
float RS540_PID(float speed,float actualSpeed)
{
	uint8 index;
	pid.SetSpeed = speed;
	pid.ActualSpeed = actualSpeed;
	pid.Kp = 1.0;
	pid.Ki = 1.0;
	pid.Kd = 1.0;

	pid.err = pid.SetSpeed - pid.ActualSpeed;
	pid.integral += pid.err;          

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
	}	
	

	float incrementSpeed = pid.Kp*(pid.err - pid.err_next) + index*pid.Ki*pid.err + pid.Kd*(pid.err - 2 * pid.err_next + pid.err_last);

	pid.ActualSpeed += incrementSpeed;
	pid.err_last = pid.err_next;
	pid.err_next = pid.err;
	return pid.ActualSpeed
}
*/