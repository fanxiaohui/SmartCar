#include "SD5.h"
#include "PID.h"

struct _pid SD5_pid;
uint8    SD5_flag = SD5_Normal;     //舵机状态

/*
*  @brief      SD5初始化，调整到中值
*/
void SD5_Init()
{
	ftm_pwm_init(SD5_FTM, SD5_FTM_CH, SD5_FTM_Freq, SD5_Duty_Middle);
}

/*
*  @brief      SD5转动函数
*  @param      Duty         占空比
*/
void SD5_Turn(uint16 Duty)
{
	ftm_pwm_duty(SD5_FTM, SD5_FTM_CH, Duty);
}

/*
*  @brief      SD5_PID初始化函数
*  @param      
*/
void SD5_PID_Init()
{
	SD5_pid.SetSpeed = 0;
	SD5_pid.increment = 0.0;
	SD5_pid.err = 0;
	SD5_pid.err_next = 0;
	SD5_pid.err_last = 0;
	SD5_pid.Kp = 0.1;
	SD5_pid.Ki = 0.3;
	SD5_pid.Kd = 0.08;
}

/*
*  @brief      SD5_PID运算函数
*  @param      actual_err 当前偏差
*/
float SD5_PID(int16 actual_err)
{
	float PID;

	SD5_pid.err = actual_err;

	SD5_pid.increment = SD5_pid.Kp*(SD5_pid.err - SD5_pid.err_next) + SD5_pid.Ki*SD5_pid.err + SD5_pid.Kd*(SD5_pid.err - 2 * SD5_pid.err_next + SD5_pid.err_last);
	SD5_pid.err_last = SD5_pid.err_next;
	SD5_pid.err_next = SD5_pid.err;

	if (abs(SD5_pid.increment) <= 10)
	{
		PID = 4 * SD5_pid.increment;
	} 
	else if (abs(SD5_pid.increment > 10 && abs(SD5_pid.increment) <= 30))
	{
		PID = 7 * SD5_pid.increment;
	}
	else if(abs(SD5_pid.increment > 30 && abs(SD5_pid.increment) <=60))
	{
		PID = 9 * SD5_pid.increment;
	}
	else if (abs(SD5_pid.increment) > 60 && abs(SD5_pid.increment) <= 80)
	{
		PID = 10 * SD5_pid.increment;
	}
	

	return PID;
}

void  SD5_Contral(int16 average)
{
	int16 SD5_Duty;              //舵机占空比

	if (SD5_flag == SD5_Normal)  //如果偏差在正常范围
	{
		SD5_Duty = (int16)SD5_PID(average) + SD5_Duty_Middle;

		if (SD5_Duty >= SD5_Left_max && SD5_Duty <= SD5_Right_max)    //如果占空比在可调范围内
		{
			SD5_Turn(SD5_Duty);
		}
		else if (SD5_Duty <= SD5_Left_max)                            //如果占空比小于最左
		{
			SD5_Turn(SD5_Left_max);
		}
		else                                                          //如果占空比大于最右
		{
			SD5_Turn(SD5_Right_max);
		}
	}
	else if (SD5_flag == SD5_Left)    //如果偏差应让舵机打到最左
	{
		SD5_Turn(SD5_Left_max);

	}
	else                              //如果偏差应让舵机打到最右
	{
		SD5_Turn(SD5_Right_max);

	}
	



}