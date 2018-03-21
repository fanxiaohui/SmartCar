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
	SD5_pid.Kp = 0.5;
	SD5_pid.Ki = 0.35;
	SD5_pid.Kd = 0.01;
}

/*
*  @brief      SD5_PID获取函数
*  @param      内部更改kp,ki,kd,清除路况标志
*/
void  Get_SD5_PID(void)
{
	if (cross_flag)              //十字
	{
		SD5_pid.Kp = 0.5;
		SD5_pid.Ki = 0.35;
		SD5_pid.Kd = 0.01;

		cross_flag = 0;
	} 
	else if (L_Sharp_turn_flag)    //左急弯
	{
		SD5_pid.Kp = 0.5;
		SD5_pid.Ki = 0.35;
		SD5_pid.Kd = 0.01;

		L_Sharp_turn_flag = 0;
	} 
	else if (R_Sharp_turn_flag)    //右急弯
	{
		SD5_pid.Kp = 0.5;
		SD5_pid.Ki = 0.35;
		SD5_pid.Kd = 0.01;

		R_Sharp_turn_flag = 0;
	}
	else if (Big_Turn_flag)      //大弯
	{
		SD5_pid.Kp = 0.5;
		SD5_pid.Ki = 0.35;
		SD5_pid.Kd = 0.01;

		Big_Turn_flag = 0;
	}
	else if (S_flag)             //连续S小弯
	{
		SD5_pid.Kp = 0.5;
		SD5_pid.Ki = 0.35;
		SD5_pid.Kd = 0.01;

		S_flag = 0;
	}
	else if (Barrier_flag)       //障碍
	{
		SD5_pid.Kp = 0.5;
		SD5_pid.Ki = 0.35;
		SD5_pid.Kd = 0.01;

		Barrier_flag = 0;
	}
	else if (L_Loop_flag)          //左侧环岛
	{
		SD5_pid.Kp = 0.5;
		SD5_pid.Ki = 0.35;
		SD5_pid.Kd = 0.01;

		L_Loop_flag = 0;
	}
	else if (R_Loop_flag)          //左侧环岛
	{
		SD5_pid.Kp = 0.5;
		SD5_pid.Ki = 0.35;
		SD5_pid.Kd = 0.01;

		R_Loop_flag = 0;
	}
	else                         //正常
	{
		SD5_pid.Kp = 0.5;
		SD5_pid.Ki = 0.35;
		SD5_pid.Kd = 0.01;
	}
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
		PID = 6 * SD5_pid.increment;
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
		                                                              //用于限制占空比的最大最小值
		if (SD5_Duty >= SD5_Left_max && SD5_Duty <= SD5_Right_max)    
		{
			SD5_Turn(SD5_Duty);
		}
		else if (SD5_Duty <= SD5_Left_max)                            
		{
			SD5_Turn(SD5_Left_max);
		}
		else                                                         
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