#include "include.h"
#include "common.h"

/***********************************************************************************************************************************/

int16 encoder_in1;         //编码器输入变量1
int16 encoder_in2;         //编码器输入变量2

/***********************************************************************************************************************************/

void servo_init();         //舵机初始化函数
void servo_control();      //舵机PWM波输出函数

void motor_init();         //电机初始化函数
void motor_control();      //电机PWM波输出函数

void encoder_init();       //编码器初始化函数
void encoder_input();      //编码器正交解码输入函数

/***********************************************************************************************************************************/

void servo_init()          //舵机初始化函数
{
      ftm_pwm_init( SERVO_FTM , SERVO_CH , SERVO_HZ , SERVO_MID );      //初始化 舵机 PWM
}

/***********************************************************************************************************************************/

void servo_output()       //舵机PWM波输出函数
{
      ftm_pwm_duty( SERVO_FTM , SERVO_CH , servo_out );    //精度为0.01%
}

/***********************************************************************************************************************************/

void motor_init()          //电机初始化函数
{
      ftm_pwm_init(MOTOR_FTM, MOTOR0_PWM,MOTOR_HZ,0);      //初始化 电机 PWM
      ftm_pwm_init(MOTOR_FTM, MOTOR1_PWM,MOTOR_HZ,0);      //初始化 电机 PWM
      
      motor0_out=0;     //电机输出变量1赋初值  0
      motor1_out=0;     //电机输出变量2赋初值  0
}

/***********************************************************************************************************************************/

void motor_output()       //电机PWM波输出函数
{
 
  ftm_pwm_duty(MOTOR_FTM, MOTOR0_PWM,motor0_out);     //（左轮反转输出）精度为0.1%
  ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,motor1_out);     //（左轮正转输出）精度为0.1%
}

/***********************************************************************************************************************************/

void encoder_init()        //编码器初始化函数
{
      
}

/***********************************************************************************************************************************/

void encoder_input()       //编码器正交解码输入函数
{
      
}

/***********************************************************************************************************************************/

