#ifndef _FTM_FUNCTION_H_
#define _FTM_FUNCTION_H_

/*------------------------------宏定义区--------------------------------*/

        /*舵机输出宏定义参数*/

#define SERVO_IO    PTC9           //舵机输出 FTM 引脚号

#define SERVO_FTM   FTM3           //舵机输出 FTM 模块号
#define SERVO_CH    FTM_CH5        //舵机输出 FTM 通道号
#define SERVO_HZ    (50)           //舵机输出 频率 （一般为 50~100 HZ）


        /*电机输出宏定义参数*/

#define MOTOR0_IO   PTD5           //电机输出2 FTM 引脚号
#define MOTOR1_IO   PTD4           //电机输出4 FTM 引脚号

#define MOTOR_FTM   FTM0           //电机输出 FTM 模块号
#define MOTOR0_PWM  FTM_CH5        //电机输出反转 FTM 通道号
#define MOTOR1_PWM  FTM_CH4        //电机输出正转 FTM 通道号

//滑行模式下，频率应该是 30~100。
//常规模式下，频率应该是 20k 左右

#define MOTOR_HZ    (20*1000)


        /*编码器输入宏定义参数*/



/*----------------------------------------------------------------------*/

/*------------------------------变量声明区------------------------------*/

extern int16 encoder_in1;      //编码器输入变量1
extern int16 encoder_in2;      //编码器输入变量2

/*----------------------------------------------------------------------*/

/*------------------------------函数声明区------------------------------*/

extern void servo_init();      //舵机初始化函数
extern void servo_output();    //舵机PWM波输出函数

extern void motor_init();      //电机初始化函数
extern void motor_output();    //电机PWM波输出函数

extern void encoder_init();    //编码器初始化函数
extern void encoder_input();   //编码器正交解码输入函数

/*----------------------------------------------------------------------*/




#endif


