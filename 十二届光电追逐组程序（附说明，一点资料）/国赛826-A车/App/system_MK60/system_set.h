#ifndef _SYSTEM_SET_H_
#define _SYSTEM_SET_H_

/*------------------------------宏定义枚举-----------------------------*/

typedef enum
{
      CAR_WAIT,      //小车等待状态
      CAR_RUN,       //小车行驶状态
      CAR_STOPP,     //小车停止状态
}CAR_STATUS;         //小车状态枚举变量

/*----------------------------------------------------------------------*/

/*------------------------------变量声明区------------------------------*/

extern uint8 car_flag;          //前后车标志位（0为前车，1为后车）
extern uint8 car_start_flag;
extern uint8 double_car_flag;   //双车（1启动双车模式）
extern CAR_STATUS car_status;
extern uint8 servo_enable,motor_enable;
extern uint8 send_error_flag,send_start_flag;
extern uint32 send_error;
/*----------------------------------------------------------------------*/

/*------------------------------函数声明区------------------------------*/

extern void car_init();
extern void car_wait();
extern void car_judge();
extern uint8 QianChe_extract();
extern void qipao_tongxin();

/*----------------------------------------------------------------------*/








#endif


