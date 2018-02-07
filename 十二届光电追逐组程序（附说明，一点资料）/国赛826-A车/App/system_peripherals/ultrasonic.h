#ifndef _ULTRASONIC1_H_
#define _ULTRASONIC1_H_

/*------------------------------宏定义枚举-----------------------------*/


#define CHAOSHENGBODELAY_TIME1  2
#define PORT_CSB      PTD2
/*----------------------------------------------------------------------*/

/*------------------------------变量声明区------------------------------*/
extern uint8 chaoshengbo_flag;
extern int8 open;
extern int32 car_distance;    //距离
extern int32 car_distance0[10];
extern uint32 timevar;         //定时器计数值，转换成时间
extern int16 chaoshengbo_delay_cnt1;

/*----------------------------------------------------------------------*/

/*------------------------------函数声明区------------------------------*/

extern void ultrasonic_init();     //超声波初始化函数
extern void chaoshengbo(void);

/*----------------------------------------------------------------------*/


#endif //