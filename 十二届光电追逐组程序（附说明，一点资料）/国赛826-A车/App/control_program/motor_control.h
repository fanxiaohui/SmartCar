#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_

/*------------------------------宏定义枚举-----------------------------*/

#define POSITION_JIZHUN_DOWN  59
#define POSITION_JIZHUN_UP    49
#define IN_CURVE_ROW_MIN     250
#define POSITION_ERROR_RANGE  79.5
/*----------------------------------------------------------------------*/

/*------------------------------变量声明区------------------------------*/
extern uint8 qibu_flag;
extern int16 speedout;          //电机输出正向变量
extern uint8 get_motor_Ku_flag;
extern uint8 speed_yun_flag;
extern int16 motor_goal;        //电机输出给定速度
extern int16 motor_goal_temp;     //电机输出给定速度中间变量
extern int16 speed_step;
extern int16 speed_minl;
extern int16 speed_maxl;
extern int16 speed_minr;
extern int16 speed_maxr;
extern int16 speed_s;
extern int16 speed_z;
extern int16 speed_step2;
extern int16 speed_poa;
extern int16 speed_pob;
extern int16 speed_za;
extern int16 speed_zb;
extern int16 speed_ha;
extern int16 speed_hb;

extern uint8 in_L_curve_flag,in_R_curve_flag;
extern uint8 in_d_curve_flag;
extern uint8 in_d_L_curve_flag;
extern uint8 in_d_R_curve_flag;
extern double motor_step;
extern double motor_a;
extern double motor_b;
extern double motor_c;
extern double motor_d;
extern double motor_e;
extern double motor_f;


extern int16 motor0_out;          //电机输出变量1
extern int16 motor1_out;          //电机输出变量2
extern int16 speedout_count;      //编码器反馈的实际速度

extern int32 speedout_val;        //编码器反馈速度统计值


extern uint8 in_curve_flag;
extern uint8 in_curve_speed_set;
extern double car_position;
extern int16 motor_error[3];
extern uint8 car_stop_bangbang;
extern uint8 stop_wait;
extern uint8 stop_wait_over;
extern uint8 overtake_po_stop;
extern uint8 overtake_po_stop_over;
extern int32 bangbang_tingcnt;
extern int32 bangbang_tingcnt2;
extern int32 bangbang_tingcnt3;
extern uint8 out_curve_flag;
/*----------------------------------------------------------------------*/

/*------------------------------函数声明区------------------------------*/

extern void motor_control();        //电机速度给定函数

extern void motor_control_pit();    //电机速度PID控制函数

/*----------------------------------------------------------------------*/








#endif


