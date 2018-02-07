#ifndef _SERVO_CONTROL_H_
#define _SERVO_CONTROL_H_

/*------------------------------宏定义区--------------------------------*/

//#define JIZHUN_UP       30        //计算偏差的中线截取部分的上边界26
//#define JIZHUN_DOWN     50        //计算偏差的中线截取部分的下边界35
//#define JIZHUN_ROW      40        //中心权值行

#define JIZHUN_COL      79.5        //图像中线基准

#define LENGTH_IN       160         //图像近处降低权值范围（逆透视单位）
#define LENGTH_OUT      320         //图像远处降低权值范围（逆透视单位）
#define pull_over_dis   120         //小车靠边停车前瞻设定（逆透视单位）
#define pull_over_dis_po  160          //小车靠边停车前瞻设定（逆透视单位）
#define pull_over_dis_cross   160         //小车靠边停车前瞻设定（逆透视单位）
#define pull_over_dis_qi  80         //小车靠边停车前瞻设定（逆透视单位）
/*----------------------------------------------------------------------*/

/*------------------------------变量声明区------------------------------*/

extern uint32 jizhun_step;
extern uint32 JIZHUN_ROW;    //中心权值行
extern uint32 JIZHUN_UP;     //计算偏差的中线截取部分的上边界26
extern uint32 JIZHUN_DOWN;   //计算偏差的中线截取部分的下边界35

extern uint32 servo_m_step;
extern uint32 SERVO_MID;     // 637
extern uint32 SERVO_L;       //舵机左极限  756(卧式舵机）
extern uint32 SERVO_R;       //舵机右极限  576（卧式舵机）

extern double servo_step;
extern double servo_a;
extern double servo_b;
extern double servo_c;
extern double servo_d;
extern double servo_e;
extern double servo_f;

extern double servo_Kp;           //舵机 P系数
extern double servo_Ki;           //舵机 I系数
extern double servo_Kd;           //舵机 D系数
extern double middle_Error[2];    //舵机偏差存储函数
extern double midline_avg;        //中心线平均值
extern int16 servo_out;            //舵机输出值

extern double inv_weight[CAMERA_H];  //中心线权值数组

/*----------------------------------------------------------------------*/

/*------------------------------函数声明区------------------------------*/

extern void servo_control();   //舵机控制主函数
extern void get_weight();      //中心线权值获取函数
extern void middle_avg();      //中心线加权平均求偏差函数
extern void PID_control();     //舵机PID控制

/*----------------------------------------------------------------------*/




#endif