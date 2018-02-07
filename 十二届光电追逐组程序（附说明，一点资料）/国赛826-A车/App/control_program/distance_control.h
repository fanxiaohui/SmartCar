#ifndef _DISTANCE_CONTROL_H_
#define _DISTANCE_CONTROL_H_

extern void distance_control();
extern void dis_control();
extern void Q_dis_control();
extern void Q_dis_fache();
extern void H_dis_control();
extern void H_dis_fache();
extern void distance_speed(double a,double b,double c);
extern void distance_speed0(double a,double b,double c);

extern  double distance_step;
extern double distance_a;  //防止距离过近的近处误差，单位厘米
extern double distance_b;  //
extern double distance_c;  //防止距离过近的近处误差，单位厘米(弯道时)
extern double distance_d;  //
extern double distance_e;
extern double distance_f;
extern  double distance2_step;
extern double distance2_a;  //防止距离过近的近处误差，单位厘米
extern double distance2_b;  //
extern double distance2_c;  //防止距离过近的近处误差，单位厘米(弯道时)
extern double distance2_d;  //
extern double distance2_e;
extern double distance2_f;

extern int32 Q_distance,H_distance[2];
extern uint8 dis_huaxing;
extern uint8 H_csb_get;
extern uint8 H_QC_get;

#define DIS_DANGEROUS    (200)//距离下限
#define DIS_SHACHE       (500)//开始急刹车
#define MAX_SPEED       (800)//(motor_goal_temp*2+50)
#define MIN_SPEED       (-1000)

#define limit_z_shang    (1+distance2_d*0.01)
#define limit_z_xia      (1-distance2_d*0.01)
#define limit_pu_shang   (1+distance2_e*0.01)
#define limit_pu_xia     (1-distance2_e*0.01)
#define limit_w_shang    (1+distance2_f*0.01)
#define limit_w_xia      (1-distance2_f*0.01)

#endif