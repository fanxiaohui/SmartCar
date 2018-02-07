#ifndef _CAR_SKILL_H_
#define _CAR_SKILL_H_
/*------------------------------宏定义枚举-----------------------------*/
#define OVERTAKE_TIME     100
#define LOOP_TEMP_delay_time1 6000
#define LOOP_TEMP_delay_time2 6000
#define STRAIGHT_delay_time1 50
#define IN_CURVE_DELAY_delay_time1 50
#define S_delay_time1    150
#define CROSS_delay_time1  50
#define CROSS_delay_time2  150
#define S_CURVE_delay_time1  50
#define L_CURVE_delay_time1  100
#define R_CURVE_delay_time1  100
#define B_car_delay_time1     50
#define car_pull_over_delay_time 30
#define BARRIER_TIME1 200

/*----------------------------------------------------------------------*/

/*------------------------------变量声明区------------------------------*/
extern int16 cnt_step;
extern int16 cnt_a;//直道次数
extern int16 cnt_b;//环道次数
extern int16 cnt_c;//坡道次数
extern int16 cnt_d;
extern int16 cnt_e;
extern int16 cnt_f;

//////////////////////////////////////////////////////////////////////////
extern int16 barrier_time;
/////////////////////////////////////////////////////////////////////////
/******************************************************************/
extern int16 END_LINE_TEMP_delay_time2;
/******************************************************************/
extern uint8 IN_CURVE_delay_flag1;
extern int16 IN_CURVER_delay_cnt1;
/******************************************************************/
extern uint8 END_LINE_TEMP_delay_flag2;
extern int16 END_LINE_TEMP_delay_cnt2;
/******************************************************************/
extern uint8 car_L_pull_over_flag;//左停车
extern uint8 car_R_pull_over_flag;//右停车
extern uint8 car_M_pull_over_flag;//停车
extern uint8 car_pull_over_flag;//启动停车
extern int16 car_pull_over_delay_cnt;//停车时长
extern uint8 car_pull_over_finish_flag;//停车完毕
/******************************************************************/
extern int16 B_car_delay_cnt1;
extern uint8 B_car_delay_flag1;
extern int16 B_car_delay_cnt2;
extern uint8 B_car_delay_flag2;
extern int16 bangbang_cnt;
extern uint8 bangbang;
extern uint8 bangbang_begin;
/******************************************************************/
extern int16 LOOP_delay_cnt1;
extern uint8 LOOP_delay_flag1;
/******************************************************************/
extern int16 STRAIGHT_delay_cnt1;
extern uint8 STRAIGHT_delay_flag1;
/******************************************************************/
extern int16  L_CURVE_delay_cnt1;
extern uint8  L_CURVE_delay_flag1;
extern int16  R_CURVE_delay_cnt1;
extern uint8  R_CURVE_delay_flag1;
/******************************************************************/
extern int16  S_CURVE_delay_cnt1;
extern uint8  S_CURVE_delay_flag1;
/******************************************************************/
extern int16  S_delay_cnt1;
extern uint8  S_delay_flag1;
/******************************************************************/
extern int16  CROSS_delay_cnt1;
extern uint8  CROSS_delay_flag1;
extern int16  CROSS_delay_cnt2;
extern uint8  CROSS_delay_flag2;
/******************************************************************/
///////////////////////////////////////////////////////////////////////////////
//---------------------------------------------------------------------------//
///////////////////////////////////////////////////////////////////////////////
/*********通信部分**********/
extern uint8 H_Q_overtake_outrow_ok;
extern uint8 H_Q_overtake_outrow_flag;
extern uint8 H_Q_overtake_ring_ok;
extern uint8 H_Q_overtake_ring_flag;//后车对前车

extern uint8 Q_H_overtake_cross_flag;
extern uint8 Q_H_stop_ok;
extern uint8 Q_H_overtake_ring_ok;
extern uint8 Q_H_overtake_ring_flag;
extern uint8 Q_H_overtake_po_flag;
extern uint8 Q_H_obstacle_flag;//前车对后车
/*********发车部分*********/
extern uint8 speedout_val_start1_flag;
extern int32 speedout_val_start1;
extern uint8 overtake_start_ok;
extern uint8 LOR_turn_send_ok;
extern int32 l_barrier_val;
extern uint8 barrier_over_start;
extern uint8 l_barrier_over;

extern uint8 dir_overtake_start;
extern uint8 overtake_start_flag;//发车模式
extern uint8 START_LINE_TEMP;//记录已经过起跑线，只使用一次
extern uint8 overtake_ok_flag;//成功超车
extern int32 overtake_start_sum;//超车次数，0不超车，1正常速度超一次，2直道太短慢速超
extern uint8 houche_speed_flag;//后车超完速度回归正常
extern uint8 speedout_val0_flag;//记步标志
extern int32 speedout_val0;//过起跑线的距离，用于定点停车
extern uint8 speedout_val5_flag;//记步标志
extern int32 speedout_val5;//最终的停车
/*********十字部分*********/
extern uint8 overtake_cross_flag;
extern uint8 overtake_cross_sum;
extern uint8 Q_overtake_cross;
extern uint8 overtake_cross_stop;
extern uint8 overtake_cross_ing_flag;
extern uint8 speedout_val_cross_flag;
extern int32 speedout_val_cross;
extern uint8 H_voertake_cross;
extern uint8 Q_stop_ok;
extern uint8 overtake_cross_stop_over;

extern uint8 speedout_val_cross1_flag;
extern int32 speedout_val_cross1;
extern uint8 overtake_cross_ok;
extern uint8 speedout_val_cross_ting_flag;
/*********环道超车部分*********/
extern uint8 overtake_outrow_flag;

extern uint8 ringout_able_flag;
extern uint8 overtake_ring_flag;//环道超车模式
extern uint8 overtake1_ring_ok_flag;
extern uint8 overtake1_ring_stop_flag;
extern uint8 overtake_ring_stop;

extern uint8 loop_LOR_flag1;//第一个环的方向
extern uint8 loop_LOR_flag2;//第二个环的方向
extern uint8 loop_LOR_flag3;//第三个环的方向
extern uint8 loop_LR2_flag;//补线用的环的方向
extern int32 overtake_ring_sum;//环超车次数
extern int32 ring_sum;//过环的次数，用于环道方向判断
/*********直道超车部分*********/
extern uint8 dir_overtake_zhi;
extern uint8 overtake_zhi_flag;//直道超车模式
extern uint8 overtake_zhi_ing_flag;//前车超车准备
extern uint8 chaoche_zhi_flag;//后车超车准备
extern uint8 over_zhangai;//障碍记数
extern uint8 over_podao;//坡道记数
extern uint8 over_ring;//环道记数
extern int32 overtake_zhi_sum;//直道超车次数，用于每次超车前置条件的判断
/*********坡道超车部分*********/
extern uint8 po_cnt_S;
extern uint8 dir_overtake_po_1;
extern uint8 dir_overtake_po_2;
extern uint8 overtake_po_ok;
extern uint8 dir_overtake_po;
extern uint8 overtake_po_flag;//坡道超车模式
extern uint8 overtake_po_ing_flag;//前车超车准备
extern int32 overtake_po_sum;//坡道超车次数
extern uint8 speedout_val_po_flag;//记步标志
extern int32 speedout_val_po;
extern uint8 speedout_val_po1_flag;
extern int32 speedout_val_po1;
/*********避障部分*********/
extern uint8 barrier_able_flag;//后车避障模式
extern uint8 barrier_ing_flag;//正在通过障碍物

/*********停车部分*********/
extern uint8 dir_car_stop;
extern uint8 STOP_PATTERN_able;
extern uint8 QAQ_STOP;//出界停车
extern uint8 car_stop_flag;//停车
extern uint8 car_stop_ing_flag;//停车前的缓冲滑行
extern uint8 STOP_PATTERN;

extern uint8 Bee_flag;//蜂鸣器 鸣
extern uint32 Bee_time;//蜂鸣器计时
extern double CAR_position;//底部偏差，代表车身位置

extern uint8 zhengpao;
extern int32 overtake_sum;

extern uint8 CAR_STOPP_flag;
extern uint8 keep_side;

extern uint8 LOR_turn_flag;
extern uint8 LOR_turn_flag1;
extern uint8 LOR_turn_flag2;
extern uint8 po_pull_over;

extern uint8 cross_jian;
extern uint8 cross_qian_pull_over;
extern uint8 cross_hou_pull_over;
/*----------------------------------------------------------------------*/

/*------------------------------函数声明区------------------------------*/
extern void car_control();
extern void f_voertake_no();

extern void overtake_start_extract();
extern void f_overtake_start_barrier();
extern void f_overtake_start();
extern void f_overtake_ring();
extern void f_overtake_po();
extern void f_overtake_cross();

extern void f_overtake_zhi_judge();
extern void f_STOP_PATTERN();

extern void f_ring_out();
extern void f_key_extract();

extern uint8 straightline_R();
extern uint8 straightline_L();

extern void get_curve_distance();
extern void ring_dir();
extern double get_car_position();

extern void shiboqi();
extern  int16 get_points_distance(Site_xy a,Site_xy b);
/*----------------------------------------------------------------------*/






#endif
