#ifndef _EDGE_EXTRACT_H_
#define _EDGE_EXTRACT_H_

/*------------------------------宏定义区--------------------------------*/


/*
int16 bar_range_col[CAMERA_H]={10,11,12,13,14,15,16,17,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,31,32,33,34,35,36,37,38,39,40,41,42,43,44,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,58,59,60,61,62,63,64,65};
a车宽度*/

/*常规部分宏定义*/
#define PI                3.1415926       //pi值

#define DOT_B             0               //黑点的灰度值
#define DOT_W             255             //白点的灰度值
#define DOT_M             127             //中间的灰度值

#define ROW_START         0               //行扫描起始点
#define ROW_END           (CAMERA_H-1)    //行扫描终止点

#define COL_START         0               //列扫描起始点
#define COL_END           (CAMERA_W-1)    //列扫描终止点

#define ROW_W_SUM         8               //上搜行数总值
#define ROW_W_NUM         4               //上搜得到的白点数阀值，超过阀值则判断为干扰边沿
#define COL_W_NUM         8               //橫搜得到的白点数阀值，超过阀值则判断为干扰边沿


#define curve_div         220             //弯道边沿消除距离


#define s_start_row       (CAMERA_H-20)   //终点线行搜索结束行

#define straight_end      0               //直道搜索结束行数
#define straight_start    54              //直道搜索起始行数

#define loop_out_end      5               //环道出口行搜索结束行
#define loop_per          75              //环道面积比例
#define loop_out_l        3000            //出环距离

#define end_line_end      7               //终点线行搜索结束行
#define end_line_gray_add   10            //终点线行灰度检测增加值


/*十字部分宏定义*/
#define CROSS_UP_ROW      2               //结束搜索上十字直角拐点行
#define CROSS_UP_COL      4               //边沿突变判断上十字直角拐点列阀值
#define CROSS_ERTRACT     10              //搜索上十字直角拐点缩进行数
#define CROSS_DOWN_ROW    4               //结束搜索下十字直角拐点行
#define CROSS_DOWN_COL    8               //边沿突变判断下十字直角拐点列阀值
#define CROSS_START_ROW   (CAMERA_H-10)    //开始搜索十字直角拐点行

#define L_CROSS_ADD       40              //左边沿左倾并且有左边沿十字标志时，向右搜索范围
#define L_LOST_CROSS      6               //左边沿出现时距离边界列数，判断左边十字（大于此值）
#define R_CROSS_ADD       40              //右边沿右倾并且有右边沿十字标志时，向左搜索范围
#define R_LOST_CROSS      (CAMERA_W-7)    //右边沿出现时距离边界列数，判断右边十字（小于此值）

#define cross_k_add       40              //上十字搜索范围
#define cross_k_add_1     70              //上十字搜索范围
#define cross_k_1         0.16            //上十字搜索范围系数
#define cross_k_2         280             //上十字搜索范围系数


/*障碍部分宏定义*/
#define bar_search_end    35             //近处使用宽度搜索障碍结束行
#define BAR_UP_ROW        10             //结束搜索障碍行
#define BAR_ADD           6              //障碍外扩行数
#define BAR_DIV           8              //障碍外扩除数（使用原理：bar_range_col[i]/BAR_DIV）


/*搜线部分宏定义*/
#define L_LOST            119             //左边沿丢失时，向右搜索范围
#define L_EDGE_IN         60              //左边沿没有趋势时，向内搜索范围
#define L_EDGE_OUT        60              //左边沿没有趋势时，向外搜索范围
#define L_IN_2_IN         60              //左边沿内倾时，向内搜索范围
#define L_IN_2_OUT        40              //左边沿内倾时，向外搜索范围
#define L_OUT_2_IN        40              //左边沿外倾时，向内搜索范围
#define L_OUT_2_OUT       40              //左边沿外倾时，向外搜索范围
#define L_OUT_END         20              //左边沿外倾丢失结束判断
#define L_LOOP            20              //左边沿丢失环道搜索范围

#define R_LOST            40              //右边沿丢失时，向右搜索范围
#define R_EDGE_IN         60              //左边沿没有趋势时，向内搜索范围
#define R_EDGE_OUT        60              //左边沿没有趋势时，向外搜索范围
#define R_IN_2_IN         60              //右边沿内倾时，向内搜索范围
#define R_IN_2_OUT        40              //右边沿内倾时，向外搜索范围
#define R_OUT_2_IN        40              //右边沿外倾时，向内搜索范围
#define R_OUT_2_OUT       40              //右边沿外倾时，向外搜索范围
#define R_OUT_END         139             //右边沿外倾丢失结束判断
#define R_LOOP            140             //右边沿丢失环道搜索范围


/*边沿清除函数*/
#define edge_clean();           memset(left_edge,-1,sizeof(left_edge));             \
                                memset(right_edge,-1,sizeof(right_edge));           \
                                memset(L_down_cross,-1,sizeof(L_down_cross));       \
                                memset(L_up_cross,-1,sizeof(L_up_cross));           \
                                memset(R_down_cross,-1,sizeof(R_down_cross));       \
                                memset(R_up_cross,-1,sizeof(R_up_cross));           \
                                memset(midline,-1,sizeof(midline));                 \
                                //memset(left_down_edge,-1,sizeof(left_down_edge));   \
                                memset(right_down_edge,-1,sizeof(right_down_edge)); \
                                memset(midline_down_x,-1,sizeof(midline_down_x));   \
                                memset(midline_down_y,-1,sizeof(midline_down_y));   \

/*----------------------------------------------------------------------*/

/*------------------------------变量声明区------------------------------*/

        /*以下变量可能需要用于外部函数*/


extern double angle_aaa[4];

extern uint8 cross_num;                  //十字计数

extern uint8 L_bar_num;                  //左边障碍物计数
extern uint8 R_bar_num;                  //右边障碍物计数

extern uint8 loop_num;                   //环道计数
extern uint8 end_line_num;             //起跑线计数

extern uint8 car_over_able;              //超车识别使能标志位
extern uint8 car_over_ok;                //超车成功标志位
extern int32 bar_car_val;                //超车距离

extern uint8 L_EDGE_NUM;                 //左边沿个数
extern uint8 R_EDGE_NUM;                 //右边沿个数

extern uint8 L_LINK_NUM;                 //连续左边沿个数（搜索完边沿右作为向下搜索边沿个数）
extern uint8 R_LINK_NUM;                 //连续右边沿个数（搜索完边沿右作为向下搜索边沿个数）

extern uint8 L_cross_flag;               //左边沿判断进入十字标志
extern uint8 R_cross_flag;               //右边沿判断进入十字标志
extern uint8 cross_flag;                 //判断进入十字标志

extern uint8 L_search_end;               //左边沿搜索结束标志
extern int16 L_end_row;                  //左边沿搜索结束时，结束行数
extern int16 L_down_end_row;             //左边沿向下搜索结束时，结束行数

extern uint8 R_search_end;               //右边沿搜索结束标志
extern int16 R_end_row;                  //右边沿搜索结束时，结束行数
extern int16 R_down_end_row;             //右边沿向下搜索结束时，结束行数

extern uint8 loop_able;                  //环型赛道使能标志
extern uint8 LOOP_FLAG;                  //环型赛道标志
extern uint8 LOOP_TEMP;                  //环型赛道暂存标志
extern uint32 LOOP_TEMP_1;               //环型赛道暂存标志_1
extern uint8 LOOP_IN;                    //环型赛道进入标志
extern uint8 LOOP_OUT;                   //环型赛道将要离开标志
extern uint8 loop_l_r_flag;              //环型赛道转向判断标志（为1时左转，为0时右转）
extern int16 LOOP_IN_ROW;                //环型赛道入口行
extern int16 LOOP_OUT_ROW;               //环型赛道出口行
extern int16 out_row[2];                 //环型赛道出口行存储数组
extern int32 loop_val;                   //环道距离

extern uint8 barrier_able;               //使能检测障碍标志
extern uint8 L_barrier_flag;             //左边沿障碍标志
extern uint8 R_barrier_flag;             //右边沿障碍标志
extern int16 L_barrier_down_row;         //左边沿障碍物下边沿行
extern int16 L_barrier_up_row;           //左边沿障碍物上边沿行
extern int16 R_barrier_down_row;         //右边沿障碍物下边沿行
extern int16 R_barrier_up_row;           //右边沿障碍物上边沿行
extern int16 car_extract_row;            //检测前车距离行
extern uint8 car_extract_able;           //前车检测使能标志

extern uint8 end_line_able;              //终点线使能检测标志
extern uint8 END_LINE_FLAG;              //终点线标志
extern uint8 END_LINE_TEMP;              //终点线暂存标志
extern uint8 END_LINE_TEMP_1;            //终点线暂存标志_1
extern int16 end_line_row;               //终点线所在行
extern int16 end_line_2[2];              //终点线所在行数组

extern uint8 L_TURN;                     //内倾锁死，判断为左转向标志
extern uint8 R_TURN;                     //内倾锁死，判断为右转向标志
extern int16 curve_in_row;               //弯道入口行
extern int16 curve_car_row;              //弯道前车行数

extern uint8 STRAIGHT_FLAG;              //直道标志

extern uint8 S_FLAG;                     //S型 赛道标志
extern int16 L_s_temp;                   //左边沿判断S型 赛道标志
extern int16 R_s_temp;                   //右边沿判断S型 赛道标志
extern double s_down_dis;               //S型 赛道降前瞻距离

extern uint8 RAMP_FLAG;                  //坡道标志
extern uint8 RAMP_TEMP;                  //坡道暂存标志
extern int32 ramp_val;                   //坡道距离
extern uint8 ramp_able;                  //坡道使能检测标志

extern uint8 little_s_flag;              //小S连续弯标志

extern double inv_distance[CAMERA_H];    //逆透视获取每行实际距离数组
extern double bar_range_col[CAMERA_H];   //障碍物搜索范围数组

extern int16 L_down_cross[2];            //左边沿十字下方直角拐点数组（[0]为十字拐点行坐标值，[1]为十字拐点列坐标值）
extern int16 L_up_cross[2];              //左边沿十字上方直角拐点数组（[0]为十字拐点行坐标值，[1]为十字拐点列坐标值）

extern int16 R_down_cross[2];            //右边沿十字下方直角拐点数组（[0]为十字拐点行坐标值，[1]为十字拐点列坐标值）
extern int16 R_up_cross[2];              //右边沿十字上方直角拐点数组（[0]为十字拐点行坐标值，[1]为十字拐点列坐标值）

extern int16 left_edge[CAMERA_H];              //左边沿存放数组
extern int16 right_edge[CAMERA_H];             //右边沿存放数组

extern int16 left_down_edge[10];         //下降左边沿存放数组
extern int16 right_down_edge[10];        //下降右边沿存放数组

/*----------------------------------------------------------------------*/

/*------------------------------函数声明区------------------------------*/

extern void edge_extract();             //此函数为边沿提取主函数

extern void tan_angle();                //此函数为斜率角度制表函数

extern void bar_range();                //此函数为每行障碍物搜索范围获取函数

extern void bar_car();                  //此函数为超车判断函数
/*----------------------------------------------------------------------*/






#endif