#include "include.h"
#include "common.h"

/***********************************************************************************************************************************/


        /*以下变量不需要用于外部函数*/

double cross_wide;                //用于补十字边沿的中间变量
double cross_high;                //用于补十字边沿的中间变量
double cross_edge;                //用于补十字边沿的中间变量

int16 cross_row;                  //用于检测十字直角拐点的中间变量
int16 cross_col;                  //用于检测十字直角拐点的中间变量

int16 EXTRACT_ROW;                //当前搜索边沿的行数
int16 EXTRACT_COL;                //当前搜索边沿的列数

int16 W_num_row;                  //用于白块计数的行坐标（也用于其他判断时的中间变量）
int16 W_num_col;                  //用于白块计数的列坐标（也用于其他判断时的中间变量）
int16 W_num;                      //白块计数值（也用于其他判断时的中间变量）

uint8 L_edge_nearest;             //上一行左边沿列坐标
uint8 R_edge_nearest;             //上一行右边沿列坐标

uint8 L_get_flag;                 //左边沿搜索得到标志
uint8 L_lost_num;                 //左边沿连续丢失个数

uint8 R_get_flag;                 //右边沿搜索得到标志
uint8 R_lost_num;                 //右边沿连续丢失个数

uint8 L_trend_in;                 //左边沿内倾趋势判断标志（等于 1 时为内倾,等于 0 时为外倾）
uint8 R_trend_in;                 //右边沿内倾趋势判断标志（等于 1 时为内倾,等于 0 时为外倾）

uint8 l_cross_able;               //环道左边沿十字使能标志
uint8 r_cross_able;               //环道右边沿十字使能标志
uint8 end_bar_able;               //终点线障碍物使能标志
uint8 car_extract_able;           //前车检测使能标志


        /*以下变量可能需要用于外部函数*/

double angle_aaa[4];

uint8 cross_num=0;                //十字计数
uint8 cross_temp=0;               //十字状态标志
uint8 cross_temp_1;               //十字暂存标志_1
int16 cross_down[2];              //十字行暂存

uint8 L_bar_num=0;                //左边障碍物计数
uint8 R_bar_num=0;                //右边障碍物计数
uint8 bar_temp=0;                 //障碍物状态标志
uint8 bar_temp_1;                 //障碍物暂存标志_1
uint8 bar_temp_2;                 //障碍物暂存标志_2
int16 bar_row[2];                 //障碍物行暂存

uint8 loop_num=0;                 //环道计数
uint8 end_line_num=0;             //起跑线计数

uint8 car_over_able=0;            //超车识别使能标志位
uint8 car_over_ok=0;              //超车成功标志位
int32 bar_car_val=-1;             //超车距离
uint8 bar_car_temp_1=0;

uint8 L_EDGE_NUM;                 //左边沿个数
uint8 R_EDGE_NUM;                 //右边沿个数

uint8 L_LINK_NUM;                 //连续左边沿个数（搜索完边沿右作为向下搜索边沿个数）
uint8 R_LINK_NUM;                 //连续右边沿个数（搜索完边沿右作为向下搜索边沿个数）

uint8 L_cross_flag;               //左边沿判断进入十字标志
double L_cross_k;                //左边沿十字斜率
uint8 R_cross_flag;               //右边沿判断进入十字标志
double R_cross_k;                //右边沿十字斜率
uint8 cross_flag;                 //判断进入十字标志

uint8 L_search_end;               //左边沿搜索结束标志
int16 L_end_row;                  //左边沿搜索结束时，结束行数
int16 L_down_end_row;             //左边沿向下搜索结束时，结束行数

uint8 R_search_end;               //右边沿搜索结束标志
int16 R_end_row;                  //右边沿搜索结束时，结束行数
int16 R_down_end_row;             //右边沿向下搜索结束时，结束行数

uint8 loop_able=1;                //环型赛道使能标志
uint8 LOOP_FLAG;                  //环型赛道标志
uint8 LOOP_TEMP=0;                //环型赛道暂存标志
uint32 LOOP_TEMP_1=0;             //环型赛道暂存标志_1
uint8 LOOP_IN=0;                  //环型赛道进入标志
int16 LOOP_IN_1=0;                //环型赛道进入标志_1
uint8 LOOP_OUT_1=0;               //环型赛道离开标志_1
uint8 LOOP_OUT=0;                 //环型赛道将要离开标志
uint8 loop_l_r_flag;              //环型赛道转向判断标志（为1时左转，为0时右转）
int16 LOOP_IN_ROW;                //环型赛道入口行
int16 in_row[2]={-1,-1};          //环型赛道入口行存储数组
int16 LOOP_OUT_ROW;               //环型赛道出口行
int16 out_row[2]={-1,-1};         //环型赛道出口行存储数组
int32 loop_val=0;                 //环道距离

uint8 barrier_able=1;             //使能检测障碍标志
uint8 L_barrier_flag;             //左边沿障碍标志
uint8 R_barrier_flag;             //右边沿障碍标志
int16 L_barrier_down_row;         //左边沿障碍物下边沿行
int16 L_barrier_up_row;           //左边沿障碍物上边沿行
int16 R_barrier_down_row;         //右边沿障碍物下边沿行
int16 R_barrier_up_row;           //右边沿障碍物上边沿行
int16 car_extract_row;            //检测前车距离行

uint8 end_line_able=1;            //终点线使能检测标志
uint8 END_LINE_FLAG;              //终点线标志
uint8 END_LINE_TEMP=0;            //终点线暂存标志
uint8 END_LINE_TEMP_1;            //终点线暂存标志_1
uint8 END_LINE_TEMP_2;            //终点线暂存标志_2
int16 end_line_row=-1;            //终点线所在行
int32 end_line_val=0;             //终点线所在距离
int16 end_line_2[2]={-1,-1};      //终点线所在行数组

uint8 L_TURN;                     //内倾锁死，判断为左转向标志
uint8 R_TURN;                     //内倾锁死，判断为右转向标志
int16 curve_in_row;               //弯道入口行
int16 curve_car_row;              //弯道前车行数

uint8 S_FLAG;                     //S型 赛道标志
int16 L_s_temp=0;                 //左边沿判断S型 赛道标志
int16 R_s_temp=0;                 //右边沿判断S型 赛道标志
int16 L_s_row;                    //左边沿判断S型 赛道行
int16 R_s_row;                    //右边沿判断S型 赛道行
int16 s_curve_row;                //S型 赛道行数
double s_down_dis;               //S型 赛道降前瞻距离

uint8 ramp_able=1;                //坡道使能检测标志
uint8 RAMP_FLAG;                  //坡道标志
uint8 RAMP_TEMP;                  //坡道暂存标志
int32 ramp_val=0;                 //坡道距离

uint8 STRAIGHT_FLAG;              //直道标志
uint8 little_s_flag;              //小S连续弯标志

double tan_table[30];
int16 inv_cross[3];               //上搜边沿判断十字

double inv_distance[CAMERA_H];    //逆透视获取每行实际距离数组
double bar_range_col[CAMERA_H];   //障碍物搜索范围数组
double car_col[CAMERA_H];         //车模路径识别数组

/*十字直角拐点数组*/
int16 L_down_cross[2];            //左边沿十字下方直角拐点数组（[0]为十字拐点行坐标值，[1]为十字拐点列坐标值）
int16 L_up_cross[2];              //左边沿十字上方直角拐点数组（[0]为十字拐点行坐标值，[1]为十字拐点列坐标值）

/*十字直角拐点数组*/
int16 R_down_cross[2];            //右边沿十字下方直角拐点数组（[0]为十字拐点行坐标值，[1]为十字拐点列坐标值）
int16 R_up_cross[2];              //右边沿十字上方直角拐点数组（[0]为十字拐点行坐标值，[1]为十字拐点列坐标值）

int16 left_edge[CAMERA_H];              //左边沿存放数组
int16 right_edge[CAMERA_H];             //右边沿存放数组

int16 left_down_edge[10];         //下降左边沿存放数组
int16 right_down_edge[10];        //下降右边沿存放数组


uint8 chao_direction[5]={1,1,1,1,1};

/***********************************************************************************************************************************/

void edge_extract();             //此函数为边沿提取主函数

void L_down_cross_judge();       //此函数为左边沿下方直角拐点判断函数（边沿转折时使用）
void L_up_cross_judge();         //此函数为左边沿上方直角拐点判断函数（边沿内倾突变时使用）
void L_up_cross_extract();       //此函数为左边沿上方直角拐点判断函数（找到下十字后向内搜索上十字）
void L_up_cross_error();         //此函数为左边沿上方直角拐点错误纠正函数

void R_down_cross_judge();       //此函数为右边沿下方直角拐点判断函数（边沿转折时使用）
void R_up_cross_judge();         //此函数为右边沿上方直角拐点判断函数（边沿内倾突变时使用）
void R_up_cross_extract();       //此函数为右边沿上方直角拐点判断函数（找到下十字后向内搜索上十字）
void R_up_cross_error();         //此函数为右边沿上方直角拐点错误纠正函数

void barrier_down_extract();     //此函数为障碍物下边沿行检测函数
void barrier_up_extract();       //此函数为障碍物上边沿行检测函数

void end_line_extract();         //此函数为终点线行检测函数

void cross_edge_add();           //此函数为十字边沿补充函数

void edge_error_process();       //此函数为边沿错误处理函数

void road_judge();               //此函数为赛道类型判断函数

void bar_process();              //此函数为障碍物边沿补充函数

void road_num();                 //此函数为赛道类型计数函数（十字，障碍物个数）

void loop_judge();               //此函数为环型赛道判断函数

void down_extract();             //此函数为下降边沿搜索函数

void bar_car();                  //此函数为超车判断函数

void tan_angle();                //此函数为斜率角度制表函数

void bar_range();                //此函数为每行障碍物搜索范围获取函数


/***********************************************************************************************************************************/

void edge_extract()       //此函数为边沿提取主函数
{
      edge_clean();        //清空当前边沿提取数组数据
      
      
      /*以下变量可能需要用于外部函数*/
      
      L_EDGE_NUM=0;                //给左边沿个数赋初值  0
      R_EDGE_NUM=0;                //给右边沿个数赋初值  0
      
      L_cross_flag=0;              //左边沿判断进入十字标志置  0
      R_cross_flag=0;              //右边沿判断进入十字标志置  0
      cross_flag=0;                //判断进入十字标志置  0
      L_cross_k=10;                //左边沿十字斜率置  10
      R_cross_k=10;                //右边沿十字斜率置  10
      
      L_search_end=0;              //左边沿搜索结束标志置  0
      R_search_end=0;              //右边沿搜索结束标志置  0
      L_end_row=-1;                //左边沿搜索结束行数置  -1
      R_end_row=-1;                //右边沿搜索结束行数置  -1
      
      L_down_end_row=-1;           //左边沿向下搜索结束行数置  -1
      R_down_end_row=-1;           //右边沿向下搜索结束行数置  -1
      
      L_barrier_flag=0;            //左边沿障碍标志置  0
      R_barrier_flag=0;            //右边沿障碍标志置  0
      L_barrier_down_row=-1;       //左边沿障碍物下边沿行置  -1
      L_barrier_up_row=-1;         //左边沿障碍物上边沿行置  -1
      R_barrier_down_row=-1;       //右边沿障碍物下边沿行置  -1
      R_barrier_up_row=-1;         //右边沿障碍物上边沿行置  -1
      car_extract_row=-1;          //检测前车距离行置  -1
      
      LOOP_FLAG=0;                 //环型赛道标志置  0
      
      END_LINE_FLAG=0;             //终点线标志置  0
      
      L_TURN=0;                    //左弯标志置  0
      R_TURN=0;                    //右弯标志置  0
      curve_in_row=-1;             //弯道入口行置  -1
      curve_car_row=-1;            //弯道前车行数  -1
      
      S_FLAG=0;                    //S型 赛道标志置  0
      s_curve_row=-1;              //S型 赛道行数置  0
      
      STRAIGHT_FLAG=0;             //直道标志置  0
      little_s_flag=0;             //小S连续弯标志置  0
      RAMP_FLAG=0;                 //坡道标志置  0
      
      
      /*以下变量不需要用于外部函数*/
      
      L_LINK_NUM=0;        //给连续左边沿个数赋初值  0
      R_LINK_NUM=0;        //给连续右边沿个数赋初值  0
      
      L_get_flag=0;        //左边沿搜索得到标志置  0
      L_lost_num=1;        //左边沿连续丢失个数置  1（先定义为丢失边沿）
      
      R_get_flag=0;        //右边沿搜索得到标志置  0
      R_lost_num=1;        //左边沿连续丢失个数置  1（先定义为丢失边沿）
      
      L_trend_in=0;        //左边沿内倾趋势判断标志置  0
      R_trend_in=0;        //右边沿内倾趋势判断标志置  0
      
      if( LOOP_TEMP )
      {
            if( out_row[0]!=-1 )
            {
                  double l_out;
                  Site_xy1 xy1;
                  l_out=1.0*(loop_val-abs(speedout_val))/25;
                  xy1=get_invinv_img(l_out,0);
                  LOOP_OUT_ROW=xy1.x;
            }
      }
      
      end_bar_able=1;
      
      if( overtake_cross_flag )
      {
            if(car_L_pull_over_flag && !car_R_pull_over_flag)   //判断为左侧靠边
            {
                  l_cross_able=1;
                  r_cross_able=0;
            }
            else
            {
                  if(car_R_pull_over_flag && !car_L_pull_over_flag)   //判断为右侧靠边
                  {
                        l_cross_able=0;
                        r_cross_able=1;
                  }
                  else
                  {
                        l_cross_able=1;
                        r_cross_able=1;
                  }
            }
      }
      else
      {
            l_cross_able=1;
            r_cross_able=1;
      }
      int16 l_cross_edge=0,r_cross_edge=0,l_cross_1=0,r_cross_1=0;
      
      for( EXTRACT_ROW=ROW_END ; EXTRACT_ROW>=ROW_START ; EXTRACT_ROW-- )       //检测整幅图像的边沿
      {
            L_get_flag=0;               //对左边沿搜索得到标志清0
            R_get_flag=0;               //对右边沿搜索得到标志清0
            
            if( LOOP_TEMP )  //环道十字使能
            {
                  if( LOOP_IN )
                  {
                        if( out_row[0]!=-1 )
                        {
                              if( inv_distance[EXTRACT_ROW]<inv_distance[LOOP_IN_ROW]+60 )
                              {
                                    l_cross_able=0;
                                    r_cross_able=0;
                              }
                              else
                              {
                                    if( loop_l_r_flag==1 )
                                    {
                                          l_cross_able=0;
                                          r_cross_able=1;
                                    }
                                    else
                                    {
                                          if( loop_l_r_flag==0 )
                                          {
                                                l_cross_able=1;
                                                r_cross_able=0;
                                          }
                                    }
                              }
                        }
                        else
                        {
                              l_cross_able=0;
                              r_cross_able=0;
                        }
                  }
                  else
                  {
                        if( inv_distance[EXTRACT_ROW]>inv_distance[LOOP_IN_ROW]-60 )
                        {
                              l_cross_able=0;
                              r_cross_able=0;
                        }
                  }
            }
            
            if( END_LINE_TEMP==1 )  //终点线障碍使能
            {
                  if( end_line_row!=-1 )
                  {
                        if( inv_distance[EXTRACT_ROW]>inv_distance[end_line_row]+60 )
                        {
                              end_bar_able=1;
                              l_cross_able=1;
                              r_cross_able=1;
                        }
                        else
                        {
                              end_bar_able=0;
                              l_cross_able=0;
                              r_cross_able=0;
                        }
                  }
                  else
                  {
                        if( inv_distance[EXTRACT_ROW]>100 )
                        {
                              end_bar_able=1;
                              l_cross_able=1;
                              r_cross_able=1;
                        }
                        else
                        {
                              end_bar_able=0;
                              l_cross_able=0;
                              r_cross_able=0;
                        }
                  }
            }
            
            /*以下程序段为左边沿检测程序*/
            
            if( L_search_end==0 )       //当左边沿搜索未结束时进入
            {
                  if( L_lost_num==0 )         //当左边沿没有丢失时进入处理
                  {
                        if( L_LINK_NUM>=3 )   //当左边沿连续数大于等于3，判断出边沿趋势时进入处理
                        {
                              if( L_trend_in==1 )    //当左边沿内倾时进入处理
                              {
                                    if( img_data[EXTRACT_ROW][L_edge_nearest]==DOT_B )     //当上一行左边沿点的上方一点为黑
                                    {
                                          for( EXTRACT_COL=L_edge_nearest+1 ; EXTRACT_COL<=L_edge_nearest+L_IN_2_IN ; EXTRACT_COL++ )      //左边沿内倾向内搜索
                                          {
                                                if( EXTRACT_COL>COL_END )    //当搜索超过图像范围时，结束搜索
                                                {
                                                      break;
                                                }
                                                if( img_data[EXTRACT_ROW][EXTRACT_COL] == DOT_W )  //找到黑白跳变点
                                                {
                                                      left_edge[EXTRACT_ROW]=EXTRACT_COL-1;        //储存本行左边沿列坐标
                                                      
                                                      L_EDGE_NUM++;                                //左边沿个数加一
                                                      
                                                      L_LINK_NUM++;                                //连续左边沿个数加一
                                                      
                                                      L_get_flag=1;                                //左边沿搜索得到标志置  1
                                                      
                                                      L_edge_nearest=EXTRACT_COL-1;                //存储最近一个左边沿的列坐标
                                                      
                                                      L_up_cross_judge();                          //判断是否为上十字直角拐点
                                                      
                                                      break;                                      //本行左边沿搜索结束
                                                }
                                          }
                                          
                                          if( inv_distance[EXTRACT_ROW]>120 && left_edge[EXTRACT_ROW]!=-1 && left_edge[EXTRACT_ROW+1]!=-1 )
                                          {
                                                if( left_edge[EXTRACT_ROW]-left_edge[EXTRACT_ROW+1]>=40 )
                                                {
                                                      W_num=0;
                                                      for( EXTRACT_COL=left_edge[EXTRACT_ROW+1]+1 ; EXTRACT_COL<=left_edge[EXTRACT_ROW] ; EXTRACT_COL++ )      //左边沿内倾向内搜索
                                                      {
                                                            if( img_data[EXTRACT_ROW+1][EXTRACT_COL] == DOT_B )
                                                            {
                                                                  W_num++;
                                                            }
                                                      }
                                                      if( W_num>=0.5*abs(left_edge[EXTRACT_ROW]-left_edge[EXTRACT_ROW+1]) )
                                                      {
                                                            left_edge[EXTRACT_ROW]=-1;
                                                            L_EDGE_NUM--;
                                                            L_LINK_NUM--;
                                                            L_get_flag=0;
                                                      }
                                                      
                                                }
                                          }
                                          
                                          if( L_get_flag==0 )      //当左边沿内倾向内未搜到边沿
                                          {
                                                if( L_down_cross[0]!=-1 && L_up_cross[0]!=-1 )
                                                {
                                                      W_num=0;
                                                      for( W_num_row=EXTRACT_ROW+1 ; W_num_row<=EXTRACT_ROW+15 ; W_num_row++ )      //右边沿内倾向内搜索
                                                      {
                                                            if( W_num_row>=ROW_END )
                                                            {
                                                                  break;
                                                            }
                                                            if( left_edge[W_num_row]<left_edge[W_num_row+1] )
                                                            {
                                                                  W_num++;
                                                            }
                                                      }
                                                      if( W_num<=1 )
                                                      {
                                                            W_num=0;
                                                            for( W_num_row=EXTRACT_ROW+1 ; W_num_row<=EXTRACT_ROW+15 ; W_num_row++ )      //右边沿内倾向内搜索
                                                            {
                                                                  if( W_num_row>=ROW_END )
                                                                  {
                                                                        break;
                                                                  }
                                                                  if( left_edge[W_num_row]<left_edge[W_num_row+1] )
                                                                  {
                                                                        W_num++;
                                                                  }
                                                            }
                                                            if( W_num<=1 )
                                                            {
                                                                  R_TURN=1;                   //右弯标志置  1
                                                                  
                                                                  L_end_row=EXTRACT_ROW+1;    //将上一行的行数值赋给左边沿结束行数值
                                                                  
                                                                  L_search_end=1;             //当左边沿右倾时，向内搜索不到白点，判断为左边沿搜索结束
                                                                  R_search_end=1;             //当左边沿右倾时，向内搜索不到白点，判断为右边沿搜索结束
                                                            }
                                                      }
                                                }
                                                else
                                                {
                                                      if( L_LINK_NUM>=10 )
                                                      {
                                                            W_num=0;
                                                            for( W_num_row=EXTRACT_ROW+1 ; W_num_row<=EXTRACT_ROW+10 ; W_num_row++ )      //右边沿内倾向内搜索
                                                            {
                                                                  if( W_num_row>=ROW_END )
                                                                  {
                                                                        break;
                                                                  }
                                                                  if( left_edge[W_num_row]<left_edge[W_num_row+1] )
                                                                  {
                                                                        W_num++;
                                                                  }
                                                            }
                                                            if( W_num<=1 )
                                                            {
                                                                  R_TURN=1;                   //右弯标志置  1
                                                                  
                                                                  L_end_row=EXTRACT_ROW+1;    //将上一行的行数值赋给左边沿结束行数值
                                                                  
                                                                  L_search_end=1;             //当左边沿右倾时，向内搜索不到白点，判断为左边沿搜索结束
                                                                  R_search_end=1;             //当左边沿右倾时，向内搜索不到白点，判断为右边沿搜索结束
                                                            }
                                                      }
                                                }
                                                
                                                L_LINK_NUM=0;               //连续左边沿个数置  0
                                                
                                                L_lost_num++;               //当没有搜索到左边沿点时，左边沿丢失数加一
                                          }
                                    }
                                    else                                                   //当上一行左边沿点的上方一点为白
                                    {
                                          for( EXTRACT_COL=L_edge_nearest-1 ; EXTRACT_COL>=L_edge_nearest-L_IN_2_OUT ; EXTRACT_COL-- )      //左边沿内倾向外搜索
                                          {
                                                if( EXTRACT_COL<COL_START )    //当搜索超过图像范围时，结束搜索
                                                {
                                                      break;
                                                }
                                                if( img_data[EXTRACT_ROW][EXTRACT_COL] == DOT_B )
                                                {
                                                      left_edge[EXTRACT_ROW]=EXTRACT_COL;          //储存本行左边沿列坐标
                                                      
                                                      L_EDGE_NUM++;                                //左边沿个数加一
                                                      
                                                      L_LINK_NUM++;                                //连续左边沿个数加一
                                                      
                                                      L_get_flag=1;                                //左边沿搜索得到标志置  1
                                                      
                                                      L_edge_nearest=EXTRACT_COL;                  //存储最近一个左边沿的列坐标
                                                      
                                                      L_trend_in=0;                                //将左边沿内倾标志置  0
                                                      
                                                      L_down_cross_judge();                        //判断是否为下十字直角拐点
                                                      
                                                      break;                                      //本行左边沿搜索结束
                                                }
                                          }
                                          if( L_get_flag==0 )      //当左边沿内倾向外未搜到边沿
                                          {
                                                L_LINK_NUM=0;           //连续左边沿个数置  0
                                                
                                                L_lost_num++;           //当没有搜索到左边沿点时，左边沿丢失数加一
                                                
                                                L_down_cross_judge();   //判断是否为下十字直角拐点
                                          }
                                    }
                              }
                              else                   //当左边沿外倾时进入处理
                              {
                                    if( img_data[EXTRACT_ROW][L_edge_nearest+1]==DOT_B )     //当上一行左边沿点的上方一点为黑
                                    {
                                          for( EXTRACT_COL=L_edge_nearest+1 ; EXTRACT_COL<=L_edge_nearest+L_OUT_2_IN ; EXTRACT_COL++ )      //左边沿外倾向内搜索
                                          {
                                                if( EXTRACT_COL>COL_END )    //当搜索超过图像范围时，结束搜索
                                                {
                                                      break;
                                                }
                                                if( img_data[EXTRACT_ROW][EXTRACT_COL] == DOT_W )  //找到黑白跳变点
                                                {
                                                      left_edge[EXTRACT_ROW]=EXTRACT_COL-1;        //储存本行左边沿列坐标
                                                      
                                                      L_EDGE_NUM++;                                //左边沿个数加一
                                                      
                                                      L_LINK_NUM++;                                //连续左边沿个数加一
                                                      
                                                      L_get_flag=1;                                //左边沿搜索得到标志置  1
                                                      
                                                      L_edge_nearest=EXTRACT_COL-1;                //存储最近一个左边沿的列坐标
                                                      
                                                      L_trend_in=1;                                //左边沿内倾趋势标志置  1
                                                      
                                                      break;                                      //本行左边沿搜索结束
                                                }
                                          }
                                          if( L_get_flag==0 )      //当左边沿外倾向右未搜到边沿
                                          {
                                                if( L_LINK_NUM>=10 && L_LINK_NUM>R_LINK_NUM )
                                                {
                                                      L_end_row=EXTRACT_ROW+1;    //将上一行的行数值赋给左边沿结束行数值
                                                      
                                                      L_search_end=1;             //当左边沿外倾时，向内搜索不到白点，判断为左边沿搜索结束
                                                      R_search_end=1;             //当左边沿外倾时，向内搜索不到白点，判断为右边沿搜索结束
                                                }
                                                
                                                L_LINK_NUM=0;               //连续左边沿个数置  0
                                                
                                                L_lost_num++;               //当没有搜索到左边沿点时，左边沿丢失数加一
                                          }
                                    }
                                    else          //当上一行左边沿点的上方一点为白
                                    {
                                          if( L_cross_flag==1 )    //当搜索得到左边沿上十字
                                          {
                                                for( EXTRACT_COL=L_edge_nearest ; EXTRACT_COL>=L_edge_nearest-L_OUT_2_OUT ; EXTRACT_COL-- )     //开始向外搜索
                                                {
                                                      if( EXTRACT_COL<COL_START )    //当搜索超过图像范围时，结束搜索
                                                      {
                                                            break;
                                                      }
                                                      if( img_data[EXTRACT_ROW][EXTRACT_COL] == DOT_B )
                                                      {
                                                            left_edge[EXTRACT_ROW]=EXTRACT_COL;          //储存本行左边沿列坐标
                                                            
                                                            L_EDGE_NUM++;                                //左边沿个数加一
                                                            
                                                            L_LINK_NUM++;                                //连续左边沿个数加一
                                                            
                                                            L_get_flag=1;                                //左边沿搜索得到标志置  1
                                                            
                                                            L_edge_nearest=EXTRACT_COL;                  //存储最近一个左边沿的列坐标
                                                            
                                                            break;                                      //本行左边沿搜索结束
                                                      }
                                                }
                                                if( L_get_flag==0 )
                                                {
                                                      L_LINK_NUM=0;                                //连续左边沿个数置  0
                                                      
                                                      L_lost_num++;                                //当左边沿外倾时，向外搜索不到白点，左边沿丢失数加一
                                                }
                                          }
                                          else                     //当未搜索得到左边沿上十字
                                          {
                                                for( EXTRACT_COL=L_edge_nearest-1 ; EXTRACT_COL>=L_edge_nearest-L_OUT_2_OUT ; EXTRACT_COL-- )     //开始向外搜索
                                                {
                                                      if( EXTRACT_COL<COL_START )  //出界停止
                                                      {
                                                            break;
                                                      }
                                                      if( img_data[EXTRACT_ROW][EXTRACT_COL] == DOT_B )
                                                      {
                                                            W_num=0;            //清除白块计数值
                                                            for( W_num_col=L_edge_nearest ; W_num_col>=EXTRACT_COL ; W_num_col-- )     //在搜索到的黑白跳变点后，在上一行搜索白点，判断是否为另一边点
                                                            {
                                                                  if( img_data[EXTRACT_ROW+1][W_num_col] == DOT_W )
                                                                  {
                                                                        W_num++;
                                                                  }
                                                            }
                                                            if( W_num>COL_W_NUM && L_barrier_flag==0 )             //当白块计数值超过阀值，则判断为左边沿丢失             /*这里的 W_num 的阀值需要测试*/
                                                            {
                                                                  L_search_end=1;             //当左边沿左倾并且左边沿十字标志为0时，向左搜索异常，判断为左边沿搜索结束
                                                                  L_end_row=EXTRACT_ROW+1;    //将上一行的行数值赋给左边沿结束行数值
                                                                  
                                                                  break;
                                                            }
                                                            
                                                            left_edge[EXTRACT_ROW]=EXTRACT_COL;          //储存本行左边沿列坐标
                                                            
                                                            L_EDGE_NUM++;                                //左边沿个数加一
                                                            
                                                            L_LINK_NUM++;                                //连续左边沿个数加一
                                                            
                                                            L_get_flag=1;                                //左边沿搜索得到标志置  1
                                                            
                                                            L_edge_nearest=EXTRACT_COL;                  //存储最近一个左边沿的列坐标
                                                            
                                                            break;                                      //本行左边沿搜索结束
                                                      }
                                                }
                                                if( L_get_flag==0 )
                                                {
                                                      L_LINK_NUM=0;                                //连续左边沿个数置  0
                                                      
                                                      L_lost_num++;                                //当左边沿左倾时，向左搜索不到白点，左边沿丢失数加一
                                                }
                                          }
                                    }
                              }
                              
                              if( L_get_flag==1 )         //当搜索到边沿
                              {
                                    L_up_cross_extract();       //向内搜索上十字直角拐点
                              }
                        }
                        else                  //当左边沿没有判断出边沿趋势时进入处理
                        {
                              if( img_data[EXTRACT_ROW][L_edge_nearest] == DOT_B )     //当上一行左边沿点的上方一点为黑
                              {
                                    for( EXTRACT_COL=L_edge_nearest+1 ; EXTRACT_COL<=L_edge_nearest+L_EDGE_IN ; EXTRACT_COL++ )            //开始向右搜索
                                    {
                                          if( EXTRACT_COL>COL_END-4 )    //当搜索超过图像范围时，结束搜索
                                          {
                                                break;
                                          }
                                          if( img_data[EXTRACT_ROW][EXTRACT_COL] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL+2] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL+4] == DOT_W )   //找到黑白跳变点
                                          {
                                                left_edge[EXTRACT_ROW]=EXTRACT_COL-1;          //储存本行左边沿列坐标
                                                
                                                L_EDGE_NUM++;                                //左边沿个数加一
                                                
                                                L_LINK_NUM++;                                //连续左边沿个数加一
                                                
                                                L_get_flag=1;                                //左边沿搜索得到标志置  1
                                                
                                                L_edge_nearest=EXTRACT_COL-1;                  //存储最近一个左边沿的列坐标
                                                
                                                L_up_cross_judge();                    //如果左边沿十字标志任为1，进行十字直角拐点判断
                                                
                                                break;                                      //本行左边沿搜索结束
                                          }
                                    }
                                    if( L_get_flag==0 )
                                    {
                                          L_LINK_NUM=0;               //连续左边沿个数置  0
                                          
                                          L_lost_num++;               //当没有搜索到左边沿点时，左边沿丢失数加一
                                    }
                              }
                              else     //当上一行左边沿点的上方一点为白
                              {
                                    if(  img_data[EXTRACT_ROW][L_edge_nearest+1] == DOT_W && img_data[EXTRACT_ROW][L_edge_nearest+2] == DOT_W )        //之后两点也为白
                                    {
                                          for( EXTRACT_COL=L_edge_nearest-1 ; EXTRACT_COL>=L_edge_nearest-L_EDGE_OUT ; EXTRACT_COL-- )           //开始向左搜索黑白跳变点
                                          {
                                                if( EXTRACT_COL<COL_START+2 )    //当搜索超过图像范围时，结束搜索
                                                {
                                                      break;
                                                }
                                                if( img_data[EXTRACT_ROW][EXTRACT_COL] == DOT_B && img_data[EXTRACT_ROW][EXTRACT_COL-1] == DOT_B && img_data[EXTRACT_ROW][EXTRACT_COL-2] == DOT_B )   //找到黑白跳变点
                                                {
                                                      left_edge[EXTRACT_ROW]=EXTRACT_COL;          //储存本行左边沿列坐标
                                                      
                                                      L_EDGE_NUM++;                                //左边沿个数加一
                                                      
                                                      L_LINK_NUM++;                                //连续左边沿个数加一
                                                      
                                                      L_get_flag=1;                                //左边沿搜索得到标志置  1
                                                      
                                                      L_edge_nearest=EXTRACT_COL;                  //存储最近一个左边沿的列坐标
                                                      
                                                      break;                                      //本行左边沿搜索结束
                                                }
                                          }
                                          if( L_get_flag==0 )
                                          {
                                                L_LINK_NUM=0;       //连续左边沿个数置  0
                                                
                                                L_lost_num++;       //当没有搜索到左边沿点时，左边沿丢失数加一
                                          }
                                    }
                                    else           //之后两点有黑点,则判断为干扰
                                    {
                                          for( EXTRACT_COL=L_edge_nearest+1 ; EXTRACT_COL<=L_edge_nearest+L_EDGE_IN ; EXTRACT_COL++ )            //开始向右搜索黑白跳变点
                                          {
                                                if( EXTRACT_COL>COL_END-5 )    //当搜索超过图像范围时，结束搜索
                                                {
                                                      break;
                                                }
                                                if( img_data[EXTRACT_ROW][EXTRACT_COL] == DOT_B && img_data[EXTRACT_ROW][EXTRACT_COL+1] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL+3] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL+5] == DOT_W )   //找到黑白跳变点
                                                {
                                                      left_edge[EXTRACT_ROW]=EXTRACT_COL;          //储存本行左边沿列坐标
                                                      
                                                      L_EDGE_NUM++;                                //左边沿个数加一
                                                      
                                                      L_LINK_NUM++;                                //连续左边沿个数加一
                                                      
                                                      L_get_flag=1;                                //左边沿搜索得到标志置  1
                                                      
                                                      L_edge_nearest=EXTRACT_COL;                  //存储最近一个左边沿的列坐标
                                                      
                                                      break;                                      //本行左边沿搜索结束
                                                }
                                          }
                                          if( L_get_flag==0 )
                                          {
                                                L_LINK_NUM=0;               //连续左边沿个数置  0
                                                
                                                L_lost_num++;               //当没有搜索到左边沿点时，左边沿丢失数加一
                                          }
                                    }
                              }
                              
                              if( L_LINK_NUM>=3 )  //当搜索完这一行后，连续边沿数大于等于3
                              {
                                    if(  left_edge[EXTRACT_ROW]>= left_edge[EXTRACT_ROW+1] || left_edge[EXTRACT_ROW]>= left_edge[EXTRACT_ROW+2] )
                                    {
                                          L_trend_in=1;     //满足条件，左边沿内倾趋势置  1
                                    }
                                    else
                                    {
                                          L_trend_in=0;     //不满足条件，左边沿内倾趋势置  0
                                    }
                              }
                        }
                  }
                  else                        //当左边沿丢失时进入处理
                  {
                        if( L_cross_flag==1 ) //当左边沿十字标志为1时
                        {
                              if( L_down_cross[0]!=-1 )    //当检测到左边沿下方十字直角拐点
                              {
                                    if( L_down_cross[1]-CROSS_ERTRACT>COL_START )
                                    {
                                          W_num_col=L_down_cross[1]-CROSS_ERTRACT;
                                    }
                                    else
                                    {
                                          W_num_col=COL_START;
                                    }
                                    for( EXTRACT_COL=W_num_col ; EXTRACT_COL<=L_down_cross[1]+80 ; EXTRACT_COL++ )    //开始搜索
                                    {
                                          if( EXTRACT_COL>COL_END-5 )   //超过图像范围停止
                                          {
                                                break;
                                          }
                                          if( img_data[EXTRACT_ROW][EXTRACT_COL] == DOT_B && img_data[EXTRACT_ROW][EXTRACT_COL+1] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL+3] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL+5] == DOT_W )      //当搜索到黑白跳变点
                                          {
                                                W_num=0;            //清除白块计数值
                                                for( W_num_col=EXTRACT_ROW-1 ; W_num_col>=EXTRACT_ROW-ROW_W_SUM ; W_num_col-- )     //在搜索到的黑白跳变点后，向上搜索白点，判断是否为干扰点
                                                {
                                                      if( W_num_col<ROW_START )   //超过图像范围停止
                                                      {
                                                            break;
                                                      }
                                                      if( img_data[W_num_col][EXTRACT_COL] == DOT_W )
                                                      {
                                                            W_num++;
                                                      }
                                                }
                                                if( W_num<ROW_W_NUM )              //当白块计数值没超过阀值，则判断为左边沿点          /*这里的 W_num 的阀值需要测试*/
                                                {
                                                      L_up_cross_error();                          //左边沿上方直角拐点错误纠正
                                                      
                                                      if( L_get_flag==1 )
                                                      {
                                                            left_edge[EXTRACT_ROW]=EXTRACT_COL;          //储存本行左边沿列坐标
                                                            
                                                            L_EDGE_NUM++;                                //左边沿个数加一
                                                            
                                                            L_LINK_NUM++;                                //连续左边沿个数加一
                                                            
                                                            L_edge_nearest=EXTRACT_COL;                  //存储最近一个左边沿的列坐标
                                                            
                                                            L_lost_num=0;                                //连续左边沿丢失数清0
                                                            
                                                            if( cross_flag==0 && L_up_cross[0]!=-1 )
                                                            {
                                                                  cross_flag=1;                    //十字标志置  1
                                                            }
                                                      }
                                                      
                                                      break;                                      //本行左边沿搜索结束
                                                }
                                          }
                                    }
                                    if( L_get_flag==0 )
                                    {
                                          L_lost_num++;                          //连续左边沿丢失数加1
                                    }
                              }
                              else                         //当未检测到左边沿下方十字直角拐点
                              {
                                    for( EXTRACT_COL=COL_START ; EXTRACT_COL<=L_LOST ; EXTRACT_COL++ )    //开始搜索
                                    {
                                          if( EXTRACT_COL>COL_END-5 )   //超过图像范围停止
                                          {
                                                break;
                                          }
                                          if( img_data[EXTRACT_ROW][EXTRACT_COL] == DOT_B && img_data[EXTRACT_ROW][EXTRACT_COL+1] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL+3] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL+5] == DOT_W )      //当搜索到黑白跳变点
                                          {
                                                L_up_cross_error();                          //左边沿上方直角拐点错误纠正
                                                
                                                if( L_get_flag==1 )
                                                {
                                                      left_edge[EXTRACT_ROW]=EXTRACT_COL;          //储存本行左边沿列坐标
                                                      
                                                      L_EDGE_NUM++;                                //左边沿个数加一
                                                      
                                                      L_LINK_NUM++;                                //连续左边沿个数加一
                                                      
                                                      L_edge_nearest=EXTRACT_COL;                  //存储最近一个左边沿的列坐标
                                                      
                                                      L_lost_num=0;                                //连续左边沿丢失数清0
                                                      
                                                      if( cross_flag==0 && L_up_cross[0]!=-1 )
                                                      {
                                                            cross_flag=1;                    //十字标志置  1
                                                      }
                                                }
                                                
                                                break;                                                  //本行左边沿搜索结束
                                          }
                                    }
                                    if( L_get_flag==0 )
                                    {
                                          L_lost_num++;                          //连续左边沿丢失数加1
                                    }
                              }
                        }
                        else                  //当左边沿十字标志为0时
                        {
                              if( img_data[EXTRACT_ROW][COL_START] == DOT_W && img_data[EXTRACT_ROW][COL_START+1] == DOT_W && img_data[EXTRACT_ROW][COL_START+2] == DOT_W )    //当前三个点为白
                              {
                                    if( LOOP_TEMP==0 )    //当不处于环道
                                    {
                                          for( EXTRACT_COL=COL_START+3 ; EXTRACT_COL<L_LOST ; EXTRACT_COL++ )    //开始搜索
                                          {
                                                if( img_data[EXTRACT_ROW][EXTRACT_COL] == DOT_B && img_data[EXTRACT_ROW][EXTRACT_COL+1] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL+3] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL+5] == DOT_W )      //当搜索到黑白跳变点
                                                {
                                                      if( EXTRACT_COL>20 && EXTRACT_ROW>40 )      //当搜索到错误边沿
                                                      {
                                                            if( img_data[EXTRACT_ROW][EXTRACT_COL-3] == DOT_W || img_data[EXTRACT_ROW][EXTRACT_COL-6] == DOT_W || img_data[EXTRACT_ROW][EXTRACT_COL-9] == DOT_W || img_data[EXTRACT_ROW][EXTRACT_COL-12] == DOT_W || img_data[EXTRACT_ROW][EXTRACT_COL-15] == DOT_W )
                                                            {
                                                                  continue;
                                                            }
                                                      }
                                                      W_num=0;            //清除白块计数值
                                                      for( W_num_col=EXTRACT_ROW-1 ; W_num_col>=EXTRACT_ROW-ROW_W_SUM ; W_num_col-- )     //在搜索到的黑白跳变点后，向上搜索白点，判断是否为干扰点
                                                      {
                                                            if( W_num_col<ROW_START )   //超过图像范围停止
                                                            {
                                                                  break;
                                                            }
                                                            if( img_data[W_num_col][EXTRACT_COL] == DOT_W )
                                                            {
                                                                  W_num++;
                                                            }
                                                      }
                                                      if( W_num<ROW_W_NUM )
                                                      {
                                                            left_edge[EXTRACT_ROW]=EXTRACT_COL;          //储存本行左边沿列坐标
                                                            
                                                            L_EDGE_NUM++;                                //左边沿个数加一
                                                            
                                                            L_LINK_NUM++;                                //连续左边沿个数加一
                                                            
                                                            L_get_flag=1;                                //左边沿搜索得到标志置  1
                                                            
                                                            L_edge_nearest=EXTRACT_COL;                  //存储最近一个左边沿的列坐标
                                                            
                                                            L_lost_num=0;                                //连续左边沿丢失数清0
                                                            
                                                            break;                                      //本行左边沿搜索结束
                                                      }
                                                      
                                                }
                                          }
                                          if( L_get_flag==0 )
                                          {
                                                L_lost_num++;             //当左边沿丢失时，向右搜索不到黑白跳变点，判断为左边沿丢失
                                          }
                                    }
                                    else    //当处于环道
                                    {
                                          if( loop_l_r_flag==0 )
                                          {
                                                for( EXTRACT_COL=COL_START+3 ; EXTRACT_COL<L_LOST ; EXTRACT_COL++ )    //开始搜索
                                                {
                                                      if( img_data[EXTRACT_ROW][EXTRACT_COL] == DOT_B && img_data[EXTRACT_ROW][EXTRACT_COL+1] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL+3] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL+5] == DOT_W )      //当搜索到黑白跳变点
                                                      {
                                                            if( EXTRACT_COL>20 && EXTRACT_ROW>40 )      //当搜索到错误边沿
                                                            {
                                                                  if( img_data[EXTRACT_ROW][EXTRACT_COL-3] == DOT_W || img_data[EXTRACT_ROW][EXTRACT_COL-6] == DOT_W || img_data[EXTRACT_ROW][EXTRACT_COL-9] == DOT_W || img_data[EXTRACT_ROW][EXTRACT_COL-12] == DOT_W || img_data[EXTRACT_ROW][EXTRACT_COL-15] == DOT_W )
                                                                  {
                                                                        continue;
                                                                  }
                                                            }
                                                            W_num=0;            //清除白块计数值
                                                            for( W_num_col=EXTRACT_ROW-1 ; W_num_col>=EXTRACT_ROW-ROW_W_SUM ; W_num_col-- )     //在搜索到的黑白跳变点后，向上搜索白点，判断是否为干扰点
                                                            {
                                                                  if( W_num_col<ROW_START )   //超过图像范围停止
                                                                  {
                                                                        break;
                                                                  }
                                                                  if( img_data[W_num_col][EXTRACT_COL] == DOT_W )
                                                                  {
                                                                        W_num++;
                                                                  }
                                                            }
                                                            if( W_num<ROW_W_NUM )
                                                            {
                                                                  left_edge[EXTRACT_ROW]=EXTRACT_COL;          //储存本行左边沿列坐标
                                                                  
                                                                  L_EDGE_NUM++;                                //左边沿个数加一
                                                                  
                                                                  L_LINK_NUM++;                                //连续左边沿个数加一
                                                                  
                                                                  L_get_flag=1;                                //左边沿搜索得到标志置  1
                                                                  
                                                                  L_edge_nearest=EXTRACT_COL;                  //存储最近一个左边沿的列坐标
                                                                  
                                                                  L_lost_num=0;                                //连续左边沿丢失数清0
                                                                  
                                                                  break;                                      //本行左边沿搜索结束
                                                            }
                                                            
                                                      }
                                                }
                                                if( L_get_flag==0 )
                                                {
                                                      L_lost_num++;             //当左边沿丢失时，向右搜索不到黑白跳变点，判断为左边沿丢失
                                                }
                                          }
                                          else
                                          {
                                                L_lost_num++;             //当左边沿丢失时，向右搜索不到黑白跳变点，判断为左边沿丢失
                                          }
                                    }
                              }
                              else         //当前三个点不全为白
                              {
                                    if( LOOP_TEMP==1 && loop_l_r_flag==1 )
                                    {
                                          if( L_EDGE_NUM>=3 || EXTRACT_ROW<40 )
                                          {
                                                for( EXTRACT_COL=COL_START ; EXTRACT_COL<=L_LOOP ; EXTRACT_COL++ )    //开始搜索
                                                {
                                                      if( img_data[EXTRACT_ROW][EXTRACT_COL] == DOT_B && img_data[EXTRACT_ROW][EXTRACT_COL+1] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL+3] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL+5] == DOT_W )      //当搜索到黑白跳变点
                                                      {
                                                            left_edge[EXTRACT_ROW]=EXTRACT_COL;        //储存本行左边沿列坐标
                                                            
                                                            L_EDGE_NUM++;                                //左边沿个数加一
                                                            
                                                            L_LINK_NUM++;                                //连续左边沿个数加一
                                                            
                                                            L_get_flag=1;                                //左边沿搜索得到标志置  1
                                                            
                                                            L_edge_nearest=EXTRACT_COL;                //存储最近一个左边沿的列坐标
                                                            
                                                            L_lost_num=0;                                //连续左边沿丢失数清0
                                                            
                                                            break;                                      //本行左边沿搜索结束
                                                      }
                                                }
                                                if( L_get_flag==0 )
                                                {
                                                      L_lost_num++;             //当左边沿丢失时，向右搜索不到黑白跳变点，判断为左边沿丢失
                                                }
                                          }
                                          else
                                          {
                                                for( EXTRACT_COL=COL_START ; EXTRACT_COL<=L_LOST ; EXTRACT_COL++ )    //开始搜索
                                                {
                                                      if( img_data[EXTRACT_ROW][EXTRACT_COL] == DOT_B && img_data[EXTRACT_ROW][EXTRACT_COL+1] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL+3] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL+5] == DOT_W )      //当搜索到黑白跳变点
                                                      {
                                                            left_edge[EXTRACT_ROW]=EXTRACT_COL;        //储存本行左边沿列坐标
                                                            
                                                            L_EDGE_NUM++;                                //左边沿个数加一
                                                            
                                                            L_LINK_NUM++;                                //连续左边沿个数加一
                                                            
                                                            L_get_flag=1;                                //左边沿搜索得到标志置  1
                                                            
                                                            L_edge_nearest=EXTRACT_COL;                //存储最近一个左边沿的列坐标
                                                            
                                                            L_lost_num=0;                                //连续左边沿丢失数清0
                                                            
                                                            break;                                      //本行左边沿搜索结束
                                                      }
                                                }
                                                if( L_get_flag==0 )
                                                {
                                                      L_lost_num++;             //当左边沿丢失时，向右搜索不到黑白跳变点，判断为左边沿丢失
                                                }
                                          }
                                    }
                                    else
                                    {
                                          for( EXTRACT_COL=COL_START ; EXTRACT_COL<=L_LOST ; EXTRACT_COL++ )    //开始搜索
                                          {
                                                if( img_data[EXTRACT_ROW][EXTRACT_COL] == DOT_B && img_data[EXTRACT_ROW][EXTRACT_COL+1] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL+3] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL+5] == DOT_W )      //当搜索到黑白跳变点
                                                {
                                                      left_edge[EXTRACT_ROW]=EXTRACT_COL;        //储存本行左边沿列坐标
                                                      
                                                      L_EDGE_NUM++;                                //左边沿个数加一
                                                      
                                                      L_LINK_NUM++;                                //连续左边沿个数加一
                                                      
                                                      L_get_flag=1;                                //左边沿搜索得到标志置  1
                                                      
                                                      L_edge_nearest=EXTRACT_COL;                //存储最近一个左边沿的列坐标
                                                      
                                                      L_lost_num=0;                                //连续左边沿丢失数清0
                                                      
                                                      break;                                      //本行左边沿搜索结束
                                                }
                                          }
                                          if( L_get_flag==0 )
                                          {
                                                L_lost_num++;             //当左边沿丢失时，向右搜索不到黑白跳变点，判断为左边沿丢失
                                          }
                                    }
                              }
                        }
                  }
            }
            else
            {
                  if( L_TURN==0 && R_TURN==0 )
                  {
                        if( img_data[EXTRACT_ROW][COL_START] == DOT_W && img_data[EXTRACT_ROW][COL_START+2] == DOT_W && img_data[EXTRACT_ROW][COL_START+4] == DOT_W )    //当前三个点为白
                        {
                              L_lost_num++;             //判断为左边沿丢失
                        }
                  }
            }
            
            
            
            /*以下程序段为右边沿检测程序*/
            
            if( R_search_end==0 )       //当右边沿搜索未结束时进入
            {
                  if( R_lost_num==0 )         //当右边沿没有丢失时进入处理
                  {
                        if( R_LINK_NUM>=3 )   //当右边沿连续数大于等于3，判断出边沿趋势时进入处理
                        {
                              if( R_trend_in==1 )       //当右边沿内倾时进入处理
                              {
                                    if( img_data[EXTRACT_ROW][R_edge_nearest]==DOT_B )     //当上一行右边沿点的上方一点为黑
                                    {
                                          for( EXTRACT_COL=R_edge_nearest-1 ; EXTRACT_COL>=R_edge_nearest-R_IN_2_IN ; EXTRACT_COL-- )      //右边沿内倾向内搜索
                                          {
                                                if( EXTRACT_COL<COL_START )    //当搜索超过图像范围时，结束搜索
                                                {
                                                      break;
                                                }
                                                if( img_data[EXTRACT_ROW][EXTRACT_COL] == DOT_W )  //找到黑白跳变点
                                                {
                                                      right_edge[EXTRACT_ROW]=EXTRACT_COL+1;       //储存本行右边沿列坐标
                                                      
                                                      R_EDGE_NUM++;                                //右边沿个数加一
                                                      
                                                      R_LINK_NUM++;                                //连续右边沿个数加一
                                                      
                                                      R_get_flag=1;                                //右边沿搜索得到标志置  1
                                                      
                                                      R_edge_nearest=EXTRACT_COL+1;                //存储最近一个右边沿的列坐标
                                                      
                                                      R_up_cross_judge();                          //如果右边沿十字标志任为1，进行十字直角拐点判断
                                                      
                                                      break;                                      //本行右边沿搜索结束
                                                }
                                          }
                                          
                                          if( inv_distance[EXTRACT_ROW]>120 && right_edge[EXTRACT_ROW]!=-1 && right_edge[EXTRACT_ROW+1]!=-1 )
                                          {
                                                if( right_edge[EXTRACT_ROW+1]-right_edge[EXTRACT_ROW]>=40 )
                                                {
                                                      W_num=0;
                                                      for( EXTRACT_COL=right_edge[EXTRACT_ROW+1]-1 ; EXTRACT_COL>=right_edge[EXTRACT_ROW] ; EXTRACT_COL-- )      //左边沿内倾向内搜索
                                                      {
                                                            if( img_data[EXTRACT_ROW+1][EXTRACT_COL] == DOT_B )
                                                            {
                                                                  W_num++;
                                                            }
                                                      }
                                                      if( W_num>=0.5*abs(right_edge[EXTRACT_ROW]-right_edge[EXTRACT_ROW+1]) )
                                                      {
                                                            right_edge[EXTRACT_ROW]=-1;
                                                            R_EDGE_NUM--;
                                                            R_LINK_NUM--;
                                                            R_get_flag=0;
                                                      }
                                                }
                                          }
                                          
                                          if( R_get_flag==0 )      //当右边沿内倾向内未搜到边沿
                                          {
                                                if( R_down_cross[0]!=-1 && R_up_cross[0]!=-1 )
                                                {
                                                      W_num=0;
                                                      for( W_num_row=EXTRACT_ROW+1 ; W_num_row<=EXTRACT_ROW+15 ; W_num_row++ )      //右边沿内倾向内搜索
                                                      {
                                                            if( W_num_row>=ROW_END )
                                                            {
                                                                  break;
                                                            }
                                                            if( right_edge[W_num_row]>right_edge[W_num_row+1] )
                                                            {
                                                                  W_num++;
                                                            }
                                                      }
                                                      if( W_num<=1 )
                                                      {
                                                            L_TURN=1;                //左弯标志置  1
                                                            
                                                            R_end_row=EXTRACT_ROW+1; //将上一行的行数值赋给右边沿结束行数值
                                                            
                                                            R_search_end=1;             //当右边沿左倾时，向左搜索不到白点，判断为右边沿搜索结束
                                                            L_search_end=1;             //当右边沿左倾时，向左搜索不到白点，判断为左边沿搜索结束
                                                      }
                                                }
                                                else
                                                {
                                                      if( R_LINK_NUM>=10 )
                                                      {
                                                            W_num=0;
                                                            for( W_num_row=EXTRACT_ROW+1 ; W_num_row<=EXTRACT_ROW+10 ; W_num_row++ )      //右边沿内倾向内搜索
                                                            {
                                                                  if( W_num_row>=ROW_END )
                                                                  {
                                                                        break;
                                                                  }
                                                                  if( right_edge[W_num_row]>right_edge[W_num_row+1] )
                                                                  {
                                                                        W_num++;
                                                                  }
                                                            }
                                                            if( W_num<=1 )
                                                            {
                                                                  L_TURN=1;                //左弯标志置  1
                                                                  
                                                                  R_end_row=EXTRACT_ROW+1; //将上一行的行数值赋给右边沿结束行数值
                                                                  
                                                                  R_search_end=1;             //当右边沿左倾时，向左搜索不到白点，判断为右边沿搜索结束
                                                                  L_search_end=1;             //当右边沿左倾时，向左搜索不到白点，判断为左边沿搜索结束
                                                            }
                                                      }
                                                }
                                                
                                                R_LINK_NUM=0;               //连续右边沿个数置  0
                                                
                                                R_lost_num++;               //当没有搜索到右边沿点时，右边沿丢失数加一
                                          }
                                    }
                                    else                                                   //当上一行右边沿点的上方一点为白
                                    {
                                          for( EXTRACT_COL=R_edge_nearest+1 ; EXTRACT_COL<=R_edge_nearest+R_IN_2_OUT ; EXTRACT_COL++ )      //右边沿内倾向外搜索
                                          {
                                                if( EXTRACT_COL>COL_END )    //当搜索超过图像范围时，结束搜索
                                                {
                                                      break;
                                                }
                                                if( img_data[EXTRACT_ROW][EXTRACT_COL] == DOT_B )
                                                {
                                                      right_edge[EXTRACT_ROW]=EXTRACT_COL;          //储存本行右边沿列坐标
                                                      
                                                      R_EDGE_NUM++;                                //右边沿个数加一
                                                      
                                                      R_LINK_NUM++;                                //连续右边沿个数加一
                                                      
                                                      R_get_flag=1;                                //右边沿搜索得到标志置  1
                                                      
                                                      R_edge_nearest=EXTRACT_COL;                  //存储最近一个右边沿的列坐标
                                                      
                                                      R_trend_in=0;                                //将右边沿内倾标志置  0
                                                      
                                                      R_down_cross_judge();                        //判断是否为十字直角拐点
                                                      
                                                      break;                                      //本行右边沿搜索结束
                                                }
                                          }
                                          if( R_get_flag==0 )      //当右边沿内倾向外未搜到边沿
                                          {
                                                R_LINK_NUM=0;       //连续右边沿个数置  0
                                                
                                                R_lost_num++;       //当没有搜索到右边沿点时，右边沿丢失数加一
                                                
                                                R_down_cross_judge();                        //判断是否为十字直角拐点
                                          }
                                    }
                              }
                              else       //当右边沿外倾时进入处理
                              {
                                    if( img_data[EXTRACT_ROW][R_edge_nearest-1]==DOT_B )     //当上一行右边沿点的上方一点为黑
                                    {
                                          for( EXTRACT_COL=R_edge_nearest-1 ; EXTRACT_COL>=R_edge_nearest-R_OUT_2_IN ; EXTRACT_COL-- )      //右边沿外倾向内搜索
                                          {
                                                if( EXTRACT_COL<COL_START )    //当搜索超过图像范围时，结束搜索
                                                {
                                                      break;
                                                }
                                                if( img_data[EXTRACT_ROW][EXTRACT_COL] == DOT_W )  //找到黑白跳变点
                                                {
                                                      right_edge[EXTRACT_ROW]=EXTRACT_COL+1;        //储存本行右边沿列坐标
                                                      
                                                      R_EDGE_NUM++;                                //右边沿个数加一
                                                      
                                                      R_LINK_NUM++;                                //连续右边沿个数加一
                                                      
                                                      R_get_flag=1;                                //右边沿搜索得到标志置  1
                                                      
                                                      R_edge_nearest=EXTRACT_COL+1;                //存储最近一个右边沿的列坐标
                                                      
                                                      R_trend_in=1;                                //右边沿内倾趋势标志置  1
                                                      
                                                      break;                                      //本行右边沿搜索结束
                                                }
                                          }
                                          if( R_get_flag==0 )      //当右边沿外倾向内未搜到边沿
                                          {
                                                if( R_LINK_NUM>=10 && R_LINK_NUM>L_LINK_NUM )
                                                {
                                                      R_end_row=EXTRACT_ROW+1; //将上一行的行数值赋给右边沿结束行数值
                                                      
                                                      R_search_end=1;             //当右边沿外倾时，向左搜索不到白点，判断为右边沿搜索结束
                                                      L_search_end=1;             //当右边沿外倾时，向左搜索不到白点，判断为左边沿搜索结束
                                                }
                                                
                                                R_LINK_NUM=0;               //连续右边沿个数置  0
                                                
                                                R_lost_num++;               //当没有搜索到右边沿点时，右边沿丢失数加一
                                          }
                                    }
                                    else          //当上一行右边沿点的上方一点为白
                                    {
                                          if( R_cross_flag==1 )    //当右边沿下十字搜索得到
                                          {
                                                for( EXTRACT_COL=R_edge_nearest ; EXTRACT_COL<=R_edge_nearest+R_OUT_2_OUT ; EXTRACT_COL++ )     //开始向外搜索
                                                {
                                                      if( EXTRACT_COL>COL_END )    //当搜索超过图像范围时，结束搜索
                                                      {
                                                            break;
                                                      }
                                                      if( img_data[EXTRACT_ROW][EXTRACT_COL] == DOT_B )
                                                      {
                                                            right_edge[EXTRACT_ROW]=EXTRACT_COL;          //储存本行右边沿列坐标
                                                            
                                                            R_EDGE_NUM++;                                //右边沿个数加一
                                                            
                                                            R_LINK_NUM++;                                //连续右边沿个数加一
                                                            
                                                            R_get_flag=1;                                //右边沿搜索得到标志置  1
                                                            
                                                            R_edge_nearest=EXTRACT_COL;                  //存储最近一个右边沿的列坐标
                                                            
                                                            break;                                      //本行右边沿搜索结束
                                                      }
                                                }
                                                if( R_get_flag==0 )
                                                {
                                                      R_LINK_NUM=0;                                //连续右边沿个数置  0
                                                      
                                                      R_lost_num++;                                //当右边沿右倾时，向外搜索不到白点，判断为右边沿丢失边沿
                                                }
                                          }
                                          else                    //当右边沿十字标志为0时
                                          {
                                                for( EXTRACT_COL=R_edge_nearest+1 ; EXTRACT_COL<=R_edge_nearest+R_OUT_2_OUT ; EXTRACT_COL++ )     //开始向外搜索
                                                {
                                                      if( EXTRACT_COL>COL_END )  //出界停止
                                                      {
                                                            break;
                                                      }
                                                      if( img_data[EXTRACT_ROW][EXTRACT_COL] == DOT_B )
                                                      {
                                                            W_num=0;            //清除白块计数值
                                                            for( W_num_col=R_edge_nearest ; W_num_col<=EXTRACT_COL ; W_num_col++ )     //在搜索到的黑白跳变点后，在上一行搜索白点，判断是否为另一边点
                                                            {
                                                                  if( img_data[EXTRACT_ROW+1][W_num_col] == DOT_W )
                                                                  {
                                                                        W_num++;
                                                                  }
                                                            }
                                                            if( W_num>COL_W_NUM && R_barrier_flag==0 )             //当白块计数值超过阀值，则判断为右边沿丢失             /*这里的 W_num 的阀值需要测试*/
                                                            {
                                                                  R_search_end=1;             //当右边沿外倾并且右边沿十字标志为0时，向外搜索异常，判断为右边沿搜索结束
                                                                  R_end_row=EXTRACT_ROW+1;    //将上一行的行数值赋给右边沿结束行数值
                                                                  
                                                                  break;
                                                            }
                                                            
                                                            right_edge[EXTRACT_ROW]=EXTRACT_COL;          //储存本行右边沿列坐标
                                                            
                                                            R_EDGE_NUM++;                                //右边沿个数加一
                                                            
                                                            R_LINK_NUM++;                                //连续右边沿个数加一
                                                            
                                                            R_get_flag=1;                                //右边沿搜索得到标志置  1
                                                            
                                                            R_edge_nearest=EXTRACT_COL;                  //存储最近一个右边沿的列坐标
                                                            
                                                            break;                                      //本行右边沿搜索结束
                                                      }
                                                }
                                                if( R_get_flag==0 )
                                                {
                                                      R_LINK_NUM=0;                                //连续右边沿个数置  0
                                                      
                                                      R_lost_num++;                                //当右边沿右倾时，向外搜索不到白点，判断为右边沿丢失边沿
                                                }
                                          }
                                    }
                              }
                              
                              if( R_get_flag==1 )               //当搜索到边沿
                              {
                                    R_up_cross_extract();       //向内搜索上十字
                              }
                        }
                        else                  //当右边沿没有判断出边沿趋势时进入处理
                        {
                              if( img_data[EXTRACT_ROW][R_edge_nearest] == DOT_B )     //当上一行右边沿点的上方一点为黑
                              {
                                    for( EXTRACT_COL=R_edge_nearest-1 ; EXTRACT_COL>=R_edge_nearest-R_EDGE_IN ; EXTRACT_COL-- )            //开始向内搜索
                                    {
                                          if( EXTRACT_COL<COL_START+4 )    //当搜索超过图像范围时，结束搜索
                                          {
                                                break;
                                          }
                                          if( img_data[EXTRACT_ROW][EXTRACT_COL] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL-2] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL-4] == DOT_W )   //找到黑白跳变点
                                          {
                                                right_edge[EXTRACT_ROW]=EXTRACT_COL+1;          //储存本行右边沿列坐标
                                                
                                                R_EDGE_NUM++;                                //右边沿个数加一
                                                
                                                R_LINK_NUM++;                                //连续右边沿个数加一
                                                
                                                R_get_flag=1;                                //右边沿搜索得到标志置  1
                                                
                                                R_edge_nearest=EXTRACT_COL+1;                  //存储最近一个右边沿的列坐标
                                                
                                                R_up_cross_judge();                    //如果右边沿十字标志任为1，进行十字直角拐点判断
                                                
                                                break;                                      //本行右边沿搜索结束
                                          }
                                    }
                                    if( R_get_flag==0 )
                                    {
                                          R_LINK_NUM=0;               //连续右边沿个数置  0
                                          
                                          R_lost_num++;               //当没有搜索到右边沿点时，右边沿丢失数加一
                                    }
                              }
                              else     //当上一行右边沿点的上方一点为白
                              {
                                    if(  img_data[EXTRACT_ROW][R_edge_nearest-1] == DOT_W && img_data[EXTRACT_ROW][R_edge_nearest-2] == DOT_W )        //之后两点也为白
                                    {
                                          for( EXTRACT_COL=R_edge_nearest+1 ; EXTRACT_COL<=R_edge_nearest+R_EDGE_OUT ; EXTRACT_COL++ )           //开始向外搜索黑白跳变点
                                          {
                                                if( EXTRACT_COL>COL_END-2 )    //当搜索超过图像范围时，结束搜索
                                                {
                                                      break;
                                                }
                                                if( img_data[EXTRACT_ROW][EXTRACT_COL] == DOT_B )   //找到黑白跳变点
                                                {
                                                      right_edge[EXTRACT_ROW]=EXTRACT_COL;          //储存本行右边沿列坐标
                                                      
                                                      R_EDGE_NUM++;                                //右边沿个数加一
                                                      
                                                      R_LINK_NUM++;                                //连续右边沿个数加一
                                                      
                                                      R_get_flag=1;                                //右边沿搜索得到标志置  1
                                                      
                                                      R_edge_nearest=EXTRACT_COL;                  //存储最近一个右边沿的列坐标
                                                      
                                                      break;                                      //本行右边沿搜索结束
                                                }
                                          }
                                          if( R_get_flag==0 )
                                          {
                                                R_LINK_NUM=0;       //连续右边沿个数置  0
                                                
                                                R_lost_num++;       //当没有搜索到右边沿点时，右边沿丢失数加一
                                          }
                                    }
                                    else           //之后两点有黑点,则判断为干扰
                                    {
                                          for( EXTRACT_COL=R_edge_nearest-1 ; EXTRACT_COL>=R_edge_nearest-R_EDGE_IN ; EXTRACT_COL-- )            //开始向外搜索黑白跳变点
                                          {
                                                if( EXTRACT_COL<COL_START+5 )    //当搜索超过图像范围时，结束搜索
                                                {
                                                      break;
                                                }
                                                if( img_data[EXTRACT_ROW][EXTRACT_COL] == DOT_B && img_data[EXTRACT_ROW][EXTRACT_COL-1] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL-3] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL-5] == DOT_W )   //找到黑白跳变点
                                                {
                                                      right_edge[EXTRACT_ROW]=EXTRACT_COL;          //储存本行右边沿列坐标
                                                      
                                                      R_EDGE_NUM++;                                //右边沿个数加一
                                                      
                                                      R_LINK_NUM++;                                //连续右边沿个数加一
                                                      
                                                      R_get_flag=1;                                //右边沿搜索得到标志置  1
                                                      
                                                      R_edge_nearest=EXTRACT_COL;                  //存储最近一个右边沿的列坐标
                                                      
                                                      break;                                      //本行左边沿搜索结束
                                                }
                                          }
                                          if( R_get_flag==0 )
                                          {
                                                R_LINK_NUM=0;               //连续右边沿个数置  0
                                                
                                                R_lost_num++;               //当没有搜索到右边沿点时，右边沿丢失数加一
                                          }
                                    }
                              }
                              
                              if( R_LINK_NUM>=3 )  //当搜索完这一行后，连续边沿数大于等于3
                              {
                                    if(  right_edge[EXTRACT_ROW]<=right_edge[EXTRACT_ROW+1] || right_edge[EXTRACT_ROW]<=right_edge[EXTRACT_ROW+2] )
                                    {
                                          R_trend_in=1;     //满足条件，右边沿内倾趋势置  1
                                    }
                                    else
                                    {
                                          R_trend_in=0;     //不满足条件，右边沿内倾趋势置  0
                                    }
                              }
                        }
                  }
                  else                        //当右边沿丢失时进入处理
                  {
                        if( R_cross_flag==1 ) //当右边沿十字标志为1时
                        {
                              if( R_down_cross[0]!=-1 )    //当检测到右边沿下方十字直角拐点
                              {
                                    if( R_down_cross[1]+CROSS_ERTRACT<COL_END )
                                    {
                                          W_num_col=R_down_cross[1]+CROSS_ERTRACT;
                                    }
                                    else
                                    {
                                          W_num_col=COL_END;
                                    }
                                    for( EXTRACT_COL=W_num_col ; EXTRACT_COL>=R_down_cross[1]-80 ; EXTRACT_COL-- )    //开始搜索
                                    {
                                          if( EXTRACT_COL<COL_START+5 )   //超过图像范围停止
                                          {
                                                break;
                                          }
                                          if( img_data[EXTRACT_ROW][EXTRACT_COL] == DOT_B && img_data[EXTRACT_ROW][EXTRACT_COL-1] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL-3] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL-5] == DOT_W )      //当搜索到黑白跳变点
                                          {
                                                W_num=0;            //清除白块计数值
                                                for( W_num_col=EXTRACT_ROW-1 ; W_num_col>=EXTRACT_ROW-ROW_W_SUM ; W_num_col-- )     //在搜索到的黑白跳变点后，向上搜索白点，判断是否为干扰点
                                                {
                                                      if( W_num_col<ROW_START )   //超过图像范围停止
                                                      {
                                                            break;
                                                      }
                                                      if( img_data[W_num_col][EXTRACT_COL] == DOT_W )
                                                      {
                                                            W_num++;
                                                      }
                                                }
                                                if( W_num<ROW_W_NUM )              //当白块计数值没超过阀值，则判断为右边沿点          /*这里的 W_num 的阀值需要测试*/
                                                {
                                                      R_up_cross_error();                          //右边沿上方直角拐点错误纠正
                                                      
                                                      if( R_get_flag==1 )
                                                      {
                                                            right_edge[EXTRACT_ROW]=EXTRACT_COL;          //储存本行右边沿列坐标
                                                            
                                                            R_EDGE_NUM++;                                //右边沿个数加一
                                                            
                                                            R_LINK_NUM++;                                //连续右边沿个数加一
                                                            
                                                            R_edge_nearest=EXTRACT_COL;                  //存储最近一个右边沿的列坐标
                                                            
                                                            R_lost_num=0;                                //连续右边沿丢失数清0
                                                            
                                                            if( cross_flag==0 && R_up_cross[0]!=-1 )
                                                            {
                                                                  cross_flag=1;                          //十字标志置  1
                                                            }
                                                      }
                                                      
                                                      break;                                      //本行右边沿搜索结束
                                                }
                                          }
                                    }
                                    if( R_get_flag==0 )
                                    {
                                          R_lost_num++;                          //连续右边沿丢失数加1
                                    }
                              }
                              else                         //当检测未到右边沿下方十字直角拐点
                              {
                                    for( EXTRACT_COL=COL_END ; EXTRACT_COL>=R_LOST ; EXTRACT_COL-- )    //开始搜索
                                    {
                                          if( EXTRACT_COL<COL_START+5 )   //超过图像范围停止
                                          {
                                                break;
                                          }
                                          if( img_data[EXTRACT_ROW][EXTRACT_COL] == DOT_B && img_data[EXTRACT_ROW][EXTRACT_COL-1] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL-3] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL-5] == DOT_W )      //当搜索到黑白跳变点
                                          {
                                                R_up_cross_error();                          //右边沿上方直角拐点错误纠正
                                                
                                                if( R_get_flag==1 )
                                                {
                                                      right_edge[EXTRACT_ROW]=EXTRACT_COL;          //储存本行右边沿列坐标
                                                      
                                                      R_EDGE_NUM++;                                //右边沿个数加一
                                                      
                                                      R_LINK_NUM++;                                //连续右边沿个数加一
                                                      
                                                      R_edge_nearest=EXTRACT_COL;                  //存储最近一个右边沿的列坐标
                                                      
                                                      R_lost_num=0;                                //连续右边沿丢失数清0
                                                      
                                                      if( cross_flag==0 && R_up_cross[0]!=-1 )
                                                      {
                                                            cross_flag=1;                          //十字标志置  1
                                                      }
                                                }
                                                
                                                break;                                      //本行右边沿搜索结束
                                          }
                                    }
                                    if( R_get_flag==0 )
                                    {
                                          R_lost_num++;                          //连续右边沿丢失数加1
                                    }
                              }
                        }
                        else                  //当右边沿十字标志为0时
                        {
                              if( img_data[EXTRACT_ROW][COL_END] == DOT_W && img_data[EXTRACT_ROW][COL_END-1] == DOT_W && img_data[EXTRACT_ROW][COL_END-2] == DOT_W )    //当前三个点为白
                              {
                                    if( LOOP_TEMP==0 )    //当不处于环道
                                    {
                                          for( EXTRACT_COL=COL_END-3 ; EXTRACT_COL>R_LOST ; EXTRACT_COL-- )    //开始搜索
                                          {
                                                if( img_data[EXTRACT_ROW][EXTRACT_COL] == DOT_B && img_data[EXTRACT_ROW][EXTRACT_COL-1] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL-3] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL-5] == DOT_W )      //当搜索到黑白跳变点
                                                {
                                                      if( EXTRACT_COL<139 && EXTRACT_ROW>40 )      //当搜索到错误边沿
                                                      {
                                                            if( img_data[EXTRACT_ROW][EXTRACT_COL+3] == DOT_W || img_data[EXTRACT_ROW][EXTRACT_COL+6] == DOT_W || img_data[EXTRACT_ROW][EXTRACT_COL+9] == DOT_W || img_data[EXTRACT_ROW][EXTRACT_COL+12] == DOT_W || img_data[EXTRACT_ROW][EXTRACT_COL+15] == DOT_W )
                                                            {
                                                                  continue;
                                                            }
                                                      }
                                                      W_num=0;            //清除白块计数值
                                                      for( W_num_col=EXTRACT_ROW-1 ; W_num_col>=EXTRACT_ROW-ROW_W_SUM ; W_num_col-- )     //在搜索到的黑白跳变点后，向上搜索白点，判断是否为干扰点
                                                      {
                                                            if( W_num_col<ROW_START )   //超过图像范围停止
                                                            {
                                                                  break;
                                                            }
                                                            if( img_data[W_num_col][EXTRACT_COL] == DOT_W )
                                                            {
                                                                  W_num++;
                                                            }
                                                      }
                                                      if( W_num<ROW_W_NUM )
                                                      {
                                                            right_edge[EXTRACT_ROW]=EXTRACT_COL;          //储存本行右边沿列坐标
                                                            
                                                            R_EDGE_NUM++;                                //右边沿个数加一
                                                            
                                                            R_LINK_NUM++;                                //连续右边沿个数加一
                                                            
                                                            R_get_flag=1;                                //右边沿搜索得到标志置  1
                                                            
                                                            R_edge_nearest=EXTRACT_COL;                  //存储最近一个右边沿的列坐标
                                                            
                                                            R_lost_num=0;                                //连续右边沿丢失数清0
                                                            
                                                            break;                                      //本行右边沿搜索结束
                                                      }
                                                      
                                                }
                                          }
                                          if( R_get_flag==0 )
                                          {
                                                R_lost_num++;             //当右边沿丢失时，向左搜索不到黑白跳变点，判断为右边沿丢失
                                          }
                                    }
                                    else    //当处于环道
                                    {
                                          if( loop_l_r_flag==1 )
                                          {
                                                for( EXTRACT_COL=COL_END-3 ; EXTRACT_COL>R_LOST ; EXTRACT_COL-- )    //开始搜索
                                                {
                                                      if( img_data[EXTRACT_ROW][EXTRACT_COL] == DOT_B && img_data[EXTRACT_ROW][EXTRACT_COL-1] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL-3] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL-5] == DOT_W )      //当搜索到黑白跳变点
                                                      {
                                                            if( EXTRACT_COL<139 && EXTRACT_ROW>40 )      //当搜索到错误边沿
                                                            {
                                                                  if( img_data[EXTRACT_ROW][EXTRACT_COL+3] == DOT_W || img_data[EXTRACT_ROW][EXTRACT_COL+6] == DOT_W || img_data[EXTRACT_ROW][EXTRACT_COL+9] == DOT_W || img_data[EXTRACT_ROW][EXTRACT_COL+12] == DOT_W || img_data[EXTRACT_ROW][EXTRACT_COL+15] == DOT_W )
                                                                  {
                                                                        continue;
                                                                  }
                                                            }
                                                            W_num=0;            //清除白块计数值
                                                            for( W_num_col=EXTRACT_ROW-1 ; W_num_col>=EXTRACT_ROW-ROW_W_SUM ; W_num_col-- )     //在搜索到的黑白跳变点后，向上搜索白点，判断是否为干扰点
                                                            {
                                                                  if( W_num_col<ROW_START )   //超过图像范围停止
                                                                  {
                                                                        break;
                                                                  }
                                                                  if( img_data[W_num_col][EXTRACT_COL] == DOT_W )
                                                                  {
                                                                        W_num++;
                                                                  }
                                                            }
                                                            if( W_num<ROW_W_NUM )
                                                            {
                                                                  right_edge[EXTRACT_ROW]=EXTRACT_COL;          //储存本行右边沿列坐标
                                                                  
                                                                  R_EDGE_NUM++;                                //右边沿个数加一
                                                                  
                                                                  R_LINK_NUM++;                                //连续右边沿个数加一
                                                                  
                                                                  R_get_flag=1;                                //右边沿搜索得到标志置  1
                                                                  
                                                                  R_edge_nearest=EXTRACT_COL;                  //存储最近一个右边沿的列坐标
                                                                  
                                                                  R_lost_num=0;                                //连续右边沿丢失数清0
                                                                  
                                                                  break;                                      //本行右边沿搜索结束
                                                            }
                                                            
                                                      }
                                                }
                                                if( R_get_flag==0 )
                                                {
                                                      R_lost_num++;             //当右边沿丢失时，向左搜索不到黑白跳变点，判断为右边沿丢失
                                                }
                                          }
                                          else
                                          {
                                                R_lost_num++;             //当右边沿丢失时，向左搜索不到黑白跳变点，判断为右边沿丢失
                                          }
                                    }
                              }
                              else         //当前三个点不全为白
                              {
                                    if( LOOP_TEMP==1 && loop_l_r_flag==0 )
                                    {
                                          if( R_EDGE_NUM>=3 || EXTRACT_ROW<40 )
                                          {
                                                for( EXTRACT_COL=COL_END ; EXTRACT_COL>=R_LOOP ; EXTRACT_COL-- )    //开始搜索
                                                {
                                                      if( img_data[EXTRACT_ROW][EXTRACT_COL] == DOT_B && img_data[EXTRACT_ROW][EXTRACT_COL-1] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL-3] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL-5] == DOT_W )      //当搜索到黑白跳变点
                                                      {
                                                            right_edge[EXTRACT_ROW]=EXTRACT_COL;        //储存本行右边沿列坐标
                                                            
                                                            R_EDGE_NUM++;                                //右边沿个数加一
                                                            
                                                            R_LINK_NUM++;                                //连续右边沿个数加一
                                                            
                                                            R_get_flag=1;                                //右边沿搜索得到标志置  1
                                                            
                                                            R_edge_nearest=EXTRACT_COL;                //存储最近一个右边沿的列坐标
                                                            
                                                            R_lost_num=0;                                //连续右边沿丢失数清0
                                                            
                                                            break;                                      //本行右边沿搜索结束
                                                      }
                                                }
                                                if( R_get_flag==0 )
                                                {
                                                      R_lost_num++;             //当右边沿丢失时，向左搜索不到黑白跳变点，判断为右边沿丢失
                                                }
                                          }
                                          else
                                          {
                                                for( EXTRACT_COL=COL_END ; EXTRACT_COL>=R_LOST ; EXTRACT_COL-- )    //开始搜索
                                                {
                                                      if( img_data[EXTRACT_ROW][EXTRACT_COL] == DOT_B && img_data[EXTRACT_ROW][EXTRACT_COL-1] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL-3] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL-5] == DOT_W )      //当搜索到黑白跳变点
                                                      {
                                                            right_edge[EXTRACT_ROW]=EXTRACT_COL;        //储存本行右边沿列坐标
                                                            
                                                            R_EDGE_NUM++;                                //右边沿个数加一
                                                            
                                                            R_LINK_NUM++;                                //连续右边沿个数加一
                                                            
                                                            R_get_flag=1;                                //右边沿搜索得到标志置  1
                                                            
                                                            R_edge_nearest=EXTRACT_COL;                //存储最近一个右边沿的列坐标
                                                            
                                                            R_lost_num=0;                                //连续右边沿丢失数清0
                                                            
                                                            break;                                      //本行右边沿搜索结束
                                                      }
                                                }
                                                if( R_get_flag==0 )
                                                {
                                                      R_lost_num++;             //当右边沿丢失时，向左搜索不到黑白跳变点，判断为右边沿丢失
                                                }
                                          }
                                    }
                                    else
                                    {
                                          for( EXTRACT_COL=COL_END ; EXTRACT_COL>=R_LOST ; EXTRACT_COL-- )    //开始搜索
                                          {
                                                if( img_data[EXTRACT_ROW][EXTRACT_COL] == DOT_B && img_data[EXTRACT_ROW][EXTRACT_COL-1] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL-3] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL-5] == DOT_W )      //当搜索到黑白跳变点
                                                {
                                                      right_edge[EXTRACT_ROW]=EXTRACT_COL;        //储存本行右边沿列坐标
                                                      
                                                      R_EDGE_NUM++;                                //右边沿个数加一
                                                      
                                                      R_LINK_NUM++;                                //连续右边沿个数加一
                                                      
                                                      R_get_flag=1;                                //右边沿搜索得到标志置  1
                                                      
                                                      R_edge_nearest=EXTRACT_COL;                //存储最近一个右边沿的列坐标
                                                      
                                                      R_lost_num=0;                                //连续右边沿丢失数清0
                                                      
                                                      break;                                      //本行右边沿搜索结束
                                                }
                                          }
                                          if( R_get_flag==0 )
                                          {
                                                R_lost_num++;             //当右边沿丢失时，向左搜索不到黑白跳变点，判断为右边沿丢失
                                          }
                                    }
                              }
                        }
                  }
            }
            else
            {
                  if( L_TURN==0 && R_TURN==0 )
                  {
                        if( img_data[EXTRACT_ROW][COL_END] == DOT_W && img_data[EXTRACT_ROW][COL_END-2] == DOT_W && img_data[EXTRACT_ROW][COL_END-4] == DOT_W )    //当前三个点为白
                        {
                              R_lost_num++;             //判断为左边沿丢失
                        }
                  }
            }
            
            
            
            /*以下程序段为边沿点判断修正程序*/
            
            if( L_get_flag==1 && R_get_flag==1 )
            {
                  if( right_edge[EXTRACT_ROW]-left_edge[EXTRACT_ROW]<=0 )  //当同一行两个边沿都采集到，但是左右边沿位置异常时
                  {
                        if( L_up_cross[0]==EXTRACT_ROW || R_up_cross[0]==EXTRACT_ROW )  //当在处理左右边沿上方直角拐点时
                        {
                              if( L_down_cross[0]!=-1 && R_down_cross[0]!=-1 )      //当已经找到左右边沿下方直角拐点时
                              {
                                    if( L_up_cross[0]!=-1 && R_up_cross[0]!=-1 )
                                    {
                                          if( abs(L_up_cross[1]-L_down_cross[1]) > abs(R_up_cross[1]-R_down_cross[1]) )
                                          {
                                                left_edge[EXTRACT_ROW]=-1;            //清除本行左边沿列坐标
                                                
                                                L_EDGE_NUM--;                         //左边沿个数减一
                                                
                                                L_LINK_NUM=0;                         //清除连续左边沿个数
                                                
                                                L_lost_num++;                         //连续丢失左边沿个数加一
                                                
                                                L_up_cross[0]=-1;                     //清除十字拐点行坐标
                                                
                                                L_up_cross[1]=-1;                     //清除十字拐点列坐标
                                                
                                                L_cross_flag=0;                       //清除左边沿十字标志
                                          }
                                          else
                                          {
                                                right_edge[EXTRACT_ROW]=-1;            //清除本行左边沿列坐标
                                                
                                                R_EDGE_NUM--;                         //左边沿个数减一
                                                
                                                R_LINK_NUM=0;                         //清除连续左边沿个数
                                                
                                                R_lost_num++;                         //连续丢失左边沿个数加一
                                                
                                                R_up_cross[0]=-1;                     //清除十字拐点行坐标
                                                
                                                R_up_cross[1]=-1;                     //清除十字拐点列坐标
                                                
                                                R_cross_flag=0;                       //清除左边沿十字标志
                                          }
                                    }
                              }
                              if( L_down_cross[0]!=-1 && R_down_cross[0]==-1 )      //当已经找到左边沿下方直角拐点时
                              {
                                    right_edge[EXTRACT_ROW]=-1;           //清除本行右边沿列坐标
                                    
                                    R_EDGE_NUM--;                         //右边沿个数减一
                                    
                                    R_LINK_NUM=0;                         //清除连续右边沿个数
                                    
                                    R_lost_num++;                         //连续丢失右边沿个数加一
                                    
                                    R_up_cross[0]=-1;                     //清除十字拐点行坐标
                                    
                                    R_up_cross[1]=-1;                     //清除十字拐点列坐标
                                    
                                    R_cross_flag=0;                       //清除右边沿十字标志
                              }
                              if( L_down_cross[0]==-1 && R_down_cross[0]!=-1 )      //当已经找到右边沿下方直角拐点时
                              {
                                    left_edge[EXTRACT_ROW]=-1;            //清除本行左边沿列坐标
                                    
                                    L_EDGE_NUM--;                         //左边沿个数减一
                                    
                                    L_LINK_NUM=0;                         //清除连续左边沿个数
                                    
                                    L_lost_num++;                         //连续丢失左边沿个数加一
                                    
                                    L_up_cross[0]=-1;                     //清除十字拐点行坐标
                                    
                                    L_up_cross[1]=-1;                     //清除十字拐点列坐标
                                    
                                    L_cross_flag=0;                       //清除左边沿十字标志
                              }
                              if( L_down_cross[0]==-1 && R_down_cross[0]==-1 )      //当不存在十字下直角拐点时
                              {
                                    if( overtake_cross_flag )   //判断是否进入十字超车
                                    {
                                          if(car_L_pull_over_flag && !car_R_pull_over_flag)   //判断为左侧靠边
                                          {
                                                right_edge[EXTRACT_ROW]=-1;           //清除本行右边沿列坐标
                                                
                                                R_EDGE_NUM--;                         //右边沿个数减一
                                                
                                                R_LINK_NUM=0;                         //清除连续右边沿个数
                                                
                                                R_lost_num++;                         //连续丢失右边沿个数加一
                                                
                                                R_up_cross[0]=-1;                     //清除十字拐点行坐标
                                                
                                                R_up_cross[1]=-1;                     //清除十字拐点列坐标
                                                
                                                R_cross_flag=0;                       //清除右边沿十字标志
                                          }
                                          else if(car_R_pull_over_flag && !car_L_pull_over_flag)   //判断为右侧靠边
                                          {
                                                left_edge[EXTRACT_ROW]=-1;            //清除本行左边沿列坐标
                                                
                                                L_EDGE_NUM--;                         //左边沿个数减一
                                                
                                                L_LINK_NUM=0;                         //清除连续左边沿个数
                                                
                                                L_lost_num++;                         //连续丢失左边沿个数加一
                                                
                                                L_up_cross[0]=-1;                     //清除十字拐点行坐标
                                                
                                                L_up_cross[1]=-1;                     //清除十字拐点列坐标
                                                
                                                L_cross_flag=0;                       //清除左边沿十字标志
                                          }
                                    }
                                    else
                                    {
                                          if( L_up_cross[0]>EXTRACT_ROW )      //当已经找到左边沿上方直角拐点时
                                          {
                                                right_edge[EXTRACT_ROW]=-1;           //清除本行右边沿列坐标
                                                
                                                R_EDGE_NUM--;                         //右边沿个数减一
                                                
                                                R_LINK_NUM=0;                         //清除连续右边沿个数
                                                
                                                R_lost_num++;                         //连续丢失右边沿个数加一
                                                
                                                R_up_cross[0]=-1;                     //清除十字拐点行坐标
                                                
                                                R_up_cross[1]=-1;                     //清除十字拐点列坐标
                                                
                                                R_cross_flag=0;                       //清除右边沿十字标志
                                          }
                                          if( R_up_cross[0]>EXTRACT_ROW )      //当已经找到右边沿上方直角拐点时
                                          {
                                                left_edge[EXTRACT_ROW]=-1;            //清除本行左边沿列坐标
                                                
                                                L_EDGE_NUM--;                         //左边沿个数减一
                                                
                                                L_LINK_NUM=0;                         //清除连续左边沿个数
                                                
                                                L_lost_num++;                         //连续丢失左边沿个数加一
                                                
                                                L_up_cross[0]=-1;                     //清除十字拐点行坐标
                                                
                                                L_up_cross[1]=-1;                     //清除十字拐点列坐标
                                                
                                                L_cross_flag=0;                       //清除左边沿十字标志
                                          }
                                    }
                              }
                        }
                        else   //如果图像为普通赛道
                        {
                              if( L_LINK_NUM>=3 && R_LINK_NUM>=3 )  //当左右边沿连续时，判断边沿变化剧烈情况
                              {
                                    if( abs(left_edge[EXTRACT_ROW]-left_edge[EXTRACT_ROW+1]) > abs(left_edge[EXTRACT_ROW]-left_edge[EXTRACT_ROW+1]) )
                                    {
                                          left_edge[EXTRACT_ROW]=-1;            //清除本行左边沿列坐标
                                          
                                          L_EDGE_NUM--;                         //左边沿个数减一
                                          
                                          L_LINK_NUM=0;                         //清除连续左边沿个数
                                          
                                          L_lost_num++;                         //连续丢失左边沿个数加一
                                    }
                                    else
                                    {
                                          right_edge[EXTRACT_ROW]=-1;           //清除本行右边沿列坐标
                                          
                                          R_EDGE_NUM--;                         //右边沿个数减一
                                          
                                          R_LINK_NUM=0;                         //清除连续右边沿个数
                                          
                                          R_lost_num++;                         //连续丢失右边沿个数加一
                                    }
                              }
                              else
                              {
                                    W_num_row=0;
                                    W_num_col=0;
                                    for( EXTRACT_COL=EXTRACT_ROW+1 ; EXTRACT_COL<=EXTRACT_COL+5 ; EXTRACT_COL++ )
                                    {
                                          if( EXTRACT_COL>ROW_END )
                                          {
                                                break;
                                          }
                                          if( left_edge[EXTRACT_COL]!=-1 )
                                          {
                                                W_num_row++;
                                          }
                                          if( right_edge[EXTRACT_COL]!=-1 )
                                          {
                                                W_num_col++;
                                          }
                                    }
                                    if( W_num_row>W_num_col )   //当左边沿连续，右边不沿连续时
                                    {
                                          right_edge[EXTRACT_ROW]=-1;           //清除本行右边沿列坐标
                                          
                                          R_EDGE_NUM--;                         //右边沿个数减一
                                          
                                          R_LINK_NUM=0;                         //清除连续右边沿个数
                                          
                                          R_lost_num++;                         //连续丢失右边沿个数加一
                                    }
                                    else
                                    {
                                          if( W_num_row<W_num_col )   //当左边不沿连续，右边沿连续时
                                          {
                                                left_edge[EXTRACT_ROW]=-1;            //清除本行左边沿列坐标
                                                
                                                L_EDGE_NUM--;                         //左边沿个数减一
                                                
                                                L_LINK_NUM=0;                         //清除连续左边沿个数
                                                
                                                L_lost_num++;                         //连续丢失左边沿个数加一
                                          }
                                          else    //当左右边都不沿连续时
                                          {
                                                if( (left_edge[EXTRACT_ROW]+right_edge[EXTRACT_ROW])<CAMERA_W )
                                                {
                                                      right_edge[EXTRACT_ROW]=-1;           //清除本行右边沿列坐标
                                                      
                                                      R_EDGE_NUM--;                         //右边沿个数减一
                                                      
                                                      R_LINK_NUM=0;                         //清除连续右边沿个数
                                                      
                                                      R_lost_num++;                         //连续丢失右边沿个数加一
                                                }
                                                else
                                                {
                                                      left_edge[EXTRACT_ROW]=-1;            //清除本行左边沿列坐标
                                                      
                                                      L_EDGE_NUM--;                         //左边沿个数减一
                                                      
                                                      L_LINK_NUM=0;                         //清除连续左边沿个数
                                                      
                                                      L_lost_num++;                         //连续丢失左边沿个数加一
                                                }
                                          }
                                    }
                              }
                        }
                  }
                  else
                  {
                        if( double_car_flag==0 )   //如果为单车模式
                        {
                              if( EXTRACT_ROW<=30 && EXTRACT_ROW>5 && right_edge[EXTRACT_ROW]-left_edge[EXTRACT_ROW]<=bar_range_col[EXTRACT_ROW] )    //当赛道宽度异常变小，判断光线干扰
                              {
                                    if( L_LINK_NUM>R_LINK_NUM+10 && R_LINK_NUM<5 )
                                    {
                                          if( L_down_cross[0]<EXTRACT_ROW+5 )
                                          {
                                                L_down_cross[0]=-1;
                                                L_down_cross[1]=-1;
                                                L_cross_flag=0;
                                          }
                                          for( W_num_col=right_edge[EXTRACT_ROW] ; W_num_col<=COL_END-3 ; W_num_col++ )   //重新搜索左边沿
                                          {
                                                if( img_data[EXTRACT_ROW][W_num_col] == DOT_B && img_data[EXTRACT_ROW][W_num_col+1] == DOT_W && img_data[EXTRACT_ROW][W_num_col+3] == DOT_W )
                                                {
                                                      left_edge[EXTRACT_ROW]=W_num_col;          //储存本行左边沿列坐标
                                                      
                                                      L_EDGE_NUM++;                                //左边沿个数加一
                                                      
                                                      L_LINK_NUM++;                                //连续左边沿个数加一
                                                      
                                                      L_get_flag=1;                                //左边沿搜索得到标志置  1
                                                      
                                                      L_edge_nearest=W_num_col;                  //存储最近一个左边沿的列坐标
                                                      
                                                      L_lost_num=0;                                //连续左边沿丢失数清0
                                                      
                                                      R_lost_num++;                                //当右边沿丢失时，判断为右边沿丢失
                                                      
                                                      right_edge[EXTRACT_ROW]=-1;                  //清除本行右边沿列坐标
                                                      
                                                      break;                                      //本行左边沿搜索结束
                                                }
                                          }
                                          if( right_edge[EXTRACT_ROW]!=-1 )   //当为搜索到左边沿
                                          {
                                                W_num=0;
                                                for( W_num_row=EXTRACT_ROW+1 ; W_num_row<=EXTRACT_ROW+10 ; W_num_row++ )      //右边沿内倾向内搜索
                                                {
                                                      if( W_num_row>=ROW_END )
                                                      {
                                                            break;
                                                      }
                                                      if( left_edge[W_num_row]<left_edge[W_num_row+1] )
                                                      {
                                                            W_num++;
                                                      }
                                                }
                                                if( W_num<=1 )
                                                {
                                                      R_TURN=1;                   //右弯标志置  1
                                                      
                                                      L_end_row=EXTRACT_ROW+1;    //将上一行的行数值赋给左边沿结束行数值
                                                      
                                                      L_search_end=1;             //当左边沿右倾时，向内搜索不到白点，判断为左边沿搜索结束
                                                      R_search_end=1;             //当左边沿右倾时，向内搜索不到白点，判断为右边沿搜索结束
                                                }
                                          }
                                    }
                                    else
                                    {
                                          if( R_LINK_NUM>L_LINK_NUM+10 && L_LINK_NUM<5 )
                                          {
                                                if( R_down_cross[0]<EXTRACT_ROW+5 )
                                                {
                                                      R_down_cross[0]=-1;
                                                      R_down_cross[1]=-1;
                                                      R_cross_flag=0;
                                                }
                                                for( W_num_col=left_edge[EXTRACT_ROW] ; W_num_col>=COL_START+3 ; W_num_col-- )   //重新搜索左边沿
                                                {
                                                      if( img_data[EXTRACT_ROW][W_num_col] == DOT_B && img_data[EXTRACT_ROW][W_num_col-1] == DOT_W && img_data[EXTRACT_ROW][W_num_col-3] == DOT_W )
                                                      {
                                                            right_edge[EXTRACT_ROW]=W_num_col;          //储存本行右边沿列坐标
                                                            
                                                            R_EDGE_NUM++;                                //右边沿个数加一
                                                            
                                                            R_LINK_NUM++;                                //连续右边沿个数加一
                                                            
                                                            R_get_flag=1;                                //右边沿搜索得到标志置  1
                                                            
                                                            R_edge_nearest=W_num_col;                  //存储最近一个右边沿的列坐标
                                                            
                                                            R_lost_num=0;                                //连续右边沿丢失数清0
                                                            
                                                            L_lost_num++;                                //当左边沿丢失时，判断为左边沿丢失
                                                            
                                                            left_edge[EXTRACT_ROW]=-1;                   //清除本行左边沿列坐标
                                                            
                                                            break;                                      //本行右边沿搜索结束
                                                      }
                                                }
                                                if( left_edge[EXTRACT_ROW]!=-1 )   //当为搜索到左边沿
                                                {
                                                      W_num=0;
                                                      for( W_num_row=EXTRACT_ROW+1 ; W_num_row<=EXTRACT_ROW+10 ; W_num_row++ )      //右边沿内倾向内搜索
                                                      {
                                                            if( W_num_row>=ROW_END )
                                                            {
                                                                  break;
                                                            }
                                                            if( right_edge[W_num_row]>right_edge[W_num_row+1] )
                                                            {
                                                                  W_num++;
                                                            }
                                                      }
                                                      if( W_num<=1 )
                                                      {
                                                            L_TURN=1;                //左弯标志置  1
                                                            
                                                            R_end_row=EXTRACT_ROW+1; //将上一行的行数值赋给右边沿结束行数值
                                                            
                                                            R_search_end=1;             //当右边沿左倾时，向左搜索不到白点，判断为右边沿搜索结束
                                                            L_search_end=1;             //当右边沿左倾时，向左搜索不到白点，判断为左边沿搜索结束
                                                      }
                                                }
                                          }
                                    }
                              }
                        }
                        else   //如果为双车模式
                        {
                              if( car_flag==0 )  //如果为前车
                              {
                                    if( EXTRACT_ROW<=30 && EXTRACT_ROW>5 && right_edge[EXTRACT_ROW]-left_edge[EXTRACT_ROW]<=bar_range_col[EXTRACT_ROW] )    //当赛道宽度异常变小，判断光线干扰
                                    {
                                          if( L_LINK_NUM>R_LINK_NUM+10 && R_LINK_NUM<5 )
                                          {
                                                if( L_down_cross[0]<EXTRACT_ROW+5 )
                                                {
                                                      L_down_cross[0]=-1;
                                                      L_down_cross[1]=-1;
                                                      L_cross_flag=0;
                                                }
                                                for( W_num_col=right_edge[EXTRACT_ROW] ; W_num_col<=COL_END-3 ; W_num_col++ )   //重新搜索左边沿
                                                {
                                                      if( img_data[EXTRACT_ROW][W_num_col] == DOT_B && img_data[EXTRACT_ROW][W_num_col+1] == DOT_W && img_data[EXTRACT_ROW][W_num_col+3] == DOT_W )
                                                      {
                                                            left_edge[EXTRACT_ROW]=W_num_col;          //储存本行左边沿列坐标
                                                            
                                                            L_EDGE_NUM++;                                //左边沿个数加一
                                                            
                                                            L_LINK_NUM++;                                //连续左边沿个数加一
                                                            
                                                            L_get_flag=1;                                //左边沿搜索得到标志置  1
                                                            
                                                            L_edge_nearest=W_num_col;                  //存储最近一个左边沿的列坐标
                                                            
                                                            L_lost_num=0;                                //连续左边沿丢失数清0
                                                            
                                                            R_lost_num++;                                //当右边沿丢失时，判断为右边沿丢失
                                                            
                                                            right_edge[EXTRACT_ROW]=-1;                  //清除本行右边沿列坐标
                                                            
                                                            break;                                      //本行左边沿搜索结束
                                                      }
                                                }
                                                if( right_edge[EXTRACT_ROW]!=-1 )   //当为搜索到左边沿
                                                {
                                                      W_num=0;
                                                      for( W_num_row=EXTRACT_ROW+1 ; W_num_row<=EXTRACT_ROW+10 ; W_num_row++ )      //右边沿内倾向内搜索
                                                      {
                                                            if( W_num_row>=ROW_END )
                                                            {
                                                                  break;
                                                            }
                                                            if( left_edge[W_num_row]<left_edge[W_num_row+1] )
                                                            {
                                                                  W_num++;
                                                            }
                                                      }
                                                      if( W_num<=1 )
                                                      {
                                                            R_TURN=1;                   //右弯标志置  1
                                                            
                                                            L_end_row=EXTRACT_ROW+1;    //将上一行的行数值赋给左边沿结束行数值
                                                            
                                                            L_search_end=1;             //当左边沿右倾时，向内搜索不到白点，判断为左边沿搜索结束
                                                            R_search_end=1;             //当左边沿右倾时，向内搜索不到白点，判断为右边沿搜索结束
                                                      }
                                                }
                                          }
                                          else
                                          {
                                                if( R_LINK_NUM>L_LINK_NUM+10 && L_LINK_NUM<5 )
                                                {
                                                      if( R_down_cross[0]<EXTRACT_ROW+5 )
                                                      {
                                                            R_down_cross[0]=-1;
                                                            R_down_cross[1]=-1;
                                                            R_cross_flag=0;
                                                      }
                                                      for( W_num_col=left_edge[EXTRACT_ROW] ; W_num_col>=COL_START+3 ; W_num_col-- )   //重新搜索左边沿
                                                      {
                                                            if( img_data[EXTRACT_ROW][W_num_col] == DOT_B && img_data[EXTRACT_ROW][W_num_col-1] == DOT_W && img_data[EXTRACT_ROW][W_num_col-3] == DOT_W )
                                                            {
                                                                  right_edge[EXTRACT_ROW]=W_num_col;          //储存本行右边沿列坐标
                                                                  
                                                                  R_EDGE_NUM++;                                //右边沿个数加一
                                                                  
                                                                  R_LINK_NUM++;                                //连续右边沿个数加一
                                                                  
                                                                  R_get_flag=1;                                //右边沿搜索得到标志置  1
                                                                  
                                                                  R_edge_nearest=W_num_col;                  //存储最近一个右边沿的列坐标
                                                                  
                                                                  R_lost_num=0;                                //连续右边沿丢失数清0
                                                                  
                                                                  L_lost_num++;                                //当左边沿丢失时，判断为左边沿丢失
                                                                  
                                                                  left_edge[EXTRACT_ROW]=-1;                   //清除本行左边沿列坐标
                                                                  
                                                                  break;                                      //本行右边沿搜索结束
                                                            }
                                                      }
                                                      if( left_edge[EXTRACT_ROW]!=-1 )   //当为搜索到左边沿
                                                      {
                                                            W_num=0;
                                                            for( W_num_row=EXTRACT_ROW+1 ; W_num_row<=EXTRACT_ROW+10 ; W_num_row++ )      //右边沿内倾向内搜索
                                                            {
                                                                  if( W_num_row>=ROW_END )
                                                                  {
                                                                        break;
                                                                  }
                                                                  if( right_edge[W_num_row]>right_edge[W_num_row+1] )
                                                                  {
                                                                        W_num++;
                                                                  }
                                                            }
                                                            if( W_num<=1 )
                                                            {
                                                                  L_TURN=1;                //左弯标志置  1
                                                                  
                                                                  R_end_row=EXTRACT_ROW+1; //将上一行的行数值赋给右边沿结束行数值
                                                                  
                                                                  R_search_end=1;             //当右边沿左倾时，向左搜索不到白点，判断为右边沿搜索结束
                                                                  L_search_end=1;             //当右边沿左倾时，向左搜索不到白点，判断为左边沿搜索结束
                                                            }
                                                      }
                                                }
                                          }
                                    }
                              }
                              else if( car_flag==1 )  //如果为后车
                              {
                                    if( car_extract_able==1 )  //如果不处于类知道超车和避障模式，检测前车
                                    {
                                          if( EXTRACT_ROW<=40 && right_edge[EXTRACT_ROW]-left_edge[EXTRACT_ROW]<=1.5*bar_range_col[EXTRACT_ROW] )    //当赛道宽度异常变小，判断前车干扰
                                          {
                                                if( left_edge[EXTRACT_ROW]<=40 )
                                                {
                                                      for( W_num_row=EXTRACT_ROW ; W_num_row<=ROW_END ; W_num_row++ )   //清除错误左边沿
                                                      {
                                                            if( fabsf(inv_distance[EXTRACT_ROW]-inv_distance[W_num_row])>60 )
                                                            {
                                                                  break;
                                                            }
                                                            if( left_edge[W_num_row]!=-1 )
                                                            {
                                                                  left_edge[W_num_row]=-1;
                                                                  L_EDGE_NUM--;
                                                                  if( L_down_cross[0]==W_num_row )
                                                                  {
                                                                        L_down_cross[0]=-1;
                                                                        L_down_cross[1]=-1;
                                                                        L_cross_flag=0;
                                                                        cross_flag=0;
                                                                  }
                                                            }
                                                            else
                                                            {
                                                                  break;
                                                            }
                                                      }
                                                      L_LINK_NUM=0;
                                                      L_lost_num++;         //判断为左边沿丢失
                                                }
                                                else
                                                {
                                                      if( right_edge[EXTRACT_ROW]>=119 )
                                                      {
                                                            for( W_num_row=EXTRACT_ROW ; W_num_row<=ROW_END ; W_num_row++ )   //清除错误右边沿
                                                            {
                                                                  if( fabsf(inv_distance[EXTRACT_ROW]-inv_distance[W_num_row])>60 )
                                                                  {
                                                                        break;
                                                                  }
                                                                  if( right_edge[W_num_row]!=-1 )
                                                                  {
                                                                        right_edge[W_num_row]=-1;
                                                                        R_EDGE_NUM--;
                                                                        if( R_down_cross[0]==W_num_row )
                                                                        {
                                                                              R_down_cross[0]=-1;
                                                                              R_down_cross[1]=-1;
                                                                              R_cross_flag=0;
                                                                              cross_flag=0;
                                                                        }
                                                                  }
                                                                  else
                                                                  {
                                                                        break;
                                                                  }
                                                            }
                                                            R_LINK_NUM=0;
                                                            R_lost_num++;         //判断为右边沿丢失
                                                      }
                                                      else
                                                      {
                                                            if( R_LINK_NUM>L_LINK_NUM+10 )
                                                            {
                                                                  for( W_num_row=EXTRACT_ROW ; W_num_row<=ROW_END ; W_num_row++ )   //清除错误左边沿
                                                                  {
                                                                        if( fabsf(inv_distance[EXTRACT_ROW]-inv_distance[W_num_row])>60 )
                                                                        {
                                                                              break;
                                                                        }
                                                                        if( left_edge[W_num_row]!=-1 )
                                                                        {
                                                                              left_edge[W_num_row]=-1;
                                                                              L_EDGE_NUM--;
                                                                              if( L_down_cross[0]==W_num_row )
                                                                              {
                                                                                    L_down_cross[0]=-1;
                                                                                    L_down_cross[1]=-1;
                                                                                    L_cross_flag=0;
                                                                                    cross_flag=0;
                                                                              }
                                                                        }
                                                                        else
                                                                        {
                                                                              break;
                                                                        }
                                                                  }
                                                                  L_LINK_NUM=0;
                                                                  L_lost_num++;         //判断为左边沿丢失
                                                            }
                                                            else if( L_LINK_NUM>R_LINK_NUM+10 )
                                                            {
                                                                  for( W_num_row=EXTRACT_ROW ; W_num_row<=ROW_END ; W_num_row++ )   //清除错误右边沿
                                                                  {
                                                                        if( fabsf(inv_distance[EXTRACT_ROW]-inv_distance[W_num_row])>60 )
                                                                        {
                                                                              break;
                                                                        }
                                                                        if( right_edge[W_num_row]!=-1 )
                                                                        {
                                                                              right_edge[W_num_row]=-1;
                                                                              R_EDGE_NUM--;
                                                                              if( R_down_cross[0]==W_num_row )
                                                                              {
                                                                                    R_down_cross[0]=-1;
                                                                                    R_down_cross[1]=-1;
                                                                                    R_cross_flag=0;
                                                                                    cross_flag=0;
                                                                              }
                                                                        }
                                                                        else
                                                                        {
                                                                              break;
                                                                        }
                                                                  }
                                                                  R_LINK_NUM=0;
                                                                  R_lost_num++;         //判断为右边沿丢失
                                                            }
                                                      }
                                                }
                                          }
                                    }
                              }
                        }
                  }
            }
            
            end_line_extract();      //检车停车线
            barrier_up_extract();    //障碍物上边沿行检测
            barrier_down_extract();  //障碍物下边沿行检测
            
            if( left_edge[EXTRACT_ROW]==-1 && left_edge[EXTRACT_ROW+1]!=-1 && left_edge[EXTRACT_ROW+1]>L_OUT_END)
            {
                  if( LOOP_TEMP==0 && L_cross_flag==0 && R_cross_flag==0 && L_barrier_flag==0 && R_barrier_flag==0 )
                  {
                        L_search_end=1;
                  }
            }
            
            if( right_edge[EXTRACT_ROW]==-1 && right_edge[EXTRACT_ROW+1]!=-1 && right_edge[EXTRACT_ROW+1]<R_OUT_END)
            {
                  if( LOOP_TEMP==0 && L_cross_flag==0 && R_cross_flag==0 && L_barrier_flag==0 && R_barrier_flag==0 )
                  {
                        R_search_end=1;
                  }
            }
            
            if( inv_distance[EXTRACT_ROW]<60 )    //当边沿在图像近处结束搜索时，强制继续搜索
            {
                  if( L_search_end==1 || R_search_end==1 )
                  {
                        L_search_end=0;
                        R_search_end=0;
                        L_TURN=0;
                        R_TURN=0;
                  }
            }
            
            if( L_lost_num>=5 && R_lost_num>=5 && L_up_cross[0]==-1 && R_up_cross[0]==-1 && l_cross_able==1 && r_cross_able==1 )  //判断是否为十字
            {
                  L_search_end=0;
                  R_search_end=0;
                  L_cross_flag=1; //如果左右边沿同时丢边数大于阀值，且不是环道，则判断为十字
                  R_cross_flag=1; //如果左右边沿同时丢边数大于阀值，且不是环道，则判断为十字
                  cross_flag=1;   //如果左右边沿同时丢边数大于阀值，且不是环道，则判断为十字
            }
            if( overtake_cross_flag )   //判断是否进入十字超车
            {
                  if(car_L_pull_over_flag && !car_R_pull_over_flag)   //判断为左侧靠边
                  {
                        if( L_lost_num>=5 && L_up_cross[0]==-1 )  //十字超车判断是否为十字
                        {
                              L_search_end=0;
                              L_cross_flag=1; //如果左右边沿同时丢边数大于阀值，且不是环道，则判断为十字
                              cross_flag=1;   //如果左右边沿同时丢边数大于阀值，且不是环道，则判断为十字
                        }
                  }
                  else if(car_R_pull_over_flag && !car_L_pull_over_flag)   //判断为右侧靠边
                  {
                        if( R_lost_num>=5 && R_up_cross[0]==-1 )  //十字超车判断是否为十字
                        {
                              R_search_end=0;
                              R_cross_flag=1; //如果左右边沿同时丢边数大于阀值，且不是环道，则判断为十字
                              cross_flag=1;   //如果左右边沿同时丢边数大于阀值，且不是环道，则判断为十字
                        }
                  }
            }
            else
            {
                  if( EXTRACT_ROW<=ROW_END-1 )
                  {
                        if( L_down_cross[0]==-1 &&  R_down_cross[0]==-1 )
                        {
                              if( left_edge[EXTRACT_ROW]!=-1 && left_edge[EXTRACT_ROW+1]!=-1 && right_edge[EXTRACT_ROW]!=-1 && right_edge[EXTRACT_ROW+1]!=-1 )
                              {
                                    if( l_cross_able==1 && left_edge[EXTRACT_ROW]<left_edge[EXTRACT_ROW+1] )
                                    {
                                          l_cross_1++;
                                          l_cross_edge=l_cross_edge+abs(left_edge[EXTRACT_ROW+1]-left_edge[EXTRACT_ROW]);
                                          if( l_cross_1>=8 && l_cross_edge>=15 )
                                          {
                                                l_cross_able=0;
                                          }
                                    }
                                    
                                    if( r_cross_able==1 && right_edge[EXTRACT_ROW]>right_edge[EXTRACT_ROW+1] )
                                    {
                                          r_cross_1++;
                                          r_cross_edge=r_cross_edge+abs(right_edge[EXTRACT_ROW]-right_edge[EXTRACT_ROW+1]);
                                          if( r_cross_1>=8 && r_cross_edge>=15 )
                                          {
                                                r_cross_able=0;
                                          }
                                    }
                              }
                        }
                  }
            }
            
            if( loop_able==1 && LOOP_FLAG==0 && LOOP_IN==0 && EXTRACT_ROW>5 && L_barrier_flag==0 && R_barrier_flag==0 && END_LINE_TEMP==0 )    //判断是否为环型赛道
            {
                  if( L_LINK_NUM>=10 && R_LINK_NUM>=10 )
                  {
                        W_num=0;
                        cross_row=-1;
                        for( W_num_row=EXTRACT_ROW ; W_num_row<=EXTRACT_ROW+10 ; W_num_row++ )
                        {
                              if( W_num_row>ROW_END-2 )
                              {
                                    break;
                              }
                              if( left_edge[W_num_row]!=-1 && left_edge[W_num_row+1]!=-1 )
                              {
                                    if( left_edge[W_num_row+1]-left_edge[W_num_row]>=2 )
                                    {
                                          W_num++;
                                    }
                                    else
                                    {
                                          if( left_edge[W_num_row+2]!=-1 )    //结束搜索
                                          {
                                                if( left_edge[W_num_row]-left_edge[W_num_row+1]>=0 && left_edge[W_num_row+1]-left_edge[W_num_row+2]>=0 )
                                                {
                                                      cross_row=W_num_row;
                                                      break;
                                                }
                                          }
                                    }
                              }
                        }
                        W_num_col=0;
                        cross_col=-1;
                        for( W_num_row=EXTRACT_ROW ; W_num_row<=EXTRACT_ROW+10 ; W_num_row++ )
                        {
                              if( right_edge[W_num_row]!=-1 && right_edge[W_num_row+1]!=-1 )
                              {
                                    if( right_edge[W_num_row]-right_edge[W_num_row+1]>=2 )
                                    {
                                          W_num_col++;
                                    }
                                    else
                                    {
                                          if( right_edge[W_num_row+2]!=-1 )    //结束搜索
                                          {
                                                if( right_edge[W_num_row]-right_edge[W_num_row+1]<=0 && right_edge[W_num_row+1]-right_edge[W_num_row+2]<=0 )
                                                {
                                                      cross_col=W_num_row;
                                                      break;
                                                }
                                          }
                                    }
                              }
                        }
                        
                        if( W_num>=4 && W_num_col>=4 && cross_row!=-1 && cross_col!=-1 )
                        {
                              if( cross_row>cross_col )
                              {
                                    cross_col=0;
                              }
                              else
                              {
                                    cross_row=cross_col;
                                    cross_col=0;
                              }
                              for( W_num_row=cross_row ; W_num_row<=cross_row+10 ; W_num_row++ )
                              {
                                    if( W_num_row>ROW_END )
                                    {
                                          break;
                                    }
                                    if( left_edge[W_num_row]!=-1 && right_edge[W_num_row]!=-1 )
                                    {
                                          if( abs(right_edge[W_num_row]-left_edge[W_num_row])<2.5*bar_range_col[W_num_row] )
                                          {
                                                if( abs(right_edge[W_num_row]-left_edge[W_num_row])>1.5*bar_range_col[W_num_row] )
                                                {
                                                      cross_col++;
                                                }
                                          }
                                    }
                              }
                              if( cross_col>=8 )
                              {
                                    LOOP_FLAG=1;                      //判断为环型赛道
                                    
                                    LOOP_IN_ROW=cross_row;            //环型赛道入口行赋值
                              }
                        }
                  }
            }
      }
      
      cross_edge_add();     //十字边沿补充（修正十字错误边沿）
      
      edge_error_process(); //边沿错误处理（修正普通赛道错误边沿，修正左右拐，识别S弯）
      
      road_judge();         //判断赛道类型（识别直道，坡道，小S，出弯口）
      
      bar_car();            //超车判断
      
      bar_process();        //障碍物边沿补充
      
      road_num();           //赛道类型计数（十字，障碍物个数）
      
      loop_judge();         //环型赛道判断
      
      
      
      
      
      
      /*蜂鸣器*/
      
//      if( RAMP_TEMP==1 && angle_aaa[0]==0 )   //END_LINE_FLAG    //L_cross_flag==1 || R_cross_flag==1 || cross_flag==1      //RAMP_FLAG   //RAMP_TEMP  //down_flag  //LOOP_FLAG
//      {
//            Bee_flag=1;
//      }
//      if( RAMP_TEMP==0 && angle_aaa[0]==1 )   //END_LINE_FLAG    //L_cross_flag==1 || R_cross_flag==1 || cross_flag==1      //RAMP_FLAG   //RAMP_TEMP  //down_flag  //LOOP_FLAG
//      {
//            Bee_flag=1;
//      }
      
      
      
      
      
      
      /**检测十字和环道**/
//      if( L_down_cross[1]!=-1 && L_cross_k!=10 )
//      {
//            if( R_down_cross[1]!=-1 && R_cross_k!=10 )
//            {
//                  double aa;
//                  Site_xy xy_l,xy_r;
//                  Site_xy1 xy1,xy2;
//                  int16 l_edge,r_edge;
//                  int16 r_start,r_end,b_point,edge_able=0;
//                  xy_l=get_inv_img( L_down_cross[0] , L_down_cross[1] );
//                  xy_r=get_inv_img( R_down_cross[0] , R_down_cross[1] );
//                  
//                  if( xy_l.x<320 && xy_r.x<320 && fabsf(xy_l.y-xy_r.y)>120 && fabsf(xy_l.y-xy_r.y)<200 )
//                  {
//                        aa=0.5*(xy_l.x+xy_l.x)+220;
//                        xy1=get_invinv_img( aa , 0 );
//                        r_start=xy1.x;
//                        if( r_start<0 )
//                        {
//                              r_start=0;
//                        }
//                        aa=0.5*(xy_l.x+xy_l.x)+400;
//                        xy1=get_invinv_img( aa , 0 );
//                        r_end=xy1.x;
//                        if( r_end<0 )
//                        {
//                              r_end=0;
//                        }
//                        
//                        for( W_num_row=r_start ; W_num_row>=r_end ; W_num_row-- )
//                        {
//                              aa=xy_l.y-L_cross_k*fabsf(inv_distance[W_num_row]-xy_l.x);
//                              xy1=get_invinv_img( inv_distance[W_num_row] , aa );
//                              aa=xy_r.y-R_cross_k*fabsf(inv_distance[W_num_row]-xy_r.x);
//                              xy2=get_invinv_img( inv_distance[W_num_row] , aa );
//                              if( xy1.y>=COL_END )
//                              {
//                                    break;
//                              }
//                              else
//                              {
//                                    if( xy1.y<COL_START )
//                                    {
//                                          l_edge=COL_START;
//                                    }
//                                    else
//                                    {
//                                          l_edge=xy1.y;
//                                    }
//                              }
//                              
//                              if( xy2.y<=COL_START )
//                              {
//                                    break;
//                              }
//                              else
//                              {
//                                    if( xy2.y>COL_END )
//                                    {
//                                          r_edge=COL_END;
//                                    }
//                                    else
//                                    {
//                                          r_edge=xy2.y;
//                                    }
//                              }
//                              for( W_num_col=l_edge ; W_num_col<=r_edge ; W_num_col++ )
//                              {
//                                    img_data[W_num_row][W_num_col]=DOT_M;
//                              }
//                        }
//                  }
//            }
//            else
//            {
//                  double xxx=0,yyy=0;
//                  int16 xx;
//                  
//                  Site_xy1 xy2;
//                  Site_xy xy1;
//                  
//                  xy1=get_inv_img( L_down_cross[0] , L_down_cross[1] );
//                  
//                  for( W_num_row=L_down_cross[0] ; W_num_row>=ROW_START+5 ; W_num_row-- )
//                  {
//                        if( fabsf(inv_distance[W_num_row]-xy1.x)>cross_k_2 )
//                        {
//                              //break;
//                        }
//                        else
//                        {
//                              xxx=cross_k_1*fabsf(inv_distance[W_num_row]-xy1.x);
//                        }
//                        yyy=xy1.y-L_cross_k*fabsf(inv_distance[W_num_row]-xy1.x);
//                        xy2=get_invinv_img( inv_distance[W_num_row] , yyy );
//                        if( xy2.y>0 && xy2.y<159 )
//                        {
//                              img_data[W_num_row][xy2.y]=DOT_M;
//                              img_data[W_num_row][xy2.y-1]=DOT_M;
//                        }
//                        
//                        if( fabsf(inv_distance[W_num_row]-xy1.x)>180 && fabsf(inv_distance[W_num_row]-xy1.x)<400 )
//                        {
//                              for( W_num_col=xy2.y ; W_num_col<=xy2.y+2.0*bar_range_col[W_num_row] ; W_num_col++ )
//                              {
//                                    if( W_num_col>0 && W_num_col<159 )
//                                    {
//                                          img_data[W_num_row][W_num_col]=DOT_M;
//                                    }
//                              }
//                        }
//                        
//                        yyy=xy1.y-L_cross_k*fabsf(inv_distance[W_num_row]-xy1.x)+xxx;
//                        xy2=get_invinv_img( inv_distance[W_num_row] , yyy );
//                        if( xy2.y>0 && xy2.y<159 )
//                        {
//                              img_data[W_num_row][xy2.y]=DOT_M;
//                              img_data[W_num_row][xy2.y-1]=DOT_M;
//                        }
//                        
//                        xy2=get_invinv_img( inv_distance[W_num_row] , yyy+cross_k_add );
//                        if( xy2.y>0 && xy2.y<159 )
//                        {
//                              img_data[W_num_row][xy2.y]=DOT_M;
//                              img_data[W_num_row][xy2.y-1]=DOT_M;
//                        }
//                        xy2=get_invinv_img( inv_distance[W_num_row] , yyy-cross_k_add_1 );
//                        if( xy2.y>0 && xy2.y<159 )
//                        {
//                              img_data[W_num_row][xy2.y]=DOT_M;
//                              img_data[W_num_row][xy2.y-1]=DOT_M;
//                        }
//                  }
//            }
//      }
//      else
//      {
//            if( R_down_cross[1]!=-1 && R_cross_k!=10 )
//            {
//                  double xxx=0,yyy=0;
//                  int16 xx;
//                  
//                  Site_xy1 xy2;
//                  Site_xy xy1;
//                  
//                  xy1=get_inv_img( R_down_cross[0] , R_down_cross[1] );
//                  
//                  for( W_num_row=R_down_cross[0] ; W_num_row>=ROW_START ; W_num_row-- )
//                  {
//                        if( fabsf(inv_distance[W_num_row]-xy1.x)>cross_k_2 )
//                        {
//                              //break;
//                        }
//                        else
//                        {
//                              xxx=cross_k_1*fabsf(inv_distance[W_num_row]-xy1.x);
//                        }
//                        yyy=xy1.y-R_cross_k*fabsf(inv_distance[W_num_row]-xy1.x);  //cross_k_add
//                        xy2=get_invinv_img( inv_distance[W_num_row] , yyy );
//                        if( xy2.y>0 && xy2.y<159 )
//                        {
//                              img_data[W_num_row][xy2.y]=DOT_M;
//                              img_data[W_num_row][xy2.y+1]=DOT_M;
//                        }
//                        
//                        if( fabsf(inv_distance[W_num_row]-xy1.x)>180 && fabsf(inv_distance[W_num_row]-xy1.x)<400 )
//                        {
//                              for( W_num_col=xy2.y ; W_num_col>=xy2.y-2.0*bar_range_col[W_num_row] ; W_num_col-- )
//                              {
//                                    if( W_num_col>0 && W_num_col<159 )
//                                    {
//                                          img_data[W_num_row][W_num_col]=DOT_M;
//                                    }
//                              }
//                        }
//                        
//                        yyy=xy1.y-R_cross_k*fabsf(inv_distance[W_num_row]-xy1.x)-xxx;  //cross_k_add
//                        xy2=get_invinv_img( inv_distance[W_num_row] , yyy );
//                        if( xy2.y>0 && xy2.y<159 )
//                        {
//                              img_data[W_num_row][xy2.y]=DOT_M;
//                              img_data[W_num_row][xy2.y+1]=DOT_M;
//                        }
//                        
//                        xy2=get_invinv_img( inv_distance[W_num_row] , yyy-cross_k_add );
//                        if( xy2.y>0 && xy2.y<159 )
//                        {
//                              img_data[W_num_row][xy2.y]=DOT_M;
//                              img_data[W_num_row][xy2.y+1]=DOT_M;
//                        }
//                        xy2=get_invinv_img( inv_distance[W_num_row] , yyy+cross_k_add_1 );
//                        if( xy2.y>0 && xy2.y<159 )
//                        {
//                              img_data[W_num_row][xy2.y]=DOT_M;
//                              img_data[W_num_row][xy2.y+1]=DOT_M;
//                        }
//                  }
//            }
//      }
//      
      
      
      
      
      /**检测逆透视**/
//      double edge_1[CAMERA_H],edge_2[CAMERA_H],edge_3[CAMERA_H];
//       
//      for( W_num_row=ROW_START ; W_num_row<=ROW_END ; W_num_row++ )
//      {
//            if( right_edge[W_num_row]!=-1 && left_edge[W_num_row]!=-1 )
//            {
//                  edge_1[W_num_row]=1.0*(left_edge[W_num_row]+right_edge[W_num_row])/2;
//                  edge_2[W_num_row]=1.0*(right_edge[W_num_row]-left_edge[W_num_row])/2;
//            }
//            else
//            {
//                  edge_1[W_num_row]=-1;
//                  edge_2[W_num_row]=-1;
//            }
//      }
//      double xxx=0,yyy=0;
//      for( W_num_row=ROW_START+5 ; W_num_row<=ROW_END ; W_num_row++ )
//      {
//            if( edge_1[W_num_row]!=-1 )
//            {
//                  xxx=xxx+edge_1[W_num_row];
//                  yyy++;
//            }
//      }
//      xxx=xxx/yyy;
//      for( W_num_row=ROW_START ; W_num_row<=ROW_END ; W_num_row++ )
//      {
//            if( edge_2[W_num_row]!=-1 )
//            {
//                  edge_3[W_num_row]=2.0*(edge_2[W_num_row]-bar_range_col[W_num_row]);
//            }
//            else
//            {
//                  edge_3[W_num_row]=-1;
//            }
//      }
//      Site_xy xy3;
//      double edge_4[CAMERA_H],edge_5[CAMERA_H];
//      double edge_l=0,edge_r=0,edge_l_f=0,edge_r_f=0;
//      int16 edge_l_num=0,edge_r_num=0;
//      for( W_num_row=ROW_START ; W_num_row<=ROW_END ; W_num_row++ )
//      {
//            if( left_edge[W_num_row]!=-1 )
//            {
//                  xy3=get_inv_img( W_num_row , left_edge[W_num_row] );
//                  edge_4[W_num_row]=xy3.y;
//                  edge_l_f=edge_l_f+fabsf(80.0+edge_4[W_num_row]);
//                  edge_l=edge_l+edge_4[W_num_row];
//                  edge_l_num++;
//            }
//            else
//            {
//                  edge_4[W_num_row]=-1;
//            }
//            if( right_edge[W_num_row]!=-1 )
//            {
//                  xy3=get_inv_img( W_num_row , right_edge[W_num_row] );
//                  edge_5[W_num_row]=xy3.y;
//                  edge_r_f=edge_r_f+fabsf(80.0-edge_5[W_num_row]);
//                  edge_r=edge_r+edge_5[W_num_row];
//                  edge_r_num++;
//            }
//            else
//            {
//                  edge_5[W_num_row]=-1;
//            }
//      }
//      angle_aaa[0]=edge_l/edge_l_num;
//      angle_aaa[1]=edge_r/edge_r_num;
//      angle_aaa[2]=100.0*edge_l_f/60;
//      angle_aaa[3]=100.0*edge_r_f/60;
      
      
}

/***********************************************************************************************************************************/

void L_down_cross_judge()           //此函数为左边沿下方直角拐点判断函数
{
      if( l_cross_able==1 && L_up_cross[0]==-1 && L_down_cross[0]==-1 && left_edge[EXTRACT_ROW+1]>L_LOST_CROSS && EXTRACT_ROW>CROSS_DOWN_ROW && EXTRACT_ROW<=CROSS_START_ROW && L_barrier_flag==0 && R_barrier_flag==0 )      //当要判断的点在范围内
      {
            W_num=0;            //清除白块计数值
            for( W_num_row=EXTRACT_ROW+2 ; W_num_row<=EXTRACT_ROW+9 ; W_num_row++ )  //检测前8行内倾数
            {
                  if( left_edge[W_num_row]!=-1 )
                  {
                        if( left_edge[W_num_row-1]-left_edge[W_num_row]<0 )
                        {
                              W_num++;
                        }
                  }
                  else
                  {
                        W_num=10;
                        break;
                  }
            }
            
            if( W_num<=1 )   //前几行左边沿内倾数满足阀值
            {
                  for( W_num=0 ; W_num<=2 ; W_num++ )
                  {
                        inv_cross[W_num]=-1;
                  }
                  uint8 num=0;        //记录丢边点个数
                  W_num=0;            //记录内倾点个数
                  if( left_edge[EXTRACT_ROW]!=-1 )
                  {
                        if( EXTRACT_ROW>=15 && left_edge[EXTRACT_ROW+1]-left_edge[EXTRACT_ROW]<CROSS_DOWN_COL )
                        {
                              EXTRACT_COL=left_edge[EXTRACT_ROW];
                              for( W_num_row=EXTRACT_ROW-1 ; W_num_row>=EXTRACT_ROW-6 ; W_num_row-- )   //向上搜索边沿
                              {
                                    if( img_data[W_num_row][EXTRACT_COL+1]==DOT_B )
                                    {
                                          W_num++;
                                          continue;
                                    }
                                    for( W_num_col=EXTRACT_COL ; W_num_col>=COL_START ; W_num_col-- )
                                    {
                                          if( img_data[W_num_row][W_num_col]==DOT_B && img_data[W_num_row][W_num_col+1]==DOT_W )
                                          {
                                                EXTRACT_COL=W_num_col;
                                                break;
                                          }
                                    }
                                    if( W_num_col<COL_START )
                                    {
                                          if( img_data[W_num_row][COL_START]==DOT_W && img_data[W_num_row][COL_START+1]==DOT_W )
                                          {
                                                EXTRACT_COL=COL_START;
                                                num++;
                                          }
                                    }
                                    if( (EXTRACT_ROW-W_num_row)==3 )
                                    {
                                          inv_cross[0]=EXTRACT_COL;
                                    }
                                    if( (EXTRACT_ROW-W_num_row)==4 )
                                    {
                                          inv_cross[1]=EXTRACT_COL;
                                    }
                                    if( (EXTRACT_ROW-W_num_row)==5 )
                                    {
                                          inv_cross[2]=EXTRACT_COL;
                                    }
                              }
                        }
                  }
                  else
                  {
                        if( EXTRACT_ROW>=15 )
                        {
                              EXTRACT_COL=COL_START;
                              for( W_num_row=EXTRACT_ROW-1 ; W_num_row>=EXTRACT_ROW-6 ; W_num_row-- )   //向上搜索边沿
                              {
                                    if( img_data[W_num_row][EXTRACT_COL+1]==DOT_B )
                                    {
                                          W_num++;
                                          continue;
                                    }
                                    for( W_num_col=EXTRACT_COL ; W_num_col>=COL_START ; W_num_col-- )
                                    {
                                          if( img_data[W_num_row][W_num_col]==DOT_B && img_data[W_num_row][W_num_col+1]==DOT_W )
                                          {
                                                EXTRACT_COL=W_num_col;
                                                break;
                                          }
                                    }
                                    if( W_num_col<COL_START )
                                    {
                                          if( img_data[W_num_row][COL_START]==DOT_W && img_data[W_num_row][COL_START+1]==DOT_W )
                                          {
                                                EXTRACT_COL=COL_START;
                                                num++;
                                          }
                                    }
                                    if( (EXTRACT_ROW-W_num_row)==3 )
                                    {
                                          inv_cross[0]=EXTRACT_COL;
                                    }
                                    if( (EXTRACT_ROW-W_num_row)==4 )
                                    {
                                          inv_cross[1]=EXTRACT_COL;
                                    }
                                    if( (EXTRACT_ROW-W_num_row)==5 )
                                    {
                                          inv_cross[2]=EXTRACT_COL;
                                    }
                              }
                        }
                  }
                  if( W_num<=1 )   //上搜白点数满足阀值
                  {
                        if( left_edge[EXTRACT_ROW]==-1 || left_edge[EXTRACT_ROW+1]-left_edge[EXTRACT_ROW]>=CROSS_DOWN_COL )       //当边沿剧烈变化时，判断为下十字直角
                        {
                              L_down_cross[0]=EXTRACT_ROW+1;                           //存储左边沿十字下方直角拐点行坐标
                              L_down_cross[1]=left_edge[EXTRACT_ROW+1];                //存储左边沿十字下方直角拐点列坐标
                              
                              L_cross_flag=1;      //当符合条件时，判断为边沿突变，左边沿十字标志置  1
                        }
                        else
                        {
                              if( num>=4 )    //上搜边沿数过少，直接判断为十字
                              {
                                    L_down_cross[0]=EXTRACT_ROW+1;                           //存储左边沿十字下方直角拐点行坐标
                                    L_down_cross[1]=left_edge[EXTRACT_ROW+1];                //存储左边沿十字下方直角拐点列坐标
                                    
                                    L_cross_flag=1;      //当符合条件时，判断为边沿突变，左边沿十字标志置  1
                              }
                              else              //上搜边沿数足够，逆透视校正角度判断十字
                              {
                                    int16 k1=0,k2=0,k3=0;
                                    double x=0.0,y=0.0,x1,y1,k=0.0;
                                    Site_xy xy1,xy2;
                                    int16 row_add;
                                    row_add=EXTRACT_ROW/12;
                                    
                                    xy1=get_inv_img( EXTRACT_ROW+1 , left_edge[EXTRACT_ROW+1] );    //获取拐点逆透视坐标
                                    
                                    W_num_row=4;
                                    
                                    for( W_num=1 ; W_num<=W_num_row ; W_num++ )   //获取拐点向下的拟合斜率
                                    {
                                          W_num_col=EXTRACT_ROW+W_num+row_add;
                                          if( W_num_col>ROW_END )
                                          {
                                                break;
                                          }
                                          if( left_edge[W_num_col]!=-1 )
                                          {
                                                xy2=get_inv_img( W_num_col , left_edge[W_num_col] );
                                                x1=xy2.x-xy1.x;
                                                y1=xy2.y-xy1.y;
                                                x=x+x1*y1;
                                                y=y+x1*x1;
                                          }
                                    }
                                    if( x!=0 )
                                    {
                                          k=(-1)*x/y;    //得到拐点向下的拟合斜率
                                    }
                                    else
                                    {
                                          k==0;
                                    }
                                    if( k>=0 )
                                    {
                                          for( W_num=0 ; W_num<=29 ; W_num++ )
                                          {
                                                if( k<=tan_table[W_num] )
                                                {
                                                      k1=180-3*W_num;
                                                      break;
                                                }
                                          }
                                          if( W_num==30 )
                                          {
                                                k1=90;
                                          }
                                    }
                                    else
                                    {
                                          k=fabsf(k);
                                          for( W_num=0 ; W_num<=29 ; W_num++ )
                                          {
                                                if( k<=tan_table[W_num] )
                                                {
                                                      k1=180+3*W_num;
                                                      break;
                                                }
                                          }
                                          if( W_num==30 )
                                          {
                                                k1=270;
                                          }
                                    }
                                    
                                    x=0.0;
                                    y=0.0;
                                    for( W_num=0 ; W_num<=2 ; W_num++ )   //获取拐点向上的拟合斜率
                                    {
                                          if( inv_cross[W_num]!=-1 )
                                          {
                                                xy2=get_inv_img( EXTRACT_ROW-W_num-2 , inv_cross[W_num] );
                                                x1=xy2.x-xy1.x;
                                                y1=xy2.y-xy1.y;
                                                x=x+x1*y1;
                                                y=y+x1*x1;
                                          }
                                    }
                                    if( x!=0 )
                                    {
                                          k=(-1)*x/y;    //得到拐点向上的拟合斜率
                                    }
                                    else
                                    {
                                          k==0;
                                    }
                                    if( k>=0 )
                                    {
                                          for( W_num=0 ; W_num<=29 ; W_num++ )
                                          {
                                                if( k<=tan_table[W_num] )
                                                {
                                                      k2=-3*W_num;
                                                      break;
                                                }
                                          }
                                          if( W_num==30 )
                                          {
                                                k2=-90;
                                          }
                                    }
                                    else
                                    {
                                          k=fabsf(k);
                                          for( W_num=0 ; W_num<=29 ; W_num++ )
                                          {
                                                if( k<=tan_table[W_num] )
                                                {
                                                      k2=3*W_num;
                                                      break;
                                                }
                                          }
                                          if( W_num==30 )
                                          {
                                                k2=90;
                                          }
                                    }
                                    
                                    k3=abs(360+k2-k1);
                                    angle_aaa[0]=k3;
                                    if( k3<=117 )  //通过角度校正判断为十字
                                    {
                                          L_down_cross[0]=EXTRACT_ROW+1;                           //存储左边沿十字下方直角拐点行坐标
                                          L_down_cross[1]=left_edge[EXTRACT_ROW+1];                //存储左边沿十字下方直角拐点列坐标
                                          
                                          L_cross_flag=1;      //当符合条件时，判断为边沿突变，左边沿十字标志置  1
                                    }
                              }
                        }
                        
                        if( L_down_cross[0]!=-1 )
                        {
                              double x=0.0,y=0.0,x1,y1,k=0.0,a;
                              Site_xy xy1,xy2;
                              Site_xy1 xy3;
                              
                              xy1=get_inv_img( L_down_cross[0] , L_down_cross[1] );    //获取拐点逆透视坐标
                              
                              a=xy1.x-50;
                              xy3=get_invinv_img( a , 0 );
                              if( xy3.x-L_down_cross[0]>15 )
                              {
                                    W_num_row=15;
                              }
                              else
                              {
                                    if( xy3.x-L_down_cross[0]<3 )
                                    {
                                          W_num_row=3;
                                    }
                                    else
                                    {
                                          W_num_row=abs(xy3.x-L_down_cross[0]);
                                    }
                              }
                              
                              for( W_num=1 ; W_num<=W_num_row ; W_num++ )   //获取拐点向下的拟合斜率
                              {
                                    W_num_col=EXTRACT_ROW+W_num+1;
                                    if( W_num_col>ROW_END )
                                    {
                                          break;
                                    }
                                    if( left_edge[W_num_col]!=-1 )
                                    {
                                          xy2=get_inv_img( W_num_col , left_edge[W_num_col] );
                                          x1=xy2.x-xy1.x;
                                          y1=xy2.y-xy1.y;
                                          x=x+x1*y1;
                                          y=y+x1*x1;
                                    }
                                    else
                                    {
                                          break;
                                    }
                              }
                              if( x!=0 )
                              {
                                    k=(-1)*x/y;    //得到拐点向下的拟合斜率
                              }
                              else
                              {
                                    k==0;
                              }
                              
                              L_cross_k=k;        //取向下拟合斜率
                        }
                  }
            }
      }
}

/***********************************************************************************************************************************/

void L_up_cross_judge()             //此函数为左边沿上方直角拐点判断函数
{
      if( l_cross_able==1 && L_cross_flag==1 && EXTRACT_ROW<=CROSS_START_ROW && EXTRACT_ROW>CROSS_UP_ROW )         //当左边沿十字标为1时
      {
            if( L_down_cross[0]!=-1 )   //获取下十字拐点
            {
                  if( fabsf(inv_distance[EXTRACT_ROW]-inv_distance[L_down_cross[0]])<cross_k_2 )
                  {
                        if( L_LINK_NUM>=2 && L_LINK_NUM<(L_down_cross[0]-EXTRACT_ROW-1) )     //当连续的边沿大于1时
                        {
                              if( L_up_cross[0]!=-1 )
                              {
                                    Site_xy xy1,xy2;
                                    xy1=get_inv_img(EXTRACT_ROW,left_edge[EXTRACT_ROW]);
                                    xy2=get_inv_img(L_up_cross[0],L_up_cross[1]);
                                    if( 2.0*xy1.x-xy1.y<=2.0*xy2.x-xy2.y )     //符合条件时，判断为十字直角拐点
                                    {
                                          double xx,aa;
                                          xy2=get_inv_img( L_down_cross[0] , L_down_cross[1] );
                                          
                                          if( fabsf(xy1.x-xy2.x)>cross_k_2 )
                                          {
                                                aa=cross_k_1*cross_k_2;
                                          }
                                          else
                                          {
                                                aa=cross_k_1*fabsf(xy1.x-xy2.x);
                                          }
                                          xx=xy2.y-L_cross_k*fabsf(xy1.x-xy2.x)+cross_k_add+aa;
                                          
                                          if( xy1.y<=xx )
                                          {
                                                L_up_cross[0]=EXTRACT_ROW;                        //存储左边沿十字下方直角拐点行坐标
                                                L_up_cross[1]=left_edge[EXTRACT_ROW];             //存储左边沿十字下方直角拐点列坐标
                                          }
                                    }
                              }
                              else
                              {
                                    double xx,aa;
                                    Site_xy xy1,xy2;
                                    xy1=get_inv_img(EXTRACT_ROW,left_edge[EXTRACT_ROW]);
                                    xy2=get_inv_img( L_down_cross[0] , L_down_cross[1] );
                                    
                                    if( fabsf(xy1.x-xy2.x)>cross_k_2 )
                                    {
                                          aa=cross_k_1*cross_k_2;
                                    }
                                    else
                                    {
                                          aa=cross_k_1*fabsf(xy1.x-xy2.x);
                                    }
                                    xx=xy2.y-L_cross_k*fabsf(xy1.x-xy2.x)+cross_k_add+aa;
                                    
                                    if( xy1.y<=xx )
                                    {
                                          L_up_cross[0]=EXTRACT_ROW;                        //存储左边沿十字下方直角拐点行坐标
                                          L_up_cross[1]=left_edge[EXTRACT_ROW];             //存储左边沿十字下方直角拐点列坐标
                                    }
                              }
                        }
                  }
                  else
                  {
                        L_cross_flag=0;
                        cross_flag=1;
                  }
            }
            else   //未获取下十字拐点
            {
                  if( L_up_cross[0]!=-1 )
                  {
                        if( L_LINK_NUM>=2 )     //当连续的边沿大于1时
                        {
                              Site_xy xy1,xy2;
                              xy1=get_inv_img(EXTRACT_ROW,left_edge[EXTRACT_ROW]);
                              xy2=get_inv_img(L_up_cross[0],L_up_cross[1]);
                              if( 2.0*xy1.x-xy1.y<=2.0*xy2.x-xy2.y )     //符合条件时，判断为十字直角拐点
                              {
                                    L_up_cross[0]=EXTRACT_ROW;                        //存储左边沿十字下方直角拐点行坐标
                                    L_up_cross[1]=left_edge[EXTRACT_ROW];             //存储左边沿十字下方直角拐点列坐标
                              }
                              else
                              {
                                    L_cross_flag=0;
                                    cross_flag=1;
                              }
                        }
                  }
                  else
                  {
                        if( L_LINK_NUM>=2 )     //当连续的边沿大于1时
                        {
                              if( left_edge[EXTRACT_ROW]>=L_LOST_CROSS )     //符合条件时，判断为十字直角拐点
                              {
                                    L_up_cross[0]=EXTRACT_ROW;                        //存储左边沿十字下方直角拐点行坐标
                                    L_up_cross[1]=left_edge[EXTRACT_ROW];             //存储左边沿十字下方直角拐点列坐标
                              }
                        }
                  }
            }
      }
}

/***********************************************************************************************************************************/

void L_up_cross_extract()           //此函数为左边沿上方直角拐点判断函数（找到下十字后，未丢边，向内搜索上十字）
{
      if( l_cross_able==1 && L_down_cross[0]!=-1 && L_up_cross[0]==-1 && EXTRACT_ROW>CROSS_UP_ROW && fabsf(inv_distance[EXTRACT_ROW]-inv_distance[L_down_cross[0]])<cross_k_2 )
      {
            if( L_LINK_NUM>=(L_down_cross[0]-EXTRACT_ROW) && left_edge[EXTRACT_ROW]!=-1 )
            {
                  if( left_edge[EXTRACT_ROW]<=L_down_cross[1] )
                  {
                        double xxx=0,yyy=0;
                        Site_xy1 xy2,xy3;
                        Site_xy xy1;
                        
                        xy1=get_inv_img( L_down_cross[0] , L_down_cross[1] );
                        xxx=cross_k_1*fabsf(inv_distance[EXTRACT_ROW]-xy1.x);
                        
                        yyy=xy1.y-L_cross_k*fabsf(inv_distance[EXTRACT_ROW]-xy1.x)+cross_k_add+xxx;
                        xy2=get_invinv_img( inv_distance[EXTRACT_ROW] , yyy );
                        yyy=xy1.y-L_cross_k*fabsf(inv_distance[EXTRACT_ROW]-xy1.x)-cross_k_add_1+xxx;
                        xy3=get_invinv_img( inv_distance[EXTRACT_ROW] , yyy );
                        if( xy3.y<L_down_cross[1]+2 )
                        {
                              xy3.y=L_down_cross[1]+2;
                        }
                        
                        for( EXTRACT_COL=xy3.y ; EXTRACT_COL<=xy2.y ; EXTRACT_COL++ )  //搜索十字拐点
                        {
                              if( EXTRACT_COL>COL_END-3 )    //当搜索超过图像范围时，结束搜索
                              {
                                    break;
                              }
                              if( img_data[EXTRACT_ROW][EXTRACT_COL]==DOT_B && img_data[EXTRACT_ROW][EXTRACT_COL+1]==DOT_W  && img_data[EXTRACT_ROW][EXTRACT_COL+3]==DOT_W )
                              {
                                    W_num=0;            //清除白块计数值
                                    for( W_num_row=EXTRACT_ROW-1 ; W_num_row>=EXTRACT_ROW-ROW_W_SUM ; W_num_row-- )     //在搜索到的黑白跳变点后，向跳变点上方搜索十个点，判断是否连续
                                    {
                                          if( W_num_row<ROW_START )    //当搜索超过图像范围时，结束搜索
                                          {
                                                break;
                                          }
                                          if( img_data[W_num_row][EXTRACT_COL] == DOT_W )
                                          {
                                                W_num++;
                                          }
                                    }
                                    if( W_num<ROW_W_NUM )             //当白块计数值没超过阀值，则判断为左边沿          /*这里的 W_num 的阀值需要测试*/
                                    {
                                          L_up_cross_error();                          //左边沿上方直角拐点错误纠正
                                          
                                          if( L_get_flag==1 )
                                          {
                                                left_edge[EXTRACT_ROW]=EXTRACT_COL;       //储存本行左边沿列坐标
                                                
                                                L_EDGE_NUM++;                                //左边沿个数加1
                                                
                                                L_LINK_NUM=1;                                //连续左边沿个数赋  1
                                                
                                                L_edge_nearest=EXTRACT_COL;                  //存储最近一个左边沿的列坐标
                                                
                                                if( L_cross_flag==1 && L_up_cross[0]!=-1 )
                                                {
                                                      L_cross_flag=0;                              //左十字标志置  0
                                                      
                                                      cross_flag=1;                                //十字标志置  1
                                                }
                                          }
                                          
                                          break;                                      //本行左边沿搜索结束
                                    }
                              }
                        }
                  }
                  else
                  {
                        if( inv_distance[EXTRACT_ROW]>120 && fabsf(inv_distance[EXTRACT_ROW]-inv_distance[L_down_cross[0]])<100 )
                        {
                              L_down_cross[0]=-1;
                              L_down_cross[1]=-1;
                              L_cross_flag=0;                              //左十字标志置  0
                              l_cross_able=0;
                        }
                  }
            }
      }
}

/***********************************************************************************************************************************/

void L_up_cross_error()             //此函数为左边沿上方直角拐点错误纠正函数
{
      if( l_cross_able==1 && EXTRACT_COL>L_LOST_CROSS && EXTRACT_ROW>CROSS_UP_ROW && L_up_cross[0]==-1 )
      {
            W_num=0;            //清除白块计数值
            
            if( L_cross_k!=10 && L_down_cross[0]!=-1 )
            {
                  Site_xy xy1,xy2;
                  double xx,yy,aa;
                  xy1=get_inv_img( EXTRACT_ROW , EXTRACT_COL );
                  xy2=get_inv_img( L_down_cross[0] , L_down_cross[1] );
                  
                  aa=cross_k_1*fabsf(xy1.x-xy2.x);
                  
                  xx=xy2.y-L_cross_k*fabsf(xy1.x-xy2.x)+cross_k_add+aa;
                  yy=xy2.y-L_cross_k*fabsf(xy1.x-xy2.x)-cross_k_add_1+aa;
                  
                  if( xy1.y<xx && xy1.y>yy && fabsf(xy1.x-xy2.x)<cross_k_2 )
                  {
                        W_num=10;
                  }
                  if( fabsf(xy1.x-xy2.x)>cross_k_2 )
                  {
                        L_get_flag=0;                    //左边沿搜索得到标志置  0
                  }
            }
            else
            {
                  for( W_num_row=EXTRACT_ROW-1 ; W_num_row>=EXTRACT_ROW-5 ; W_num_row-- )     //在搜索到的黑白跳变点后，向上搜索白点，判断是否为干扰点
                  {
                        for( W_num_col=EXTRACT_COL+1 ; W_num_col<=EXTRACT_COL+20 ; W_num_col++ )     //在搜索到的黑白跳变点后，向上搜索白点，判断是否为干扰点
                        {
                              if( W_num_col>COL_END-3 )   //超过图像范围停止
                              {
                                    break;
                              }
                              if( img_data[W_num_row][W_num_col] == DOT_B && img_data[W_num_row][W_num_col+1] == DOT_W && img_data[W_num_row][W_num_col+3] == DOT_W )
                              {
                                    W_num++;
                              }
                        }
                  }
            }
            
            if( W_num>=3 )
            {
                  L_up_cross[0]=EXTRACT_ROW;        //存储十字拐点行坐标
                  L_up_cross[1]=EXTRACT_COL;        //存储十字拐点列坐标
                  
                  L_get_flag=1;                     //左边沿搜索得到标志置  1
            }
      }
}

/***********************************************************************************************************************************/

void R_down_cross_judge()           //此函数为右边沿下方直角拐点判断函数
{
      if( r_cross_able==1 && R_up_cross[0]==-1 && R_down_cross[0]==-1 && right_edge[EXTRACT_ROW+1]<R_LOST_CROSS && EXTRACT_ROW>CROSS_DOWN_ROW && EXTRACT_ROW<=CROSS_START_ROW && L_barrier_flag==0 && R_barrier_flag==0 )     //当要判断的点在范围内
      {
            W_num=0;
            for( W_num_row=EXTRACT_ROW+2 ; W_num_row<=EXTRACT_ROW+9 ; W_num_row++ )  //检测前8行内倾数
            {
                  if( right_edge[W_num_row]!=-1 )
                  {
                        if( right_edge[W_num_row-1]-right_edge[W_num_row]>0 )
                        {
                              W_num++;
                        }
                  }
                  else
                  {
                        W_num=10;
                        break;
                  }
            }
            
            if( W_num<=1 )   //前几行右边沿全部都为内倾
            {
                  for( W_num=0 ; W_num<=2 ; W_num++ )
                  {
                        inv_cross[W_num]=-1;
                  }
                  uint8 num=0;        //记录丢边点个数
                  W_num=0;            //记录内倾点个数
                  if( right_edge[EXTRACT_ROW]!=-1 )
                  {
                        if( EXTRACT_ROW>=15 && right_edge[EXTRACT_ROW]-right_edge[EXTRACT_ROW+1]<CROSS_DOWN_COL )
                        {
                              EXTRACT_COL=right_edge[EXTRACT_ROW];
                              for( W_num_row=EXTRACT_ROW-1 ; W_num_row>=EXTRACT_ROW-6 ; W_num_row-- )   //向上搜索边沿
                              {
                                    if( img_data[W_num_row][EXTRACT_COL-1]==DOT_B )
                                    {
                                          W_num++;
                                          continue;
                                    }
                                    for( W_num_col=EXTRACT_COL ; W_num_col<=COL_END ; W_num_col++ )
                                    {
                                          if( img_data[W_num_row][W_num_col]==DOT_B && img_data[W_num_row][W_num_col-1]==DOT_W )
                                          {
                                                EXTRACT_COL=W_num_col;
                                                break;
                                          }
                                    }
                                    if( W_num_col>COL_END )
                                    {
                                          if( img_data[W_num_row][COL_END-1]==DOT_W && img_data[W_num_row][COL_END]==DOT_W )
                                          {
                                                EXTRACT_COL=COL_END;
                                                num++;
                                          }
                                    }
                                    if( (EXTRACT_ROW-W_num_row)==3 )
                                    {
                                          inv_cross[0]=EXTRACT_COL;
                                    }
                                    if( (EXTRACT_ROW-W_num_row)==4 )
                                    {
                                          inv_cross[1]=EXTRACT_COL;
                                    }
                                    if( (EXTRACT_ROW-W_num_row)==5 )
                                    {
                                          inv_cross[2]=EXTRACT_COL;
                                    }
                              }
                        }
                  }
                  else
                  {
                        if( EXTRACT_ROW>=15 )
                        {
                              EXTRACT_COL=COL_END;
                              for( W_num_row=EXTRACT_ROW-1 ; W_num_row>=EXTRACT_ROW-6 ; W_num_row-- )   //向上搜索边沿
                              {
                                    if( img_data[W_num_row][EXTRACT_COL-1]==DOT_B )
                                    {
                                          W_num++;
                                          continue;
                                    }
                                    for( W_num_col=EXTRACT_COL ; W_num_col<=COL_END ; W_num_col++ )
                                    {
                                          if( img_data[W_num_row][W_num_col]==DOT_B && img_data[W_num_row][W_num_col-1]==DOT_W )
                                          {
                                                EXTRACT_COL=W_num_col;
                                                break;
                                          }
                                    }
                                    if( W_num_col>COL_END )
                                    {
                                          if( img_data[W_num_row][COL_END-1]==DOT_W && img_data[W_num_row][COL_END]==DOT_W )
                                          {
                                                EXTRACT_COL=COL_END;
                                                num++;
                                          }
                                    }
                                    if( (EXTRACT_ROW-W_num_row)==3 )
                                    {
                                          inv_cross[0]=EXTRACT_COL;
                                    }
                                    if( (EXTRACT_ROW-W_num_row)==4 )
                                    {
                                          inv_cross[1]=EXTRACT_COL;
                                    }
                                    if( (EXTRACT_ROW-W_num_row)==5 )
                                    {
                                          inv_cross[2]=EXTRACT_COL;
                                    }
                              }
                        }
                  }
                  if( W_num<=1 )   //上搜白点数满足阀值
                  {
                        if( right_edge[EXTRACT_ROW]==-1 || right_edge[EXTRACT_ROW]-right_edge[EXTRACT_ROW+1]>=CROSS_DOWN_COL )       //当边沿剧烈变化时，判断为下十字直角
                        {
                              R_down_cross[0]=EXTRACT_ROW+1;                           //存储右边沿十字下方直角拐点行坐标
                              R_down_cross[1]=right_edge[EXTRACT_ROW+1];                //存储右边沿十字下方直角拐点列坐标
                              
                              R_cross_flag=1;      //当符合条件时，判断为边沿突变，右边沿十字标志置  1
                        }
                        else
                        {
                              if( num>=4 )    //上搜边沿数过少，直接判断为十字
                              {
                                    R_down_cross[0]=EXTRACT_ROW+1;                           //存储右边沿十字下方直角拐点行坐标
                                    R_down_cross[1]=right_edge[EXTRACT_ROW+1];                //存储右边沿十字下方直角拐点列坐标
                                    
                                    R_cross_flag=1;      //当符合条件时，判断为边沿突变，右边沿十字标志置  1
                              }
                              else              //上搜边沿数足够，逆透视校正角度判断十字
                              {
                                    int16 k1=0,k2=0,k3=0;
                                    double x=0.0,y=0.0,x1,y1,k=0.0;
                                    Site_xy xy1,xy2;
                                    int16 row_add;
                                    row_add=EXTRACT_ROW/12;
                                    
                                    xy1=get_inv_img( EXTRACT_ROW+1 , right_edge[EXTRACT_ROW+1] );    //获取拐点逆透视坐标
                                    
                                    W_num_row=4;
                                    
                                    for( W_num=1 ; W_num<=W_num_row ; W_num++ )   //获取拐点向下的拟合斜率
                                    {
                                          W_num_col=EXTRACT_ROW+W_num+row_add;
                                          if( W_num_col>ROW_END )
                                          {
                                                break;
                                          }
                                          if( right_edge[W_num_col]!=-1 )
                                          {
                                                xy2=get_inv_img( W_num_col , right_edge[W_num_col] );
                                                x1=xy2.x-xy1.x;
                                                y1=xy2.y-xy1.y;
                                                x=x+x1*y1;
                                                y=y+x1*x1;
                                          }
                                    }
                                    if( x!=0 )
                                    {
                                          k=(-1)*x/y;    //得到拐点向下的拟合斜率
                                    }
                                    else
                                    {
                                          k==0;
                                    }
                                    if( k>=0 )
                                    {
                                          for( W_num=0 ; W_num<=29 ; W_num++ )
                                          {
                                                if( k<=tan_table[W_num] )
                                                {
                                                      k1=180-3*W_num;
                                                      break;
                                                }
                                          }
                                          if( W_num==30 )
                                          {
                                                k1=90;
                                          }
                                    }
                                    else
                                    {
                                          k=fabsf(k);
                                          for( W_num=0 ; W_num<=29 ; W_num++ )
                                          {
                                                if( k<=tan_table[W_num] )
                                                {
                                                      k1=180+3*W_num;
                                                      break;
                                                }
                                          }
                                          if( W_num==30 )
                                          {
                                                k1=270;
                                          }
                                    }
                                    
                                    x=0.0;
                                    y=0.0;
                                    for( W_num=0 ; W_num<=2 ; W_num++ )   //获取拐点向上的拟合斜率
                                    {
                                          if( inv_cross[W_num]!=-1 )
                                          {
                                                xy2=get_inv_img( EXTRACT_ROW-W_num-2 , inv_cross[W_num] );
                                                x1=xy2.x-xy1.x;
                                                y1=xy2.y-xy1.y;
                                                x=x+x1*y1;
                                                y=y+x1*x1;
                                          }
                                    }
                                    if( x!=0 )
                                    {
                                          k=(-1)*x/y;    //得到拐点向上的拟合斜率
                                    }
                                    else
                                    {
                                          k==0;
                                    }
                                    if( k>=0 )
                                    {
                                          for( W_num=0 ; W_num<=29 ; W_num++ )
                                          {
                                                if( k<=tan_table[W_num] )
                                                {
                                                      k2=-3*W_num;
                                                      break;
                                                }
                                          }
                                          if( W_num==30 )
                                          {
                                                k2=-90;
                                          }
                                    }
                                    else
                                    {
                                          k=fabsf(k);
                                          for( W_num=0 ; W_num<=29 ; W_num++ )
                                          {
                                                if( k<=tan_table[W_num] )
                                                {
                                                      k2=3*W_num;
                                                      break;
                                                }
                                          }
                                          if( W_num==30 )
                                          {
                                                k2=90;
                                          }
                                    }
                                    
                                    k3=abs(k1-k2);
                                    angle_aaa[1]=k3;
                                    if( k3<=117 )  //通过角度校正判断为十字
                                    {
                                          R_down_cross[0]=EXTRACT_ROW+1;                           //存储右边沿十字下方直角拐点行坐标
                                          R_down_cross[1]=right_edge[EXTRACT_ROW+1];                //存储右边沿十字下方直角拐点列坐标
                                          
                                          R_cross_flag=1;      //当符合条件时，判断为边沿突变，右边沿十字标志置  1
                                    }
                              }
                        }
                        
                        if( R_down_cross[0]!=-1 )
                        {
                              double x=0.0,y=0.0,x1,y1,k=0.0,a;
                              Site_xy xy1,xy2;
                              Site_xy1 xy3;
                              
                              xy1=get_inv_img( R_down_cross[0] , R_down_cross[1] );    //获取拐点逆透视坐标
                              
                              a=xy1.x-50;
                              xy3=get_invinv_img( a , 0 );
                              if( xy3.x-R_down_cross[0]>15 )
                              {
                                    W_num_row=15;
                              }
                              else
                              {
                                    if( xy3.x-R_down_cross[0]<3 )
                                    {
                                          W_num_row=3;
                                    }
                                    else
                                    {
                                          W_num_row=abs(xy3.x-R_down_cross[0]);
                                    }
                              }
                              
                              for( W_num=1 ; W_num<=W_num_row ; W_num++ )   //获取拐点向下的拟合斜率
                              {
                                    W_num_col=EXTRACT_ROW+W_num+1;
                                    if( W_num_col>ROW_END )
                                    {
                                          break;
                                    }
                                    if( right_edge[W_num_col]!=-1 )
                                    {
                                          xy2=get_inv_img( W_num_col , right_edge[W_num_col] );
                                          x1=xy2.x-xy1.x;
                                          y1=xy2.y-xy1.y;
                                          x=x+x1*y1;
                                          y=y+x1*x1;
                                    }
                                    else
                                    {
                                          break;
                                    }
                              }
                              if( x!=0 )
                              {
                                    k=(-1)*x/y;    //得到拐点向下的拟合斜率
                              }
                              else
                              {
                                    k==0;
                              }
                              
                              R_cross_k=k;        //取向下拟合斜率
                        }
                  }
            }
      }
}

/***********************************************************************************************************************************/

void R_up_cross_judge()             //此函数为右边沿上方直角拐点判断函数
{
      if( r_cross_able==1 && R_cross_flag==1 && EXTRACT_ROW<=CROSS_START_ROW && EXTRACT_ROW>CROSS_UP_ROW )         //当右边沿十字标为1时
      {
            if( R_down_cross[0]!=-1 )   //获取下十字拐点
            {
                  if( fabsf(inv_distance[EXTRACT_ROW]-inv_distance[R_down_cross[0]])<cross_k_2 )
                  {
                        if( R_LINK_NUM>=2 && R_LINK_NUM<(R_down_cross[0]-EXTRACT_ROW-1) )     //当连续的边沿大于3时
                        {
                              if( R_up_cross[0]!=-1 )
                              {
                                    Site_xy xy1,xy2;
                                    xy1=get_inv_img(EXTRACT_ROW,right_edge[EXTRACT_ROW]);
                                    xy2=get_inv_img(R_up_cross[0],R_up_cross[1]);
                                    if( 2.0*xy1.x+xy1.y<=2.0*xy2.x+xy2.y )     //符合条件时，判断为十字直角拐点
                                    {
                                          double xx,aa;
                                          xy2=get_inv_img( R_down_cross[0] , R_down_cross[1] );
                                          
                                          if( fabsf(xy1.x-xy2.x)>cross_k_2 )
                                          {
                                                aa=cross_k_1*cross_k_2;
                                          }
                                          else
                                          {
                                                aa=cross_k_1*fabsf(xy1.x-xy2.x);
                                          }
                                          xx=xy2.y-R_cross_k*fabsf(xy1.x-xy2.x)-cross_k_add-aa;
                                          
                                          if( xy1.y>=xx )
                                          {
                                                R_up_cross[0]=EXTRACT_ROW;                        //存储右边沿十字下方直角拐点行坐标
                                                R_up_cross[1]=right_edge[EXTRACT_ROW];            //存储右边沿十字下方直角拐点列坐标
                                          }
                                    }
                              }
                              else
                              {
                                    double xx,aa;
                                    Site_xy xy1,xy2;
                                    xy1=get_inv_img(EXTRACT_ROW,right_edge[EXTRACT_ROW]);
                                    xy2=get_inv_img( R_down_cross[0] , R_down_cross[1] );
                                    
                                    if( fabsf(xy1.x-xy2.x)>cross_k_2 )
                                    {
                                          aa=cross_k_1*cross_k_2;
                                    }
                                    else
                                    {
                                          aa=cross_k_1*fabsf(xy1.x-xy2.x);
                                    }
                                    xx=xy2.y-R_cross_k*fabsf(xy1.x-xy2.x)-cross_k_add-aa;
                                    
                                    if( xy1.y>=xx )
                                    {
                                          R_up_cross[0]=EXTRACT_ROW;                        //存储右边沿十字下方直角拐点行坐标
                                          R_up_cross[1]=right_edge[EXTRACT_ROW];            //存储右边沿十字下方直角拐点列坐标
                                    }
                              }
                        }
                  }
                  else
                  {
                        R_cross_flag=0;
                        cross_flag=1;
                  }
            }
            else   //未获取下十字拐点
            {
                  if( R_up_cross[0]!=-1 )
                  {
                        if( R_LINK_NUM>=2 )     //当连续的边沿大于1时
                        {
                              Site_xy xy1,xy2;
                              xy1=get_inv_img(EXTRACT_ROW,right_edge[EXTRACT_ROW]);
                              xy2=get_inv_img(R_up_cross[0],R_up_cross[1]);
                              if( 2.0*xy1.x+xy1.y<=2.0*xy2.x+xy2.y )     //符合条件时，判断为十字直角拐点
                              {
                                    R_up_cross[0]=EXTRACT_ROW;                        //存储右边沿十字下方直角拐点行坐标
                                    R_up_cross[1]=right_edge[EXTRACT_ROW];            //存储右边沿十字下方直角拐点列坐标
                              }
                              else
                              {
                                    R_cross_flag=0;
                                    cross_flag=1;
                              }
                        }
                  }
                  else
                  {
                        if( R_LINK_NUM>=2 )     //当连续的边沿大于1时
                        {
                              if( right_edge[EXTRACT_ROW]<=R_LOST_CROSS )     //符合条件时，判断为十字直角拐点
                              {
                                    R_up_cross[0]=EXTRACT_ROW;                        //存储右边沿十字下方直角拐点行坐标
                                    R_up_cross[1]=right_edge[EXTRACT_ROW];            //存储右边沿十字下方直角拐点列坐标
                              }
                        }
                  }
            }
      }
}

/***********************************************************************************************************************************/

void R_up_cross_extract()           //此函数为右边沿上方直角拐点判断函数（找到下十字后，未丢边，向内搜索上十字）
{
      if( r_cross_able==1 && R_down_cross[0]!=-1 && R_up_cross[0]==-1 && EXTRACT_ROW>CROSS_UP_ROW && fabsf(inv_distance[EXTRACT_ROW]-inv_distance[R_down_cross[0]])<cross_k_2 )
      {
            if( R_LINK_NUM>=(R_down_cross[0]-EXTRACT_ROW) && right_edge[EXTRACT_ROW]!=-1 )
            {
                  if( right_edge[EXTRACT_ROW]>=R_down_cross[1] )
                  {
                        double xxx=0,yyy=0;
                        Site_xy1 xy2,xy3;
                        Site_xy xy1;
                        
                        xy1=get_inv_img( R_down_cross[0] , R_down_cross[1] );
                        xxx=cross_k_1*fabsf(inv_distance[EXTRACT_ROW]-xy1.x);
                        
                        yyy=xy1.y-R_cross_k*fabsf(inv_distance[EXTRACT_ROW]-xy1.x)-cross_k_add-xxx;
                        xy2=get_invinv_img( inv_distance[EXTRACT_ROW] , yyy );
                        yyy=xy1.y-R_cross_k*fabsf(inv_distance[EXTRACT_ROW]-xy1.x)+cross_k_add_1-xxx;
                        xy3=get_invinv_img( inv_distance[EXTRACT_ROW] , yyy );
                        if( xy3.y>R_down_cross[1]-2 )
                        {
                              xy3.y=R_down_cross[1]-2;
                        }
                        
                        for( EXTRACT_COL=xy3.y ; EXTRACT_COL>=xy2.y ; EXTRACT_COL-- )  //搜索十字拐点
                        {
                              if( EXTRACT_COL<COL_START+3 )    //当搜索超过图像范围时，结束搜索
                              {
                                    break;
                              }
                              if( img_data[EXTRACT_ROW][EXTRACT_COL]==DOT_B && img_data[EXTRACT_ROW][EXTRACT_COL-1]==DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL-3]==DOT_W )
                              {
                                    W_num=0;            //清除白块计数值
                                    for( W_num_row=EXTRACT_ROW-1 ; W_num_row>=EXTRACT_ROW-ROW_W_SUM ; W_num_row-- )     //在搜索到的黑白跳变点后，向跳变点上方搜索十个点，判断是否连续
                                    {
                                          if( W_num_row<ROW_START )    //当搜索超过图像范围时，结束搜索
                                          {
                                                break;
                                          }
                                          if( img_data[W_num_row][EXTRACT_COL] == DOT_W )
                                          {
                                                W_num++;
                                          }
                                    }
                                    if( W_num<ROW_W_NUM )             //当白块计数值没超过阀值，则判断为左边沿          /*这里的 W_num 的阀值需要测试*/
                                    {
                                          R_up_cross_error();                          //右边沿上方直角拐点错误纠正
                                          
                                          if( R_get_flag==1 )
                                          {
                                                right_edge[EXTRACT_ROW]=EXTRACT_COL;         //储存本行右边沿列坐标
                                                
                                                R_EDGE_NUM++;                                //右边沿个数加1
                                                
                                                R_LINK_NUM=1;                                //连续右边沿个数赋  1
                                                
                                                R_edge_nearest=EXTRACT_COL;                  //存储最近一个右边沿的列坐标
                                                
                                                if( R_cross_flag==1 && R_up_cross[0]!=-1 )
                                                {
                                                      R_cross_flag=0;                              //右十字标志置  0
                                                      
                                                      cross_flag=1;                                //十字标志置  1
                                                }
                                          }
                                          
                                          break;                                      //本行右边沿搜索结束
                                    }
                              }
                        }
                  }
                  else
                  {
                        if( inv_distance[EXTRACT_ROW]>120 && fabsf(inv_distance[EXTRACT_ROW]-inv_distance[R_down_cross[0]])<100 )
                        {
                              R_down_cross[0]=-1;
                              R_down_cross[1]=-1;
                              R_cross_flag=0;                              //右十字标志置  0
                              r_cross_able=0;
                        }
                  }
            }
      }
}

/***********************************************************************************************************************************/

void R_up_cross_error()             //此函数为右边沿上方直角拐点错误纠正函数
{
      if( r_cross_able==1 && EXTRACT_COL<R_LOST_CROSS && EXTRACT_ROW>CROSS_UP_ROW && R_up_cross[0]==-1 )
      {
            W_num=0;            //清除白块计数值
            
            if( R_cross_k!=10 && R_down_cross[0]!=-1 )
            {
                  Site_xy xy1,xy2;
                  double xx,yy,aa;
                  xy1=get_inv_img( EXTRACT_ROW , EXTRACT_COL );
                  xy2=get_inv_img( R_down_cross[0] , R_down_cross[1] );
                  
                  aa=cross_k_1*fabsf(xy1.x-xy2.x);
                  
                  xx=xy2.y-R_cross_k*fabsf(xy1.x-xy2.x)-cross_k_add-aa;
                  yy=xy2.y-R_cross_k*fabsf(xy1.x-xy2.x)+cross_k_add_1-aa;
                  
                  if( xy1.y>xx && xy1.y<yy && fabsf(xy1.x-xy2.x)<cross_k_2 )
                  {
                        W_num=10;
                  }
                  if( fabsf(xy1.x-xy2.x)>cross_k_2 )
                  {
                        R_get_flag=0;                    //右边沿搜索得到标志置  0
                  }
            }
            else
            {
                  for( W_num_row=EXTRACT_ROW-1 ; W_num_row>=EXTRACT_ROW-5 ; W_num_row-- )     //在搜索到的黑白跳变点后，向上搜索白点，判断是否为干扰点
                  {
                        for( W_num_col=EXTRACT_COL-1 ; W_num_col>=EXTRACT_COL-20 ; W_num_col-- )     //在搜索到的黑白跳变点后，向上搜索白点，判断是否为干扰点
                        {
                              if( W_num_col<COL_START+3 )   //超过图像范围停止
                              {
                                    break;
                              }
                              if( img_data[W_num_row][W_num_col] == DOT_B && img_data[W_num_row][W_num_col-1] == DOT_W && img_data[W_num_row][W_num_col-3] == DOT_W )
                              {
                                    W_num++;
                              }
                        }
                  }
            }
            
            if( W_num>=3 )
            {
                  R_up_cross[0]=EXTRACT_ROW;        //存储十字拐点行坐标
                  R_up_cross[1]=EXTRACT_COL;        //存储十字拐点列坐标
                  
                  R_get_flag=1;                     //右边沿搜索得到标志置  1
            }
      }
}

/***********************************************************************************************************************************/

void barrier_down_extract()         //此函数为障碍物下边沿行检测函数
{
      if( L_get_flag==1 && R_get_flag==1 && L_barrier_down_row==-1 && R_barrier_down_row==-1 && LOOP_TEMP==0 && EXTRACT_ROW>=BAR_UP_ROW && EXTRACT_ROW<=50 && end_bar_able==1 )
      {
            if( L_LINK_NUM>=3 && R_trend_in==1 )  //左边沿突变
            {
                  if( EXTRACT_ROW<=20 )
                  {
                        if( (left_edge[EXTRACT_ROW]-left_edge[EXTRACT_ROW+1])>0.4*bar_range_col[EXTRACT_ROW+1] )
                        {
                              L_barrier_down_row=EXTRACT_ROW;           //检测出左边沿障碍物下边沿
                              
                              if( barrier_able==1 )
                              {
                                    L_barrier_flag=1;                         //左边沿障碍标志置  1
                              }
                        }
                  }
                  else
                  {
                        if( (left_edge[EXTRACT_ROW]-left_edge[EXTRACT_ROW+2])>0.4*bar_range_col[EXTRACT_ROW+2] )
                        {
                              L_barrier_down_row=EXTRACT_ROW;           //检测出左边沿障碍物下边沿
                              
                              if( barrier_able==1 )
                              {
                                    L_barrier_flag=1;                         //左边沿障碍标志置  1
                              }
                        }
                  }
            }
            
            if( R_LINK_NUM>=3 && L_trend_in==1 )  //右边沿突变
            {
                  if( EXTRACT_ROW<=20 )
                  {
                        if( (right_edge[EXTRACT_ROW+1]-right_edge[EXTRACT_ROW])>0.4*bar_range_col[EXTRACT_ROW+1] )
                        {
                              R_barrier_down_row=EXTRACT_ROW;              //检测出右边沿障碍物下边沿
                              
                              if( barrier_able==1 )
                              {
                                    R_barrier_flag=1;                            //右边沿障碍标志置  1
                              }
                        }
                  }
                  else
                  {
                        if( (right_edge[EXTRACT_ROW+2]-right_edge[EXTRACT_ROW])>0.4*bar_range_col[EXTRACT_ROW+2] )
                        {
                              R_barrier_down_row=EXTRACT_ROW;              //检测出右边沿障碍物下边沿
                              
                              if( barrier_able==1 )
                              {
                                    R_barrier_flag=1;                            //右边沿障碍标志置  1
                              }
                        }
                  }
            }
            
            if( L_barrier_down_row==-1 && R_barrier_down_row==-1 )
            {
                  if( abs(right_edge[EXTRACT_ROW]-left_edge[EXTRACT_ROW])<=(1.5*bar_range_col[EXTRACT_ROW]) && EXTRACT_ROW>30 )    //当赛道宽度过小，判断为障碍
                  {
                        if( up_flag==0 )
                        {
                              if( left_edge[EXTRACT_ROW]-left_edge[EXTRACT_ROW+1]>0.4*bar_range_col[EXTRACT_ROW] && L_LINK_NUM>=5 )  //当左边沿位置突变，判断左边沿障碍物
                              {
                                    if( EXTRACT_ROW>=ROW_END-10 )
                                    {
                                          L_barrier_down_row=EXTRACT_ROW;     //检测出左边沿障碍物下边沿
                                          
                                          if( barrier_able==1 )
                                          {
                                                L_barrier_flag=1;                   //左边沿障碍标志置  1
                                          }
                                    }
                                    else
                                    {
                                          if( R_LINK_NUM>=10 )
                                          {
                                                L_barrier_down_row=EXTRACT_ROW;     //检测出左边沿障碍物下边沿
                                                
                                                if( barrier_able==1 )
                                                {
                                                      L_barrier_flag=1;                   //左边沿障碍标志置  1
                                                }
                                          }
                                    }
                              }
                              else
                              {
                                    if( right_edge[EXTRACT_ROW+1]-right_edge[EXTRACT_ROW]>0.4*bar_range_col[EXTRACT_ROW] && R_LINK_NUM>=5 )  //当右边沿位置突变，判断右边沿障碍物
                                    {
                                          if( EXTRACT_ROW>=ROW_END-10 )
                                          {
                                                R_barrier_down_row=EXTRACT_ROW;     //检测出右边沿障碍物下边沿
                                                
                                                if( barrier_able==1 )
                                                {
                                                      R_barrier_flag=1;                   //右边沿障碍标志置  1
                                                }
                                          }
                                          else
                                          {
                                                if( L_LINK_NUM>=10 )
                                                {
                                                      R_barrier_down_row=EXTRACT_ROW;     //检测出右边沿障碍物下边沿
                                                      
                                                      if( barrier_able==1 )
                                                      {
                                                            R_barrier_flag=1;                   //右边沿障碍标志置  1
                                                      }
                                                }
                                          }
                                    }
                                    else
                                    {
                                          if( EXTRACT_ROW>bar_search_end )     //当未判断出突变，赋值标志位
                                          {
                                                L_barrier_down_row=EXTRACT_ROW;
                                                R_barrier_down_row=EXTRACT_ROW;
                                                if( barrier_able==1 )
                                                {
                                                      L_barrier_flag=1;
                                                      R_barrier_flag=1;
                                                }
                                          }
                                    }
                              }
                        }
                  }
                  else    //当赛道宽度正常，搜索障碍
                  {
                        if( abs(right_edge[EXTRACT_ROW]-left_edge[EXTRACT_ROW])<=(2.5*bar_range_col[EXTRACT_ROW]) )
                        {
                              int16 l_edge=-1,r_edge=-1;
                              for( EXTRACT_COL=L_edge_nearest+1 ; EXTRACT_COL<=R_edge_nearest-3 ; EXTRACT_COL++ )
                              {
                                    if( img_data[EXTRACT_ROW][EXTRACT_COL]==DOT_B && img_data[EXTRACT_ROW][EXTRACT_COL+1]==DOT_B && img_data[EXTRACT_ROW][EXTRACT_COL+2]==DOT_B )
                                    {
                                          l_edge=EXTRACT_COL;
                                          break;
                                    }
                              }
                              if( l_edge!=-1 )
                              {
                                    for( EXTRACT_COL=R_edge_nearest-1 ; EXTRACT_COL>=l_edge+3 ; EXTRACT_COL-- )
                                    {
                                          if( img_data[EXTRACT_ROW][EXTRACT_COL]==DOT_B && img_data[EXTRACT_ROW][EXTRACT_COL-1]==DOT_B && img_data[EXTRACT_ROW][EXTRACT_COL-2]==DOT_B )
                                          {
                                                r_edge=EXTRACT_COL;
                                                break;
                                          }
                                    }
                                    if( r_edge!=-1 )   //当左右都搜索到跳变
                                    {
                                          if( abs(r_edge-l_edge)>0.35*bar_range_col[EXTRACT_ROW] )
                                          {
                                                W_num=0;
                                                W_num_col=(r_edge+l_edge)/2;
                                                for( W_num_row=EXTRACT_ROW-1 ; W_num_row>=EXTRACT_ROW-5 ; W_num_row-- )
                                                {
                                                      if( W_num_row<ROW_START )
                                                      {
                                                            break;
                                                      }
                                                      if( img_data[W_num_row][W_num_col]==DOT_W )
                                                      {
                                                            W_num++;
                                                      }
                                                }
                                                W_num_row=0;
                                                for( W_num_col=l_edge ; W_num_col<=r_edge ; W_num_col++ )
                                                {
                                                      if( img_data[EXTRACT_ROW][W_num_col]==DOT_B )
                                                      {
                                                            W_num_row++;
                                                      }
                                                }
                                                if( W_num<=1 && W_num_row>=(r_edge-l_edge) )    //判断有障碍物
                                                {
                                                      if( (l_edge+r_edge)<(L_edge_nearest+R_edge_nearest) )  //判断为左障碍物
                                                      {
                                                            if( EXTRACT_ROW>=30 )
                                                            {
                                                                  if( abs(right_edge[EXTRACT_ROW]-r_edge)>bar_range_col[EXTRACT_ROW] )
                                                                  {
                                                                        L_barrier_down_row=EXTRACT_ROW;           //检测出左边沿障碍物下边沿
                                                                        
                                                                        if( barrier_able==1 )
                                                                        {
                                                                              left_edge[EXTRACT_ROW]=r_edge;       //储存本行左边沿列坐标
                                                                              
                                                                              L_edge_nearest=r_edge;               //存储最近一个左边沿的列坐标
                                                                              
                                                                              L_barrier_flag=1;                         //左边沿障碍标志置  1
                                                                        }
                                                                  }
                                                                  else
                                                                  {
                                                                        if( car_extract_row==-1 )
                                                                        {
                                                                              car_extract_row=EXTRACT_ROW;
                                                                        }
                                                                  }
                                                            }
                                                            else
                                                            {
                                                                  if( L_LINK_NUM>=5 && R_LINK_NUM>=5 )
                                                                  {
                                                                        if( abs(right_edge[EXTRACT_ROW]-r_edge)>bar_range_col[EXTRACT_ROW] )
                                                                        {
                                                                              L_barrier_down_row=EXTRACT_ROW;           //检测出左边沿障碍物下边沿
                                                                              
                                                                              if( barrier_able==1 )
                                                                              {
                                                                                    left_edge[EXTRACT_ROW]=r_edge;       //储存本行左边沿列坐标
                                                                                    
                                                                                    L_edge_nearest=r_edge;               //存储最近一个左边沿的列坐标
                                                                                    
                                                                                    L_barrier_flag=1;                         //左边沿障碍标志置  1
                                                                              }
                                                                        }
                                                                        else
                                                                        {
                                                                              if( car_extract_row==-1 )
                                                                              {
                                                                                    car_extract_row=EXTRACT_ROW;
                                                                              }
                                                                        }
                                                                  }
                                                            }
                                                      }
                                                      else  //判断为右障碍物
                                                      {
                                                            if( EXTRACT_ROW>=30 )
                                                            {
                                                                  if( abs(l_edge-left_edge[EXTRACT_ROW])>bar_range_col[EXTRACT_ROW] )
                                                                  {
                                                                        R_barrier_down_row=EXTRACT_ROW;              //检测出右边沿障碍物下边沿
                                                                        
                                                                        if( barrier_able==1 )
                                                                        {
                                                                              right_edge[EXTRACT_ROW]=l_edge;         //储存本行右边沿列坐标
                                                                              
                                                                              R_edge_nearest=l_edge;                  //存储最近一个右边沿的列坐标
                                                                              
                                                                              R_barrier_flag=1;                            //右边沿障碍标志置  1
                                                                        }
                                                                  }
                                                                  else
                                                                  {
                                                                        if( car_extract_row==-1 )
                                                                        {
                                                                              car_extract_row=EXTRACT_ROW;
                                                                        }
                                                                  }
                                                            }
                                                            else
                                                            {
                                                                  if( L_LINK_NUM>=5 && R_LINK_NUM>=5 )
                                                                  {
                                                                        if( abs(l_edge-left_edge[EXTRACT_ROW])>bar_range_col[EXTRACT_ROW] )
                                                                        {
                                                                              R_barrier_down_row=EXTRACT_ROW;              //检测出右边沿障碍物下边沿
                                                                              
                                                                              if( barrier_able==1 )
                                                                              {
                                                                                    right_edge[EXTRACT_ROW]=l_edge;         //储存本行右边沿列坐标
                                                                                    
                                                                                    R_edge_nearest=l_edge;                  //存储最近一个右边沿的列坐标
                                                                                    
                                                                                    R_barrier_flag=1;                            //右边沿障碍标志置  1
                                                                              }
                                                                        }
                                                                        else
                                                                        {
                                                                              if( car_extract_row==-1 )
                                                                              {
                                                                                    car_extract_row=EXTRACT_ROW;
                                                                              }
                                                                        }
                                                                  }
                                                            }
                                                      }
                                                }
                                          }
                                    }
                              }
                        }
                  }
            }
      }
}

/***********************************************************************************************************************************/

void barrier_up_extract()           //此函数为障碍物上边沿行检测函数
{
      if( L_barrier_flag==1 && L_barrier_up_row==-1 )
      {
            if( left_edge[EXTRACT_ROW]!=-1 && left_edge[EXTRACT_ROW+1]!=-1 )
            {
                  W_num_col=left_edge[EXTRACT_ROW];
                  for( W_num_row=EXTRACT_ROW-1 ; W_num_row>=EXTRACT_ROW-3 ; W_num_row-- )
                  {
                        if( W_num_row<ROW_START )
                        {
                              break;
                        }
                        if( img_data[W_num_row][W_num_col]==DOT_B )
                        {
                              continue;
                        }
                        for( EXTRACT_COL=W_num_col ; EXTRACT_COL>=COL_START+3 ; EXTRACT_COL-- )
                        {
                              if( EXTRACT_COL<COL_START+3 )    //当搜索超过图像范围时，结束搜索
                              {
                                    break;
                              }
                              if( img_data[W_num_row][EXTRACT_COL]==DOT_W && img_data[W_num_row][EXTRACT_COL-1]==DOT_B && img_data[W_num_row][EXTRACT_COL-3]==DOT_B )
                              {
                                    W_num_col=EXTRACT_COL;
                                    break;
                              }
                        }
                  }
                  if( left_edge[EXTRACT_ROW+1]-W_num_col>=bar_range_col[EXTRACT_ROW]/3 )
                  {
                        L_barrier_up_row=EXTRACT_ROW+1;              //检测出左边沿障碍物上边沿
                        R_barrier_flag=0;
                  }
            }
            else
            {
                  if( left_edge[EXTRACT_ROW]==-1 && left_edge[EXTRACT_ROW+1]!=-1 && L_search_end==0 )
                  {
                        if( left_edge[EXTRACT_ROW+1]>=COL_START+bar_range_col[EXTRACT_ROW+1]/3 )
                        {
                              L_barrier_up_row=EXTRACT_ROW+1;              //检测出左边沿障碍物上边沿
                              R_barrier_flag=0;
                        }
                  }
            }
      }
      if( R_barrier_flag==1 && R_barrier_up_row==-1 )
      {
            if( right_edge[EXTRACT_ROW]!=-1 && right_edge[EXTRACT_ROW+1]!=-1 )
            {
                  W_num_col=right_edge[EXTRACT_ROW];
                  for( W_num_row=EXTRACT_ROW-1 ; W_num_row>=EXTRACT_ROW-3 ; W_num_row-- )
                  {
                        if( W_num_row<ROW_START )
                        {
                              break;
                        }
                        if( img_data[W_num_row][W_num_col]==DOT_B )
                        {
                              continue;
                        }
                        for( EXTRACT_COL=W_num_col ; EXTRACT_COL<=COL_END-3 ; EXTRACT_COL++ )
                        {
                              if( img_data[W_num_row][EXTRACT_COL]==DOT_W && img_data[W_num_row][EXTRACT_COL+1]==DOT_B && img_data[W_num_row][EXTRACT_COL+3]==DOT_B )
                              {
                                    W_num_col=EXTRACT_COL;
                                    break;
                              }
                        }
                  }
                  if( W_num_col-right_edge[EXTRACT_ROW+1]>=bar_range_col[EXTRACT_ROW]/3 )
                  {
                        R_barrier_up_row=EXTRACT_ROW+1;              //检测出右边沿障碍物上边沿
                        L_barrier_flag=0;
                  }
            }
            else
            {
                  if( right_edge[EXTRACT_ROW]==-1 && right_edge[EXTRACT_ROW+1]!=-1 && R_search_end==0 )
                  {
                        if( right_edge[EXTRACT_ROW+1]<=COL_END-bar_range_col[EXTRACT_ROW+1]/3 )
                        {
                              R_barrier_up_row=EXTRACT_ROW+1;              //检测出右边沿障碍物上边沿
                              L_barrier_flag=0;
                        }
                  }
            }
      }
}

/***********************************************************************************************************************************/

void end_line_extract()             //此函数为终点线行检测函数
{
      if( LOOP_TEMP==0 && L_trend_in==1 && R_trend_in==1 && L_LINK_NUM>=5 && R_LINK_NUM>=5 && EXTRACT_ROW>=end_line_end && END_LINE_FLAG==0 && end_line_able==1 )
      {
            if( left_edge[EXTRACT_ROW]!=-1 && right_edge[EXTRACT_ROW]!=-1 && abs(right_edge[EXTRACT_ROW]-left_edge[EXTRACT_ROW])<2.5*bar_range_col[EXTRACT_ROW] )
            {
                  if( EXTRACT_ROW<30 )
                  {
                        int16 l_edge,r_edge;
                        W_num=0;
                        for( W_num_col=left_edge[EXTRACT_ROW]+2 ; W_num_col<=right_edge[EXTRACT_ROW]-3 ; W_num_col++ )
                        {
                              if( img_data[EXTRACT_ROW][W_num_col]<=img_threshold+end_line_gray_add && img_data[EXTRACT_ROW][W_num_col+1]>img_threshold+end_line_gray_add )
                              {
                                    W_num++;
                                    if( W_num==1 )
                                    {
                                          l_edge=W_num_col;
                                    }
                                    else
                                    {
                                          r_edge=W_num_col;
                                    }
                              }
                        }
                        if( W_num>=5 && abs(r_edge-l_edge)>1.5*bar_range_col[EXTRACT_ROW] )
                        {
                              END_LINE_FLAG=1;
                              
                              END_LINE_TEMP=1;
                              
                              END_LINE_TEMP_1=0;
                              
                              END_LINE_TEMP_2=0;
                              
                              end_line_row=EXTRACT_ROW;
                              
                              end_line_val=abs(speedout_val)+25*inv_distance[end_line_row]+1000;
                        }
                  }
                  else
                  {
                        int16 l_edge,r_edge;
                        W_num=0;
                        for( W_num_col=left_edge[EXTRACT_ROW]+2 ; W_num_col<=right_edge[EXTRACT_ROW]-5 ; W_num_col++ )
                        {
                              if( img_data[EXTRACT_ROW][W_num_col]==DOT_B && img_data[EXTRACT_ROW][W_num_col+1]==DOT_B && img_data[EXTRACT_ROW][W_num_col+2]==DOT_W && img_data[EXTRACT_ROW][W_num_col+3]==DOT_W )
                              {
                                    W_num++;
                                    if( W_num==1 )
                                    {
                                          l_edge=W_num_col;
                                    }
                                    else
                                    {
                                          r_edge=W_num_col;
                                    }
                              }
                        }
                        if( W_num>=5 && abs(r_edge-l_edge)>1.5*bar_range_col[EXTRACT_ROW] )
                        {
                              END_LINE_FLAG=1;
                              
                              END_LINE_TEMP=1;
                              
                              END_LINE_TEMP_1=0;
                              
                              END_LINE_TEMP_2=0;
                              
                              end_line_row=EXTRACT_ROW;
                              
                              end_line_val=abs(speedout_val)+25*inv_distance[end_line_row]+1000;
                        }
                  }
            }
      }
}

/***********************************************************************************************************************************/

void cross_edge_add()               //此函数为十字边沿补充函数（修正十字错误边沿）
{
      if( L_cross_flag==1 || R_cross_flag==1 || cross_flag==1 )   //当检测出十字
      {
            if( L_up_cross[0]==-1 && R_up_cross[0]==-1 )   //清除错误十字拐点
            {
                  if( L_down_cross[0]!=-1 )
                  {
                        if( R_down_cross[0]==-1 )
                        {
                              Site_xy xy;
                              Site_xy1 xy1;
                              xy=get_inv_img( L_down_cross[0] , L_down_cross[1] );
                              if( xy.y<40 )
                              {
                                    int16 r_start,r_end;
                                    xy1=get_invinv_img( xy.x-60 , 0 );
                                    r_start=xy1.x;
                                    if( r_start>ROW_END )
                                    {
                                          r_start=ROW_END;
                                    }
                                    xy1=get_invinv_img( xy.x+120 , 0 );
                                    r_end=xy1.x;
                                    if( L_TURN==1 && R_end_row!=-1 )
                                    {
                                          if( R_end_row>r_end )
                                          {
                                                r_end=R_end_row;
                                          }
                                    }
                                    else
                                    {
                                          if( r_end<ROW_START )
                                          {
                                                r_end=ROW_START;
                                          }
                                    }
                                    
                                    W_num=0;
                                    for( W_num_row=r_start-1 ; W_num_row>=r_end ; W_num_row-- )
                                    {
                                          if( right_edge[W_num_row]!=-1 && right_edge[W_num_row+1]!=-1 && right_edge[W_num_row]<=right_edge[W_num_row+1] )
                                          {
                                                continue;
                                          }
                                          else
                                          {
                                                W_num++;
                                          }
                                    }
                                    if( W_num<=2 )
                                    {
                                          L_down_cross[0]=-1;
                                          L_down_cross[1]=-1;
                                          L_cross_flag=0;
                                          cross_flag=0;
                                    }
                              }
                        }
                  }
                  else
                  {
                        if( R_down_cross[0]!=-1 )
                        {
                              Site_xy xy;
                              Site_xy1 xy1;
                              xy=get_inv_img( R_down_cross[0] , R_down_cross[1] );
                              if( xy.y>-40 )
                              {
                                    int16 r_start,r_end;
                                    xy1=get_invinv_img( xy.x-60 , 0 );
                                    r_start=xy1.x;
                                    if( r_start>ROW_END )
                                    {
                                          r_start=ROW_END;
                                    }
                                    xy1=get_invinv_img( xy.x+120 , 0 );
                                    r_end=xy1.x;
                                    if( R_TURN==1 && L_end_row!=-1 )
                                    {
                                          if( R_end_row>r_end )
                                          {
                                                r_end=R_end_row;
                                          }
                                    }
                                    else
                                    {
                                          if( r_end<ROW_START )
                                          {
                                                r_end=ROW_START;
                                          }
                                    }
                                    
                                    W_num=0;
                                    for( W_num_row=r_start-1 ; W_num_row>=r_end ; W_num_row-- )
                                    {
                                          if( left_edge[W_num_row]!=-1 && left_edge[W_num_row+1]!=-1 && left_edge[W_num_row]>=left_edge[W_num_row+1] )
                                          {
                                                continue;
                                          }
                                          else
                                          {
                                                W_num++;
                                          }
                                    }
                                    if( W_num<=2 )
                                    {
                                          R_down_cross[0]=-1;
                                          R_down_cross[1]=-1;
                                          R_cross_flag=0;
                                          cross_flag=0;
                                    }
                              }
                        }
                  }
            }
            
            if( loop_able==1 && LOOP_IN==0 && LOOP_FLAG==0 )    //环道补充判断，斜入判断
            {
                  if( L_down_cross[0]!=-1 )   //通过边沿趋势判断环道
                  {
                        if( R_down_cross[0]!=-1 )
                        {
                              if( L_down_cross[1]<=40 && L_cross_k>0 && inv_distance[L_down_cross[0]]<240 )
                              {
                                    W_num=0;
                                    for( W_num_row=L_down_cross[0]-1 ; W_num_row>=ROW_START ; W_num_row-- )
                                    {
                                          if( left_edge[W_num_row]!=-1 )
                                          {
                                                W_num++;
                                          }
                                          else
                                          {
                                                break;
                                          }
                                    }
                                    if( W_num<=3 )   //发现类似斜入环特征
                                    {
                                          if( L_up_cross[0]!=-1 )    //搜索环道特征边沿
                                          {
                                                double aa;
                                                Site_xy xy1,xy2;
                                                xy1=get_inv_img( L_down_cross[0] , L_down_cross[1] );
                                                
                                                W_num=0;
                                                W_num_col=0;
                                                for( W_num_row=L_up_cross[0] ; W_num_row>=ROW_START ; W_num_row-- )
                                                {
                                                      if( fabs(inv_distance[W_num_row]-inv_distance[L_down_cross[0]])>400 )
                                                      {
                                                            break;
                                                      }
                                                      if( left_edge[W_num_row]!=-1 )
                                                      {
                                                            xy2=get_inv_img( W_num_row , left_edge[W_num_row] );
                                                            aa=xy1.y-L_cross_k*fabsf(inv_distance[W_num_row]-xy1.x);
                                                            if( xy2.y>aa+120 )
                                                            {
                                                                  W_num++;
                                                                  if( W_num>4 )
                                                                  {
                                                                        break;
                                                                  }
                                                            }
                                                            else
                                                            {
                                                                  if( xy2.y<aa+40 )
                                                                  {
                                                                        W_num_col++;
                                                                        if( W_num_col>=7 )
                                                                        {
                                                                              break;
                                                                        }
                                                                  }
                                                            }
                                                      }
                                                }
                                                if( W_num>4 )
                                                {
                                                      LOOP_FLAG=1;                          //判断为环型赛道
                                                      
                                                      LOOP_IN_ROW=L_down_cross[0];            //环型赛道入口行赋值
                                                }
                                          }
                                    }
                              }
                              
                              if( R_down_cross[1]>=119 && R_cross_k<0 && inv_distance[R_down_cross[0]]<240 )
                              {
                                    W_num=0;
                                    for( W_num_row=R_down_cross[0]-1 ; W_num_row>=ROW_START ; W_num_row-- )
                                    {
                                          if( right_edge[W_num_row]!=-1 )
                                          {
                                                W_num++;
                                          }
                                          else
                                          {
                                                break;
                                          }
                                    }
                                    if( W_num<=3 )   //发现类似斜入环特征
                                    {
                                          if( R_up_cross[0]!=-1 )    //搜索环道特征边沿
                                          {
                                                double aa;
                                                Site_xy xy1,xy2;
                                                W_num=0;
                                                xy1=get_inv_img( R_down_cross[0] , R_down_cross[1] );
                                                
                                                
                                                W_num=0;
                                                W_num_col=0;
                                                for( W_num_row=R_up_cross[0] ; W_num_row>=ROW_START ; W_num_row-- )
                                                {
                                                      if( fabs(inv_distance[W_num_row]-inv_distance[R_down_cross[0]])>400 )
                                                      {
                                                            break;
                                                      }
                                                      if( right_edge[W_num_row]!=-1 )
                                                      {
                                                            xy2=get_inv_img( W_num_row , right_edge[W_num_row] );
                                                            aa=xy1.y-R_cross_k*fabsf(inv_distance[W_num_row]-xy1.x);
                                                            if( xy2.y<aa-120 )
                                                            {
                                                                  W_num++;
                                                                  if( W_num>4 )
                                                                  {
                                                                        break;
                                                                  }
                                                            }
                                                            else
                                                            {
                                                                  if( xy2.y>aa-40 )
                                                                  {
                                                                        W_num_col++;
                                                                        if( W_num_col>=7 )
                                                                        {
                                                                              break;
                                                                        }
                                                                  }
                                                            }
                                                      }
                                                }
                                                if( W_num>4 )
                                                {
                                                      LOOP_FLAG=1;                          //判断为环型赛道
                                                      
                                                      LOOP_IN_ROW=R_down_cross[0];            //环型赛道入口行赋值
                                                }
                                          }
                                    }
                              }
                        }
                        else
                        {
                              if( L_down_cross[1]<=40 && L_cross_k>0 && inv_distance[L_down_cross[0]]<240 )
                              {
                                    W_num=0;
                                    for( W_num_row=L_down_cross[0]-1 ; W_num_row>=ROW_START ; W_num_row-- )
                                    {
                                          if( left_edge[W_num_row]!=-1 )
                                          {
                                                W_num++;
                                          }
                                          else
                                          {
                                                break;
                                          }
                                    }
                                    if( W_num<=3 )   //发现类似斜入环特征
                                    {
                                          if( L_up_cross[0]!=-1 )   //当存在上十字
                                          {
                                                W_num=0;
                                                for( W_num_row=L_down_cross[0]-3 ; W_num_row>=ROW_START ; W_num_row-- )  //搜索环道圆环
                                                {
                                                      if( fabs(inv_distance[W_num_row]-inv_distance[L_down_cross[0]])>400 )
                                                      {
                                                            break;
                                                      }
                                                      if( left_edge[W_num_row]!=-1 )
                                                      {
                                                            for( W_num_col=left_edge[W_num_row]-1 ; W_num_col>=COL_START+3 ; W_num_col-- )
                                                            {
                                                                  if( img_data[EXTRACT_ROW][EXTRACT_COL] == DOT_B && img_data[EXTRACT_ROW][EXTRACT_COL-1] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL-3] == DOT_W )
                                                                  {
                                                                        W_num++;
                                                                        break;
                                                                  }
                                                            }
                                                      }
                                                }
                                                if( W_num>=5 )  //搜索得到环道圆环
                                                {
                                                      LOOP_FLAG=1;                          //判断为环型赛道
                                                      
                                                      LOOP_IN_ROW=L_down_cross[0];            //环型赛道入口行赋值
                                                }
                                                else    //搜索环道特征边沿
                                                {
                                                      double aa;
                                                      Site_xy xy1,xy2;
                                                      xy1=get_inv_img( L_down_cross[0] , L_down_cross[1] );
                                                      
                                                      W_num=0;
                                                      W_num_col=0;
                                                      for( W_num_row=L_up_cross[0] ; W_num_row>=ROW_START ; W_num_row-- )
                                                      {
                                                            if( fabs(inv_distance[W_num_row]-inv_distance[L_down_cross[0]])>400 )
                                                            {
                                                                  break;
                                                            }
                                                            if( left_edge[W_num_row]!=-1 )
                                                            {
                                                                  xy2=get_inv_img( W_num_row , left_edge[W_num_row] );
                                                                  aa=xy1.y-L_cross_k*fabsf(inv_distance[W_num_row]-xy1.x);
                                                                  if( xy2.y>aa+120 )
                                                                  {
                                                                        W_num++;
                                                                        if( W_num>4 )
                                                                        {
                                                                              break;
                                                                        }
                                                                  }
                                                                  else
                                                                  {
                                                                        if( xy2.y<aa+40 )
                                                                        {
                                                                              W_num_col++;
                                                                              if( W_num_col>=7 )
                                                                              {
                                                                                    break;
                                                                              }
                                                                        }
                                                                  }
                                                            }
                                                      }
                                                      if( W_num>4 )
                                                      {
                                                            LOOP_FLAG=1;                          //判断为环型赛道
                                                            
                                                            LOOP_IN_ROW=L_down_cross[0];            //环型赛道入口行赋值
                                                      }
                                                }
                                          }
                                    }
                              }
                        }
                  }
                  else
                  {
                        if( R_down_cross[0]!=-1 )
                        {
                              if( R_down_cross[1]>=119 && R_cross_k<0 && inv_distance[R_down_cross[0]]<240 )
                              {
                                    W_num=0;
                                    for( W_num_row=R_down_cross[0]-1 ; W_num_row>=ROW_START ; W_num_row-- )
                                    {
                                          if( right_edge[W_num_row]!=-1 )
                                          {
                                                W_num++;
                                          }
                                          else
                                          {
                                                break;
                                          }
                                    }
                                    if( W_num<=3 )   //发现类似斜入环特征
                                    {
                                          if( R_up_cross[0]!=-1 )
                                          {
                                                W_num=0;
                                                for( W_num_row=R_down_cross[0]-3 ; W_num_row>=ROW_START ; W_num_row-- )  //搜索环道圆环
                                                {
                                                      if( fabs(inv_distance[W_num_row]-inv_distance[R_down_cross[0]])>400 )
                                                      {
                                                            break;
                                                      }
                                                      if( right_edge[W_num_row]!=-1 )
                                                      {
                                                            for( W_num_col=right_edge[W_num_row]-1 ; W_num_col<=COL_END-3 ; W_num_col++ )
                                                            {
                                                                  if( img_data[EXTRACT_ROW][EXTRACT_COL] == DOT_B && img_data[EXTRACT_ROW][EXTRACT_COL+1] == DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL+3] == DOT_W )
                                                                  {
                                                                        W_num++;
                                                                        break;
                                                                  }
                                                            }
                                                      }
                                                }
                                                if( W_num>=5 )  //搜索得到环道圆环
                                                {
                                                      LOOP_FLAG=1;                          //判断为环型赛道
                                                      
                                                      LOOP_IN_ROW=R_down_cross[0];            //环型赛道入口行赋值
                                                }
                                                else    //搜索环道特征边沿
                                                {
                                                      double aa;
                                                      Site_xy xy1,xy2;
                                                      xy1=get_inv_img( R_down_cross[0] , R_down_cross[1] );
                                                      
                                                      W_num=0;
                                                      W_num_col=0;
                                                      for( W_num_row=R_up_cross[0] ; W_num_row>=ROW_START ; W_num_row-- )
                                                      {
                                                            if( fabs(inv_distance[W_num_row]-inv_distance[R_down_cross[0]])>400 )
                                                            {
                                                                  break;
                                                            }
                                                            if( right_edge[W_num_row]!=-1 )
                                                            {
                                                                  xy2=get_inv_img( W_num_row , right_edge[W_num_row] );
                                                                  aa=xy1.y-R_cross_k*fabsf(inv_distance[W_num_row]-xy1.x);
                                                                  if( xy2.y<aa-120 )
                                                                  {
                                                                        W_num++;
                                                                        if( W_num>4 )
                                                                        {
                                                                              break;
                                                                        }
                                                                  }
                                                                  else
                                                                  {
                                                                        if( xy2.y>aa-40 )
                                                                        {
                                                                              W_num_col++;
                                                                              if( W_num_col>=7 )
                                                                              {
                                                                                    break;
                                                                              }
                                                                        }
                                                                  }
                                                            }
                                                      }
                                                      if( W_num>4 )
                                                      {
                                                            LOOP_FLAG=1;                          //判断为环型赛道
                                                            
                                                            LOOP_IN_ROW=R_down_cross[0];            //环型赛道入口行赋值
                                                      }
                                                }
                                          }
                                    }
                              }
                        }
                  }
                  
                  if( LOOP_FLAG==0 )   //通过前方黑块判断环道
                  {
                        if( L_down_cross[0]!=-1 )
                        {
                              if( R_down_cross[0]!=-1 )
                              {
                                    if( L_up_cross[0]==-1 || R_up_cross[0]==-1 )
                                    {
                                          if( fabsf(inv_distance[L_down_cross[0]]-inv_distance[R_down_cross[0]])<80 )
                                          {
                                                double aa;
                                                Site_xy xy_l,xy_r;
                                                Site_xy1 xy1,xy2;
                                                int16 l_edge,r_edge;
                                                int16 r_start,r_end,b_point,edge_able=0;
                                                xy_l=get_inv_img( L_down_cross[0] , L_down_cross[1] );
                                                xy_r=get_inv_img( R_down_cross[0] , R_down_cross[1] );
                                                
                                                if( xy_l.x<320 && xy_r.x<320 && fabsf(xy_l.y-xy_r.y)>120 && fabsf(xy_l.y-xy_r.y)<200 )
                                                {
                                                      aa=0.5*(xy_l.x+xy_l.x)+220;
                                                      xy1=get_invinv_img( aa , 0 );
                                                      r_start=xy1.x;
                                                      if( r_start<0 )
                                                      {
                                                            r_start=0;
                                                      }
                                                      aa=0.5*(xy_l.x+xy_l.x)+400;
                                                      xy1=get_invinv_img( aa , 0 );
                                                      r_end=xy1.x;
                                                      if( r_end<0 )
                                                      {
                                                            r_end=0;
                                                      }
                                                      
                                                      W_num=0;
                                                      b_point=0;
                                                      for( W_num_row=r_start ; W_num_row>=r_end ; W_num_row-- )
                                                      {
                                                            aa=xy_l.y-L_cross_k*fabsf(inv_distance[W_num_row]-xy_l.x);
                                                            xy1=get_invinv_img( inv_distance[W_num_row] , aa );
                                                            aa=xy_r.y-R_cross_k*fabsf(inv_distance[W_num_row]-xy_r.x);
                                                            xy2=get_invinv_img( inv_distance[W_num_row] , aa );
                                                            if( xy1.y>=COL_END )
                                                            {
                                                                  break;
                                                            }
                                                            else
                                                            {
                                                                  if( xy1.y<COL_START )
                                                                  {
                                                                        l_edge=COL_START;
                                                                  }
                                                                  else
                                                                  {
                                                                        l_edge=xy1.y;
                                                                  }
                                                            }
                                                            
                                                            if( xy2.y<=COL_START )
                                                            {
                                                                  break;
                                                            }
                                                            else
                                                            {
                                                                  if( xy2.y>COL_END )
                                                                  {
                                                                        r_edge=COL_END;
                                                                  }
                                                                  else
                                                                  {
                                                                        r_edge=xy2.y;
                                                                  }
                                                            }
                                                            for( W_num_col=l_edge ; W_num_col<=r_edge ; W_num_col++ )
                                                            {
                                                                  W_num++;
                                                                  if( img_data[W_num_row][W_num_col]==DOT_B )
                                                                  {
                                                                        b_point++;
                                                                  }
                                                            }
                                                      }
                                                      if( W_num==0)
                                                      {
                                                            aa=0;
                                                      }
                                                      else
                                                      {
                                                            aa=100.0*b_point/W_num;
                                                      }
                                                      
                                                      //angle_aaa[0]=aa;
                                                      //angle_aaa[1]=b_point;
                                                      //angle_aaa[2]=W_num;
                                                      
                                                      if( aa>loop_per && W_num>=250 )
                                                      {
                                                            LOOP_FLAG=1;                          //判断为环型赛道
                                                            
                                                            LOOP_IN_ROW=L_down_cross[0];            //环型赛道入口行赋值
                                                      }
                                                }
                                          }
                                    }
                              }
                              else
                              {
                                    if( L_up_cross[0]==-1 )
                                    {
                                          double aa;
                                          Site_xy xy;
                                          Site_xy1 xy1;
                                          int16 r_start,r_end,b_point,edge_able=0;
                                          xy=get_inv_img( L_down_cross[0] , L_down_cross[1] );
                                          
                                          if( xy.x<320 && xy.y<40 )
                                          {
                                                if( xy.x>240 )
                                                {
                                                      edge_able=1;
                                                }
                                                
                                                aa=xy.x+220;
                                                xy1=get_invinv_img( aa , 0 );
                                                r_start=xy1.x;
                                                if( r_start<0 )
                                                {
                                                      r_start=0;
                                                }
                                                aa=xy.x+400;
                                                xy1=get_invinv_img( aa , 0 );
                                                r_end=xy1.x;
                                                if( r_end<0 )
                                                {
                                                      r_end=0;
                                                }
                                                
                                                W_num=0;
                                                b_point=0;
                                                for( W_num_row=r_start ; W_num_row>=r_end ; W_num_row-- )
                                                {
                                                      if( L_TURN==1 && R_end_row!=-1 && W_num_row<R_end_row )
                                                      {
                                                            break;
                                                      }
                                                      
                                                      if( fabsf(inv_distance[W_num_row]-xy.x)>cross_k_2 )
                                                      {
                                                            aa=cross_k_1*cross_k_2;
                                                      }
                                                      else
                                                      {
                                                            aa=cross_k_1*fabsf(inv_distance[W_num_row]-xy.x);
                                                      }
                                                      aa=xy.y-L_cross_k*fabsf(inv_distance[W_num_row]-xy.x)+aa;
                                                      xy1=get_invinv_img( inv_distance[W_num_row] , aa );
                                                      if( xy1.y>=COL_END )
                                                      {
                                                            break;
                                                      }
                                                      if( edge_able==1 )
                                                      {
                                                            if( right_edge[W_num_row]!=-1 &&  xy1.y>right_edge[W_num_row] )
                                                            {
                                                                  break;
                                                            }
                                                      }
                                                      
                                                      for( W_num_col=xy1.y ; W_num_col<=xy1.y+2*bar_range_col[W_num_row] ; W_num_col++ )
                                                      {
                                                            if( W_num_col>COL_END )
                                                            {
                                                                  break;
                                                            }
                                                            W_num++;
                                                            if( img_data[W_num_row][W_num_col]==DOT_B )
                                                            {
                                                                  b_point++;
                                                            }
                                                      }
                                                }
                                                if( W_num==0)
                                                {
                                                      aa=0;
                                                }
                                                else
                                                {
                                                      aa=100.0*b_point/W_num;
                                                }
                                                
                                                //angle_aaa[0]=aa;
                                                //angle_aaa[1]=b_point;
                                                //angle_aaa[2]=W_num;
                                                
                                                if( aa>loop_per && W_num>=250 )
                                                {
                                                      LOOP_FLAG=1;                          //判断为环型赛道
                                                      
                                                      LOOP_IN_ROW=L_down_cross[0];            //环型赛道入口行赋值
                                                }
                                          }
                                    }
                              }
                        }
                        else
                        {
                              if( R_down_cross[0]!=-1 && R_up_cross[0]==-1 )
                              {
                                    double aa;
                                    Site_xy xy;
                                    Site_xy1 xy1;
                                    int16 r_start,r_end,b_point,edge_able=0;
                                    xy=get_inv_img( R_down_cross[0] , R_down_cross[1] );
                                    
                                    if( xy.x<320 && xy.y>-40 )
                                    {
                                          if( xy.x>240 )
                                          {
                                                edge_able=1;
                                          }
                                          aa=xy.x+220;
                                          xy1=get_invinv_img( aa , 0 );
                                          r_start=xy1.x;
                                          if( r_start<0 )
                                          {
                                                r_start=0;
                                          }
                                          aa=xy.x+400;
                                          xy1=get_invinv_img( aa , 0 );
                                          r_end=xy1.x;
                                          if( r_end<0 )
                                          {
                                                r_end=0;
                                          }
                                          
                                          W_num=0;
                                          b_point=0;
                                          for( W_num_row=r_start ; W_num_row>=r_end ; W_num_row-- )
                                          {
                                                if( R_TURN==1 && L_end_row!=-1 && W_num_row<L_end_row )
                                                {
                                                      break;
                                                }
                                                
                                                if( fabsf(inv_distance[W_num_row]-xy.x)>cross_k_2 )
                                                {
                                                      aa=cross_k_1*cross_k_2;
                                                }
                                                else
                                                {
                                                      aa=cross_k_1*fabsf(inv_distance[W_num_row]-xy.x);
                                                }
                                                aa=xy.y-R_cross_k*fabsf(inv_distance[W_num_row]-xy.x)-aa;
                                                xy1=get_invinv_img( inv_distance[W_num_row] , aa );
                                                if( xy1.y<=COL_START )
                                                {
                                                      break;
                                                }
                                                if( edge_able==1 )
                                                {
                                                      if( left_edge[W_num_row]!=-1 &&  xy1.y<left_edge[W_num_row] )
                                                      {
                                                            break;
                                                      }
                                                }
                                                
                                                for( W_num_col=xy1.y ; W_num_col>=xy1.y-2*bar_range_col[W_num_row] ; W_num_col-- )
                                                {
                                                      if( W_num_col<COL_START )
                                                      {
                                                            break;
                                                      }
                                                      W_num++;
                                                      if( img_data[W_num_row][W_num_col]==DOT_B )
                                                      {
                                                            b_point++;
                                                      }
                                                }
                                          }
                                          if( W_num==0)
                                          {
                                                aa=0;
                                          }
                                          else
                                          {
                                                aa=100.0*b_point/W_num;
                                          }
                                          
                                          //angle_aaa[0]=aa;
                                          //angle_aaa[1]=b_point;
                                          //angle_aaa[2]=W_num;
                                          
                                          if( aa>loop_per && W_num>=250 )
                                          {
                                                LOOP_FLAG=1;                          //判断为环型赛道
                                                
                                                LOOP_IN_ROW=R_down_cross[0];            //环型赛道入口行赋值
                                          }
                                    }
                              }
                        }
                  }
            }
            
            
            if( L_up_cross[0]!=-1 )  //左边沿上十字误判处理
            {
                  double aa=0;
                  W_num=0;
                  for(  W_num_row=L_up_cross[0]-1 ; W_num_row>=L_up_cross[0]-10 ; W_num_row--  )  //选取合适的上下拐点
                  {
                        if( W_num_row<ROW_START )
                        {
                              break;
                        }
                        if( left_edge[W_num_row]!=-1 )
                        {
                              W_num++;
                              aa=aa+fabsf(inv_distance[W_num_row]-inv_distance[W_num_row+1]);
                        }
                  }
                  if( W_num<=4 && aa<100 )
                  {
                        L_up_cross[0]=-1;
                        L_up_cross[1]=-1;
                        L_cross_flag=0;
                        cross_flag=0;
                  }
            }
            if( R_up_cross[0]!=-1 )  //右边沿上十字误判处理
            {
                  double aa=0;
                  W_num=0;
                  for(  W_num_row=R_up_cross[0]-1 ; W_num_row>=R_up_cross[0]-10 ; W_num_row--  )  //选取合适的上下拐点
                  {
                        if( W_num_row<ROW_START )
                        {
                              break;
                        }
                        if( right_edge[W_num_row]!=-1 )
                        {
                              W_num++;
                              aa=aa+fabsf(inv_distance[W_num_row]-inv_distance[W_num_row+1]);
                        }
                  }
                  if( W_num<=4 && aa<100 )
                  {
                        R_up_cross[0]=-1;
                        R_up_cross[1]=-1;
                        R_cross_flag=0;
                        cross_flag=0;
                  }
            }
            
            /*出Q弯时*/
            if( L_up_cross[0]==-1 && L_down_cross[1]!=-1 && L_down_cross[1]>=110 && L_down_cross[0]<40 )  //清除错误边沿
            {
                  L_TURN=0;
                  R_TURN=1;
                  L_up_cross[0]=-1;
                  L_up_cross[1]=-1;
                  L_end_row=-1;
                  R_end_row=-1;
                  for(  W_num_row=L_down_cross[0]-1 ; W_num_row>=ROW_START ; W_num_row--  )
                  {
                        if( left_edge[W_num_row]!=-1 )
                        {
                              left_edge[W_num_row]=-1;
                              L_EDGE_NUM--;
                        }
                        if( right_edge[W_num_row]!=-1 )
                        {
                              right_edge[W_num_row]=-1;
                              R_EDGE_NUM--;
                        }
                  }
                  
                  /*十字唯一向上连线*/
                  if( L_cross_k!=10 )   //向下搜索得到斜率
                  {
                        double aa;
                        Site_xy1 xy2;
                        Site_xy xy1;
                        
                        xy1=get_inv_img( L_down_cross[0] , L_down_cross[1] );
                        
                        for( cross_row=L_down_cross[0]-1 ; cross_row>=ROW_START ; cross_row-- )
                        {
                              aa=xy1.y-L_cross_k*fabsf(inv_distance[cross_row]-xy1.x);
                              xy2=get_invinv_img( inv_distance[cross_row] , aa );
                              
                              if( xy2.y>COL_END )
                              {
                                    break;
                              }
                              
                              left_edge[cross_row]=xy2.y;
                              L_EDGE_NUM++;
                        }
                  }
            }
            else
            {
                  if( L_down_cross[0]<20 && L_up_cross[0]==-1 )
                  {
                        for(  W_num_row=L_down_cross[0]-1 ; W_num_row>=ROW_START ; W_num_row--  )
                        {
                              if( left_edge[W_num_row]!=-1 )
                              {
                                    left_edge[W_num_row]=-1;
                                    L_EDGE_NUM--;
                              }
                        }
                  }
            }
            if( R_up_cross[0]==-1 && R_down_cross[1]!=-1 && R_down_cross[1]<=50 && R_down_cross[0]<40 )  //清除错误边沿
            {
                  L_TURN=1;
                  R_TURN=0;
                  R_up_cross[0]=-1;
                  R_up_cross[1]=-1;
                  L_end_row=-1;
                  R_end_row=-1;
                  for(  W_num_row=R_down_cross[0]-1 ; W_num_row>=ROW_START ; W_num_row--  )
                  {
                        if( left_edge[W_num_row]!=-1 )
                        {
                              left_edge[W_num_row]=-1;
                              L_EDGE_NUM--;
                        }
                        if( right_edge[W_num_row]!=-1 )
                        {
                              right_edge[W_num_row]=-1;
                              R_EDGE_NUM--;
                        }
                  }
                  
                  /*十字唯一向上连线*/
                  if( R_cross_k!=10 )   //向下搜索得到斜率
                  {
                        double aa;
                        Site_xy1 xy2;
                        Site_xy xy1;
                        
                        xy1=get_inv_img( R_down_cross[0] , R_down_cross[1] );
                        
                        for( cross_row=R_down_cross[0]-1 ; cross_row>=ROW_START ; cross_row-- )
                        {
                              aa=xy1.y-R_cross_k*fabsf(inv_distance[cross_row]-xy1.x);
                              xy2=get_invinv_img( inv_distance[cross_row] , aa );
                              
                              if( xy2.y<COL_START )
                              {
                                    break;
                              }
                              
                              right_edge[cross_row]=xy2.y;
                              R_EDGE_NUM++;
                        }
                  }
            }
            else
            {
                  if( R_down_cross[0]<20 && R_up_cross[0]==-1 )
                  {
                        for(  W_num_row=R_down_cross[0]-1 ; W_num_row>=ROW_START ; W_num_row--  )
                        {
                              if( right_edge[W_num_row]!=-1 )
                              {
                                    right_edge[W_num_row]=-1;
                                    R_EDGE_NUM--;
                              }
                        }
                  }
            }
            
            /*以下程序段为左边沿十字补边程序*/
            
            if( L_down_cross[0]!=-1 && L_up_cross[0]!=-1 )   //左边沿上下十字直角拐点检测出
            {
                  int16 down_edge[2]={-1,-1},up_edge[2]={-1,-1};
                  for(  W_num_row=L_down_cross[0]+2 ; W_num_row<=L_down_cross[0]+4 ; W_num_row++  )  //选取合适的上下拐点
                  {
                        if( W_num_row>ROW_END )
                        {
                              break;
                        }
                        if( left_edge[W_num_row]!=-1 )
                        {
                              down_edge[0]=W_num_row;
                              
                              down_edge[1]=left_edge[W_num_row];
                              
                              break;
                        }
                  }
                  if( down_edge[0]==-1 )
                  {
                        down_edge[0]=L_down_cross[0];
                        down_edge[1]=L_down_cross[1];
                  }
                  for(  W_num_row=L_up_cross[0]-2 ; W_num_row>=L_up_cross[0]-4 ; W_num_row--  )  //选取合适的上下拐点
                  {
                        if( W_num_row<ROW_START )
                        {
                              break;
                        }
                        if( left_edge[W_num_row]!=-1 && left_edge[W_num_row]-L_up_cross[1]>=0 )
                        {
                              up_edge[0]=W_num_row;
                              
                              up_edge[1]=left_edge[W_num_row];
                              
                              break;
                        }
                  }
                  if( up_edge[0]==-1 )
                  {
                        up_edge[0]=L_up_cross[0];
                        up_edge[1]=L_up_cross[1];
                  }
                  
                  cross_high=down_edge[0]-up_edge[0];
                  cross_wide=up_edge[1]-down_edge[1];
                  cross_wide=cross_wide/cross_high;   //为正值
                  cross_edge=down_edge[1];
                  for( cross_row=down_edge[0]-1 ; cross_row>up_edge[0] ; cross_row-- )
                  {
                        cross_edge=cross_edge+cross_wide;
                        cross_col=round(cross_edge);
                        left_edge[cross_row]=cross_col;
                        L_EDGE_NUM++;
                  }
            }
            else    //左边沿上下十字直角拐点未全部检测出
            {
                  if( L_down_cross[0]==-1 && L_up_cross[0]!=-1 )  //只有左边沿上十字直角拐点检测出
                  {
                        Site_xy1 xy1;
                        xy1=get_invinv_img(inv_distance[L_up_cross[0]]-120,0);
                        if( xy1.x<ROW_END )
                        {
                              W_num=0;
                              for( W_num_row=ROW_END ; W_num_row>=xy1.x ; W_num_row-- )
                              {
                                    if( left_edge[W_num_row]!=-1 && left_edge[W_num_row-1]!=-1 )
                                    {
                                          if( left_edge[W_num_row]<=left_edge[W_num_row-1] && left_edge[W_num_row]+5>=left_edge[W_num_row-1] )
                                          {
                                                L_down_cross[0]=W_num_row-1;
                                                L_down_cross[1]=left_edge[W_num_row-1];
                                          }
                                          else
                                          {
                                                W_num++;
                                                if( W_num>=4 )
                                                {
                                                      break;
                                                }
                                          }
                                    }
                                    else
                                    {
                                          if( L_down_cross[0]!=-1 )
                                          {
                                                W_num++;
                                                if( W_num>=4 )
                                                {
                                                      break;
                                                }
                                          }
                                    }
                              }
                        }
                        
                        if( L_down_cross[0]==-1 )
                        {
                              int16 up_edge[2]={-1,-1};
                              for( W_num_row=L_up_cross[0]-2 ; W_num_row>=L_up_cross[0]-4 ; W_num_row-- )  //选取合适的上下拐点
                              {
                                    if( W_num_row<ROW_START )
                                    {
                                          break;
                                    }
                                    if( left_edge[W_num_row]!=-1 )
                                    {
                                          up_edge[0]=W_num_row;
                                          
                                          up_edge[1]=left_edge[W_num_row];
                                          
                                          break;
                                    }
                              }
                              if( up_edge[0]==-1 )
                              {
                                    up_edge[0]=L_up_cross[0];
                                    up_edge[1]=L_up_cross[1];
                              }
                              
                              double x=0.0,y=0.0,x1,y1,k=0.0,a;
                              Site_xy xy1,xy2;
                              Site_xy1 xy3;
                              
                              xy1=get_inv_img( up_edge[0] , up_edge[1] );    //获取拐点逆透视坐标
                              
                              a=xy1.x+50;
                              xy3=get_invinv_img( a , 0 );
                              if( up_edge[0]-xy3.x>15 )
                              {
                                    W_num_row=15;
                              }
                              else
                              {
                                    if( up_edge[0]-xy3.x<3 )
                                    {
                                          W_num_row=3;
                                    }
                                    else
                                    {
                                          W_num_row=abs(up_edge[0]-xy3.x);
                                    }
                              }
                              
                              for( W_num=1 ; W_num<=W_num_row ; W_num++ )   //获取拐点向下的拟合斜率
                              {
                                    W_num_col=up_edge[0]-W_num;
                                    if( W_num_col<ROW_START )
                                    {
                                          break;
                                    }
                                    if( left_edge[W_num_col]!=-1 )
                                    {
                                          xy2=get_inv_img( W_num_col , left_edge[W_num_col] );
                                          x1=xy2.x-xy1.x;
                                          y1=xy2.y-xy1.y;
                                          x=x+x1*y1;
                                          y=y+x1*x1;
                                    }
                                    else
                                    {
                                          break;
                                    }
                              }
                              if( x!=0 )
                              {
                                    k=(-1)*x/y;    //得到拐点向下的拟合斜率
                              }
                              else
                              {
                                    k==0;
                              }
                              cross_wide=k;        //取向下拟合斜率
                              
                              cross_edge=up_edge[1];
                              for( cross_row=up_edge[0]+1 ; cross_row<=ROW_END ; cross_row++ )
                              {
                                    a=xy1.y+cross_wide*fabsf(inv_distance[cross_row]-xy1.x);
                                    xy3=get_invinv_img( inv_distance[cross_row] , a );
                                    if( xy3.y<COL_START )
                                    {
                                          break;
                                    }
                                    
                                    left_edge[cross_row]=xy3.y;
                                    L_EDGE_NUM++;
                              }
                        }
                        else
                        {
                              int16 up_edge[2]={-1,-1};
                              for(  W_num_row=L_up_cross[0]-2 ; W_num_row>=L_up_cross[0]-4 ; W_num_row--  )  //选取合适的上下拐点
                              {
                                    if( W_num_row<ROW_START )
                                    {
                                          break;
                                    }
                                    if( left_edge[W_num_row]!=-1 )
                                    {
                                          up_edge[0]=W_num_row;
                                          
                                          up_edge[1]=left_edge[W_num_row];
                                          
                                          break;
                                    }
                              }
                              if( up_edge[0]==-1 )
                              {
                                    up_edge[0]=L_up_cross[0];
                                    up_edge[1]=L_up_cross[1];
                              }
                              
                              cross_high=L_down_cross[0]-up_edge[0];
                              cross_wide=up_edge[1]-left_edge[L_down_cross[0]];
                              cross_wide=cross_wide/cross_high;   //为正值
                              cross_edge=left_edge[L_down_cross[0]];
                              for( cross_row=L_down_cross[0]-1 ; cross_row>up_edge[0] ; cross_row-- )
                              {
                                    cross_edge=cross_edge+cross_wide;
                                    cross_col=round(cross_edge);
                                    left_edge[cross_row]=cross_col;
                                    L_EDGE_NUM++;
                              }
                        }
                  }
            }
            
            /*以下程序段为右边沿十字补边程序*/
            
            if( R_down_cross[0]!=-1 && R_up_cross[0]!=-1 )   //右边沿上下十字直角拐点检测出
            {
                  int16 down_edge[2]={-1,-1},up_edge[2]={-1,-1};
                  for(  W_num_row=R_down_cross[0]+2 ; W_num_row<=R_down_cross[0]+4 ; W_num_row++  )  //选取合适的上下拐点
                  {
                        if( W_num_row>ROW_END )
                        {
                              break;
                        }
                        if( right_edge[W_num_row]!=-1 )
                        {
                              down_edge[0]=W_num_row;
                              
                              down_edge[1]=right_edge[W_num_row];
                              
                              break;
                        }
                  }
                  if( down_edge[0]==-1 )
                  {
                        down_edge[0]=R_down_cross[0];
                        down_edge[1]=R_down_cross[1];
                  }
                  for(  W_num_row=R_up_cross[0]-2 ; W_num_row>=R_up_cross[0]-4 ; W_num_row--  )  //选取合适的上下拐点
                  {
                        if( W_num_row<ROW_START )
                        {
                              break;
                        }
                        if( right_edge[W_num_row]!=-1 && right_edge[W_num_row]-R_up_cross[1]<=0 )
                        {
                              up_edge[0]=W_num_row;
                              
                              up_edge[1]=right_edge[W_num_row];
                              
                              break;
                        }
                  }
                  if( up_edge[0]==-1 )
                  {
                        up_edge[0]=R_up_cross[0];
                        up_edge[1]=R_up_cross[1];
                  }
                  
                  cross_high=down_edge[0]-up_edge[0];
                  cross_wide=up_edge[1]-down_edge[1];
                  cross_wide=cross_wide/cross_high;
                  cross_edge=down_edge[1];
                  for( cross_row=down_edge[0]-1 ; cross_row>up_edge[0] ; cross_row-- )
                  {
                        cross_edge=cross_edge+cross_wide;
                        cross_col=round(cross_edge);
                        right_edge[cross_row]=cross_col;
                        R_EDGE_NUM++;
                  }
            }
            else    //右边沿上下十字直角拐点未全部检测出
            {
                  if( R_down_cross[0]==-1 && R_up_cross[0]!=-1 )  //只有右边沿上十字直角拐点检测出
                  {
                        Site_xy1 xy1;
                        xy1=get_invinv_img(inv_distance[R_up_cross[0]]-120,0);
                        if( xy1.x<ROW_END )
                        {
                              W_num=0;
                              for( W_num_row=ROW_END ; W_num_row>=xy1.x ; W_num_row-- )
                              {
                                    if( right_edge[W_num_row]!=-1 && right_edge[W_num_row-1]!=-1 )
                                    {
                                          if( right_edge[W_num_row]>=right_edge[W_num_row-1] && right_edge[W_num_row]<=right_edge[W_num_row-1]+5 )
                                          {
                                                R_down_cross[0]=W_num_row-1;
                                                R_down_cross[1]=right_edge[W_num_row-1];
                                          }
                                          else
                                          {
                                                W_num++;
                                                if( W_num>=4 )
                                                {
                                                      break;
                                                }
                                          }
                                    }
                                    else
                                    {
                                          if( R_down_cross[0]!=-1 )
                                          {
                                                W_num++;
                                                if( W_num>=4 )
                                                {
                                                      break;
                                                }
                                          }
                                    }
                              }
                        }
                        
                        if( R_down_cross[0]==-1 )
                        {
                              int16 up_edge[2]={-1,-1};
                              for(  W_num_row=R_up_cross[0]-2 ; W_num_row>=R_up_cross[0]-4 ; W_num_row--  )  //选取合适的上下拐点
                              {
                                    if( W_num_row<ROW_START )
                                    {
                                          break;
                                    }
                                    if( right_edge[W_num_row]!=-1 )
                                    {
                                          up_edge[0]=W_num_row;
                                          
                                          up_edge[1]=right_edge[W_num_row];
                                          
                                          break;
                                    }
                              }
                              if( up_edge[0]==-1 )
                              {
                                    up_edge[0]=R_up_cross[0];
                                    up_edge[1]=R_up_cross[1];
                              }
                              
                              double x=0.0,y=0.0,x1,y1,k=0.0,a;
                              Site_xy xy1,xy2;
                              Site_xy1 xy3;
                              
                              xy1=get_inv_img( up_edge[0] , up_edge[1] );    //获取拐点逆透视坐标
                              
                              a=xy1.x+50;
                              xy3=get_invinv_img( a , 0 );
                              if( up_edge[0]-xy3.x>15 )
                              {
                                    W_num_row=15;
                              }
                              else
                              {
                                    if( up_edge[0]-xy3.x<3 )
                                    {
                                          W_num_row=3;
                                    }
                                    else
                                    {
                                          W_num_row=abs(up_edge[0]-xy3.x);
                                    }
                              }
                              
                              for( W_num=1 ; W_num<=W_num_row ; W_num++ )   //获取拐点向下的拟合斜率
                              {
                                    W_num_col=up_edge[0]-W_num;
                                    if( W_num_col<ROW_START )
                                    {
                                          break;
                                    }
                                    if( right_edge[W_num_col]!=-1 )
                                    {
                                          xy2=get_inv_img( W_num_col , right_edge[W_num_col] );
                                          x1=xy2.x-xy1.x;
                                          y1=xy2.y-xy1.y;
                                          x=x+x1*y1;
                                          y=y+x1*x1;
                                    }
                                    else
                                    {
                                          break;
                                    }
                              }
                              if( x!=0 )
                              {
                                    k=(-1)*x/y;    //得到拐点向下的拟合斜率
                              }
                              else
                              {
                                    k==0;
                              }
                              cross_wide=k;        //取向下拟合斜率
                              
                              cross_edge=up_edge[1];
                              for( cross_row=up_edge[0]+1 ; cross_row<=ROW_END ; cross_row++ )
                              {
                                    a=xy1.y+cross_wide*fabsf(inv_distance[cross_row]-xy1.x);
                                    xy3=get_invinv_img( inv_distance[cross_row] , a );
                                    if( xy3.y>COL_END )
                                    {
                                          break;
                                    }
                                    
                                    right_edge[cross_row]=xy3.y;
                                    R_EDGE_NUM++;
                              }
                        }
                        else
                        {
                              int16 up_edge[2]={-1,-1};
                              for(  W_num_row=R_up_cross[0]-2 ; W_num_row>=R_up_cross[0]-4 ; W_num_row--  )  //选取合适的上下拐点
                              {
                                    if( W_num_row<ROW_START )
                                    {
                                          break;
                                    }
                                    if( right_edge[W_num_row]!=-1 )
                                    {
                                          up_edge[0]=W_num_row;
                                          
                                          up_edge[1]=right_edge[W_num_row];
                                          
                                          break;
                                    }
                              }
                              if( up_edge[0]==-1 )
                              {
                                    up_edge[0]=R_up_cross[0];
                                    up_edge[1]=R_up_cross[1];
                              }
                              
                              cross_high=R_down_cross[0]-up_edge[0];
                              cross_wide=up_edge[1]-right_edge[R_down_cross[0]];
                              cross_wide=cross_wide/cross_high;
                              cross_edge=right_edge[R_down_cross[0]];
                              for( cross_row=R_down_cross[0]-1 ; cross_row>up_edge[0] ; cross_row-- )
                              {
                                    cross_edge=cross_edge+cross_wide;
                                    cross_col=round(cross_edge);
                                    right_edge[cross_row]=cross_col;
                                    R_EDGE_NUM++;
                              }
                        }
                  }
            }
      }
}

/***********************************************************************************************************************************/

void edge_error_process()           //此函数为边沿错误处理函数（修正普通赛道错误边沿，修正左右拐，识别S弯）
{
      /*障碍物处理*/
      if( L_barrier_down_row==-1 || L_barrier_up_row==-1 || fabsf(inv_distance[L_barrier_down_row]-inv_distance[L_barrier_up_row])>180 )
      {
            L_barrier_flag=0;
      }
      if( R_barrier_down_row==-1 || R_barrier_up_row==-1 || fabsf(inv_distance[R_barrier_down_row]-inv_distance[R_barrier_up_row])>180 )
      {
            R_barrier_flag=0;
      }
      if( L_TURN==1 && R_end_row!=-1 )
      {
            if( L_barrier_flag==1 )
            {
                  if( L_barrier_down_row!=-1 )
                  {
                        if( fabsf(inv_distance[L_barrier_down_row]-inv_distance[R_end_row])<400 )
                        {
                              L_barrier_flag=0;
                        }
                  }
            }
            if( R_barrier_flag==1 )
            {
                  if( R_barrier_down_row!=-1 )
                  {
                        if( fabsf(inv_distance[R_barrier_down_row]-inv_distance[R_end_row])<400 )
                        {
                              R_barrier_flag=0;
                        }
                  }
            }
      }
      else
      {
            if( R_TURN==1 && L_end_row!=-1 )
            {
                  if( L_barrier_flag==1 )
                  {
                        if( L_barrier_down_row!=-1 )
                        {
                              if( fabsf(inv_distance[L_barrier_down_row]-inv_distance[L_end_row])<400 )
                              {
                                    L_barrier_flag=0;
                              }
                        }
                  }
                  if( R_barrier_flag==1 )
                  {
                        if( R_barrier_down_row!=-1 )
                        {
                              if( fabsf(inv_distance[R_barrier_down_row]-inv_distance[L_end_row])<400 )
                              {
                                      R_barrier_flag=0;
                              }
                        }
                  }
            }
      }
      
      if( LOOP_TEMP==0 && L_barrier_flag==0 && R_barrier_flag==0 )
      {
            int16 row_1=-1,row_2=-1,x=-1;
            /*以下为处理左边沿程序*/
            for( W_num_row=ROW_END ; W_num_row>=ROW_START ; W_num_row-- )  //开始处连续边沿不处理
            {
                  if( left_edge[W_num_row]==-1 )
                  {
                        row_1=W_num_row;
                        break;
                  }
            }
            for( W_num_row=row_1 ; W_num_row>=ROW_START ; W_num_row-- )   //从第一个不连续边沿开始处理
            {
                  if( left_edge[W_num_row]!=-1 )  //找到边沿连续起始点
                  {
                        row_1=W_num_row;
                        row_2=W_num_row;
                        for( W_num_row=row_1-1 ; W_num_row>=ROW_START ; W_num_row-- )
                        {
                              if( W_num_row==ROW_START )
                              {
                                    if( row_2==row_1 )
                                    {
                                          row_2=ROW_START;
                                          break;
                                    }
                              }
                              else
                              {
                                    if( left_edge[W_num_row]==-1 )  //找到边沿连续结束点
                                    {
                                          row_2=W_num_row;
                                          break;
                                    }
                              }
                        }
                        if( row_1-row_2<=4 )   //当连续边沿太短，直接去除
                        {
                              for( x=row_1 ; x>row_2 ; x-- )
                              {
                                    left_edge[x]=-1;
                                    L_EDGE_NUM--;
                              }
                        }
                        else   //当连续边沿足够长，判断去除噪点
                        {
                              W_num=0;
                              for( x=row_1-6 ; x>row_1-10 ; x-- )
                              {
                                    if( x<=row_2 )
                                    {
                                          break;
                                    }
                                    if( abs(left_edge[x-1]-left_edge[x])<=2 )
                                    {
                                          W_num++;
                                    }
                              }
                              if( W_num>=3 )
                              {
                                    for( x=row_1 ; x>row_2 ; x-- )
                                    {
                                          if( abs(left_edge[x-1]-left_edge[x])>=4 )
                                          {
                                                left_edge[x]=-1;
                                                L_EDGE_NUM--;
                                          }
                                          else
                                          {
                                                break;
                                          }
                                    }
                              }
                        }
                  }
            }
            
            /*以下为处理右边沿程序*/
            for( W_num_row=ROW_END ; W_num_row>=ROW_START ; W_num_row-- )  //开始处连续边沿不处理
            {
                  if( right_edge[W_num_row]==-1 )
                  {
                        row_1=W_num_row;
                        break;
                  }
            }
            for( W_num_row=row_1 ; W_num_row>=ROW_START ; W_num_row-- )   //从第一个不连续边沿开始处理
            {
                  if( right_edge[W_num_row]!=-1 )  //找到边沿连续起始点
                  {
                        row_1=W_num_row;
                        row_2=W_num_row;
                        for( W_num_row=row_1-1 ; W_num_row>=ROW_START ; W_num_row-- )
                        {
                              if( W_num_row==ROW_START )
                              {
                                    if( row_2==row_1 )
                                    {
                                          row_2=ROW_START;
                                          break;
                                    }
                              }
                              else
                              {
                                    if( right_edge[W_num_row]==-1 )  //找到边沿连续结束点
                                    {
                                          row_2=W_num_row;
                                          break;
                                    }
                              }
                        }
                        if( row_1-row_2<=4 )   //当连续边沿太短，直接去除
                        {
                              for( x=row_1 ; x>row_2 ; x-- )
                              {
                                    right_edge[x]=-1;
                                    R_EDGE_NUM--;
                              }
                        }
                        else   //当连续边沿足够长，判断去除噪点
                        {
                              W_num=0;
                              for( x=row_1-6 ; x>row_1-10 ; x-- )
                              {
                                    if( x<=row_2 )
                                    {
                                          break;
                                    }
                                    if( abs(right_edge[x-1]-right_edge[x])<=2 )
                                    {
                                          W_num++;
                                    }
                              }
                              if( W_num>=3 )
                              {
                                    for( x=row_1 ; x>row_2 ; x-- )
                                    {
                                          if( abs(right_edge[x-1]-right_edge[x])>=4 )
                                          {
                                                right_edge[x]=-1;
                                                R_EDGE_NUM--;
                                          }
                                          else
                                          {
                                                break;
                                          }
                                    }
                              }
                        }
                  }
            }
            
            
            /*以下为左右转处理程序*/
            
            if( L_cross_flag==0 && R_cross_flag==0 && cross_flag==0 && up_flag==0 && down_flag==0 )   //未识别十字，障碍，坡道，处理S弯识别
            {
                  if( L_s_temp==1 || R_s_temp==1 )    //识别出S弯
                  {
                        if( L_s_temp==1 )  //左边沿识别出S弯
                        {
                              W_num=0;
                              for( EXTRACT_ROW=ROW_END ; EXTRACT_ROW>=ROW_START ; EXTRACT_ROW-- )      //搜索异常边沿
                              {
                                    if( left_edge[EXTRACT_ROW]!=-1 )
                                    {
                                          if( s_curve_row==-1 )
                                          {
                                                s_curve_row=EXTRACT_ROW;
                                                W_num=0;
                                          }
                                          else
                                          {
                                                if( left_edge[EXTRACT_ROW]>=left_edge[s_curve_row] )
                                                {
                                                      s_curve_row=EXTRACT_ROW;
                                                      W_num=0;
                                                }
                                                else
                                                {
                                                      W_num++;
                                                }
                                          }
                                    }
                                    if( W_num>=5 )
                                    {
                                          break;
                                    }
                              }
                              if( s_curve_row!=-1 )
                              {
                                    L_s_row=s_curve_row;
                              }
                        }
                        else
                        {
                              if( R_s_temp==1 )  //右边沿识别出S弯
                              {
                                    W_num=0;
                                    for( EXTRACT_ROW=ROW_END ; EXTRACT_ROW>=ROW_START ; EXTRACT_ROW-- )      //搜索异常边沿
                                    {
                                          if( right_edge[EXTRACT_ROW]!=-1 )
                                          {
                                                if( s_curve_row==-1 )
                                                {
                                                      s_curve_row=EXTRACT_ROW;
                                                      W_num=0;
                                                }
                                                else
                                                {
                                                      if( right_edge[EXTRACT_ROW]<=right_edge[s_curve_row] )
                                                      {
                                                            s_curve_row=EXTRACT_ROW;
                                                            W_num=0;
                                                      }
                                                      else
                                                      {
                                                            W_num++;
                                                      }
                                                }
                                          }
                                          if( W_num>=5 )
                                          {
                                                break;
                                          }
                                    }
                                    if( s_curve_row!=-1 )
                                    {
                                          R_s_row=s_curve_row;
                                    }
                              }
                        }
                  }
                  else    //未识别出S弯
                  {
                        if( L_TURN==1 && R_TURN==0 && R_end_row!=-1 )    //对左弯误判修正，识别S弯
                        {
                              W_num=0;
                              for( EXTRACT_ROW=ROW_END ; EXTRACT_ROW>=R_end_row ; EXTRACT_ROW-- )      //搜索异常边沿
                              {
                                    if( left_edge[EXTRACT_ROW]!=-1 )
                                    {
                                          if( s_curve_row==-1 )
                                          {
                                                s_curve_row=EXTRACT_ROW;
                                                W_num=0;
                                          }
                                          else
                                          {
                                                if( left_edge[EXTRACT_ROW]>=left_edge[s_curve_row] )
                                                {
                                                      s_curve_row=EXTRACT_ROW;
                                                      W_num=0;
                                                }
                                                else
                                                {
                                                      W_num++;
                                                }
                                          }
                                    }
                                    else
                                    {
                                          if( s_curve_row!=-1 )
                                          {
                                                W_num++;
                                          }
                                    }
                                    if( W_num>=5 )
                                    {
                                          break;
                                    }
                              }
                              if( s_curve_row!=-1 && left_edge[s_curve_row]>=80 )
                              {
                                    S_FLAG=1;
                                    L_s_temp=1;
                                    L_s_row=s_curve_row;
                              }
                        }
                        else
                        {
                              if( L_TURN==0 && R_TURN==1 && L_end_row!=-1 )    //对右弯误判修正，识别S弯
                              {
                                    W_num=0;
                                    for( EXTRACT_ROW=ROW_END ; EXTRACT_ROW>=L_end_row ; EXTRACT_ROW-- )      //搜索异常边沿
                                    {
                                          if( right_edge[EXTRACT_ROW]!=-1 )
                                          {
                                                if( s_curve_row==-1 )
                                                {
                                                      s_curve_row=EXTRACT_ROW;
                                                      W_num=0;
                                                }
                                                else
                                                {
                                                      if( right_edge[EXTRACT_ROW]<=right_edge[s_curve_row] )
                                                      {
                                                            s_curve_row=EXTRACT_ROW;
                                                            W_num=0;
                                                      }
                                                      else
                                                      {
                                                            W_num++;
                                                      }
                                                }
                                          }
                                          else
                                          {
                                                if( s_curve_row!=-1 )
                                                {
                                                      W_num++;
                                                }
                                          }
                                          if( W_num>=5 )
                                          {
                                                break;
                                          }
                                    }
                                    if( s_curve_row!=-1 && right_edge[s_curve_row]<=80 )
                                    {
                                          S_FLAG=1;
                                          R_s_temp=1;
                                          R_s_row=s_curve_row;
                                    }
                              }
                              else    //识别S弯
                              {
                                    int16 s_row=-1;
                                    if( L_EDGE_NUM>R_EDGE_NUM+5 )
                                    {
                                          W_num=0;
                                          for( EXTRACT_ROW=ROW_END ; EXTRACT_ROW>=ROW_START ; EXTRACT_ROW-- )      //搜索异常边沿
                                          {
                                                if( left_edge[EXTRACT_ROW]!=-1 )
                                                {
                                                      if( s_row==-1 )
                                                      {
                                                            s_row=EXTRACT_ROW;
                                                            W_num=0;
                                                      }
                                                      else
                                                      {
                                                            if( left_edge[EXTRACT_ROW]>=left_edge[s_row] )
                                                            {
                                                                  s_row=EXTRACT_ROW;
                                                                  W_num=0;
                                                            }
                                                            else
                                                            {
                                                                  W_num++;
                                                            }
                                                      }
                                                }
                                                if( W_num>=5 )
                                                {
                                                      s_curve_row=s_row;
                                                      break;
                                                }
                                          }
                                          if( s_curve_row!=-1 && left_edge[s_curve_row]>=80 )
                                          {
                                                S_FLAG=1;
                                                L_s_temp=1;
                                                L_s_row=s_curve_row;
                                          }
                                    }
                                    else
                                    {
                                          if( R_EDGE_NUM>L_EDGE_NUM+5 )
                                          {
                                                W_num=0;
                                                for( EXTRACT_ROW=ROW_END ; EXTRACT_ROW>=ROW_START ; EXTRACT_ROW-- )      //搜索异常边沿
                                                {
                                                      if( right_edge[EXTRACT_ROW]!=-1 )
                                                      {
                                                            if( s_row==-1 )
                                                            {
                                                                  s_row=EXTRACT_ROW;
                                                                  W_num=0;
                                                            }
                                                            else
                                                            {
                                                                  if( right_edge[EXTRACT_ROW]<=right_edge[s_row] )
                                                                  {
                                                                        s_row=EXTRACT_ROW;
                                                                        W_num=0;
                                                                  }
                                                                  else
                                                                  {
                                                                        W_num++;
                                                                  }
                                                            }
                                                      }
                                                      if( W_num>=5 )
                                                      {
                                                            s_curve_row=s_row;
                                                            break;
                                                      }
                                                }
                                                if( s_curve_row!=-1 && right_edge[s_curve_row]<=80 )
                                                {
                                                      S_FLAG=1;
                                                      R_s_temp=1;
                                                      R_s_row=s_curve_row;
                                                }
                                          }
                                    }
                              }
                        }
                  }
                  
                  if( L_s_temp==1 )   //左边沿识别出S弯
                  {
                        if( L_s_row!=-1 )
                        {
                              Site_xy s_xy;
                              s_xy=get_inv_img( L_s_row , left_edge[L_s_row] );
                              s_down_dis=s_xy.x;
                              if( s_xy.x<100 )
                              {
                                    L_s_temp=0;
                              }
                              L_s_row=-1;
                        }
                        else
                        {
                              L_s_temp=0;
                        }
                  }
                  else
                  {
                        if( R_s_temp==1 )   //右边沿识别出S弯
                        {
                              if( R_s_row!=-1 )
                              {
                                    Site_xy s_xy;
                                    s_xy=get_inv_img( R_s_row , right_edge[R_s_row] );
                                    s_down_dis=s_xy.x;
                                    if( s_xy.x<100 )
                                    {
                                          R_s_temp=0;
                                    }
                                    R_s_row=-1;
                              }
                              else
                              {
                                    R_s_temp=0;
                              }
                        }
                  }
                  
                  
                  if( L_TURN==0 && R_TURN==0 )
                  {
                        if( L_EDGE_NUM-R_EDGE_NUM>=15 )
                        {
                              R_TURN=1;
                        }
                        if( R_EDGE_NUM-L_EDGE_NUM>=15 )
                        {
                              L_TURN=1;
                        }
                  }
                  
                  if( L_s_temp==0 && R_s_temp==0 )    //搜索弯道前车，并清除错误边沿
                  {
                        if( R_TURN==1 )
                        {
                              if( L_end_row!=-1 )
                              {
                                    double xx;
                                    Site_xy1 yy;
                                    xx=inv_distance[L_end_row];
                                    xx=xx-curve_div;
                                    yy=get_invinv_img( xx , 0 );
                                    if( yy.x>59 )
                                    {
                                          yy.x=59;
                                    }
                                    
                                    for( EXTRACT_ROW=L_end_row ; EXTRACT_ROW<=yy.x ; EXTRACT_ROW++ )      //去除异常右边沿
                                    {
                                          if( right_edge[EXTRACT_ROW]!=-1 )
                                          {
                                                right_edge[EXTRACT_ROW]=-1;
                                                R_EDGE_NUM--;
                                          }
                                    }
                              }
                        }
                        else
                        {
                              if( L_TURN==1 )
                              {
                                    if( R_end_row!=-1 )
                                    {
                                          double xx;
                                          Site_xy1 yy;
                                          xx=inv_distance[R_end_row];
                                          xx=xx-curve_div;
                                          yy=get_invinv_img( xx , 0 );
                                          if( yy.x>59 )
                                          {
                                                yy.x=59;
                                          }
                                          
                                          for( EXTRACT_ROW=R_end_row ; EXTRACT_ROW<=yy.x ; EXTRACT_ROW++ )      //去除异常右边沿
                                          {
                                                if( left_edge[EXTRACT_ROW]!=-1 )
                                                {
                                                      left_edge[EXTRACT_ROW]=-1;
                                                      L_EDGE_NUM--;
                                                }
                                          }
                                    }
                              }
                        }
                  }
            }
      }
}

/***********************************************************************************************************************************/

void road_judge()                   //此函数为赛道类型判断函数（识别直道，坡道，出弯口，小S）
{
      if( end_line_row>=30 && END_LINE_FLAG==0 )     //判断终点线丢失
      {
            END_LINE_TEMP_2++;
            if( END_LINE_TEMP_2>=2 )
            {
                  end_line_row=-1;
                  END_LINE_TEMP_2=0;
            }
      }
      if( END_LINE_TEMP==1 )
      {
            if( abs(speedout_val)>end_line_val )
            {
                  END_LINE_TEMP=0;
                  end_line_row=-1;
                  end_line_num++;
                  Bee_flag=1;     //蜂鸣器
            }
      }
      if( RAMP_TEMP==1 )
      {
            Bee_flag=1;     //蜂鸣器
            if( abs(speedout_val)>ramp_val )
            {
                  RAMP_TEMP=0;
            }
      }
      
      if( LOOP_TEMP==0 && L_barrier_flag==0 && R_barrier_flag==0 )       //当不处于环形赛道和障碍
      {
            uint8 xx=0,yy=0;
            for( EXTRACT_ROW=straight_start ; EXTRACT_ROW>=straight_end ; EXTRACT_ROW-- )      //记录左右边沿个数
            {
                  if( left_edge[EXTRACT_ROW]!=-1 )
                  {
                        xx++;
                  }
                  else
                  {
                        xx=0;
                        break;
                  }
                  if( right_edge[EXTRACT_ROW]!=-1 )
                  {
                        yy++;
                  }
                  else
                  {
                        yy=0;
                        break;
                  }
            }
            if( xx!=0 && yy!=0 )    //当提取到大部分左右边沿，判断是否为直道
            {
                  W_num=0;
                  W_num_row=0;
                  W_num_col=0;
                  for( EXTRACT_ROW=straight_start ; EXTRACT_ROW>straight_end ; EXTRACT_ROW-- )      //记录左右边沿内倾数
                  {
                        if( left_edge[EXTRACT_ROW-1]-left_edge[EXTRACT_ROW]>=0 )
                        {
                              W_num_row++;
                        }
                        else
                        {
                              W_num++;
                        }
                        
                        if( right_edge[EXTRACT_ROW]-right_edge[EXTRACT_ROW-1]>=0 )
                        {
                              W_num_col++;
                        }
                        else
                        {
                              W_num++;
                        }
                  }
                  if( W_num_row>=straight_start-straight_end-3 && W_num_col>=straight_start-straight_end-3 && W_num<=3 )   //当左右边沿内倾数大于阀值，判断为直道
                  {
                        STRAIGHT_FLAG=1;    //直道标志置  1
                  }
            }
            if( L_cross_flag==0 && R_cross_flag==0 && cross_flag==0 && up_enable_flag==1 )   //判断是否为坡道
            {
                  uint8 xx=0,yy=0;
                  for( EXTRACT_ROW=ROW_END ; EXTRACT_ROW>=ROW_START ; EXTRACT_ROW-- )      //记录左右边沿个数
                  {
                        if( left_edge[EXTRACT_ROW]!=-1 )
                        {
                              xx++;
                        }
                        if( right_edge[EXTRACT_ROW]!=-1 )
                        {
                              yy++;
                        }
                  }
                  if( xx+yy>=90 )
                  {
                        W_num_row=0;
                        W_num_col=0;
                        
                        W_num=-1;
                        for( EXTRACT_ROW=ROW_END ; EXTRACT_ROW>=ROW_START+1 ; EXTRACT_ROW-- )      //记录左右边沿内倾数
                        {
                              if( left_edge[EXTRACT_ROW]!=-1 && left_edge[EXTRACT_ROW-1]!=-1 )
                              {
                                    W_num=EXTRACT_ROW;
                                    break;
                              }
                        }
                        if( W_num!=-1 )
                        {
                              for( EXTRACT_ROW=W_num ; EXTRACT_ROW>=ROW_START+1 ; EXTRACT_ROW-- )      //记录左右边沿内倾数
                              {
                                    if( left_edge[EXTRACT_ROW-1]!=-1 )
                                    {
                                          if( left_edge[EXTRACT_ROW-1]<left_edge[EXTRACT_ROW] )
                                          {
                                                W_num_row++;
                                          }
                                    }
                                    else
                                    {
                                          W_num_col=10;
                                          break;
                                    }
                              }
                        }
                        W_num=-1;
                        for( EXTRACT_ROW=ROW_END ; EXTRACT_ROW>=ROW_START+1 ; EXTRACT_ROW-- )      //记录左右边沿内倾数
                        {
                              if( right_edge[EXTRACT_ROW]!=-1 && right_edge[EXTRACT_ROW-1]!=-1 )
                              {
                                    W_num=EXTRACT_ROW;
                                    break;
                              }
                        }
                        if( W_num!=-1 )
                        {
                              for( EXTRACT_ROW=W_num ; EXTRACT_ROW>=ROW_START+1 ; EXTRACT_ROW-- )      //记录左右边沿内倾数
                              {
                                    if( right_edge[EXTRACT_ROW-1]!=-1 )
                                    {
                                          if( right_edge[EXTRACT_ROW-1]>right_edge[EXTRACT_ROW] )
                                          {
                                                W_num_col++;
                                          }
                                    }
                                    else
                                    {
                                          W_num_col=10;
                                          break;
                                    }
                              }
                        }
                        if( W_num_row<=3 && W_num_col<=3 )   //当识别出直道后，判断是否为坡道
                        {
                              int16 aa;
                              W_num=0;
                              
                              for( EXTRACT_ROW=ROW_START ; EXTRACT_ROW<=ROW_START+20 ; EXTRACT_ROW++ )      //搜索边沿，判断是否为坡道
                              {
                                    aa=2.0*bar_range_col[EXTRACT_ROW]+17-EXTRACT_ROW/4;
                                    if( abs(left_edge[EXTRACT_ROW]-right_edge[EXTRACT_ROW])>=aa )
                                    {
                                          W_num++;
                                    }
                              }
                              if( W_num>=7 )    //判断为坡道
                              {
                                    RAMP_TEMP=1;
                                    ramp_val=abs(speedout_val)+4000;
                              }
                        }
                  }
            }
            
            if( STRAIGHT_FLAG==0 )    //当不为直道，检测入弯口行
            {
                  if( L_TURN==1 )
                  {
                        W_num=0;
                        for( W_num_row=ROW_END ; W_num_row>=10 ; W_num_row-- )
                        {
                              if( left_edge[W_num_row]!=-1 )
                              {
                                    W_num=0;
                                    if( curve_in_row==-1 )
                                    {
                                          curve_in_row=W_num_row;
                                    }
                                    else
                                    {
                                          if( left_edge[W_num_row]>=left_edge[curve_in_row] )
                                          {
                                                curve_in_row=W_num_row;
                                          }
                                    }
                              }
                              else
                              {
                                    W_num++;
                              }
                              if( W_num>=10 )
                              {
                                    break;
                              }
                        }
                  }
                  else
                  {
                        if( R_TURN==1 )
                        {
                              W_num=0;
                              for( W_num_row=ROW_END ; W_num_row>=10 ; W_num_row-- )
                              {
                                    if( right_edge[W_num_row]!=-1 )
                                    {
                                          W_num=0;
                                          if( curve_in_row==-1 )
                                          {
                                                curve_in_row=W_num_row;
                                          }
                                          else
                                          {
                                                if( right_edge[W_num_row]<=right_edge[curve_in_row] )
                                                {
                                                      curve_in_row=W_num_row;
                                                }
                                          }
                                    }
                                    else
                                    {
                                          W_num++;
                                    }
                                    if( W_num>=10 )
                                    {
                                          break;
                                    }
                              }
                        }
                  }
            }
      }
}

/***********************************************************************************************************************************/

void bar_process()                  //此函数为障碍物边沿补充函数
{
      if( L_cross_flag==1 || R_cross_flag==1 || cross_flag==1 || up_flag==1 )   //判断出十字或坡道取消障碍物处理
      {
            L_barrier_flag=0;
            R_barrier_flag=0;
      }
      
      /*以下程序段为障碍物补边程序*/
      
      int16 up=-1,down=-1;  //当判断有障碍物
      
      if( L_barrier_flag==1 )
      {
            int16 x;
            x=(L_barrier_down_row+L_barrier_up_row)/2;
            for( W_num_row=L_barrier_down_row ; W_num_row>=L_barrier_up_row ; W_num_row-- )
            {
                  if( left_edge[W_num_row]!=-1 )
                  {
                        left_edge[W_num_row]=left_edge[W_num_row]+other4_e*bar_range_col[W_num_row];
                        if( left_edge[W_num_row]>COL_END )
                        {
                              left_edge[W_num_row]=COL_END;
                        }
                  }
            }
            
            if( L_barrier_down_row!=-1 )
            {
                  down=L_barrier_down_row;
                  for( W_num_row=L_barrier_down_row+5 ; W_num_row<=ROW_END-1 ; W_num_row++ )
                  {
                        if( left_edge[W_num_row]!=-1 )
                        {
                              down=W_num_row;
                              if( left_edge[W_num_row+1]==-1 || abs(left_edge[W_num_row]-left_edge[W_num_row+1])>=5 )
                              {
                                    break;
                              }
                        }
                  }
                  cross_high=down-L_barrier_down_row;
                  cross_wide=left_edge[down]-left_edge[L_barrier_down_row];
                  cross_wide=cross_wide/cross_high;   //为正值
                  cross_edge=left_edge[L_barrier_down_row];
                  for( cross_row=L_barrier_down_row+1 ; cross_row<down ; cross_row++ )
                  {
                        cross_edge=cross_edge+cross_wide;
                        cross_col=round(cross_edge);
                        left_edge[cross_row]=cross_col;
                  }
            }
            if( L_barrier_up_row!=-1 )
            {
                  up=L_barrier_up_row;
                  for( W_num_row=L_barrier_up_row-5 ; W_num_row>=ROW_START+1 ; W_num_row-- )
                  {
                        if( left_edge[W_num_row]!=-1 )
                        {
                              up=W_num_row;
                              if( left_edge[W_num_row-1]==-1 || abs(left_edge[W_num_row]-left_edge[W_num_row-1])>=5 )
                              {
                                    break;
                              }
                        }
                  }
                  cross_wide=0;
                  for( W_num_row=L_barrier_up_row ; W_num_row<=L_barrier_up_row+5 ; W_num_row++ )
                  {
                        if( left_edge[W_num_row]!=-1 && right_edge[W_num_row]!=-1 )
                        {
                              cross_wide=1.0-1.0*abs(left_edge[W_num_row]-left_edge[W_num_row])/(2.0*bar_range_col[W_num_row]);
                              break;
                        }
                  }
                  if( cross_wide!=0 && cross_wide>0 )
                  {
                        left_edge[up]=left_edge[up]+round(cross_wide*bar_range_col[up]);
                  }
                  
                  cross_high=up-L_barrier_up_row;
                  cross_wide=left_edge[up]-left_edge[L_barrier_up_row];
                  cross_wide=cross_wide/cross_high;   //为正值
                  cross_edge=left_edge[L_barrier_up_row];
                  for( cross_row=L_barrier_up_row-1 ; cross_row>up ; cross_row-- )
                  {
                        cross_edge=cross_edge-cross_wide;
                        cross_col=round(cross_edge);
                        left_edge[cross_row]=cross_col;
                  }
            }
      }
      else
      {
            if( R_barrier_flag==1 )
            {
                  int16 x;
                  x=(R_barrier_down_row+R_barrier_up_row)/2;
                  for( W_num_row=R_barrier_down_row ; W_num_row>=R_barrier_up_row ; W_num_row-- )
                  {
                        if( right_edge[W_num_row]!=-1 )
                        {
                              right_edge[W_num_row]=right_edge[W_num_row]-other4_f*bar_range_col[W_num_row];
                              if( right_edge[W_num_row]<COL_START )
                              {
                                    right_edge[W_num_row]=COL_START;
                              }
                        }
                  }
                  if( R_barrier_down_row!=-1 )
                  {
                        down=R_barrier_down_row;
                        for( W_num_row=R_barrier_down_row+5 ; W_num_row<=ROW_END-1 ; W_num_row++ )
                        {
                              if( right_edge[W_num_row]!=-1 )
                              {
                                    down=W_num_row;
                                    if( right_edge[W_num_row+1]==-1 || abs(right_edge[W_num_row]-right_edge[W_num_row+1])>=5 )
                                    {
                                          break;
                                    }
                              }
                        }
                        cross_high=down-R_barrier_down_row;
                        cross_wide=right_edge[down]-right_edge[R_barrier_down_row];
                        cross_wide=cross_wide/cross_high;   //为正值
                        cross_edge=right_edge[R_barrier_down_row];
                        for( cross_row=R_barrier_down_row+1 ; cross_row<down ; cross_row++ )
                        {
                              cross_edge=cross_edge+cross_wide;
                              cross_col=round(cross_edge);
                              right_edge[cross_row]=cross_col;
                        }
                  }
                  if( R_barrier_up_row!=-1 )
                  {
                        up=R_barrier_up_row;
                        for( W_num_row=R_barrier_up_row-5 ; W_num_row>=ROW_START+1 ; W_num_row-- )
                        {
                              if( right_edge[W_num_row]!=-1 )
                              {
                                    up=W_num_row;
                                    if( right_edge[W_num_row-1]==-1 || abs(right_edge[W_num_row]-right_edge[W_num_row-1])>=5 )
                                    {
                                          break;
                                    }
                              }
                        }
                        cross_wide=0;
                        for( W_num_row=R_barrier_up_row ; W_num_row<=R_barrier_up_row+5 ; W_num_row++ )
                        {
                              if( left_edge[W_num_row]!=-1 && right_edge[W_num_row]!=-1 )
                              {
                                    cross_wide=1.0-1.0*abs(left_edge[W_num_row]-left_edge[W_num_row])/(2.0*bar_range_col[W_num_row]);
                                    break;
                              }
                        }
                        if( cross_wide!=0 && cross_wide>0 )
                        {
                              right_edge[up]=right_edge[up]-round(cross_wide*bar_range_col[up]);
                        }
                        
                        cross_high=up-R_barrier_up_row;
                        cross_wide=right_edge[up]-right_edge[R_barrier_up_row];
                        cross_wide=cross_wide/cross_high;   //为正值
                        cross_edge=right_edge[R_barrier_up_row];
                        for( cross_row=R_barrier_up_row-1 ; cross_row>up ; cross_row-- )
                        {
                              cross_edge=cross_edge-cross_wide;
                              cross_col=round(cross_edge);
                              right_edge[cross_row]=cross_col;
                        }
                  }
            }
      }
}

/***********************************************************************************************************************************/

void road_num()                     //此函数为赛道类型计数函数（十字，障碍物个数）
{
      if( cross_temp==0 )     //十字记录个数  cross_num
      {
            if( L_down_cross[0]!=-1 || R_down_cross[0]!=-1 )
            {
                  cross_temp_1++;
            }
            else
            {
                  cross_temp_1=0;
            }
            if( cross_temp_1>=2 )
            {
                  cross_temp=1;
                  cross_temp_1=0;
                  
                  if( L_down_cross[0]!=-1 )
                  {
                        cross_down[0]=L_down_cross[0];
                  }
                  else
                  {
                        cross_down[0]=-1;
                  }
                  if( R_down_cross[0]!=-1 )
                  {
                        cross_down[1]=R_down_cross[0];
                  }
                  else
                  {
                        cross_down[1]=-1;
                  }
            }
      }
      else
      {
            if( cross_temp==1 )
            {
                  if( L_up_cross[0]!=-1 )
                  {
                        cross_temp_1++;
                        if( cross_temp_1>=2 )
                        {
                              cross_temp=2;
                              cross_temp_1=0;
                        }
                  }
                  else
                  {
                        if( R_up_cross[0]!=-1 )
                        {
                              cross_temp_1++;
                              if( cross_temp_1>=2 )
                              {
                                    cross_temp=2;
                                    cross_temp_1=0;
                              }
                        }
                        else
                        {
                              if( L_down_cross[0]==-1 && R_down_cross[0]==-1 )
                              {
                                    cross_temp_1++;
                                    if( cross_temp_1>=2 )
                                    {
                                          cross_temp=0;
                                          cross_temp_1=0;
                                    }
                              }
                        }
                  }
            }
            else
            {
                  if( cross_temp==2 )
                  {
                        if( L_up_cross[0]!=-1 && R_up_cross[0]!=-1 )
                        {
                              W_num=0;
                              W_num_col=0;
                              for( W_num_row=ROW_END ; W_num_row>=40 ; W_num_row-- )      //记录左右边沿个数
                              {
                                    if( left_edge[W_num_row]!=-1 )
                                    {
                                          W_num++;
                                    }
                                    if( right_edge[W_num_row]!=-1 )
                                    {
                                          W_num_col++;
                                    }
                              }
                              if( W_num>=15 && W_num_col>=15 )
                              {
                                    cross_temp_1++;
                              }
                              if( cross_temp_1>=2 )
                              {
                                    cross_num++;
                                    cross_temp=3;
                                    cross_temp_1=0;
                                    //Bee_flag=1;     //蜂鸣器
                              }
                        }
                        else
                        {
                              if( L_up_cross[0]==-1 && R_up_cross[0]==-1 )
                              {
                                    cross_temp_1++;
                                    cross_temp=0;
                                    cross_temp_1=0;
                              }
                        }
                  }
                  else
                  {
                        if( cross_temp==3 )
                        {
                              if( inv_distance[L_up_cross[0]]<80 )
                              {
                                    cross_temp_1++;
                              }
                              if( inv_distance[R_up_cross[0]]<80 )
                              {
                                    cross_temp_1++;
                              }
                              if( cross_temp_1>=2 )
                              {
                                    cross_temp=4;
                                    cross_temp_1=0;
                              }
                        }
                        else
                        {
                              if( cross_temp==4 )
                              {
                                    if( L_up_cross[0]==-1 && R_up_cross[0]==-1 )
                                    {
                                          cross_temp_1++;
                                    }
                                    if( cross_temp_1>=3 )
                                    {
                                          cross_temp=0;
                                          cross_temp_1=0;
                                          //Bee_flag=1;     //蜂鸣器
                                    }
                              }
                        }
                  }
            }
      }
      
      if( bar_temp==0 )     //障碍物记录个数  bar_num
      {
            if( L_barrier_flag==1 )
            {
                  bar_temp_1++;
                  if( bar_temp_1>=3 )
                  {
                        bar_temp=1;
                        bar_row[0]=1;   //表示左边障碍物
                        bar_row[1]=L_barrier_down_row;
                        bar_temp_1=0;
                  }
            }
            else
            {
                  bar_temp_1=0;
            }
            
            if( R_barrier_flag==1 )
            {
                  bar_temp_2++;
                  if( bar_temp_2>=3 )
                  {
                        bar_temp=1;
                        bar_row[0]=2;   //表示右边障碍物
                        bar_row[1]=R_barrier_down_row;
                        bar_temp_2=0;
                  }
            }
            else
            {
                  bar_temp_2=0;
            }
      }
      else
      {
            if( bar_temp==1 )
            {
                  if( bar_row[0]==1 )
                  {
                        if( L_barrier_flag==1 )
                        {
                              if( inv_distance[bar_row[1]]<80 )
                              {
                                    bar_temp=2;
                                    L_bar_num++;
                                    //Bee_flag=1;     //蜂鸣器
                              }
                              else
                              {
                                    if( L_barrier_down_row>bar_row[1] )
                                    {
                                          bar_row[1]=L_barrier_down_row;
                                    }
                              }
                        }
                        else
                        {
                              bar_temp_1++;
                              if( bar_temp_1>=2 )
                              {
                                    bar_temp=0;
                                    bar_temp_1=0;
                              }
                        }
                  }
                  else
                  {
                        if( bar_row[0]==2 )
                        {
                              if( R_barrier_flag==1 )
                              {
                                    if( inv_distance[bar_row[1]]<80 )
                                    {
                                          bar_temp=2;
                                          R_bar_num++;
                                          //Bee_flag=1;     //蜂鸣器
                                    }
                                    else
                                    {
                                          if( R_barrier_down_row>bar_row[1] )
                                          {
                                                bar_row[1]=R_barrier_down_row;
                                          }
                                    }
                              }
                              else
                              {
                                    bar_temp_2++;
                                    if( bar_temp_2>=2 )
                                    {
                                          bar_temp=0;
                                          bar_temp_2=0;
                                    }
                              }
                        }
                  }
            }
            else
            {
                  if( bar_temp==2 )
                  {
                        if( bar_row[0]==1 )
                        {
                              if( L_barrier_flag==0 )
                              {
                                    bar_temp_1++;
                                    if( bar_temp_1>=3 )
                                    {
                                          bar_temp=0;
                                          bar_temp_1=0;
                                          //Bee_flag=1;     //蜂鸣器
                                    }
                              }
                        }
                        else
                        {
                              if( bar_row[0]==2 )
                              {
                                    if( R_barrier_flag==0 )
                                    {
                                          bar_temp_2++;
                                          if( bar_temp_2>=3 )
                                          {
                                                bar_temp=0;
                                                bar_temp_2=0;
                                                //Bee_flag=1;     //蜂鸣器
                                          }
                                    }
                              }
                        }
                  }
            }
      }
}

/***********************************************************************************************************************************/

void loop_judge()                   //此函数为环型赛道判断函数
{
      if( LOOP_TEMP==0 )       //当未处于环形赛道
      {
            if( LOOP_FLAG==1 )
            {
                  LOOP_TEMP_1++;
                  //Bee_flag=1;     //蜂鸣器
            }
            else
            {
                  LOOP_TEMP_1=0;
            }
            if( LOOP_TEMP_1>=1 )   //连续判断出环道，判断为环道
            {
                  LOOP_TEMP=1;
                  Bee_flag=1;     //蜂鸣器
                  loop_val=25*inv_distance[LOOP_IN_ROW]+300+abs(speedout_val);
                  loop_num++;
                  LOOP_TEMP_1=0;
                  LOOP_IN=0;
                  LOOP_OUT=0;
                  out_row[0]=-1;
                  out_row[1]=-1;
                  loop_l_r_flag=loop_LR2_flag;
            }
      }
      else       //当处于环形赛道
      {
            if( LOOP_IN==0 )   //当未进入环形赛道时
            {
                  if( LOOP_FLAG==1 )
                  {
                        int32 l_val;
                        
                        l_val=25*inv_distance[LOOP_IN_ROW]+300+abs(speedout_val);
                        if( l_val<loop_val+1000 )
                        {
                              loop_val=l_val;
                        }
                        else
                        {
                              double l_in_row;
                              Site_xy1 xy1;
                              l_in_row=1.0*(loop_val-abs(speedout_val)-300)/25;
                              xy1=get_invinv_img(l_in_row,0);
                              LOOP_IN_ROW=xy1.x;
                              if( LOOP_IN_ROW>59 )
                              {
                                    LOOP_IN_ROW=59;
                              }
                        }
                  }
                  else
                  {
                        double l_in_row;
                        Site_xy1 xy1;
                        l_in_row=1.0*(loop_val-abs(speedout_val)-300)/25;
                        xy1=get_invinv_img(l_in_row,0);
                        LOOP_IN_ROW=xy1.x;
                        if( LOOP_IN_ROW>59 )
                        {
                              LOOP_IN_ROW=59;
                        }
                  }
                  //angle_aaa[0]=loop_val;
                  if( abs(speedout_val)>loop_val )   //经过一段距离，判断为进入环道
                  {
                        LOOP_IN=1;
                        Bee_flag=1;     //蜂鸣器
                        LOOP_TEMP_1=0;
                        LOOP_IN_1=0;
                        LOOP_OUT_1=0;
                        loop_val=0;
                  }
                  
                  if( loop_l_r_flag==1 )    //环道边沿修正
                  {
                        int16 row=-1;
                        if( LOOP_IN_ROW<=55 )
                        {
                              row=LOOP_IN_ROW-5;
                        }
                        else
                        {
                              row=50;
                        }
                        W_num_col=-1;
                        
                        for( W_num_row=row ; W_num_row<=ROW_END ; W_num_row++ )
                        {
                              if( left_edge[W_num_row]!=-1 )
                              {
                                    if( W_num_col==-1 )
                                    {
                                          W_num_col=W_num_row;
                                    }
                                    else
                                    {
                                          if( left_edge[W_num_row]>=left_edge[W_num_col] )
                                          {
                                                W_num_col=W_num_row;
                                          }
                                    }
                              }
                        }
                        if( W_num_col==-1 )
                        {
                              for( W_num_row=LOOP_IN_ROW ; W_num_row>=ROW_START ; W_num_row-- )
                              {
                                    if( left_edge[W_num_row]!=-1 )
                                    {
                                          left_edge[W_num_row]=-1;
                                          L_EDGE_NUM--;
                                    }
                              }
                        }
                        else
                        {
                              row=-1;
                              W_num=0;
                              for( W_num_row=W_num_col-1 ; W_num_row>=ROW_START ; W_num_row-- )
                              {
                                    if( left_edge[W_num_row]==-1 )
                                    {
                                          W_num++;
                                          if( W_num>=2 )
                                          {
                                                row=W_num_row;
                                                break;
                                          }
                                    }
                                    else
                                    {
                                          if( left_edge[W_num_row]>=left_edge[W_num_col] )
                                          {
                                                W_num_col=W_num_row;
                                          }
                                    }
                              }
                              if( W_num>=2 )
                              {
                                    for( W_num_row=row-1 ; W_num_row>=ROW_START ; W_num_row-- )
                                    {
                                          if( left_edge[W_num_row]!=-1 && left_edge[W_num_row]>=left_edge[W_num_col] )
                                          {
                                                left_edge[W_num_row]=-1;
                                                L_EDGE_NUM--;
                                          }
                                    }
                              }
                        }
                  }
                  else
                  {
                        if( loop_l_r_flag==0 )    //环道边沿修正
                        {
                              int16 row=-1;
                              if( LOOP_IN_ROW<=55 )
                              {
                                    row=LOOP_IN_ROW-5;
                              }
                              else
                              {
                                    row=50;
                              }
                              W_num_col=-1;
                              
                              for( W_num_row=row ; W_num_row<=ROW_END ; W_num_row++ )
                              {
                                    if( right_edge[W_num_row]!=-1 )
                                    {
                                          if( W_num_col==-1 )
                                          {
                                                W_num_col=W_num_row;
                                          }
                                          else
                                          {
                                                if( right_edge[W_num_row]<=right_edge[W_num_col] )
                                                {
                                                      W_num_col=W_num_row;
                                                }
                                          }
                                    }
                              }
                              if( W_num_col==-1 )
                              {
                                    for( W_num_row=LOOP_IN_ROW ; W_num_row>=ROW_START ; W_num_row-- )
                                    {
                                          if( right_edge[W_num_row]!=-1 )
                                          {
                                                right_edge[W_num_row]=-1;
                                                R_EDGE_NUM--;
                                          }
                                    }
                              }
                              else
                              {
                                    row=-1;
                                    W_num=0;
                                    for( W_num_row=W_num_col-1 ; W_num_row>=ROW_START ; W_num_row-- )
                                    {
                                          if( right_edge[W_num_row]==-1 )
                                          {
                                                W_num++;
                                                if( W_num>=2 )
                                                {
                                                      row=W_num_row;
                                                      break;
                                                }
                                          }
                                          else
                                          {
                                                if( right_edge[W_num_row]<=right_edge[W_num_col] )
                                                {
                                                      W_num_col=W_num_row;
                                                }
                                          }
                                    }
                                    if( W_num>=2 )
                                    {
                                          for( W_num_row=row-1 ; W_num_row>=ROW_START ; W_num_row-- )
                                          {
                                                if( right_edge[W_num_row]!=-1 && right_edge[W_num_row]<=right_edge[W_num_col] )
                                                {
                                                      right_edge[W_num_row]=-1;
                                                      R_EDGE_NUM--;
                                                }
                                          }
                                    }
                              }
                        }
                  }
            }
            else   //当进入环形赛道时
            {
                  if( out_row[0]==-1 )  //当刚刚进入环形赛道
                  {
                        int16 out_start;
                        Site_xy1 xy1;
                        xy1=get_invinv_img(80,0);
                        out_start=xy1.x;
                        angle_aaa[0]=0;
                        angle_aaa[1]=0;
                        angle_aaa[2]=0;
                        if( loop_l_r_flag==1 ) //当判断出环道向左转
                        {
                              W_num_col=-1;
                              for( W_num_row=ROW_END ; W_num_row>=loop_out_end+3 ; W_num_row-- )
                              {
                                    if( left_edge[W_num_row]!=-1 && left_edge[W_num_row-1]!=-1 && left_edge[W_num_row-2]!=-1 )
                                    {
                                          W_num_col=W_num_row;
                                          break;
                                    }
                              }
                              if( W_num_col!=-1 )
                              {
                                    W_num=0;
                                    for( W_num_row=W_num_col-1 ; W_num_row>=W_num_col-10 ; W_num_row-- )
                                    {
                                          if( left_edge[W_num_row]!=-1 && left_edge[W_num_row]>=left_edge[W_num_col] && left_edge[W_num_row]<left_edge[W_num_col]+30 )
                                          {
                                                W_num++;
                                          }
                                    }
                                    if( W_num>=5 )
                                    {
                                          LOOP_OUT_ROW=W_num_col;
                                          W_num=0;
                                          for( W_num_row=W_num_col ; W_num_row>=loop_out_end ; W_num_row-- )
                                          {
                                                if( left_edge[W_num_row]!=-1 )
                                                {
                                                      if( left_edge[W_num_row]>=left_edge[LOOP_OUT_ROW] )
                                                      {
                                                            W_num=0;
                                                            LOOP_OUT_ROW=W_num_row;
                                                      }
                                                      else
                                                      {
                                                            W_num++;
                                                            if( W_num>=6 )
                                                            {
                                                                  break;
                                                            }
                                                      }
                                                }
                                                else
                                                {
                                                      W_num++;
                                                      if( W_num>=3 )
                                                      {
                                                            break;
                                                      }
                                                }
                                          }
                                          
                                          if( LOOP_OUT_ROW<out_start )
                                          {
                                                W_num=0;
                                                W_num_col=0;
                                                for( W_num_row=LOOP_OUT_ROW-1 ; W_num_row>=ROW_START ; W_num_row-- )
                                                {
                                                      if( left_edge[W_num_row]!=-1 )
                                                      {
                                                            if( left_edge[W_num_row]<=left_edge[W_num_row+1] )
                                                            {
                                                                  W_num++;
                                                                  if( W_num>=5 )
                                                                  {
                                                                        break;
                                                                  }
                                                            }
                                                      }
                                                      else
                                                      {
                                                            break;
                                                      }
                                                }
                                                for( W_num_row=LOOP_OUT_ROW+1 ; W_num_row<=ROW_END ; W_num_row++ )
                                                {
                                                      if( left_edge[W_num_row]!=-1 )
                                                      {
                                                            if( left_edge[W_num_row]<=left_edge[W_num_row-1] )
                                                            {
                                                                  W_num_col++;
                                                                  if( W_num_col>=8 )
                                                                  {
                                                                        break;
                                                                  }
                                                            }
                                                      }
                                                      else
                                                      {
                                                            break;
                                                      }
                                                }
                                                
                                                if( W_num>=5 && W_num_col>=8 )
                                                {
                                                      out_row[0]=LOOP_OUT_ROW;
                                                      out_row[1]=left_edge[LOOP_OUT_ROW];
                                                      
                                                      Site_xy xy;
                                                      xy=get_inv_img(out_row[0],out_row[1]);
                                                      loop_val=25*sqrt(fabsf(xy.x*xy.x)+fabsf(xy.y*xy.y))+loop_out_l+abs(speedout_val);
                                                }
                                          }
                                    }
                              }
                        }
                        else //当判断出环道向右转
                        {
                              W_num_col=-1;
                              for( W_num_row=ROW_END ; W_num_row>=loop_out_end+3 ; W_num_row-- )
                              {
                                    if( right_edge[W_num_row]!=-1 && right_edge[W_num_row-1]!=-1 && right_edge[W_num_row-2]!=-1 )
                                    {
                                          W_num_col=W_num_row;
                                          break;
                                    }
                              }
                              if( W_num_col!=-1 )
                              {
                                    W_num=0;
                                    for( W_num_row=W_num_col-1 ; W_num_row>=W_num_col-10 ; W_num_row-- )
                                    {
                                          if( right_edge[W_num_row]!=-1 && right_edge[W_num_row]<=right_edge[W_num_col] && right_edge[W_num_row]>right_edge[W_num_col]-30 )
                                          {
                                                W_num++;
                                          }
                                    }
                                    if( W_num>=5 )
                                    {
                                          LOOP_OUT_ROW=W_num_col;
                                          W_num=0;
                                          for( W_num_row=W_num_col ; W_num_row>=loop_out_end ; W_num_row-- )
                                          {
                                                if( right_edge[W_num_row]!=-1 )
                                                {
                                                      if( right_edge[W_num_row]<=right_edge[LOOP_OUT_ROW] )
                                                      {
                                                            W_num=0;
                                                            LOOP_OUT_ROW=W_num_row;
                                                      }
                                                      else
                                                      {
                                                            W_num++;
                                                            if( W_num>=6 )
                                                            {
                                                                  break;
                                                            }
                                                      }
                                                }
                                                else
                                                {
                                                      W_num++;
                                                      if( W_num>=3 )
                                                      {
                                                            break;
                                                      }
                                                }
                                          }
                                          angle_aaa[2]=LOOP_OUT_ROW;
                                          if( LOOP_OUT_ROW<out_start )
                                          {
                                                W_num=0;
                                                W_num_col=0;
                                                for( W_num_row=LOOP_OUT_ROW-1 ; W_num_row>=ROW_START ; W_num_row-- )
                                                {
                                                      if( right_edge[W_num_row]!=-1 )
                                                      {
                                                            if( right_edge[W_num_row]>=right_edge[W_num_row+1] )
                                                            {
                                                                  W_num++;
                                                                  if( W_num>=5 )
                                                                  {
                                                                        break;
                                                                  }
                                                            }
                                                      }
                                                      else
                                                      {
                                                            break;
                                                      }
                                                }
                                                for( W_num_row=LOOP_OUT_ROW+1 ; W_num_row<=ROW_END ; W_num_row++ )
                                                {
                                                      if( right_edge[W_num_row]!=-1 )
                                                      {
                                                            if( right_edge[W_num_row]>=right_edge[W_num_row-1] )
                                                            {
                                                                  W_num_col++;
                                                                  if( W_num_col>=8 )
                                                                  {
                                                                        break;
                                                                  }
                                                            }
                                                      }
                                                      else
                                                      {
                                                            break;
                                                      }
                                                }
                                                angle_aaa[0]=W_num;
                                                angle_aaa[1]=W_num_col;
                                                if( W_num>=5 && W_num_col>=8 )
                                                {
                                                      out_row[0]=LOOP_OUT_ROW;
                                                      out_row[1]=right_edge[LOOP_OUT_ROW];
                                                      
                                                      Site_xy xy;
                                                      xy=get_inv_img(out_row[0],out_row[1]);
                                                      loop_val=25*sqrt(fabsf(xy.x*xy.x)+fabsf(xy.y*xy.y))+loop_out_l+abs(speedout_val);
                                                }
                                          }
                                    }
                              }
                        }
                  }
                  else  //当进入环形赛道一段时间
                  {
                        if( out_row[0]+30>ROW_END-4 )
                        {
                              LOOP_IN_1=ROW_END-4;
                        }
                        else
                        {
                              LOOP_IN_1=out_row[0]+30;
                        }
                        LOOP_OUT_ROW=-1;
                        
                        if( inv_distance[out_row[0]]<=120 )   //出环口小于30厘米时
                        {
                              if( loop_l_r_flag==1 ) //当判断出环道向左转
                              {
                                    for( W_num_row=LOOP_IN_1 ; W_num_row>=loop_out_end ; W_num_row-- )
                                    {
                                          if( W_num_row<ROW_START+4)
                                          {
                                                break;
                                          }
                                          if( left_edge[W_num_row]!=-1 )
                                          {
                                                if( left_edge[W_num_row+4]!=-1 && left_edge[W_num_row+2]!=-1 )
                                                {
                                                      if( left_edge[W_num_row]>left_edge[W_num_row+4] && left_edge[W_num_row]>left_edge[W_num_row+2] )
                                                      {
                                                            if( left_edge[W_num_row-2]!=-1 )
                                                            {
                                                                  if( left_edge[W_num_row]>left_edge[W_num_row-2] )
                                                                  {
                                                                        if( left_edge[W_num_row-4]!=-1 )
                                                                        {
                                                                              if( left_edge[W_num_row]>left_edge[W_num_row-4] )
                                                                              {
                                                                                    LOOP_OUT_ROW=W_num_row;
                                                                                    break;
                                                                              }
                                                                        }
                                                                        else
                                                                        {
                                                                              if( left_edge[W_num_row]<=60 )
                                                                              {
                                                                                    LOOP_OUT_ROW=W_num_row;
                                                                                    break;
                                                                              }
                                                                        }
                                                                  }
                                                            }
                                                            else
                                                            {
                                                                  if(  left_edge[W_num_row-4]==-1 && left_edge[W_num_row]<=60 )
                                                                  {
                                                                        LOOP_OUT_ROW=W_num_row;
                                                                        break;
                                                                  }
                                                            }
                                                      }
                                                }
                                          }
                                    }
                              }
                              else //当判断出环道向右转
                              {
                                    for( W_num_row=LOOP_IN_1 ; W_num_row>=loop_out_end ; W_num_row-- )
                                    {
                                          if( W_num_row<ROW_START+4)
                                          {
                                                break;
                                          }
                                          if( right_edge[W_num_row]!=-1 )
                                          {
                                                if( right_edge[W_num_row+4]!=-1 && right_edge[W_num_row+2]!=-1 )
                                                {
                                                      if( right_edge[W_num_row]<right_edge[W_num_row+4] && right_edge[W_num_row]<right_edge[W_num_row+2] )
                                                      {
                                                            if( right_edge[W_num_row-2]!=-1 )
                                                            {
                                                                  if( right_edge[W_num_row]<right_edge[W_num_row-2] )
                                                                  {
                                                                        if( right_edge[W_num_row-4]!=-1 )
                                                                        {
                                                                              if( right_edge[W_num_row]<right_edge[W_num_row-4] )
                                                                              {
                                                                                    LOOP_OUT_ROW=W_num_row;
                                                                                    break;
                                                                              }
                                                                        }
                                                                        else
                                                                        {
                                                                              if( right_edge[W_num_row]>=100 )
                                                                              {
                                                                                    LOOP_OUT_ROW=W_num_row;
                                                                                    break;
                                                                              }
                                                                        }
                                                                  }
                                                            }
                                                            else
                                                            {
                                                                  if(  right_edge[W_num_row-4]==-1 && right_edge[W_num_row]>=100 )
                                                                  {
                                                                        LOOP_OUT_ROW=W_num_row;
                                                                        break;
                                                                  }
                                                            }
                                                      }
                                                }
                                          }
                                    }
                              }
                        }
                        else
                        {
                              if( loop_l_r_flag==1 ) //当判断出环道向左转
                              {
                                    for( W_num_row=LOOP_IN_1 ; W_num_row>=loop_out_end ; W_num_row-- )
                                    {
                                          if( W_num_row<ROW_START+4)
                                          {
                                                break;
                                          }
                                          if( left_edge[W_num_row]!=-1 )
                                          {
                                                if( left_edge[W_num_row+4]!=-1 && left_edge[W_num_row+2]!=-1 && left_edge[W_num_row-2]!=-1 && left_edge[W_num_row-4]!=-1 )
                                                {
                                                      if( left_edge[W_num_row]>left_edge[W_num_row+4] && left_edge[W_num_row]>left_edge[W_num_row+2] && left_edge[W_num_row]>left_edge[W_num_row-2] && left_edge[W_num_row]>left_edge[W_num_row-4] )
                                                      {
                                                            LOOP_OUT_ROW=W_num_row;
                                                            break;
                                                      }
                                                }
                                          }
                                    }
                              }
                              else //当判断出环道向右转
                              {
                                    for( W_num_row=LOOP_IN_1 ; W_num_row>=loop_out_end ; W_num_row-- )
                                    {
                                          if( W_num_row<ROW_START+4)
                                          {
                                                break;
                                          }
                                          if( right_edge[W_num_row]!=-1 )
                                          {
                                                if( right_edge[W_num_row+4]!=-1 && right_edge[W_num_row+2]!=-1 && right_edge[W_num_row-2]!=-1 && right_edge[W_num_row-4]!=-1 )
                                                {
                                                      if( right_edge[W_num_row]<right_edge[W_num_row+4] && right_edge[W_num_row]<right_edge[W_num_row+2] && right_edge[W_num_row]<right_edge[W_num_row-2] && right_edge[W_num_row]<right_edge[W_num_row-4] )
                                                      {
                                                            LOOP_OUT_ROW=W_num_row;
                                                            break;
                                                      }
                                                }
                                          }
                                    }
                              }
                        }
                        
                        
                        if( LOOP_OUT_ROW==-1 )   //当本次搜索未搜到出环行
                        {
                              LOOP_TEMP_1++;
                              if( LOOP_TEMP_1>=4 )
                              {
                                    if( out_row[0]<30 )
                                    {
                                          out_row[0]=-1;
                                          out_row[1]=-1;
                                    }
                                    else
                                    {
                                          if( abs(speedout_val)>loop_val )
                                          {
                                                LOOP_TEMP=0;   //当搜索到赛道正常，判断为出环道
                                                LOOP_IN=0;     //入环标志置  0
                                                out_row[0]=-1;
                                                out_row[1]=-1;
                                                Bee_flag=1;     //蜂鸣器
                                          }
                                    }
                              }
                              else
                              {
                                    if( abs(speedout_val)>loop_val )
                                    {
                                          LOOP_TEMP=0;   //当搜索到赛道正常，判断为出环道
                                          LOOP_IN=0;     //入环标志置  0
                                          out_row[0]=-1;
                                          out_row[1]=-1;
                                          Bee_flag=1;     //蜂鸣器
                                    }
                              }
                        }
                        else   //当本次搜索搜到出环行
                        {
                              LOOP_TEMP_1=0;
                              
                              Site_xy xy;
                              int32 l_val,val_add;
                              
                              l_val=loop_val-loop_out_l-abs(speedout_val);
                              if( l_val<3000 )
                              {
                                    val_add=1000;
                              }
                              else
                              {
                                    if( l_val>10000 )
                                    {
                                          val_add=4000;
                                    }
                                    else
                                    {
                                          val_add=1000+3*(l_val-3000)/7;
                                    }
                              }
                              if( loop_l_r_flag==1 )
                              {
                                    xy=get_inv_img(LOOP_OUT_ROW,left_edge[LOOP_OUT_ROW]);
                                    
                                    l_val=25*sqrt(fabsf(xy.x*xy.x)+fabsf(xy.y*xy.y))+loop_out_l+abs(speedout_val);
                                    
                                    if( l_val-loop_val<val_add )   //当搜索到的出环行正常，存储本次数据
                                    {
                                          loop_val=l_val;
                                          out_row[0]=LOOP_OUT_ROW;
                                          out_row[1]=left_edge[LOOP_OUT_ROW];
                                          //Bee_flag=1;     //蜂鸣器
                                    }
                              }
                              else
                              {
                                    xy=get_inv_img(LOOP_OUT_ROW,right_edge[LOOP_OUT_ROW]);
                                    
                                    l_val=25*sqrt(fabsf(xy.x*xy.x)+fabsf(xy.y*xy.y))+loop_out_l+abs(speedout_val);
                                    if( l_val-loop_val<val_add )   //当搜索到的出环行正常，存储本次数据
                                    {
                                          loop_val=l_val;
                                          out_row[0]=LOOP_OUT_ROW;
                                          out_row[1]=right_edge[LOOP_OUT_ROW];
                                    }
                              }
                              
                              if( abs(speedout_val)>loop_val )
                              {
                                    LOOP_TEMP=0;   //当搜索到赛道正常，判断为出环道
                                    LOOP_IN=0;     //入环标志置  0
                                    out_row[0]=-1;
                                    out_row[1]=-1;
                                    Bee_flag=1;     //蜂鸣器
                              }
                        }
                  }
            }
      }
}

/***********************************************************************************************************************************/

void down_extract()                 //此函数为下降边沿搜索函数
{
      L_LINK_NUM=0;        //左边沿下降边沿个数清零
      R_LINK_NUM=0;        //右边沿下降边沿个数清零
      if( L_search_end==1 || R_search_end==1 )
      {
            if( L_TURN==1 && R_end_row!=-1 )    //当图像为左弯，并且提取出右边沿结束行，右边沿向下搜索
            {
                  R_edge_nearest=right_edge[R_end_row]-1;    //设置起始搜索位置
                  for( EXTRACT_ROW=R_end_row ; EXTRACT_ROW<=R_end_row+9 ; EXTRACT_ROW++ )
                  {
                        if( EXTRACT_ROW>ROW_END )
                        {
                              break;
                        }
                        
                        R_get_flag=0;                        //对右边沿搜索得到标志清0
                        if( img_data[EXTRACT_ROW][R_edge_nearest]==DOT_B )      //当最近列的下一点为黑时，停止搜索
                        {
                              break;
                        }
                        else      //当最近列的下一点不为黑时，开始搜索
                        {
                              for( EXTRACT_COL=R_edge_nearest-1 ; EXTRACT_COL>=COL_START ; EXTRACT_COL-- )
                              {
                                    if( img_data[EXTRACT_ROW][EXTRACT_COL]==DOT_B )       //搜索找到本行黑点
                                    {
                                          right_down_edge[R_LINK_NUM]=EXTRACT_COL;        //存储本行边沿点
                                          
                                          R_LINK_NUM++;                                   //边沿点个数加一
                                          
                                          R_edge_nearest=EXTRACT_COL;                     //存储最近一个右边沿的列坐标
                                          
                                          R_get_flag=1;                                   //本行找到下降边沿点
                                          
                                          break;                                         //结束本行搜索
                                    }
                              }
                        }
                        if( R_get_flag==0 )      //如果本行未找到边沿点，停止搜索
                        {
                              break;
                        }
                  }
                  
                  if( L_end_row!=-1 )    //当图像为左弯，并且提取出左边沿结束行，左边沿向下搜索
                  {
                        L_edge_nearest=left_edge[L_end_row];       //设置起始搜索位置
                        for( EXTRACT_ROW=L_end_row ; EXTRACT_ROW<=L_end_row+9 ; EXTRACT_ROW++ )
                        {
                              if( EXTRACT_ROW>ROW_END )
                              {
                                    break;
                              }
                              
                              L_get_flag=0;                        //对左边沿搜索得到标志清0
                              for( EXTRACT_COL=L_edge_nearest ; EXTRACT_COL>=COL_START+1 ; EXTRACT_COL-- )
                              {
                                    if( img_data[EXTRACT_ROW][EXTRACT_COL-1]==DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL]==DOT_B )       //搜索找到本行黑白跳变点
                                    {
                                          left_down_edge[L_LINK_NUM]=EXTRACT_COL;         //存储本行边沿点
                                          
                                          L_LINK_NUM++;                                   //边沿点个数加一
                                          
                                          L_edge_nearest=EXTRACT_COL-1;                   //存储最近一个左边沿的列坐标
                                          
                                          L_get_flag=1;                                   //本行找到下降边沿点
                                          
                                          break;                                         //结束本行搜索
                                    }
                              }
                              if( L_get_flag==0 )      //如果本行未找到边沿点，停止搜索
                              {
                                    break;
                              }
                        }
                  }
            }
            
            if( R_TURN==1 && L_end_row!=-1 )    //当图像为右弯，并且提取出左边沿结束行，左边沿向下搜索
            {
                  L_edge_nearest=left_edge[L_end_row]+1;     //设置起始搜索位置
                  for( EXTRACT_ROW=L_end_row ; EXTRACT_ROW<=L_end_row+9 ; EXTRACT_ROW++ )
                  {
                        if( EXTRACT_ROW>ROW_END )
                        {
                              break;
                        }
                        
                        L_get_flag=0;                        //对左边沿搜索得到标志清0
                        if( img_data[EXTRACT_ROW][L_edge_nearest]==DOT_B )      //当最近列的下一点为黑时，停止搜索
                        {
                              break;
                        }
                        else      //当最近列的下一点不为黑时，开始搜索
                        {
                              for( EXTRACT_COL=L_edge_nearest+1 ; EXTRACT_COL<=COL_END ; EXTRACT_COL++ )
                              {
                                    if( img_data[EXTRACT_ROW][EXTRACT_COL]==DOT_B )       //搜索找到本行黑点
                                    {
                                          left_down_edge[L_LINK_NUM]=EXTRACT_COL;         //存储本行边沿点
                                          
                                          L_LINK_NUM++;                                   //边沿点个数加一
                                          
                                          L_edge_nearest=EXTRACT_COL;                     //存储最近一个左边沿的列坐标
                                          
                                          L_get_flag=1;                                   //本行找到下降边沿点
                                          
                                          break;                                         //结束本行搜索
                                    }
                              }
                        }
                        if( L_get_flag==0 )      //如果本行未找到边沿点，停止搜索
                        {
                              break;
                        }
                  }
                  
                  if( R_end_row!=-1 )    //当图像为右弯，并且提取出右边沿结束行，右边沿向下搜索
                  {
                        R_edge_nearest=right_edge[R_end_row];      //设置起始搜索位置
                        for( EXTRACT_ROW=R_end_row ; EXTRACT_ROW<=R_end_row+9 ; EXTRACT_ROW++ )
                        {
                              if( EXTRACT_ROW>ROW_END )
                              {
                                    break;
                              }
                              
                              R_get_flag=0;                        //对右边沿搜索得到标志清0
                              for( EXTRACT_COL=R_edge_nearest ; EXTRACT_COL<=COL_END-1 ; EXTRACT_COL++ )
                              {
                                    if( img_data[EXTRACT_ROW][EXTRACT_COL+1]==DOT_W && img_data[EXTRACT_ROW][EXTRACT_COL]==DOT_B )       //搜索找到本行黑白跳变点
                                    {
                                          right_down_edge[R_LINK_NUM]=EXTRACT_COL;        //存储本行边沿点
                                          
                                          R_LINK_NUM++;                                   //边沿点个数加一
                                          
                                          R_edge_nearest=EXTRACT_COL+1;                   //存储最近一个右边沿的列坐标
                                          
                                          R_get_flag=1;                                   //本行找到下降边沿点
                                          
                                          break;                                         //结束本行搜索
                                    }
                              }
                              if( R_get_flag==0 )      //如果本行未找到边沿点，停止搜索
                              {
                                    break;
                              }
                        }
                  }
            }
      }
}

/***********************************************************************************************************************************/

void bar_car()                    //此函数为超车判断函数
{
      if( double_car_flag==1 )   //如果为双车模式
      {
            if( car_over_able==1 )
            {
                  if( car_flag==0 )  //如果为前车
                  {
                        if( car_over_ok==0 )
                        {
                              W_num=0;
                              for( W_num_row=ROW_END ; W_num_row>=30 ; W_num_row-- )
                              {
                                    if( left_edge[W_num_row]!=-1 && right_edge[W_num_row]!=-1 )       //搜索找到本行黑白跳变点
                                    {
                                          if( abs(left_edge[W_num_row]-right_edge[W_num_row])<1.5*bar_range_col[W_num_row] )
                                          {
                                                W_num++;
                                          }
                                    }
                              }
                              if( W_num>=10 )
                              {
                                    bar_car_temp_1++;
                                    if( bar_car_temp_1>=3 )
                                    {
                                          bar_car_temp_1=0;
                                          car_over_ok=1;
                                    }
                              }
                              else
                              {
                                    bar_car_temp_1=0;
                              }
                        }
                  }
                  else if( car_flag==1 )  //如果为后车
                  {
                        if( car_over_ok==0 )
                        {
                              int16 down_row=-1,up_row=-1,aa=0;
                              W_num=0;
                              for( W_num_row=ROW_END ; W_num_row>=20 ; W_num_row-- )
                              {
                                    if( left_edge[W_num_row]!=-1 && right_edge[W_num_row]!=-1 )       //搜索找到本行黑白跳变点
                                    {
                                          if( abs(left_edge[W_num_row]-right_edge[W_num_row])<1.5*bar_range_col[W_num_row] )
                                          {
                                                aa=0;
                                                W_num++;
                                                if( down_row==-1 )
                                                {
                                                      down_row=W_num_row;
                                                }
                                                else
                                                {
                                                      up_row=W_num_row;
                                                }
                                          }
                                          else
                                          {
                                                if( W_num>0 )
                                                {
                                                      aa++;
                                                }
                                          }
                                    }
                                    else
                                    {
                                          if( W_num>0 )
                                          {
                                                aa++;
                                          }
                                    }
                                    if( aa>=5 )
                                    {
                                          break;
                                    }
                              }
                              if( W_num>=10 )
                              {
                                    if( bar_car_val==-1 )
                                    {
                                          bar_car_temp_1++;
                                          if( bar_car_temp_1>=2 )
                                          {
                                                bar_car_temp_1=0;
                                                bar_car_val=25*inv_distance[up_row]+abs(speedout_val);
                                          }
                                    }
                                    else
                                    {
                                          if( bar_car_val-abs(speedout_val)>3000 )
                                          {
                                                bar_car_val=25*inv_distance[up_row]+abs(speedout_val);
                                          }
                                          else
                                          {
                                                int32 val_1;
                                                val_1=25*inv_distance[up_row]+abs(speedout_val);
                                                if( val_1<bar_car_val+2000 )
                                                {
                                                      bar_car_val=val_1;
                                                }
                                          }
                                    }
                              }
                              else
                              {
                                    if( bar_car_val==-1 )
                                    {
                                          bar_car_temp_1=0;
                                    }
                              }
                              if( bar_car_val!=-1 )
                              {
                                    if( abs(speedout_val)>bar_car_val )
                                    {
                                          car_over_ok=1;
                                    }
                              }
                        }
                  }
            }
      }
}

/***********************************************************************************************************************************/

void tan_angle()                    //此函数为斜率角度制表函数
{
      uint8 x;
      for( x=0 ; x<=29 ; x++ )
      {
            tan_table[x]=tan((double)x*PI/60);
      }
}

/***********************************************************************************************************************************/

void bar_range()                    //此函数为每行障碍物搜索范围获取函数
{
      uint8 x;
      Site_xy xy_1,xy_2;
      Site_xy1 xy1;
      for( x=ROW_START ; x<=ROW_END ; x++ )
      {
            xy_1=get_inv_img( x , 79 );
            xy_2=get_inv_img( x , 80 );
            inv_distance[x]=(xy_1.x+xy_2.x)/2;
      }
      
      for( x=ROW_START ; x<=ROW_END ; x++ )
      {
            xy1=get_invinv_img( inv_distance[x] , ROAD_WIDE/2 );
            bar_range_col[x]=abs(xy1.y-79.5 );
      }
      
      for( x=ROW_START ; x<=ROW_END ; x++ )
      {
            xy1=get_invinv_img( inv_distance[x] , ROAD_WIDE/4 );
            car_col[x]=abs(xy1.y-79.5 );
      }
}

/***********************************************************************************************************************************/




