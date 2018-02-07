#include "include.h"
#include "common.h"

/************************************************************************************/

double distance_step=1;
double distance_a=16;  //防止距离过近的近处误差，单位分米
double distance_b=38;  //
double distance_c=5;  //防止距离过近的近处误差，单位分米(弯道时)
double distance_d=8;  //
double distance_e=16;
double distance_f=15;
double distance2_step=1;
double distance2_a=5;
double distance2_b=17;
double distance2_c=3;
double distance2_d=8;
double distance2_e=7;
double distance2_f=10;

int16 motor_goal_temp2=0;           //电机输出中间变量
int32 distance_goal=800;    //目标距离，单位毫米

uint8 fangzhuang_flag=0,dis_huaxing=0;
int32 Q_distance=0,H_distance[2];
uint8 H_csb_get=0;
uint8 H_QC_get=0;
uint8 keep_stop=0;
uint8 stop_wait_able=0;

void distance_control()
{
      if(double_car_flag)//双车模式对实际期望速度的处理
      {
            dis_control();
      }
      else//单车模式对实际期望速度的处理
      {
            motor_goal_temp2=motor_goal_temp;
      }
      
      //----------滑行与停车----------//
      if(car_stop_ing_flag)
      {
            if(LOOP_TEMP)
            {
                  motor_goal_temp2=speed_hb;
            }
            else if(overtake_po_flag)
            {
                  if(!car_flag)
                  {
                        motor_goal_temp2=speed_poa;
                  }
                  else if(car_flag)
                  {
                        motor_goal_temp2=speed_pob;
                  }
            }
            else if(overtake_cross_flag)
            {
                  if(!car_flag)
                  {
                        motor_goal_temp2=speed_za;
                  }
                  else if(car_flag)
                  {
                        motor_goal_temp2=speed_zb;
                  }
            }
            else if(overtake_start_flag)
            {
                  if(!car_flag)
                  {
                        motor_goal_temp2=speed_za;
                  }
                  else if(car_flag)
                  {
                        motor_goal_temp2=speed_zb;
                  }
            }
      }
      
      if(car_stop_flag)
      {
            motor_goal_temp2=0;
            car_stop_ing_flag=0;
      }
      if(car_status == CAR_STOPP)
      {
            motor_goal_temp2=0;
            qibu_flag=0;
      }
      //-----目标速度赋值-----//
      motor_goal=motor_goal_temp2;
}

void dis_control()//双车期望速度的判别计算
{
      if( !LOOP_TEMP && !overtake_ring_flag && !overtake_po_flag && !overtake_cross_flag && !STOP_PATTERN)//如果不是特殊模式（即是普通跟车和发车），对双车速度进行如下计算
      {
            if(!car_flag)//前车控距 (距离近加速，距离远减速，根据自己的赛道确定速度限幅)
            {//Q_distance
                  if(!overtake_start_flag)
                  {
                        //                        Q_dis_control();//超声波数据抖动可能会出现错误控制
                        motor_goal_temp2=motor_goal_temp;
                  }
                  else
                  {
                        Q_dis_fache();
                  }
            }
            else//如果是后车
            {//H_distance
                  //--------------获取距离并限幅------------//
                  if(car_distance>=100 && car_distance<=3000)
                  {
                        H_distance[0]=car_distance;
                  }
                  if(!END_LINE_TEMP && car_extract_row>0)
                  {
                        H_distance[0]=10*inv_distance[car_extract_row]/4;
                  }
                  else if(!chaoshengbo_flag)
                  {
                        H_distance[0]=1500;
                  }
                  
                  //---------------------------------------//
                  if(!overtake_start_flag)//普通跟车时
                  {
                        H_dis_control();
                  }
                  else//发车时
                  {
                        H_dis_fache();
                  }
            }
      }
      else if(STOP_PATTERN)
      {
            if(!car_flag)
            {
                  if(STOP_PATTERN_able==1)//模式二冲线，有等待
                  {
                          if(speedout_val5_flag)
                          {
                                motor_goal_temp2=motor_goal_temp;
                                stop_wait=0;
                                qibu_flag=1;
                          }
                          else
                          {
                                motor_goal_temp2=1000*other5_e;
                                if(!stop_wait_able)
                                {
                                      stop_wait=1;
                                      stop_wait_able=1;
                                }
                          }
                          if(keep_stop || H_QC_get)
                          {
                                motor_goal_temp2=motor_goal_temp;
                                qibu_flag=1;
                                keep_stop=1;
                                stop_wait=0;
                          }
                  }
            }
            else
            {
                  if(STOP_PATTERN_able==1)
                  {
                        if(car_status == CAR_STOPP)
                        {
                              stop_wait=1;
                        }
                        else
                        {
                                //--------------获取距离并限幅------------//
                              if(!END_LINE_TEMP && car_extract_row>0)
                              {
                                    H_distance[0]=10*inv_distance[car_extract_row]/4;
                              }
                              else
                              {
                                    H_distance[0]=1500;
                              }
                              H_dis_control();
                              qibu_flag=1;
                        }
                  }
            }
      }
      else//特殊模式可能需要特殊操作，目前暂无
      {
            motor_goal_temp2=motor_goal_temp;
      }
}

//-------------------------------前车普通速度给定------------------------------//
void Q_dis_control()
{//(具体思想是，距离太近时加速，距离太远时减速，其余距离不控)
      if(Q_distance<=distance2_a*100)//加速
      {
            if(STRAIGHT_FLAG)//直道限幅50%
            {
                  motor_goal_temp2=limit_z_shang*motor_goal_temp;
            }
            else if(!in_d_curve_flag)//不是弯道，限幅10%
            {
                  motor_goal_temp2=limit_pu_shang*motor_goal_temp;
            }
            else//弯道限幅2%
            {
                  motor_goal_temp2=limit_w_shang*motor_goal_temp;
            }
      }
      else if(Q_distance>=distance2_b*100)//减速
      {
            if(STRAIGHT_FLAG)//直道限幅50%
            {
                  motor_goal_temp2=limit_z_xia*motor_goal_temp;
            }
            else if(!in_d_curve_flag)//不是弯道，限幅10%
            {
                  motor_goal_temp2=limit_pu_xia*motor_goal_temp;
            }
            else//弯道限幅2%
            {
                  motor_goal_temp2=limit_w_xia*motor_goal_temp;
            }
      }
      else
      {
            motor_goal_temp2=motor_goal_temp;
      }
}
//------------------------------前车发车速度给定------------------------------//
void Q_dis_fache()
{
      if(overtake_start_sum==0)//不超车时
      {
            motor_goal_temp2=motor_goal_temp;
            overtake_start_flag=0;//退出起跑模式
      }
      else if(overtake_start_sum==1)
      {
            motor_goal_temp2=250;//超车完成自己会退出起跑模式
      }
      else if(overtake_start_sum==2)
      {
            motor_goal_temp2=400;
      }
}
//-------------------------------后车普通速度给定------------------------------//
void H_dis_control()
{
      dis_huaxing=0;
      if((STOP_PATTERN && END_LINE_TEMP) || STRAIGHT_FLAG)//直道不限幅
      {
            if(STOP_PATTERN)
            {
                  distance_speed0(1,3,1);
            }
            else
            {
                  distance_speed(distance_a,distance_b,distance_c);
            }
            if(!fangzhuang_flag)//不防撞才限幅
            {
              if(STOP_PATTERN)
              {
                  if(motor_goal_temp2>other5_d*1000)
                        motor_goal_temp2=other5_d*1000;
              }
              else
              {
                  if(motor_goal_temp2>limit_z_shang*motor_goal_temp)
                        motor_goal_temp2=limit_z_shang*motor_goal_temp;
                  if(motor_goal_temp2<limit_z_xia*motor_goal_temp)
                        motor_goal_temp2=limit_z_xia*motor_goal_temp;
              }
            }
      }
      else if(!in_d_curve_flag)//不是弯道(除弯道最多的状况，限幅太小追不上，限幅太大不稳定，所以要实际调试，)
      {
            if(STOP_PATTERN)
            {
                  distance_speed0(1,3,1);
            }
            else
            {
                  distance_speed(distance_a,distance_b,distance_c);
            }
            if(!fangzhuang_flag)//不防撞才限幅
            {
              if(motor_goal_temp2>limit_pu_shang*motor_goal_temp)
                   motor_goal_temp2=limit_pu_shang*motor_goal_temp;
              if(motor_goal_temp2<limit_pu_xia*motor_goal_temp)
                   motor_goal_temp2=limit_pu_xia*motor_goal_temp;
            }
      }
      else//弯道限幅2%
      {
            if(STOP_PATTERN)
            {
                  distance_speed0(1,3,1);
            }
            else
            {
                  distance_speed(distance_d,distance_e,distance_f);
            }
            if(motor_goal_temp2>limit_w_shang*motor_goal_temp)
                  motor_goal_temp2=limit_w_shang*motor_goal_temp;
            if(motor_goal_temp2<limit_w_xia*motor_goal_temp)
                  motor_goal_temp2=limit_w_xia*motor_goal_temp;
      }
      //---------------------------------------------------------------------//
      //以下是特殊处理，如果距离太近，就会进入滑行
      if((car_extract_row>0) && (H_distance[0]<1000) && !fangzhuang_flag && !STOP_PATTERN)
      {
            motor_goal_temp2=400;
            dis_huaxing=1;
      }
}
//------------------------------后车发车速度给定------------------------------//
uint8 H_overtake_start_flag=0;
void H_dis_fache()
{
      if(overtake_start_sum==0)//起跑不超车时
      {
            if(H_distance[0]<distance2_c*100)
            {
                  motor_goal_temp2=0;//距离近时速度为0，防止启动时后车先动，或者速度快从而撞上前车
            }
            else
            {
                  overtake_start_flag=0;//不超车时，距离达到要求就会退出起跑模式，从而正常跑
                  //超车时，超车完成会自己退出起跑模式
            }
      }
      else if(overtake_start_sum==1)
      {//后车在超车时控距，以后需要时再写
            if(H_distance[0]<2*100 && (!H_overtake_start_flag))
            {
                  motor_goal_temp2=0;//距离近时速度为0，防止启动时后车先动，或者速度快从而撞上前车
            }
            else
            {
                  if(houche_speed_flag)//超车完成，速度变正常
                  {
                        motor_goal_temp2=motor_goal_temp;
                  }
                  else
                  {
                        motor_goal_temp2=350;//起跑超车的滑行速度
                  }
                  H_overtake_start_flag=1;
            }
      }
      else if(overtake_start_sum==2)
      {
            if(H_distance[0]<2*100 && (!barrier_over_start))
            {
                  motor_goal_temp2=0;//距离近时速度为0，防止启动时后车先动，或者速度快从而撞上前车
            }
            else
            {
                  if(houche_speed_flag)//超车完成，速度变正常
                  {
                        motor_goal_temp2=motor_goal_temp;
                  }
                  else
                  {
                        motor_goal_temp2=400;//起跑超车的滑行速度
                  }
            }
      }
}

//----------距离引起的速度增量计算函数----------//
void distance_speed(double a,double b,double c)
{
      fangzhuang_flag=0;
      distance_goal=100*c;
      if(H_distance[0]<1000)//防止错误减速
      {
            if(car_extract_row>0)//看到障碍物才认为距离真的近
            {
                  if(H_distance[0]<=DIS_DANGEROUS)//限幅，防溢出
                  {
                        motor_goal_temp2=MIN_SPEED;
                        fangzhuang_flag=1;
                  }
                  else if(H_distance[0]<=DIS_SHACHE && H_distance[0]>DIS_DANGEROUS)//太近，急刹车
                  {
                        motor_goal_temp2=1.0*MIN_SPEED*(H_distance[0]-DIS_SHACHE)/(DIS_DANGEROUS-DIS_SHACHE);
                        fangzhuang_flag=1;
                  }
                  else if(H_distance[0]>DIS_SHACHE && H_distance[0]<=a*100)//较近，减速
                  {
                        motor_goal_temp2=1.0*motor_goal_temp*(H_distance[0]-DIS_SHACHE)/(a*100-DIS_SHACHE);
                  }
            }
            else
            {
                  motor_goal_temp2=motor_goal_temp;
            }
      }
      else
      {
            if(H_distance[0]>DIS_SHACHE && H_distance[0]<=a*100)//较近，减速
            {
                  if(car_extract_row>0)//看到障碍物才认为距离真的近
                  {
                        motor_goal_temp2=1.0*motor_goal_temp*(H_distance[0]-DIS_SHACHE)/(a*100-DIS_SHACHE);
                  }
                  else
                  {
                        motor_goal_temp2=motor_goal_temp;
                  }
            }
            else if(H_distance[0]>a*100 && H_distance[0]<=distance_goal)//不算近，不控
            {
                  motor_goal_temp2=motor_goal_temp;
            }
            else if((H_distance[0]>distance_goal && H_distance[0]<=b*100) && !barrier_able_flag)//远了，加速
            {
                  motor_goal_temp2=1.0*(MAX_SPEED-motor_goal_temp)*(H_distance[0]-b*100)/(b*100-distance_goal)+MAX_SPEED;
            }
            else if(!barrier_able_flag)//限幅，防溢出
            {
                  motor_goal_temp2=MAX_SPEED;
            }
      }
}
void distance_speed0(double a,double b,double c)
{
      fangzhuang_flag=0;
      distance_goal=100*c;
      if(H_distance[0]<200)//防止错误减速
      {
            if(H_distance[0]<=0)//限幅，防溢出
            {
                  motor_goal_temp2=MIN_SPEED;
                  fangzhuang_flag=1;
            }
            else if(H_distance[0]<=10 && H_distance[0]>0)//太近，急刹车
            {
                  motor_goal_temp2=1.0*MIN_SPEED*(H_distance[0]-10)/(0-10);
                  fangzhuang_flag=1;
            }
            else if(H_distance[0]>10 && H_distance[0]<=a*100)//较近，减速
            {
                  motor_goal_temp2=1.0*motor_goal_temp*(H_distance[0]-10)/(a*100-10);
            }
      }
      else
      {
            if(H_distance[0]>10 && H_distance[0]<=a*100)//较近，减速
            {
                  if(car_extract_row>0)//看到障碍物才认为距离真的近
                  {
                        motor_goal_temp2=1.0*motor_goal_temp*(H_distance[0]-10)/(a*100-10);
                  }
                  else
                  {
                        motor_goal_temp2=motor_goal_temp;
                  }
            }
            else if(H_distance[0]>a*100 && H_distance[0]<=distance_goal)//不算近，不控
            {
                  motor_goal_temp2=motor_goal_temp;
            }
            else if(H_distance[0]>distance_goal && H_distance[0]<=b*100)//远了，加速
            {
                  motor_goal_temp2=1.0*(MAX_SPEED-motor_goal_temp)*(H_distance[0]-b*100)/(b*100-distance_goal)+MAX_SPEED;
            }
            else//限幅，防溢出
            {
                  motor_goal_temp2=MAX_SPEED;
            }
      }
}