#include "include.h"
#include "common.h"

/***********************************************************************************************************************************/
int16 cnt_step;
int16 cnt_a;
int16 cnt_b;
int16 cnt_c;
int16 cnt_d;
int16 cnt_e;
int16 cnt_f;

int16 barrier_time=0;

int16 END_LINE_TEMP_delay_time2;

uint8 IN_CURVE_delay_flag1;
int16 IN_CURVER_delay_cnt1;

uint8 END_LINE_TEMP_delay_flag2;
int16 END_LINE_TEMP_delay_cnt2;
uint8 car_L_pull_over_flag;
uint8 car_R_pull_over_flag;
uint8 car_M_pull_over_flag;
uint8 car_pull_over_flag;
int16 car_pull_over_delay_cnt;
uint8 car_pull_over_finish_flag;

int16 B_car_delay_cnt1;
uint8 B_car_delay_flag1;
int16 B_car_delay_cnt2;
uint8 B_car_delay_flag2;
uint8 bangbang = 1 ;
int16 bangbang_cnt;
uint8 bangbang_begin;

int16 LOOP_delay_cnt1;
uint8 LOOP_delay_flag1;

int16  STRAIGHT_delay_cnt1;
uint8  STRAIGHT_delay_flag1;

int16  S_delay_cnt1;
uint8  S_delay_flag1;

int16  CROSS_delay_cnt1;
uint8  CROSS_delay_flag1;
int16  CROSS_delay_cnt2=0;
uint8  CROSS_delay_flag2=0;

int16  L_CURVE_delay_cnt1;
uint8  L_CURVE_delay_flag1;
int16  R_CURVE_delay_cnt1;
uint8  R_CURVE_delay_flag1;


uint8  S_CURVE;
int16  S_CURVE_delay_cnt1;
uint8  S_CURVE_delay_flag1;

int16 S_CURVE_CNT;

///////////////////////////////////////////////////////////////////////////////
//---------------------------------------------------------------------------//
///////////////////////////////////////////////////////////////////////////////
/*********通信部分**********/
uint8 H_Q_overtake_outrow_ok=0;
uint8 H_Q_overtake_outrow_flag=0;
uint8 H_Q_overtake_ring_ok=0;
uint8 H_Q_overtake_ring_flag=0;//后车对前车

uint8 Q_H_overtake_cross_flag=0;
uint8 Q_H_stop_ok=0;
uint8 Q_H_overtake_ring_ok=0;
uint8 Q_H_overtake_ring_flag=0;
uint8 Q_H_overtake_po_flag=0;
uint8 Q_H_obstacle_flag=0;//前车对后车
/*********发车部分*********/
uint8 barrier_over_start=0;
uint8 l_barrier_over=0;
uint8 r_barrier_over=0;
uint8 l_barrier_over_1=0;
uint8 r_barrier_over_1=0;
int32 l_barrier_val=0;
int32 r_barrier_val=0;
int32 qipao_distance=0;
int32 qipao_hua_val=0;
uint8 overtake_start_ok=0;
uint8 LOR_turn_send_ok=0;
uint8 speedout_val_start1_flag=0;
int32 speedout_val_start1=0;

uint8 speedout_val01_flag=0;
int32 speedout_val01=0;


uint8 dir_overtake_start=0;
uint8 overtake_start_flag=0;//发车模式
uint8 START_LINE_TEMP=0;//记录已经过起跑线，只使用一次
uint8 overtake_ok_flag=0;//成功超车
int32 overtake_start_sum=0;//超车次数，0不超车，1正常速度超一次，2直道太短慢速超
uint8 houche_speed_flag=0;//后车超完速度回归正常
uint8 speedout_val0_flag=0;//记步标志
int32 speedout_val0=0;//过起跑线的距离，用于定点停车
uint8 speedout_val5_flag=0;//记步标志
int32 speedout_val5=0;//最终的停车
/*********十字部分*********/
uint8 overtake_cross_flag=0;
uint8 overtake_cross_sum=0;
uint8 Q_overtake_cross=0;
uint8 overtake_cross_stop=0;
uint8 overtake_cross_ing_flag=0;
uint8 speedout_val_cross_flag=0;
int32 speedout_val_cross=0;
uint8 H_voertake_cross=0;
uint8 Q_stop_ok=0;
uint8 overtake_cross_stop_over=0;
uint8 speedout_val_cross2_flag=0;
int32 speedout_val_cross2=0;
uint8 speedout_val_cross1_flag=0;
int32 speedout_val_cross1=0;
uint8 speedout_val_cross_ting_flag=0;
int32 speedout_val_cross_ting_val=0;
uint8 overtake_cross_ok=0;
int32 cross_stop_val=-1;

Site_xy L_cross_down_val={-1.-1};
Site_xy R_cross_down_val={-1.-1};
Site_xy L_cross_up_val={-1.-1};
Site_xy R_cross_up_val={-1.-1};
/*********环道超车部分*********/
uint8 overtake_outrow_flag=0;
uint8 ringout_able_flag=0;
uint8 overtake_ring_flag=0;//环道超车模式
uint8 overtake1_ring_ok_flag=0;
uint8 overtake_ring_stop=0;
uint8 speedout_val_ring3_flag=0;
int32 speedout_val_ring3=0;
uint8 ring_out_enable=0;

uint8 loop_LOR_flag1=0;//第一个环的方向
uint8 loop_LOR_flag2=0;//第二个环的方向
uint8 loop_LOR_flag3=0;//第三个环的方向
uint8 loop_LR2_flag=0;//补线用的环的方向
int32 overtake_ring_sum=0;//环超车次数
int32 ring_sum=0;//过环的次数，用于环道方向判断
/*********直道超车部分*********/
uint8 dir_overtake_zhi=0;
uint8 overtake_zhi_flag=0;//直道超车模式
uint8 overtake_zhi_ing_flag=0;//前车超车准备
uint8 chaoche_zhi_flag=0;//后车超车准备
uint8 over_zhangai=0;//障碍记数
uint8 over_podao=0;//坡道记数
uint8 over_ring=0;//环道记数
int32 overtake_zhi_sum=0;
/*********坡道超车部分*********/
uint8 po_cnt_S=0;
uint8 dir_overtake_po_1=0;
uint8 dir_overtake_po_2=0;
uint8 overtake_po_ok=0;
uint8 dir_overtake_po=0;
uint8 overtake_po_flag=0;//坡道超车模式
uint8 overtake_po_ing_flag=0;//前车超车准备
int32 overtake_po_sum=0;//坡道超车次数

uint8 speedout_val_po_flag=0;//记步标志
int32 speedout_val_po=0;//最终的停车
uint8 speedout_val_po1_flag=0;
int32 speedout_val_po1=0;
/*********避障部分*********/
uint8 barrier_able_flag=0;//后车避障模式
uint8 barrier_ing_flag=0;//正在通过障碍物

/*********停车部分*********/
uint8 stop_tx=0;
uint8 dir_car_stop=0;
uint8 STOP_PATTERN_able=0;
uint8 QAQ_STOP=0;//出界停车
uint8 car_stop_flag=0;//停车
uint8 car_stop_ing_flag=0;//停车前的缓冲滑行

uint8 Bee_flag=0;//蜂鸣器 鸣
uint32 Bee_time=0;//蜂鸣器计时
double CAR_position=0;//底部偏差，代表车身位置

int32 overtake_sum=0;
uint8 keep_side=0;
uint8 LOR_turn_flag=0;
uint8 LOR_turn_flag1=0;
uint8 LOR_turn_flag2=0;
uint8 po_pull_over=0;
uint8  cross_CHAO=0;
uint8 cross_jian=0;
uint8 S_NO_CROSS=0;
int32 S_FLAG_val=0;
uint8 cross_chao_jian=0;
int32 cross_chao_jian_val=0;
uint8 cross_qian_pull_over=0;
uint8 cross_hou_pull_over=0;
/***********************************************************************************************************************************/

void car_control();
double get_car_position();
int16 get_points_distance(Site_xy a,Site_xy b);
/***********************************************************************************************************************************/

void car_control()
{
      //环道识别开关
      f_key_extract();
      if(double_car_flag)
      {
            if(overtake_start_flag)
            {
                  if(overtake_start_sum!=0)//超车，完成后自己退出
                  {
                        if(overtake_start_sum==1)
                        {
                              f_overtake_start();
                        }
                        else if(overtake_start_sum==2)
                        {
                              f_overtake_start_barrier();
                        }
                  }
            }
            else if(overtake_ring_flag)//环道超车
            {
                  f_overtake_ring();
            }
            else if(overtake_po_flag)//坡道超车模式
            {
                  f_overtake_po();
            }
            else if(overtake_cross_flag)//十字超车
            {
                  f_overtake_cross();
            }
            else//跟车模式
            {
                  f_voertake_no();
            }
      }
      nrf_read_finish_flag=0;
      nrf_send_finish_flag=0;
      if(!car_flag)
      {
            car_distance=0;
      }
}

/***********************************************************************************************************************************/
void f_voertake_no()
{
      if(car_flag)//后车
      {
            B_read_A();
            
            //---------环道超车---------//
            if(overtake_ring_sum!=0 && LOOP_TEMP)
            {
                  overtake_ring_flag=1;
            }
            
//            //---------坡道超车--------//作用是提前靠边，避免通信失败
//            if(up_flag && (overtake_po_sum!=0))
//            {
//                  car_stop_ing_flag=1;
//                  keep_side=0;
//                  if(dir_overtake_po)
//                  {
//                        car_L_pull_over_flag=1;
//                        car_R_pull_over_flag=0;
//                  }
//                  else
//                  {
//                        car_L_pull_over_flag=0;
//                        car_R_pull_over_flag=1;
//                  }
//            }
            //------------后车避障模式-----------//
            if(barrier_able_flag)
            {
                  barrier_able=1;
                  if(R_barrier_down_row>=50 || L_barrier_down_row>=50)
                  {
                        barrier_ing_flag=1;
                  }
                  if(barrier_ing_flag && (R_barrier_down_row<50 || L_barrier_down_row<50))//避障结束
                  {
                        barrier_able_flag=0;
                        barrier_able=0;
                        barrier_ing_flag=0;
                  }
            }
            
            //----------------通信----------------//
            if(QAQ_STOP)
            {
                  B_send_A();
            }
            if(STOP_PATTERN)
            {
                  B_send_A();
            }
      }
      else //前车
      {
            //-------------先读------------//
            A_read_B();
            
            //---------环道超车---------//
            if(overtake_ring_sum!=0 && LOOP_TEMP)
            {
                  overtake_ring_flag=1;
            }
            //---------坡道超车--------//
#if 1
            if((overtake_po_sum!=0)&&(((round(cnt_c)==1) && (po_cnt==po_cnt_S)) || ((round(cnt_c)==2)) ) && (END_LINE_TEMP!=1) && !LOOP_TEMP)
            {
                  if(up_flag)
                  {     
                        po_pull_over=1;
                        car_stop_ing_flag=1;
                        overtake_po_ing_flag=1;
                        if(dir_overtake_po)
                        {
                              car_L_pull_over_flag=0;
                              car_R_pull_over_flag=1;
                        }
                        else
                        {
                              car_L_pull_over_flag=1;
                              car_R_pull_over_flag=0;
                        }
                        barrier_able=0;
                  }
                  if(down_flag)
                  {
                        if(!speedout_val_po_flag)
                        {
                              speedout_val_po_flag=1;
                              speedout_val_po=speedout_val;
                        }
                  }
            }
            if(overtake_po_ing_flag && speedout_val_po_flag && abs(speedout_val-speedout_val_po)>=other3_b*10000)
            {
                  car_stop_flag=1;
            }//防通讯失败，提前停车
#endif
            
            //---------十字超车--------//
#if 1
            if(overtake_cross_sum!=0 && !Q_overtake_cross && !S_FLAG && !up_flag && !down_flag && !in_curve_flag )
            {
                  if(L_down_cross[0]!=-1) 
                  {
                        L_cross_down_val=get_inv_img(L_down_cross[0] ,L_down_cross[1]);
                  }
                  if(R_down_cross[0]!=-1)
                  {
                        R_cross_down_val=get_inv_img(R_down_cross[0] ,R_down_cross[1]);
                  }
                  if(L_up_cross[0]!=-1) 
                  {
                        L_cross_up_val=get_inv_img(L_up_cross[0] ,L_up_cross[1]);
                  }
                  if(R_up_cross[0]!=-1)
                  {
                        R_cross_up_val=get_inv_img(R_up_cross[0] ,R_up_cross[1]);
                  }
                  if(abs(speedout_val-cross_chao_jian_val)>=2.5*10000)
                  {
                    cross_chao_jian=0;
                  }
                  if(!cross_chao_jian)
                  {
                    if(((L_up_cross[0]!=-1) && (R_up_cross[0]!=-1)) && ((L_down_cross[0]!=-1) && (R_down_cross[0]!=-1)))
                    {
                      if(((L_cross_down_val.x>=(0.5*400)) || ( R_cross_down_val.x>=(0.5*400)))&&((get_points_distance(L_cross_down_val,L_cross_up_val)<=(0.6*400)) || (get_points_distance(R_cross_down_val,R_cross_up_val)<=(0.6*400)))&&((L_cross_down_val.x<=1.5*400) || ( R_cross_down_val.x<=1.5*400)))//限制距离
                      {
                        if(fabsf(((double)(SERVO_MID)-(double)(servo_out))/((double)(SERVO_L)-(double)(SERVO_MID)))<=0.5)//限制偏差
                        {
                            cross_jian=1; 
                        }
                      }
                    }
                    if(((L_up_cross[0]!=-1) && (R_up_cross[0]!=-1)) && ((L_down_cross[0]!=-1) && (R_down_cross[0]!=-1)))
                    {
                      if(((L_cross_down_val.x>=(0.5*400)) || ( R_cross_down_val.x>=(0.5*400)))&&((get_points_distance(L_cross_down_val,L_cross_up_val)<=(0.6*400)) || (get_points_distance(R_cross_down_val,R_cross_up_val)<=(0.6*400)))&&((L_cross_down_val.x<=1*400) || ( R_cross_down_val.x<=1*400)))//限制距离
                      {
                        if(fabsf(((double)(SERVO_MID)-(double)(servo_out))/((double)(SERVO_L)-(double)(SERVO_MID)))<=0.5)//限制偏差
                        {
                            overtake_cross_stop=1;//特殊减速滑行
                            car_stop_ing_flag=1;
                            overtake_cross_ing_flag=1;//通信
                            if(LOR_turn_flag)
                            {
                              car_R_pull_over_flag=1;
                              car_L_pull_over_flag=0;
                              LOR_turn_flag1=1;
                            }
                            else
                            {
                              car_R_pull_over_flag=0;
                              car_L_pull_over_flag=1;
                              LOR_turn_flag1=0;
                            }
                            keep_side=1;
                            if(!speedout_val_cross_flag)
                            {
                              Q_overtake_cross=1;
                              speedout_val_cross_flag=1;
                              speedout_val_cross=speedout_val;
                            }
                        }
                      }
                    }
                    
                  }
            }
          
            
            
            
            
             if(speedout_val_cross_flag && !speedout_val_cross_ting_flag)
            {
                  if( cross_stop_val==-1 )
                  {
                        if( car_L_pull_over_flag==1 && car_R_pull_over_flag==0 )
                        {
                              if( L_down_cross[0]!=-1 )
                              {
                                    cross_stop_val=25*inv_distance[L_down_cross[0]]+abs(speedout_val);
                              }
                        }
                        else if( car_L_pull_over_flag==0 && car_R_pull_over_flag==1 )
                        {
                              if( R_down_cross[0]!=-1 )
                              {
                                    cross_stop_val=25*inv_distance[R_down_cross[0]]+abs(speedout_val);
                              }
                        }
                  }
                  else
                  {
                        if( car_L_pull_over_flag==1 && car_R_pull_over_flag==0 )
                        {
                              if( L_down_cross[0]!=-1 )
                              {
                                    int32 val_1;
                                    val_1=25*inv_distance[L_down_cross[0]]+abs(speedout_val);
                                    if( val_1<cross_stop_val+2000 )
                                    {
                                          cross_stop_val=val_1;
                                    }
                              }
                        }
                        else if( car_L_pull_over_flag==0 && car_R_pull_over_flag==1 )
                        {
                              if( R_down_cross[0]!=-1 )
                              {
                                    int32 val_1;
                                    val_1=25*inv_distance[R_down_cross[0]]+abs(speedout_val);
                                    if( val_1<cross_stop_val+2000 )
                                    {
                                          cross_stop_val=val_1;
                                    }
                              }
                        }
                        
                        if( abs(speedout_val)>cross_stop_val )
                        {
                              if(!speedout_val_cross_ting_flag)
                              {
                                    cross_stop_val=-1;
                                    speedout_val_cross_ting_flag=1;
                                    speedout_val_cross_ting_val=speedout_val;
                                    //Bee_flag=1;
                              }
                        }
                  }

                  
                  
//                  if(((L_down_cross[0]==-1) &&(R_down_cross[0]==-1)))
//                  {
//                        if(((L_up_cross[0]!=-1) &&(L_cross_up_val.x<=(0.5*400)))||((R_up_cross[0]!=-1) &&(R_cross_up_val.x<=(0.5*400))))
//                        {
//                              if(!speedout_val_cross_ting_flag)
//                              {
//                                    speedout_val_cross_ting_flag=1;
//                                    speedout_val_cross_ting_val=speedout_val;
//                                    //Bee_flag=1;
//                              }
//                        }
//                  }
//           
            
            }
            if(speedout_val_cross_flag &&speedout_val_cross_ting_flag&& (abs(speedout_val-speedout_val_cross)>=(0.6*10000))&&(abs(speedout_val-speedout_val_cross_ting_val)>=(0.2*10000)))
            {
                  car_stop_flag=1;
                  overtake_cross_stop_over=0;
                  bangbang_tingcnt=0;
            }
#endif            
            
            
            if(STOP_PATTERN_able!=0)
            {
                  f_STOP_PATTERN();//停车
            }
            
            //-------------通讯-----------//
#if 1
            if(L_barrier_flag || R_barrier_flag)//避障开关
            {
                  Q_H_obstacle_flag=1;
            }
            else
            {
                  Q_H_obstacle_flag=0;
            }
            if(overtake_po_ing_flag)//坡道超车
            {
                  Q_H_overtake_po_flag=1;
            }
            else
            {
                  Q_H_overtake_po_flag=0;
            }
            if(overtake_cross_ing_flag)
            {
                  Q_H_overtake_cross_flag=1;
            }
            else
            {
                  Q_H_overtake_cross_flag=0;
            }
            if(STOP_PATTERN)
            {
                  stop_tx++;
                  if(stop_tx>=2)
                  {
//                        if(Q_H_obstacle_flag)//停车模式，两个周期向后车发一次，并且只有看到障碍才发
//                        {
                        A_send_B();
//                        }
                        stop_tx=0;
                  }
            }
            else
            {
                  A_send_B();
            }
            if(Q_H_overtake_cross_flag && nrf_send_finish_flag)
            {
                  overtake_cross_flag=1;
                  Q_H_overtake_cross_flag=0;
                  overtake_cross_ing_flag=0;
            }
            if(Q_H_overtake_po_flag && nrf_send_finish_flag)
            {
                  Q_H_overtake_po_flag=0;
                  overtake_po_ing_flag=0;
                  overtake_po_flag=1;
            }
            if(LOOP_TEMP && !nrf_send_finish_flag)
            {
                  overtake_ring_flag=0;
            }
#endif
      }
}

void overtake_start_extract()
{
      if( !barrier_over_start )
      {
            if( end_line_num>=1 || END_LINE_TEMP==1 )
            {
                  if( !l_barrier_over && !r_barrier_over )   //识别障碍
                  {
                        if( L_barrier_down_row!=-1 && L_barrier_up_row!=-1 )
                        {
                              l_barrier_over_1++;
                              if( l_barrier_over_1>=3 )   //识别到左障碍，设定距离
                              {
                                    l_barrier_over=1;
                                    l_barrier_val=25*inv_distance[L_barrier_up_row]+abs(speedout_val)+qipao_distance;
                              }
                        }
                        else
                        {
                              l_barrier_over_1=0;
                        }
                        
                        if( R_barrier_down_row!=-1 && R_barrier_up_row!=-1 )
                        {
                              r_barrier_over_1++;
                              if( r_barrier_over_1>=3 )   //识别到右障碍，设定距离
                              {
                                    r_barrier_over=1;
                                    r_barrier_val=25*inv_distance[R_barrier_up_row]+abs(speedout_val)+qipao_distance;
                              }
                        }
                        else
                        {
                              r_barrier_over_1=0;
                        }
                  }
                  else   //识别到障碍
                  {
                        if( l_barrier_over==1 )
                        {
                              if( L_barrier_down_row!=-1 && L_barrier_up_row!=-1 )
                              {
                                    int32 val_1;
                                    val_1=25*inv_distance[L_barrier_up_row]+abs(speedout_val)+qipao_distance;
                                    if( val_1<l_barrier_val+1000 )   //识别到左障碍，更新距离
                                    {
                                          l_barrier_val=val_1;
                                    }
                                    else
                                    {
                                          if( abs(speedout_val)>l_barrier_val )   //经过一段距离，判断为通过障碍
                                          {
                                                barrier_over_start=1;
                                                if(!car_flag)
                                                {
                                                      car_L_pull_over_flag=1;
                                                      car_R_pull_over_flag=0;
                                                }
                                                else
                                                {
                                                      car_L_pull_over_flag=0;
                                                      car_R_pull_over_flag=1;
                                                }
                                                LOR_turn_flag1=0;
                                                keep_side=1;
                                                qipao_hua_val=speedout_val;
                                                //Bee_flag=1;     //蜂鸣器
                                          }
                                    }
                              }
                              else
                              {
                                    if( abs(speedout_val)>l_barrier_val )   //经过一段距离，判断为通过障碍
                                    {
                                          barrier_over_start=1;
                                          if(!car_flag)
                                          {
                                                car_L_pull_over_flag=1;
                                                car_R_pull_over_flag=0;
                                          }
                                          else
                                          {
                                                car_L_pull_over_flag=0;
                                                car_R_pull_over_flag=1;
                                          }
                                          LOR_turn_flag1=0;
                                          keep_side=1;
                                          qipao_hua_val=speedout_val;
                                          //Bee_flag=1;     //蜂鸣器
                                    }
                              }
                        }
                        else if( r_barrier_over==1 )
                        {
                              if( R_barrier_down_row!=-1 && R_barrier_up_row!=-1 )
                              {
                                    int32 val_1;
                                    val_1=25*inv_distance[R_barrier_up_row]+abs(speedout_val)+qipao_distance;
                                    if( val_1<r_barrier_val+1000 )   //识别到右障碍，更新距离
                                    {
                                          r_barrier_val=val_1;
                                    }
                                    else
                                    {
                                          if( abs(speedout_val)>r_barrier_val )   //经过一段距离，判断为通过障碍
                                          {
                                                barrier_over_start=1;
                                                if(!car_flag)
                                                {
                                                      car_L_pull_over_flag=0;
                                                      car_R_pull_over_flag=1;
                                                }
                                                else
                                                {
                                                      car_L_pull_over_flag=1;
                                                      car_R_pull_over_flag=0;
                                                }
                                                LOR_turn_flag1=1;
                                                keep_side=1;
                                                qipao_hua_val=speedout_val;
                                                //Bee_flag=1;     //蜂鸣器
                                          }
                                    }
                              }
                              else
                              {
                                    if( abs(speedout_val)>r_barrier_val )   //经过一段距离，判断为通过障碍
                                    {
                                          barrier_over_start=1;
                                          if(!car_flag)
                                          {
                                                car_L_pull_over_flag=0;
                                                car_R_pull_over_flag=1;
                                          }
                                          else
                                          {
                                                car_L_pull_over_flag=1;
                                                car_R_pull_over_flag=0;
                                          }
                                          LOR_turn_flag1=1;
                                          keep_side=1;
                                          qipao_hua_val=speedout_val;
                                          //Bee_flag=1;     //蜂鸣器
                                    }
                              }
                        }
                  }
            }
      }
}

void f_overtake_start_barrier()
{
      if(!car_flag)
      {
            overtake_start_extract();
            if(barrier_over_start&&(abs(qipao_hua_val-speedout_val)>=10000*0.6))
            {
                  car_stop_flag=1;
            }
            else if((l_barrier_val>0 && (l_barrier_val-abs(speedout_val)<0.3*10000)) || (r_barrier_val>0 && (r_barrier_val-abs(speedout_val)<0.3*10000)))
            {
                  car_stop_ing_flag=1;
            }
            A_read_B();
      }
      else//靠边行驶，超车
      {
            B_read_A();
            barrier_able=1;
            if(barrier_over_start)
            {
                  if( car_over_able==0 )
                  {
                        car_over_able=1;
                        car_over_ok=0;
                        bar_car_val=-1;
                  }
                  if(car_over_ok && car_over_able)//超车成功
                  {
                        if(!speedout_val_start1_flag)
                        {
                              speedout_val_start1_flag=1;
                              speedout_val_start1=speedout_val;
                        }
                        gpio_set(PORT_CSB, 1);
                        houche_speed_flag=1;
                  }
                  if(speedout_val_start1_flag && abs(speedout_val-speedout_val_start1)>=0.1*10000)
                  {
                        car_L_pull_over_flag=0;
                        car_R_pull_over_flag=0;
                         keep_side=0;
                  }
                  if(speedout_val_start1_flag && abs(speedout_val-speedout_val_start1)>=0.4*10000)
                  {
                        overtake_start_ok=1;
                  }
                  if(overtake_start_ok)
                  {
                        B_send_A();
                        if(nrf_send_finish_flag)
                        {
                              overtake_start_flag=0;//退出超车模式
                              
                              overtake_start_ok=0;//清标志
                              barrier_over_start=0;
                              speedout_val_start1_flag=0;
                              houche_speed_flag=0;
                              car_over_able=0;
                              
                              car_flag=0;
                        }
                  }
            }
            else
            {
                  overtake_start_extract();
            }
      }
}

void f_overtake_start()
{
      if(!car_flag)
      {//靠边行驶，过起跑线停车
            if(dir_overtake_start)
            {
                  car_L_pull_over_flag=0;
                  car_R_pull_over_flag=1;
            }
            else
            {
                  car_L_pull_over_flag=1;
                  car_R_pull_over_flag=0;
            }
            if( END_LINE_TEMP==1 )
            {
                  START_LINE_TEMP=1;
            }
            if(START_LINE_TEMP && ( end_line_row>=50 || end_line_row==-1 ))//end_line_row 终点线上边沿行数
            {
                  if(!speedout_val0_flag)
                  {
                        speedout_val0=speedout_val;
                        speedout_val0_flag=1;
                  }
                  if(speedout_val0_flag && (abs(speedout_val-speedout_val0)>=other3_a*10000))
                  {
                        car_stop_flag=1;
                  }
            }
            A_read_B();
      }
      else//靠边行驶，超车
      {
            if(dir_overtake_start)
            {
                  car_L_pull_over_flag=1;
                  car_R_pull_over_flag=0;
            }
            else
            {
                  car_L_pull_over_flag=0;
                  car_R_pull_over_flag=1;
            }
            barrier_able=1;
            if( car_over_able==0 )
            {
                  car_over_able=1;
                  car_over_ok=0;
                  bar_car_val=-1;
            }
            if(car_over_ok && car_over_able)
            {
                  if(!speedout_val01_flag)
                  {
                        speedout_val01_flag=1;
                        speedout_val01=speedout_val;
                  }
                  gpio_set(PORT_CSB, 1);
                  houche_speed_flag=1;
            }
            if(speedout_val01_flag && abs(speedout_val-speedout_val01)>=0.15*10000)//走中
            {
                  car_L_pull_over_flag=0;
                  car_R_pull_over_flag=0;
                  keep_side=0;
            }
            if(speedout_val01_flag && abs(speedout_val-speedout_val01)>=0.4*10000)//超车成功
            {
                  overtake_ok_flag=1;
            }
            if(overtake_ok_flag)
            {
                  B_send_A();
                  if(nrf_send_finish_flag)
                  {
                        overtake_start_flag=0;
                        overtake_ok_flag=0;
                        speedout_val01_flag=0;
                        houche_speed_flag=0;
                        car_over_able=0;
                        car_flag=0;
                  }
            }
      }
}

void f_overtake_cross()
{
  if(L_down_cross[0]!=-1) 
  {
    L_cross_down_val=get_inv_img(L_down_cross[0] ,L_down_cross[1]);
  }
  if(R_down_cross[0]!=-1)
  {
    R_cross_down_val=get_inv_img(R_down_cross[0] ,R_down_cross[1]);
  }
  if(L_up_cross[0]!=-1) 
  {
    L_cross_up_val=get_inv_img(L_up_cross[0] ,L_up_cross[1]);
  }
  if(R_up_cross[0]!=-1)
  {
    R_cross_up_val=get_inv_img(R_up_cross[0] ,R_up_cross[1]);
  }
      if(!car_flag)//前车
      {
            //-----------------------------------------------------//
            if(speedout_val_cross_flag && !speedout_val_cross_ting_flag)
            {
                  if( cross_stop_val==-1 )
                  {
                        if( car_L_pull_over_flag==1 && car_R_pull_over_flag==0 )
                        {
                              if( L_down_cross[0]!=-1 )
                              {
                                    cross_stop_val=25*inv_distance[L_down_cross[0]]+abs(speedout_val);
                              }
                        }
                        else if( car_L_pull_over_flag==0 && car_R_pull_over_flag==1 )
                        {
                              if( R_down_cross[0]!=-1 )
                              {
                                    cross_stop_val=25*inv_distance[R_down_cross[0]]+abs(speedout_val);
                              }
                        }
                  }
                  else
                  {
                        if( car_L_pull_over_flag==1 && car_R_pull_over_flag==0 )
                        {
                              if( L_down_cross[0]!=-1 )
                              {
                                    int32 val_1;
                                    val_1=25*inv_distance[L_down_cross[0]]+abs(speedout_val);
                                    if( val_1<cross_stop_val+2000 )
                                    {
                                          cross_stop_val=val_1;
                                    }
                              }
                        }
                        else if( car_L_pull_over_flag==0 && car_R_pull_over_flag==1 )
                        {
                              if( R_down_cross[0]!=-1 )
                              {
                                    int32 val_1;
                                    val_1=25*inv_distance[R_down_cross[0]]+abs(speedout_val);
                                    if( val_1<cross_stop_val+2000 )
                                    {
                                          cross_stop_val=val_1;
                                    }
                              }
                        }
                        
                        if( abs(speedout_val)>cross_stop_val )
                        {
                              if(!speedout_val_cross_ting_flag)
                              {
                                    cross_stop_val=-1;
                                    speedout_val_cross_ting_flag=1;
                                    speedout_val_cross_ting_val=speedout_val;
                                    //Bee_flag=1;
                              }
                        }
                  }

                  
                  
//                  if(((L_down_cross[0]==-1) &&(R_down_cross[0]==-1)))
//                  {
//                        if(((L_up_cross[0]!=-1) &&(L_cross_up_val.x<=(0.5*400)))||((R_up_cross[0]!=-1) &&(R_cross_up_val.x<=(0.5*400))))
//                        {
//                              if(!speedout_val_cross_ting_flag)
//                              {
//                                    speedout_val_cross_ting_flag=1;
//                                    speedout_val_cross_ting_val=speedout_val;
//                                    //Bee_flag=1;
//                              }
//                        }
//                  }
//            
            }
            if(speedout_val_cross_flag &&speedout_val_cross_ting_flag&& (abs(speedout_val-speedout_val_cross)>=(0.6*10000))&&(abs(speedout_val-speedout_val_cross_ting_val)>=(0.2*10000)))
            {
                  car_stop_flag=1;
                  overtake_cross_stop_over=0;
                  bangbang_tingcnt=0;
                  Q_H_stop_ok=1;
                  speedout_val_cross_flag=0;
                  speedout_val_cross_ting_flag=0;
            }
            if(Q_H_stop_ok)
            {
                  A_send_B();
                  if(nrf_send_finish_flag)
                  {
                        Q_H_stop_ok=0;
                  }
            }
            A_read_B();
      }
      else
      {
            B_read_A();
            //-----------------------------------------------------------------//识别速度回归正常(现在有问题)
            if((car_L_pull_over_flag || car_R_pull_over_flag) && !speedout_val_cross2_flag)
            {
                  speedout_val_cross2_flag=1;
                  speedout_val_cross2=speedout_val;
            }
//            if(speedout_val_cross2_flag && abs(speedout_val-speedout_val_cross2)>=0.7*10000)
//            {
//                  if(Q_stop_ok)
//                  {
//                        car_stop_ing_flag=0;
//                        Q_stop_ok=0;
//                  }
//            }
            if(S_FLAG)
            {
              S_FLAG_val=speedout_val;
              S_NO_CROSS=1;
            }
            if(S_NO_CROSS && (abs(speedout_val-S_FLAG_val)>=0.3*10000))
            {
              S_NO_CROSS=0;
            }
            if( !S_NO_CROSS && !LOOP_TEMP)//识别靠边
            {
              if(((L_down_cross[0]!=-1) ||(R_down_cross[0]!=-1) )&&((L_cross_down_val.x>=0.5*400) || ( R_cross_down_val.x>=0.5*400))/*&&((get_points_distance(L_cross_down_val,L_cross_up_val)<=(0.6*400)) || (get_points_distance(R_cross_down_val,R_cross_up_val)<=(0.6*400)))*/&&((L_cross_down_val.x<=1.5*400) || ( R_cross_down_val.x<=1.5*400)))//限制距离
              {
                if(!in_curve_flag && (fabsf(((double)(SERVO_MID)-(double)(servo_out))/((double)(SERVO_L)-(double)(SERVO_MID)))<=0.5))//限制偏差
                {
                  cross_jian=1;
                }
              }
            }
            //------------------------------------------------------------------//
            if( !S_FLAG && !LOOP_TEMP)//识别靠边
            {
              if(((L_down_cross[0]!=-1) ||(R_down_cross[0]!=-1) )&&((L_cross_down_val.x>=0.5*400) || ( R_cross_down_val.x>=0.5*400))/*&&((get_points_distance(L_cross_down_val,L_cross_up_val)<=(0.6*400)) || (get_points_distance(R_cross_down_val,R_cross_up_val)<=(0.6*400)))*/&&((L_cross_down_val.x<=1*400) || ( R_cross_down_val.x<=1*400)))//限制距离
              {
                if((fabsf(((double)(SERVO_MID)-(double)(servo_out))/((double)(SERVO_L)-(double)(SERVO_MID)))<=0.5))//限制偏差
                {
                  car_stop_ing_flag=1;
                  overtake_cross_stop=1;
                  H_voertake_cross=1;
                  cross_hou_pull_over=1;
                }
              }
            }
            if(out_curve_flag&&cross_hou_pull_over)
            {
              if(LOR_turn_flag2)
              {
                car_L_pull_over_flag=1;
                car_R_pull_over_flag=0;
              }
              else
              {
                car_L_pull_over_flag=0;
                car_R_pull_over_flag=1;
              }
              keep_side=1;
            }
            //-----------------------------------------------------------------////识别超车成功
            if( car_over_able==0 )
            {
                  car_over_able=1;
                  car_over_ok=0;
                  bar_car_val=-1;
            }
//           
//            if(((L_up_cross[0]!=-1) &&(get_inv_img(L_up_cross[0] ,L_up_cross[1]).x<=(0.5*400)))||((R_up_cross[0]!=-1) &&(get_inv_img(R_up_cross[0] ,R_up_cross[1]).x<=(0.5*400))))
//            {
//               cross_CHAO=1;
//            }
            if(car_over_ok && car_over_able )//识别超车成功
            {
              if(!speedout_val_cross1_flag)
              {
                speedout_val_cross1_flag=1;
                speedout_val_cross1=speedout_val;
              }
              car_stop_ing_flag=0;//速度恢复
              cross_hou_pull_over=0;
              qibu_flag=1;
              overtake_cross_stop_over=0;
              cross_jian=0; 
              bangbang_tingcnt=0;
              gpio_set(PORT_CSB, 1);
              barrier_able=1;
            }
            if(speedout_val_cross1_flag && abs(speedout_val-speedout_val_cross1)>=0.25*10000)//走中
            {
                  car_L_pull_over_flag=0;
                  car_R_pull_over_flag=0;
                  keep_side=0;
            }
            if(speedout_val_cross1_flag && abs(speedout_val-speedout_val_cross1)>=other5_c*10000)//超车成功
            {
                  overtake_cross_ok=1;
            }
            if(overtake_cross_ok)//通信
            {
                  B_send_A();
                  if(nrf_send_finish_flag)
                  {
                        overtake_cross_flag=0;
                        overtake_cross_ok=0;
                        car_R_pull_over_flag=0;
                        car_L_pull_over_flag=0;
                        car_flag=0;
                        
                        cross_chao_jian=1;
                        cross_chao_jian_val=speedout_val;
                        
                        keep_side=0;
                        barrier_able=1;
                        gpio_set(PORT_CSB, 1);
                        
                        car_over_able=0;
                        Q_stop_ok=0;
                        H_voertake_cross=0;
                        overtake_cross_stop=0;
                        overtake_cross_stop_over=1;
                        bangbang_tingcnt=0;
                        speedout_val_cross1_flag=0;
                        speedout_val_cross2_flag=0;
                        in_curve_flag=1;
                        if(L_TURN )
                        {
                              in_L_curve_flag=1;
                        }
                        else
                        {
                              in_R_curve_flag=1;
                        }
                        
                        overtake_cross_sum--;
                  }
            }
      }
}

void f_overtake_po()
{
      if(!car_flag)//前车
      {
            //-----------------------------------------------------//
            if(down_flag)
            {
                  if(!speedout_val_po_flag)
                  {
                        speedout_val_po_flag=1;
                        speedout_val_po=speedout_val;
                  }
            }
            if(speedout_val_po_flag && abs(speedout_val-speedout_val_po)>=other3_b*10000)
            {
                  car_stop_flag=1;
                  overtake_ring_stop=1;
                  overtake_po_stop=1;
            }
            
            A_read_B();
      }
      else
      {
            if(up_flag)
            {
                  po_pull_over=1;
                  keep_side=0;
                  car_stop_ing_flag=1;
                  if(dir_overtake_po)
                  {
                        car_L_pull_over_flag=1;
                        car_R_pull_over_flag=0;
                  }
                  else
                  {
                        car_L_pull_over_flag=0;
                        car_R_pull_over_flag=1;
                  }
            }
            if(down_flag)
            {
                  if( car_over_able==0 )
                  {
                        car_over_able=1;
                        car_over_ok=0;
                        bar_car_val=-1;
                  }
            }
            if(car_over_ok && car_over_able)//识别超车成功
            {
                  if(!speedout_val_po1_flag)
                  {
                        speedout_val_po1_flag=1;
                        speedout_val_po1=speedout_val;
                  }
                  gpio_set(PORT_CSB, 1);
                  barrier_able=1;
                  car_stop_ing_flag=0;
                  qibu_flag=1;
            }
            if(speedout_val_po1_flag && abs(speedout_val-speedout_val_po1)>=0.15*10000)
            {
                  car_L_pull_over_flag=0;
                  car_R_pull_over_flag=0;
                  po_pull_over=0;
            }
            if(speedout_val_po1_flag && abs(speedout_val-speedout_val_po1)>=other5_c*10000)
            {
                  overtake_po_ok=1;
            }
            if(overtake_po_ok)
            {
                  B_send_A();
                  if(nrf_send_finish_flag)
                  {
                        overtake_po_ok=0;
                        car_L_pull_over_flag=0;
                        car_R_pull_over_flag=0;
                        po_pull_over=0;
                        car_flag=0;
                        barrier_able=1;
                        gpio_set(PORT_CSB, 1);
                        
                        speedout_val_po1_flag=0;
                        overtake_po_flag=0;
                        car_over_able=0;
                        
                        overtake_po_sum--;//超车次数自减
                        down_flag=0;
                  }
            }
      }
}

void f_overtake_ring()
{
      if(!car_flag)
      {
            if(overtake1_ring_ok_flag)//超车成功仍在环内
            {
                  gpio_set(PORT_CSB, 0);//关超声波
                  barrier_able=0;//关障碍检测
                  
                  car_stop_ing_flag=0;
                  car_stop_flag=0;
                  qibu_flag=1;
                  if(LOOP_IN && !Q_H_overtake_ring_ok && out_row[0]!=(-1) && inv_distance[out_row[0]]<=10000/25*0.8)//用来让后车启动
                  {
                        Q_H_overtake_ring_flag=1;
                  }
                  if(!LOOP_TEMP)
                  {
                        Q_H_overtake_ring_ok=0;
                        Q_H_overtake_ring_flag=0;
                        overtake_ring_flag=0;
                        overtake_outrow_flag=0;
                        ring_out_enable=0;
                        speedout_val_ring3_flag=0;
                        car_flag=1;
                        overtake1_ring_ok_flag=0;
                        overtake_ring_sum--;
                        ring_sum++;
                  }
            }
            else
            {
                  A_read_B();
                  if(LOOP_IN)
                  {
                        if(LOOP_IN && !speedout_val_ring3_flag)
                        {
                              speedout_val_ring3_flag=1;
                              speedout_val_ring3=speedout_val;
                        }
                        if(speedout_val_ring3_flag && abs(speedout_val-speedout_val_ring3)>=0.25*10000)
                        {
                              ring_out_enable=1;
                        }
                        if((out_row[0]!=(-1))&& ring_out_enable)//前车看到出环口
                        {
                              car_stop_ing_flag=1;//立即滑行，对小环没问题，对大环，若后车早出环则没问题，而此时一定是小角度，若为大角度，前车减速等后车出环也没大问题
                              
                              if(inv_distance[out_row[0]]<=400*0.7)//用来停车
                              {
                                    car_stop_flag=1;//停车
                                    overtake_ring_stop=1;
                                    Bee_flag=1;
                              }
                        }
                        else
                        {
                              car_stop_flag=0;
                              overtake_ring_stop=0;
                              if(overtake_outrow_flag)//后车看到出环口,前车不需要滑行
                              {
                                    car_stop_ing_flag=0;
                              }
                              else
                              {
                                    if((middle_Error[0]<35 && !loop_LR2_flag) || (middle_Error[0]>-35 && loop_LR2_flag))//粗略判断为大环
                                    {
                                          car_stop_ing_flag=0;
                                    }
                                    else if((middle_Error[0]>50 && !loop_LR2_flag) || (middle_Error[0]<-50 && loop_LR2_flag))//或者改成入环就滑行的（主要针对小环）
                                    {
                                          car_stop_ing_flag=1;
                                    }
                              }
                        }
                        if(!Q_H_overtake_ring_ok && out_row[0]!=(-1) && inv_distance[out_row[0]]<=10000/25*0.8)//用来让后车启动
                        {
                              Q_H_overtake_ring_flag=1;
                        }
                  }
            }
            if(Q_H_overtake_ring_flag)
            {
                  A_send_B();
                  if(nrf_send_finish_flag)
                  {
                        Q_H_overtake_ring_flag=0;
                        Q_H_overtake_ring_ok=1;
                  }
            }
      }
      else
      {//入环，出环停下发超车成功，等待启动  （或不停，发超车成功）
            B_read_A();
            if(LOOP_IN && out_row[0]!=(-1) && !H_Q_overtake_outrow_ok)
            {//看到出环口
                  H_Q_overtake_outrow_flag=1;
            }
            if(H_Q_overtake_outrow_flag)
            {
                  B_send_A();
                  if(nrf_send_finish_flag)
                  {
                        H_Q_overtake_outrow_flag=0;
                        H_Q_overtake_outrow_ok=1;
                  }
            }
            if(!LOOP_TEMP)
            {
                  if(ringout_able_flag)
                  {
                        car_stop_flag=0;
                         qibu_flag=1;
                  }
                  else
                  {
                        car_stop_flag=1;
                  }
                  if(!H_Q_overtake_ring_ok)
                  {
                        H_Q_overtake_ring_flag=1;
                  }
                  if(H_Q_overtake_ring_flag)
                  {
                        B_send_A();
                        if(nrf_send_finish_flag)
                        {
                              H_Q_overtake_ring_ok=1;
                              H_Q_overtake_ring_flag=0;
                        }
                  }
                  if(H_Q_overtake_ring_ok && ringout_able_flag)
                  {
                        ringout_able_flag=0;
                        H_Q_overtake_ring_ok=0;
                        H_Q_overtake_outrow_ok=0;
                        car_flag=0;
                        overtake_ring_flag=0;
                        overtake_ring_sum--;
                        ring_sum++;
                        
                        car_distance=0;
                  }
                  barrier_able=1;
                  gpio_set(PORT_CSB, 1);//开启避障，开启超声波，
            }
      }
}

void f_overtake_zhi_judge()
{
      
}

uint8 STOP_PATTERN=0;
void f_STOP_PATTERN()
{
      if(END_LINE_TEMP==1 &&(abs(speedout_val)>=4*10000))
      {
            STOP_PATTERN=1;
      }
}

//-----------识别开关------------//
void f_key_extract()
{
      if(!double_car_flag)
      {
            if(po_loop_able)
            {
                  loop_able=1;
            }
            else
            {
                  loop_able=0;
            }
      }
      else
      {
            if(overtake_start_flag || overtake_po_flag || overtake_cross_flag || overtake_zhi_flag)
            {
                  loop_able=0;
            }
            else if(overtake_ring_flag)
            {
                  loop_able=1;
            }
            else
            {
                  if(!car_flag)
                  {
                        if(po_loop_able)
                        {
                              loop_able=1;
                        }
                        else
                        {
                              loop_able=0;
                        }
                  }
                  else
                  {
                        if(Q_loop_able && po_loop_able)
                        {
                              loop_able=1;
                        }
                        else
                        {
                              loop_able=0;
                        }
                  }
            }
      }
      
      if(!car_flag)
      {
            if(middle_Error[0]>30)
            {
                  LOR_turn_flag=0;
            }
            else if(middle_Error[0]<-30)
            {
                  LOR_turn_flag=1;
            }
      }
      
      if(double_car_flag && car_flag)
      {
            if(barrier_able_flag || overtake_start_flag || overtake_po_flag || overtake_cross_flag)
            {
                car_extract_able=0;
            }
            else
            {
                car_extract_able=1;
            }
            
            if(END_LINE_TEMP==1 &&(abs(speedout_val)>=4*10000))
            {
                  if( end_line_row>=30)
                  {
                        barrier_able=1;
                  }
            }
      }
      
      if(po_cnt==1)
      {dir_overtake_po=dir_overtake_po_1;}
      else if(po_cnt==2)
      {dir_overtake_po=dir_overtake_po_2;}
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int16 L_wucha=0,R_wucha=0;
uint8 straightline_L()
{
      uint8 i=0,j=7,k=0;//i行数，j阈值，k判断标志
      for(i=0;i<HEIGHT_END;i++)
      {
            if(i<20)//上半部分
            {
                  if(left_edge[i]<=(0))
                  {
                        k=1;
                        return 0;
                  }
                  L_wucha=(left_edge[i]+left_edge[5])/2-left_edge[(5+i)/2];
                  if(ABS(L_wucha)<j)
                  {
                        
                  }
                  else
                  {
                        k=1;
                        return 0;
                  }
            }
            else
            {
                  if(left_edge[i]>=(0))
                  {
                        L_wucha=(left_edge[i]+left_edge[5])/2-left_edge[(5+i)/2];
                        if(ABS(L_wucha)<j)
                        {
                              
                        }
                        else
                        {
                              k=1;
                              return 0;
                        }
                  }
            }
      }
      if(k==1)
      {return 0;}
      else
      {return 1;}
}

uint8 straightline_R()
{
      uint8 i=0,j=7,k=0;//i行数，j阈值，k判断标志
      for(i=0;i<HEIGHT_END;i++)
      {
            if(i<20)//上半部分
            {
                  if(right_edge[i]<=(0))
                  {
                        k=1;
                        return 0;
                  }
                  R_wucha=(right_edge[i]+right_edge[5])/2-right_edge[(5+i)/2];
                  if(ABS(R_wucha)<j)
                  {
                        
                  }
                  else
                  {
                        k=1;
                        return 0;
                  }
            }
            else
            {
                  if(right_edge[i]>=(0))
                  {
                        R_wucha=(right_edge[i]+right_edge[5])/2-right_edge[(5+i)/2];
                        if(ABS(R_wucha)<j)
                        {
                              
                        }
                        else
                        {
                              k=1;
                              return 0;
                        }
                  }
            }
      }
      if(k==1)
      {return 0;}
      else
      {return 1;}
}

void ring_dir()
{
      if(double_car_flag && overtake_ring_sum!=0)
      {
            if(ring_sum==0)
            {
                  if(!car_flag)//后车，走近路
                  {
                        loop_LR2_flag=loop_LOR_flag1;
                  }
                  else
                  {
                        if(loop_LOR_flag1)
                              loop_LR2_flag=0;
                        else
                              loop_LR2_flag=1;
                  }
            }
            else if(ring_sum==1)
            {
                  if(!car_flag)
                  {
                        loop_LR2_flag=loop_LOR_flag2;
                  }
                  else
                  {
                        if(loop_LOR_flag2)
                              loop_LR2_flag=0;
                        else
                              loop_LR2_flag=1;
                  }
            }
            else if(ring_sum==2)
            {
                  if(!car_flag)
                  {
                        loop_LR2_flag=loop_LOR_flag3;
                  }
                  else
                  {
                        if(loop_LOR_flag3)
                              loop_LR2_flag=0;
                        else
                              loop_LR2_flag=1;
                  }
            }
            else//环特别多，按第一个环走
            {
                  if(!car_flag)
                  {
                        loop_LR2_flag=loop_LOR_flag1;
                  }
                  else
                  {
                        if(loop_LOR_flag1)
                              loop_LR2_flag=0;
                        else
                              loop_LR2_flag=1;
                  }
            }
      }
      else
      {
            if(loop_LOR_flag1)
                  loop_LR2_flag=0;
            else
                  loop_LR2_flag=1;
      }
      
      //--------------------//
      if(Bee_flag)
      {
            gpio_set (PTE24, 1);
      }
      else
      {
            gpio_set (PTE24, 0);
      }
      if(car_flag)
            gpio_set (PTE26, 0);
      else
            gpio_set (PTE26, 1);
      if(chaoshengbo_flag)
            gpio_set (PTA17, 0);
      else
            gpio_set (PTA17, 1);
}

double get_car_position()
{
      double x1;
      double car_position_temp=0;
      for(int16 i=POSITION_JIZHUN_DOWN ; i>=POSITION_JIZHUN_UP ; i-- )   //中心线范围比通常赛道近，求中心线加权平均值
      {     
            if( midline[i]!=-1 )
            {
                  car_position_temp=car_position_temp+inv_weight[i]*midline[i];
                  x1=x1+inv_weight[i];
            }   
      }
      car_position_temp=79.5-car_position_temp/x1;
      return car_position_temp;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int16 shiboqi_bo[8];  //发送上位机数据
void shiboqi()
{
      shiboqi_bo[0]=car_flag*500;
      shiboqi_bo[1]=overtake_ring_flag*600;
      shiboqi_bo[2]=overtake_po_flag*700;
      shiboqi_bo[3]=overtake_cross_flag*750;
      shiboqi_bo[4]=car_stop_flag*800;
      shiboqi_bo[5]=LOOP_IN*500;
      shiboqi_bo[6]=motor_goal;
      shiboqi_bo[7]=out_row[0]*10;
      vcan_sendware(&shiboqi_bo,(uint32)(sizeof(shiboqi_bo)));
}

int16 get_points_distance(Site_xy a,Site_xy b)
{
  int16 temp=0;
  temp=sqrt((a.x-b.x) * (a.x-b.x) +(a.y-b.y) * (a.y-b.y));
  return temp;
}