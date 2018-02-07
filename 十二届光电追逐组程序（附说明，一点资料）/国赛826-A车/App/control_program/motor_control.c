#include "include.h"
#include "common.h"

/***********************************************************************************************************************************/
uint8 qibu_flag=1;
uint8 speed_yun_flag=0;
uint32 stopcnt=0;
int16 speed_step=1;
int16 speed_minl=500;
int16 speed_maxl=550;
int16 speed_minr=500;
int16 speed_maxr=550;
int16 speed_s=500;
int16 speed_z=600;

int16 speed_step2=1;
int16 speed_poa=0;
int16 speed_pob=0;
int16 speed_za=0;
int16 speed_zb=0;
int16 speed_ha=500;
int16 speed_hb=500;


int16 speed_max_use;
int16 speed_min_use;
uint8 bangbang_flag1;
uint8 bangbang_flag2;
uint8 bangbang_flag3;
uint8 bangbang_cnt1;
uint8 bangbang_cnt2;
uint8 bangbang_cnt3;

// 1cm=82 50/82是基准值
double motor_step=1;
double motor_a=0;
double motor_b=0;
double motor_c=30;
double motor_d=2.3;
double motor_e=6.9;
double motor_f=0;

/***********************************************************************************************************************************/

int16 motor0_out=0;        //电机输出变量1
int16 motor1_out=0;        //电机输出变量2

int16 speedout_count=0;                //编码器反馈的实际速度
int16 speedout_count_last=0;
int16 motor_error[3]={0x0,0x0,0x0};    //电机偏差存储函数

int16 motor_goal=0;        //电机输出给定速度
int16 motor_goal_temp;     //电机输出给定速度中间变量
int16 speedout=0;          //电机输出正向变量
int16 speedout_stop=0;
int16 speedout_last=0;
int16 inv_speedout=0;      //电机输出反向变量

int16 pwm_alter=0;     //电机PWM波 总增量
int16 pwm_alter_p=0;   //电机PWM波 P增量
int16 pwm_alter_i=0;   //电机PWM波 I增量
int16 pwm_alter_d=0;   //电机PWM波 D增量

int32 speedout_val=0;  //编码器反馈速度统计值
int16 speedout_bo[3];  //发送上位机数据

uint8 in_curve_flag;
uint8 in_L_curve_flag;
uint8 in_R_curve_flag;
uint8 in_d_curve_flag;
uint8 in_d_L_curve_flag;
uint8 in_d_R_curve_flag;
uint8 z_curve_flag;
uint8 in_curve_speed_set;
int16 in_curve_flag_cnt;
double car_position;
uint8 car_stop_bangbang=0;
int32 bangbang_tingcnt=0;
int32 bangbang_tingcnt2=0;
int32 bangbang_tingcnt3=0;
uint8 stop_wait=0;
uint8 stop_wait_over=0;
uint8 overtake_po_stop=0;
uint8 overtake_po_stop_over=0;
int32 in_curve_flag_val=0;
uint8 out_curve_flag=0;
/***********************************************************************************************************************************/

void motor_control();     //电机速度给定函数

void motor_control_pit(); //电机速度PID控制函数

/***********************************************************************************************************************************/

void motor_control()      //电机速度给定函数
{
  if(in_curve_flag)//出弯
  {
    if((!L_TURN && !R_TURN) || (in_L_curve_flag && R_TURN) || (in_R_curve_flag && L_TURN))//|| (in_L_curve_flag && (R_end_row<10)) || (in_R_curve_flag && (L_end_row<10)))
    {
      in_curve_flag_cnt++;
      if(in_curve_flag_cnt==2)
      {
        in_L_curve_flag=0;
        in_R_curve_flag=0;
        in_curve_flag=0;
        z_curve_flag=0;
        in_curve_speed_set=0;
        in_curve_flag_cnt=0;
        
        in_d_L_curve_flag=0;
        in_d_R_curve_flag=0;
        in_d_curve_flag=0;
        in_curve_flag_val=speedout_val;
      }
    }
  }
  if(!in_curve_flag&&(abs(speedout_val-in_curve_flag_val)>=(motor_f*10000)))
  {
    out_curve_flag=1;
  }
  if(curve_in_row>=0)
  {
    if(L_TURN && (get_inv_img(curve_in_row,JIZHUN_COL).x <= (motor_b*100)) && !LOOP_TEMP /* && !S_FLAG*/)//入弯
    {
      in_L_curve_flag=1;
      in_curve_flag=1;
      out_curve_flag=0;
    }
    
    if(R_TURN && (get_inv_img(curve_in_row,JIZHUN_COL).x <= (motor_b*100)) && !LOOP_TEMP /*&& !S_FLAG*/)//入弯
    {
      in_R_curve_flag=1;
      in_curve_flag=1;
      out_curve_flag=0;
    }
  }
  
  if(curve_in_row>=0)
  {
    if(L_TURN && (get_inv_img(curve_in_row,JIZHUN_COL).x <= (2.5*100)) && !LOOP_TEMP /* && !S_FLAG*/)//入弯
    {
      in_d_L_curve_flag=1;
      in_d_curve_flag=1;
    }
    
    if(R_TURN && (get_inv_img(curve_in_row,JIZHUN_COL).x <= (2.5*100)) && !LOOP_TEMP /*&& !S_FLAG*/)//入弯
    {
      in_d_R_curve_flag=1;
      in_d_curve_flag=1;
    }
    if(S_FLAG)
    {
      in_d_curve_flag=1;
    }
  }
  
  /**********弯道速度设定**************/
  if(speed_yun_flag)
  {
    if(car_status == CAR_STOPP)
    {
      motor_goal_temp=0;
    }
    else if(down_flag||up_flag)
    {
      motor_goal_temp=400;
      // Bee_flag=1;
    }
    /**********环道速度设定**************/
    else if( LOOP_TEMP )
    {
      if(speed_maxr>=speed_minr)
      {
        speed_max_use=(speed_maxr+speed_maxl)/2;
        speed_min_use=(speed_minr+speed_minl)/2;
      }
      else
      {
        speed_max_use=(speed_minr+speed_minl)/2;
        speed_min_use=(speed_maxr+speed_maxl)/2;
      }
      if((LOOP_IN==0) || ( out_row[0]!=-1 ))
      {
        motor_goal_temp=speed_min_use;
      }
      else
      {
        motor_goal_temp=1.0*speed_max_use-middle_Error[0]*middle_Error[0]*(speed_max_use-speed_min_use)/6400;
      }
    }     
     else if(!L_barrier_flag && !R_barrier_flag)
    {
      if(speed_maxr>=speed_minr)
      {
        speed_max_use=(speed_maxr+speed_maxl)/2;
        speed_min_use=(speed_minr+speed_minl)/2;
      }
      else
      {
        speed_max_use=(speed_minr+speed_minl)/2;
        speed_min_use=(speed_maxr+speed_maxl)/2;
      }
      motor_goal_temp=1.0*speed_max_use-middle_Error[0]*middle_Error[0]*(speed_max_use-speed_min_use)/6400;
    }
    else  if(S_FLAG)
    {
      if(speed_maxr>=speed_minr)
      {
        speed_max_use=(speed_maxr+speed_maxl)/2;
        speed_min_use=(speed_minr+speed_minl)/2;
      }
      else
      {
        speed_max_use=(speed_minr+speed_minl)/2;
        speed_min_use=(speed_maxr+speed_maxl)/2;
      }
      motor_goal_temp=1.0*speed_max_use-middle_Error[0]*middle_Error[0]*(speed_max_use-speed_min_use)/6400;
    }
    /**********其他赛道速度设定**************/ 
    else 
    {
      motor_goal_temp= speed_s;       
    }
    
  }
  else
  {
    /**********停车速度设定**************/
    if(car_status == CAR_STOPP)
    {
      motor_goal_temp=0;
    }
    else if(down_flag||up_flag)
    {
      motor_goal_temp=400;
      // Bee_flag=1;
    }
    /**********环道速度设定**************/
    else if( LOOP_TEMP )
    {
      if(speed_maxr>=speed_minr)
      {
        speed_max_use=(speed_maxr+speed_maxl)/2;
        speed_min_use=(speed_minr+speed_minl)/2;
      }
      else
      {
        speed_max_use=(speed_minr+speed_minl)/2;
        speed_min_use=(speed_maxr+speed_maxl)/2;
      }
      if((LOOP_IN==0) || ( out_row[0]!=-1 ))
      {
        motor_goal_temp=speed_min_use;
      }
      else
      {
        motor_goal_temp=1.0*speed_max_use-middle_Error[0]*middle_Error[0]*(speed_max_use-speed_min_use)/6400;
      }
    } 
    else if(!L_barrier_flag && !R_barrier_flag)
    {
      if(speed_maxr>=speed_minr)
      {
        speed_max_use=(speed_maxr+speed_maxl)/2;
        speed_min_use=(speed_minr+speed_minl)/2;
      }
      else
      {
        speed_max_use=(speed_minr+speed_minl)/2;
        speed_min_use=(speed_maxr+speed_maxl)/2;
      }
      motor_goal_temp=1.0*speed_max_use-middle_Error[0]*middle_Error[0]*(speed_max_use-speed_min_use)/6400;
    }
    /**********其他赛道速度设定**************/
    else  if(S_FLAG)
    {
      if(speed_maxr>=speed_minr)
      {
        speed_max_use=(speed_maxr+speed_maxl)/2;
        speed_min_use=(speed_minr+speed_minl)/2;
      }
      else
      {
        speed_max_use=(speed_minr+speed_minl)/2;
        speed_min_use=(speed_maxr+speed_maxl)/2;
      }
      motor_goal_temp=1.0*speed_max_use-middle_Error[0]*middle_Error[0]*(speed_max_use-speed_min_use)/6400;
    }
    /**********普通弯道速度设定**************/
    else if(in_curve_flag)
    {  
      if(!in_curve_speed_set)//速度设定
      {
        car_position=get_car_position();
        if(in_L_curve_flag)
        {
          speed_max_use=speed_maxl;
          speed_min_use=speed_minl;
        }
        if(in_R_curve_flag)
        {
          speed_max_use=speed_maxr;
          speed_min_use=speed_minr;
        }
        motor_goal_temp=1.0*speed_max_use-middle_Error[0]*middle_Error[0]*(speed_max_use-speed_min_use)/6400;
        in_curve_speed_set=1;
      }
      else
      {
        if(fabsf(middle_Error[0])>=40)
        {
          z_curve_flag=1;
        }
        if(!z_curve_flag)
        {
          motor_goal_temp=speed_min_use;
        }
        else
        {
          motor_goal_temp=1.0*speed_max_use-middle_Error[0]*middle_Error[0]*(speed_max_use-speed_min_use)/6400;
        }
      }
      
    }
    else if(cross_jian)
    {
      if(speed_maxr>=speed_minr)
      {
         motor_goal_temp=(speed_minr+speed_minl)/2;
      }
      else
      {
         motor_goal_temp=(speed_maxr+speed_maxl)/2;
      }
    }
    /**********直道速度设定**************/
    else if(STRAIGHT_FLAG)
    {      
      
      motor_goal_temp=speed_z;       
    }
    else 
    {
      if(speed_maxr>=speed_minr)
      {
        speed_max_use=(speed_maxr+speed_maxl)/2+50;
        speed_min_use=(speed_maxr+speed_maxl)/2;
      }
      else
      {
        speed_max_use=(speed_minr+speed_minl)/2+50;
        speed_min_use=(speed_minr+speed_minl)/2;
      }
      motor_goal_temp=1.0*speed_max_use-middle_Error[0]*middle_Error[0]*(speed_max_use-speed_min_use)/6400;
      
      
      
    }
  }
  
  
}

/***********************************************************************************************************************************/

void motor_control_pit()  //电机速度PID控制函数
{
      /**获取当前偏差**/
      
      motor_error[0]=motor_goal+speedout_count;
      
      
      /**PID获得电机PWM波值**/
      
      pwm_alter_p=round(motor_c*(motor_error[0]-motor_error[1]));
      pwm_alter_i=round(motor_d*motor_error[0]);
      pwm_alter_d=round(motor_e*((motor_error[0]-motor_error[1])-(motor_error[1]-motor_error[2])));
      //  pwm_alter_p=round(13*motor_a*(motor_error[0]-motor_error[1]));
      //  pwm_alter_i=round(motor_a*motor_error[0]);
      //  pwm_alter_d=round(3*motor_a*((motor_error[0]-motor_error[1])-(motor_error[1]-motor_error[2])));
      pwm_alter=pwm_alter_p+pwm_alter_i+pwm_alter_d;
      
      speedout+=pwm_alter;
      
      /**停车处理**/
      if((car_status == CAR_STOPP))
      {
            if(!stop_wait)
            {
                  if(QAQ_STOP)
                  {
                        speedout=0;
                  }
                  else
                  {
                        if((speedout_count<speedout_count_last) || (speedout_count-speedout_count_last>=15*stopcnt) )
                        {
                              speedout_count=speedout_count_last;
                        }
                        if(speedout_count<=-10)
                        {
                              
                              if( speedout_count<-700 )
                              {
                                    speedout=0;
                              }
                              else if(speedout_count>-200)
                              {
                                    speedout=-3000;
                                    
                              }
                              else
                              {
                                    speedout = -3000*(700+speedout_count)/500;
                              }        
                              bangbang_flag1=1; 
                        } 
                        else
                        {
                              if(bangbang_flag1)
                              {
                                    speedout=0;
                                    
                                    
                              }
                        }
                        stopcnt++;
                  }
            }
      }
      else if(qibu_flag && (motor_goal>=100))
      {
            if(STOP_PATTERN)
            {
                  if(motor_error[0]<=-100)
                  {
                        qibu_flag=0;
                  }
                  else
                  {
                        speedout=9500;
                  }
            }
            else
            {
                  if(speedout_count<=(-(speed_minr+speed_minl)/2))
                  {
                        qibu_flag=0;
                  }
                  else
                  {
                        speedout=9500;
                  }
            }
      }
      else if(overtake_ring_stop)
      {
            if(motor_error[0]>-50)
            {
                  overtake_ring_stop=0;
            }
            else
            {
                  speedout=-4000;
            }
      }
      else if(overtake_cross_stop)
      {
            if(overtake_cross_stop_over)
            {
                  
                  if(!car_flag)
                  {
                        if(speedout_count>=-speed_za*0.8) //|| ((speedout_count>=-speed_zb/2 && car_stop_flag)))
                        {
                              overtake_cross_stop_over=0;
                              bangbang_tingcnt=0;
                        }
                        else
                        {
                              bangbang_tingcnt++;
                              speedout=-motor_a*bangbang_tingcnt*1000;
                        }
                  }
                  else
                  {
                        if(speedout_count>=-speed_zb*1) //|| ((speedout_count>=-speed_zb/2 && car_stop_flag)))
                        {
                              overtake_cross_stop_over=0;
                              bangbang_tingcnt=0;
                        }
                        else
                        {
                              bangbang_tingcnt++;
                              speedout=-motor_a*bangbang_tingcnt*100;
                        }
                  }
                  if(speedout<=-4000)
                  {
                        bangbang_tingcnt=4.0/motor_a-1;
                        // overtake_cross_stop_over=0;
                  }
            }
      }
      
      if(stop_wait)
      {
            if(stop_wait_over)
            {
                  
                  Bee_flag=1;
                  if(speedout_count>=-100) //|| ((speedout_count>=-speed_zb/2 && car_stop_flag)))
                  {
                        stop_wait_over=0;
                        bangbang_tingcnt2=0;
                  }
                  else
                  {
                        bangbang_tingcnt2++;
                        speedout=-motor_a*bangbang_tingcnt2*1000;
                  }
            }
      }
      
      
      
      /**限幅**/
      if( speedout>9500)
      {
            speedout=9500;
      }
      if(car_stop_flag || (car_status == CAR_STOPP) || car_stop_ing_flag || STOP_PATTERN)
      {
            if(overtake_cross_stop_over||stop_wait||overtake_po_stop)
            {
                  if( speedout<-5000)
                  {
                        speedout=-5000;
                  }
            }
            else
            {
                  if( speedout<-9500)
                  {
                        speedout=-9500;
                  }
            }
      }
      else
      {
            if( speedout<0)
            {
                  speedout=0;
            }
      }
      
      
      if(speedout<0)
      {
            motor1_out=abs(speedout);
            motor0_out=0;
      }
      else
      {
            motor0_out=speedout;
            motor1_out=0;
      }
      
      /**电机PWM波值赋值**/
      //        speedout_bo[0]=500*chaoshengbo_flag;
      //        speedout_bo[1]=car_distance;
      //        speedout_bo[2]=speedout/20;
      //        vcan_sendware(&speedout_bo,(uint32)(sizeof(speedout_bo)));
      
      /**更新偏差**/
      
      motor_error[2]=motor_error[1];
      motor_error[1]=motor_error[0];
      speedout_count_last=speedout_count;
}


/***********************************************************************************************************************************/
