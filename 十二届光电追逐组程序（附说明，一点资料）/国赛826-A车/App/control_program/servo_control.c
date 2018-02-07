#include "include.h"
#include "common.h"

/***********************************************************************************************************************************/
uint32 jizhun_step=1;
uint32 JIZHUN_ROW=30;      //中心权值行
uint32 JIZHUN_UP=11;       //计算偏差的中线截取部分的上边界14
uint32 JIZHUN_DOWN=31;     //计算偏差的中线截取部分的下边界34

uint32 servo_m_step=1;
uint32 SERVO_MID=718;      // 656
uint32 SERVO_L=789;        //舵机左极限  (卧式舵机）
uint32 SERVO_R=645;        //舵机右极限 （卧式舵机）

double servo_step=1;
double servo_a=0.15;
double servo_b=0.95;
double servo_c=2.0;
double servo_d=2.0;
double servo_e=0;
double servo_f=1.2;
/***********************************************************************************************************************************/

double servo_Kp;           //舵机 P系数
double servo_Ki;           //舵机 I系数
double servo_Kd;           //舵机 D系数
double L_R_servo_K=0;      //舵机 左右转修正系数
double middle_Error[2]={0x0,0x0};    //舵机偏差存储函数
double midline_avg;        //中心线平均值
double midline_sum;        //中心线累加值
double inv_wide[CAMERA_H];  //求中心线权值中间变量
double inv_weight[CAMERA_H];//中心线权值数组
int16 midline_use_cnt=0;    //求偏差中心线使用个数

int16 servo_adj=718;        //舵机输出中间变量
int16 servo_out=718;        //舵机输出值

int16 pull_over_row;        //小车靠边停车前瞻行
int16 pull_over_row_cross;
int16 pull_over_row_po;
int16 pull_over_row_barrier;
int16 pull_over_row_qi;
uint32 JIZHUN_UP_TEMP;     //计算偏差的中线截取部分的上边界26
uint32 JIZHUN_DOWN_TEMP;   //计算偏差的中线截取部分的下边界35
/***********************************************************************************************************************************/

void servo_control();   //舵机控制主函数
void get_weight();      //中心线权值获取函数
void middle_avg();      //中心线加权平均求偏差函数
void PID_control();     //舵机PID控制
/***********************************************************************************************************************************/

void servo_control()
{ 
  middle_avg();
  if( midline_use_cnt>=3 )
  {
    PID_control();
    
    if( servo_adj<=SERVO_L && servo_adj>=SERVO_R )
    {
      servo_adj=round(servo_adj);
    }
    else
    {
      if( servo_adj>SERVO_L )
      {
        servo_adj=SERVO_L;
      }
      if( servo_adj<SERVO_R )
      {
        servo_adj=SERVO_R;
      }
    }
  }
  else    //当中心线数量很少时
  {
    if( L_TURN )   //当判断为左弯时，左打死
    {
      servo_adj=SERVO_L;
    }
    else
    {
      if( R_TURN )    //当判断为右弯时，右打死
      {
        servo_adj=SERVO_R;
      }
      else    //当未判断出左右弯时
      {
        if( R_EDGE_NUM>L_EDGE_NUM )    //右边沿多则判断为左弯
        {
          servo_adj=SERVO_L;
        }
        else
        {
          if( L_EDGE_NUM>R_EDGE_NUM )    //左边沿多则判断为右弯
          {
            servo_adj=SERVO_R;
          }
        }
      }
    }
  }
  servo_out=servo_adj;
}

/***********************************************************************************************************************************/

void get_weight()
{
  int16 ii;
  double x0=0.0,x1=0.0;
  Site_xy1 xy;
  
  
  for( ii=ROW_START ; ii<=ROW_END ; ii++ )
  {
    inv_weight[ii]=inv_distance[ii];
  }
  
  L_R_servo_K=((double)(SERVO_MID)-(double)(SERVO_R))/((double)(SERVO_L)-(double)(SERVO_MID));
  
  xy=get_invinv_img( pull_over_dis , 0 );
  pull_over_row=xy.x;
  xy=get_invinv_img( pull_over_dis_cross , 0 );
  pull_over_row_cross=xy.x;
  xy=get_invinv_img( pull_over_dis_po , 0 );
  pull_over_row_po=xy.x;
  xy=get_invinv_img( pull_over_dis_qi , 0 );
  pull_over_row_qi=xy.x;
 
}

/***********************************************************************************************************************************/
void middle_avg()      //中线加权求偏差函数
{
  uint8 i=0;
  midline_sum=0.0;
  midline_use_cnt=0;
  double x1=0.0;
  if(qibu_flag && (motor_goal>=100))
  {
      if(speedout_count>=-200)
      {
        for( i=ROW_END ; i>=pull_over_row_qi ; i-- ) 
        {
          if( midline[i]!=-1 )
          {
            midline_sum=midline_sum+inv_weight[i]*midline[i];
            x1=x1+inv_weight[i];
            midline_use_cnt++;
          }
          else
          {
            if( L_TURN )
            {
              midline_sum=midline_sum;
              x1=x1+inv_weight[i];
              midline_use_cnt++;
            }
            if( R_TURN )
            {
              midline_sum=midline_sum+inv_weight[i]*COL_END;
              x1=x1+inv_weight[i];
              midline_use_cnt++;
            }
          }
        }
        
      }
      else
      {
        JIZHUN_DOWN_TEMP=round(JIZHUN_DOWN+(motor_goal+speedout_count)/(other2_e*10));
        if(JIZHUN_DOWN_TEMP>=ROW_END)
        {
          JIZHUN_DOWN_TEMP=ROW_END;
        }
        if(JIZHUN_DOWN_TEMP<=JIZHUN_DOWN)
        {
          JIZHUN_DOWN_TEMP=JIZHUN_DOWN;
        }
        JIZHUN_UP_TEMP=round(JIZHUN_UP+(motor_goal+speedout_count)/(other2_e*10));
        if(JIZHUN_UP_TEMP>=pull_over_row_qi)
        {
          JIZHUN_UP_TEMP=pull_over_row_qi;
        }
        if(JIZHUN_UP_TEMP<=JIZHUN_UP)
        {
          JIZHUN_UP_TEMP=JIZHUN_UP;
        }
        
        for( i=JIZHUN_DOWN_TEMP ; i>=JIZHUN_UP_TEMP ; i-- ) 
        {
          if( midline[i]!=-1 )
          {
            midline_sum=midline_sum+inv_weight[i]*midline[i];
            x1=x1+inv_weight[i];
            midline_use_cnt++;
          }
          else
          {
            if( L_TURN )
            {
              midline_sum=midline_sum;
              x1=x1+inv_weight[i];
              midline_use_cnt++;
            }
            if( R_TURN )
            {
              midline_sum=midline_sum+inv_weight[i]*COL_END;
              x1=x1+inv_weight[i];
              midline_use_cnt++;
            }
          }
        }
        
      }
    
  }
  else
  {
    if(car_status==CAR_STOPP)
    {
      if( L_barrier_flag==1 || R_barrier_flag==1 )   //当为障碍时
      { 
        for( i=JIZHUN_DOWN; i>=JIZHUN_UP; i-- )      //中心线范围取障碍物处，求中心线加权平均值
        {
          if( midline[i]!=-1 )
          {
            midline_sum=midline_sum+inv_weight[i]*midline[i];
            x1=x1+inv_weight[i];
            midline_use_cnt++;
          }
          else
          {
            if( L_TURN )
            {
              midline_sum=midline_sum;
              x1=x1+inv_weight[i];
              midline_use_cnt++;
            }
            if( R_TURN )
            {
              midline_sum=midline_sum+inv_weight[i]*COL_END;
              x1=x1+inv_weight[i];
              midline_use_cnt++;
            }
          }
          
        }
      }
      else
      {
        if(speedout_count>=-200)
        {
          for( i=ROW_END ; i>=pull_over_row_qi ; i-- ) 
          {
            if( midline[i]!=-1 )
            {
              midline_sum=midline_sum+inv_weight[i]*midline[i];
              x1=x1+inv_weight[i];
              midline_use_cnt++;
            }
            else
            {
              if( L_TURN )
              {
                midline_sum=midline_sum;
                x1=x1+inv_weight[i];
                midline_use_cnt++;
              }
              if( R_TURN )
              {
                midline_sum=midline_sum+inv_weight[i]*COL_END;
                x1=x1+inv_weight[i];
                midline_use_cnt++;
              }
            }
          }
          
        }
        else
        {
          JIZHUN_DOWN_TEMP=round(JIZHUN_DOWN+(motor_goal+speedout_count)/(other2_e*10));
          if(JIZHUN_DOWN_TEMP>=ROW_END)
          {
            JIZHUN_DOWN_TEMP=ROW_END;
          }
          if(JIZHUN_DOWN_TEMP<=JIZHUN_DOWN)
          {
            JIZHUN_DOWN_TEMP=JIZHUN_DOWN;
          }
          JIZHUN_UP_TEMP=round(JIZHUN_UP+(motor_goal+speedout_count)/(other2_e*10));
          if(JIZHUN_UP_TEMP>=pull_over_row_qi)
          {
            JIZHUN_UP_TEMP=pull_over_row_qi;
          }
          if(JIZHUN_UP_TEMP<=JIZHUN_UP)
          {
            JIZHUN_UP_TEMP=JIZHUN_UP;
          }
          
          for( i=JIZHUN_DOWN_TEMP ; i>=JIZHUN_UP_TEMP ; i-- ) 
          {
            if( midline[i]!=-1 )
            {
              midline_sum=midline_sum+inv_weight[i]*midline[i];
              x1=x1+inv_weight[i];
              midline_use_cnt++;
            }
            else
            {
              if( L_TURN )
              {
                midline_sum=midline_sum;
                x1=x1+inv_weight[i];
                midline_use_cnt++;
              }
              if( R_TURN )
              {
                midline_sum=midline_sum+inv_weight[i]*COL_END;
                x1=x1+inv_weight[i];
                midline_use_cnt++;
              }
            }
          }
          
        }
      }
      
    }
    
    else
    {
      if( LOOP_TEMP==1 )   //当为环道时
      {
        if(car_stop_ing_flag|| car_stop_flag)
        {
          
          for( i=other2_d*10; i>=other2_c*10; i-- )   //中心线范围比通常赛道近，求中心线加权平均值
          {
            if( midline[i]!=-1 )
            {
              midline_sum=midline_sum+inv_weight[i]*midline[i];
              x1=x1+inv_weight[i];
              midline_use_cnt++;
            }
            else
            {
              if( L_TURN )
              {
                midline_sum=midline_sum;
                x1=x1+inv_weight[i];
                midline_use_cnt++;
              }
              if( R_TURN )
              {
                midline_sum=midline_sum+inv_weight[i]*COL_END;
                x1=x1+inv_weight[i];
                midline_use_cnt++;
              }
            }
          }
          
        }
        else if((LOOP_IN==0) ||( out_row[0]!=-1 ))
        {
          JIZHUN_DOWN_TEMP=JIZHUN_DOWN;//+10;
          if(JIZHUN_DOWN_TEMP>=ROW_END)
          {
            JIZHUN_DOWN_TEMP=ROW_END;
          }
          for( i=JIZHUN_DOWN_TEMP; i>=JIZHUN_UP; i-- )   //中心线范围比通常赛道近，求中心线加权平均值
          {
            if( midline[i]!=-1 )
            {
              midline_sum=midline_sum+inv_weight[i]*midline[i];
              x1=x1+inv_weight[i];
              midline_use_cnt++;
            }
            else
            {
              if( L_TURN )
              {
                midline_sum=midline_sum;
                x1=x1+inv_weight[i];
                midline_use_cnt++;
              }
              if( R_TURN )
              {
                midline_sum=midline_sum+inv_weight[i]*COL_END;
                x1=x1+inv_weight[i];
                midline_use_cnt++;
              }
            }
          }
        }
        
        else     
        {
          for( i=JIZHUN_DOWN; i>=JIZHUN_UP; i-- )   //中心线范围比通常赛道近，求中心线加权平均值
          {
            if( midline[i]!=-1 )
            {
              midline_sum=midline_sum+inv_weight[i]*midline[i];
              x1=x1+inv_weight[i];
              midline_use_cnt++;
            }
            else
            {
              if( L_TURN )
              {
                midline_sum=midline_sum;
                x1=x1+inv_weight[i];
                midline_use_cnt++;
              }
              if( R_TURN )
              {
                midline_sum=midline_sum+inv_weight[i]*COL_END;
                x1=x1+inv_weight[i];
                midline_use_cnt++;
              }
            }
          }
        }
      }
      else
      {
        if( car_L_pull_over_flag==1 || car_R_pull_over_flag==1 )   //当为超车模式时
        {
          if( po_pull_over)   //当为上坡或下坡时
          {
            for( i=ROW_END ; i>=pull_over_row_po ; i-- )   //中心线范围比通常赛道近，求中心线加权平均值
            {
              
              if( midline[i]!=-1 )
              {
                midline_sum=midline_sum+inv_weight[i]*midline[i];
                x1=x1+inv_weight[i];
                midline_use_cnt++;
              }   
              
            }
          }
          
          else if(overtake_cross_flag)
          {
            if(!qibu_flag)
            {
              for( i=ROW_END ; i>=pull_over_row_cross ; i-- )   //中心线范围比通常赛道近，求中心线加权平均值
              {
                if( midline[i]!=-1 )
                {
                  midline_sum=midline_sum+inv_weight[i]*midline[i];
                  x1=x1+inv_weight[i];
                  midline_use_cnt++;
                }
                else
                {
                  if( L_TURN )
                  {
                    midline_sum=midline_sum;
                    x1=x1+inv_weight[i];
                    midline_use_cnt++;
                  }
                  if( R_TURN )
                  {
                    midline_sum=midline_sum+inv_weight[i]*COL_END;
                    x1=x1+inv_weight[i];
                    midline_use_cnt++;
                  }
                }
                
              }
            }
            else
            {
              for( i=ROW_END ; i>=pull_over_row_cross ; i-- )   //中心线范围比通常赛道近，求中心线加权平均值
              {
                if( midline[i]!=-1 )
                {
                  midline_sum=midline_sum+inv_weight[i]*midline[i];
                  x1=x1+inv_weight[i];
                  midline_use_cnt++;
                }
              }
              
            }
          }
          else
          {
            for( i=ROW_END ; i>=pull_over_row ; i-- )   //中心线范围比通常赛道近，求中心线加权平均值
            {
              if( midline[i]!=-1 )
              {
                midline_sum=midline_sum+inv_weight[i]*midline[i];
                x1=x1+inv_weight[i];
                midline_use_cnt++;
              }
              else
              {
                if( L_TURN )
                {
                  midline_sum=midline_sum;
                  x1=x1+inv_weight[i];
                  midline_use_cnt++;
                }
                if( R_TURN )
                {
                  midline_sum=midline_sum+inv_weight[i]*COL_END;
                  x1=x1+inv_weight[i];
                  midline_use_cnt++;
                }
              }
            }
          }
        }
        else
        {
          if( up_flag==1 || down_flag==1 )   //当为上坡或下坡时
          {
            for( i=ROW_END ; i>=pull_over_row_po ; i-- )   //中心线范围比通常赛道近，求中心线加权平均值
            {
              if( midline[i]!=-1 )
              {
                midline_sum=midline_sum+inv_weight[i]*midline[i];
                x1=x1+inv_weight[i];
                midline_use_cnt++;
              }
              
              
            }
          }
          
          else
          {
            if( L_barrier_flag==1 || R_barrier_flag==1 )   //当为障碍时
            { 
              for( i=JIZHUN_DOWN; i>=JIZHUN_UP; i-- )      //中心线范围取障碍物处，求中心线加权平均值
              {
                if( midline[i]!=-1 )
                {
                  midline_sum=midline_sum+inv_weight[i]*midline[i];
                  x1=x1+inv_weight[i];
                  midline_use_cnt++;
                }
                else
                {
                  if( L_TURN )
                  {
                    midline_sum=midline_sum;
                    x1=x1+inv_weight[i];
                    midline_use_cnt++;
                  }
                  if( R_TURN )
                  {
                    midline_sum=midline_sum+inv_weight[i]*COL_END;
                    x1=x1+inv_weight[i];
                    midline_use_cnt++;
                  }
                }
                
              }
            }
            else
            {
              if( L_s_temp==1 || R_s_temp==1 )
              {
                JIZHUN_DOWN_TEMP=JIZHUN_DOWN+10;
                if(JIZHUN_DOWN_TEMP>=ROW_END)
                {
                  JIZHUN_DOWN_TEMP=ROW_END;
                }
                for( i=JIZHUN_DOWN_TEMP; i>=JIZHUN_UP; i-- )  
                {
                  if( midline[i]!=-1 )
                  {
                    midline_sum=midline_sum+inv_weight[i]*midline[i];
                    x1=x1+inv_weight[i];
                    midline_use_cnt++;
                  }
                  else
                  {
                    if( L_TURN )
                    {
                      midline_sum=midline_sum;
                      x1=x1+inv_weight[i];
                      midline_use_cnt++;
                    }
                    if( R_TURN )
                    {
                      midline_sum=midline_sum+inv_weight[i]*COL_END;
                      x1=x1+inv_weight[i];
                      midline_use_cnt++;
                    }
                  }
                }
              }
              else
              {    
                for( i=JIZHUN_DOWN ; i>=JIZHUN_UP ; i-- )   //选取合适的中心线范围，求中心线加权平均值
                {
                  if( midline[i]!=-1 )
                  {
                    midline_sum=midline_sum+inv_weight[i]*midline[i];
                    x1=x1+inv_weight[i];
                    midline_use_cnt++;
                  }
                  else
                  {
                    if( L_TURN )
                    {
                      midline_sum=midline_sum;
                      x1=x1+inv_weight[i];
                      midline_use_cnt++;
                    }
                    if( R_TURN )
                    {
                      midline_sum=midline_sum+inv_weight[i]*COL_END;
                      x1=x1+inv_weight[i];
                      midline_use_cnt++;
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
  
  midline_avg=midline_sum/x1;
}

/***********************************************************************************************************************************/

void  PID_control()
{
  /**获取中线偏差**/
  
  
      middle_Error[0]=79.5-midline_avg;
      if(keep_side)    //靠边停车偏差修正
      {
            //Bee_flag=1;
            int16 pull_over_side;
            if(car_flag)
            {
                  pull_over_side=5;
            }
            else
            {
                  pull_over_side=10;
            }
            if(car_L_pull_over_flag && !car_R_pull_over_flag)
            {
                  if(middle_Error[0]<-pull_over_side)
                  {
                        keep_side=0;
                  }
                  else
                  {
                        middle_Error[0]=middle_Error[0]+pull_over_side;
                  }
            }
            if(car_R_pull_over_flag && !car_L_pull_over_flag)
            {
                  if(middle_Error[0]>pull_over_side)
                  {
                        keep_side=0;
                  }
                  else
                  {
                        middle_Error[0]=middle_Error[0]-pull_over_side;
                  }
            }
      }
  /**PID系数获取**/
  if(dis_huaxing)
  {
    servo_Kp=servo_e;
    servo_Kd=servo_c;
  }
   else if(up_flag||down_flag||(qibu_flag && (motor_goal!=0)))
  {
    servo_Kp=servo_e;
    servo_Kd=0;
  }
  else if(LOOP_TEMP)
  {
       if((LOOP_IN==0) || ( out_row[0]!=-1 ))
      {
       servo_Kp=servo_e;
       servo_Kd=0;
      }
      else
      {
        servo_Kp=servo_e;
        servo_Kd=servo_c;
      }
  }
  else if( car_L_pull_over_flag||car_R_pull_over_flag)
  {
    if(keep_side)
    {
      servo_Kp=(servo_a*70*70/10000+servo_b);
      servo_Kd=0;
    }
    else if(po_pull_over)
    {
      servo_Kp=servo_e;
      servo_Kd=servo_f;
    }
    else
    {
//      servo_Kp=1.5*servo_e;
//      servo_Kd=0;
          if( car_L_pull_over_flag==1 && car_R_pull_over_flag==0 )
          {
                if( middle_Error[0]<0 )
                {
                      servo_Kp=2*servo_e;
                      servo_Kd=0;
                }
                else
                {
                      servo_Kp=servo_e;
                      servo_Kd=0;
                }
          }
          else if( car_L_pull_over_flag==0 && car_R_pull_over_flag==1 )
          {
                if( middle_Error[0]>0 )
                {
                      servo_Kp=2*servo_e;
                      servo_Kd=0;
                }
                else
                {
                      servo_Kp=servo_e;
                      servo_Kd=0;
                }
          }
   
    
    }
  }
  else if(L_barrier_flag|| R_barrier_flag)
  { 
    if((L_barrier_down_row!=-1)&&(inv_distance[L_barrier_down_row]>=0.15*400))
    {
      servo_Kp=servo_a*70*70/10000+servo_b;
      servo_Kd=servo_c;
      
    }
    else
    {
      servo_Kp=(servo_a*70*70/10000+servo_b)*1.5;
      servo_Kd=0; 
    }
  }
  else
  {
      servo_Kp=servo_a*middle_Error[0]*middle_Error[0]/10000+servo_b;
      servo_Kd=0;
    if( L_s_temp==1 || R_s_temp==1 )
    {
      servo_Kp=(servo_a*70*70/10000+servo_b)*servo_d;
      servo_Kd=servo_c;
      if( L_s_temp==1 )
      {
        if( middle_Error[0]>0 )
        {
          L_s_temp=0;
        }
      }
      else
      {
        if( R_s_temp==1 )
        {
          if( middle_Error[0]<0 )
          {
            R_s_temp=0;
          }
        }
      }
    }
  }
  
  /**PID获得舵机调节值**/
  
  double servo_add;
  servo_add=servo_Kp*middle_Error[0]+servo_Kd*(middle_Error[0]-middle_Error[1]);
  
  
  if( servo_add<0 )
  {
    servo_add=L_R_servo_K*servo_add;
  }
  
  servo_adj=round(servo_add+SERVO_MID);
  
  /**记录偏差**/
  
  middle_Error[1]=middle_Error[0];
}

/***********************************************************************************************************************************/


 
