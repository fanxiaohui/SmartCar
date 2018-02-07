#include "include.h"

int16 tuoluoyi_y=0,tuoluoyi_x=0,tuoluoyi_z=0,tuoluoyi_y0[7];
uint8 jiance=0,count_podao;
uint8 up_hill,down_hill;
uint8 down_flag=0;
uint8 up_flag=0;
uint8 po_loop_able=1;
int podao_y[200];
int16 podao_bo[4];
int16 UP_FLAG_TEMP_1=0;
int16 DOWN_FLAG_TEMP_1=0;
int16 DOWN_FLAG_TEMP_2=0;
int16 DOWN_FLAG_TEMP_cnt2=0;
int16 po_cnt=0;
void podao_get();
void podao_get()
{
      tuoluoyi_y0[0]=Get_Gyro_y();
      tuoluoyi_y=(tuoluoyi_y0[0]+tuoluoyi_y0[1]+tuoluoyi_y0[2]+tuoluoyi_y0[3]+tuoluoyi_y0[4]+tuoluoyi_y0[5]+tuoluoyi_y0[6])/7;
      tuoluoyi_y0[6]=tuoluoyi_y0[5];
      tuoluoyi_y0[5]=tuoluoyi_y0[4];
      tuoluoyi_y0[4]=tuoluoyi_y0[3];
      tuoluoyi_y0[3]=tuoluoyi_y0[2];
      tuoluoyi_y0[2]=tuoluoyi_y0[1];
      tuoluoyi_y0[1]=tuoluoyi_y0[0];
      
      tuoluoyi_z=Get_Gyro_z();
      count_podao++;
      if(count_podao==200+1)
            count_podao=0;
      if(count_podao!=0)
      {
            podao_y[count_podao-1]=tuoluoyi_y;
      }
}
uint8 speedout_val1_flag=0,up_enable_flag=1,speedout_val2_flag=0;
int32 speedout_val1=0;
int32 speedout_val2=0;
void podao_jc()
{
      podao_get(); 
      CAR_position=get_car_position();
      if(((fabsf(CAR_position)<25)&&(fabsf(middle_Error[0])<15) && (RAMP_TEMP || (car_flag && overtake_po_flag)) && !overtake_ring_flag  && !overtake_cross_flag) || up_flag)
      {
            int16 sum_podao_y=0;
            up_hill=0;down_hill=0;
            if(count_podao>=6)
            {
                  
                  if(podao_y[count_podao-1]>UP_HILL_YUZHI)
                  {
                        up_hill=1;
                        down_hill=0;
                  }
                  else if(podao_y[count_podao-1]<DOWN_HILL_YUZHI)
                  {
                        down_hill=1;
                        up_hill=0;
                        
                  }
                  else
                  {
                        up_hill=0;
                        down_hill=0;
                  }
                  
                  if(up_hill==1)
                  {
                        if(podao_y[count_podao-1]>Get_Gyro_z())
                        {
                              for(uint8 ii=count_podao-1-5;ii<count_podao;ii++)
                              {
                                    sum_podao_y+=podao_y[ii];
                              }
                              if(sum_podao_y>UP_HILL_SUM)
                              {
                                    up_hill=2;
                                    //jiance=3;
                              }
                        }
                  }
                  else if(down_hill==1)
                  {
                        if(podao_y[count_podao-1]<Get_Gyro_z())
                        {
                              for(uint8 ii=count_podao-1-5;ii<count_podao;ii++)
                              {
                                    sum_podao_y+=podao_y[ii];
                              }
                              if(sum_podao_y<DOWN_HILL_SUM)
                              {
                                    down_hill=2;
                                    // jiance=6;
                              }
                        }
                  }
            }   
            
            if(( down_hill==2) && up_flag)
            {
                  up_flag=0;
                  down_flag=1;
                  Bee_flag=1;
                 po_loop_able=0;
            }   
            if(up_enable_flag && (up_hill==2) )
            {
                  up_flag=1;
                  down_flag=0;
                  Bee_flag=1;
                  po_loop_able=0;
                  up_enable_flag=0;
                  po_cnt++;
            }
      }
      if(up_flag && !speedout_val1_flag)
      {
            speedout_val1=speedout_val;
            speedout_val1_flag=1;
            speedout_val2_flag=1;
      }
      if(abs(speedout_val-speedout_val1)>=1.5*10000 && speedout_val2_flag)
      {
            up_flag=0;
            down_flag=0; 
            speedout_val2_flag=0;
            po_loop_able=1;
            Bee_flag=1;
      }
      if(abs(speedout_val-speedout_val1)>=4*10000 && speedout_val1_flag)
      {
            up_enable_flag=1;
            speedout_val1=0;
            speedout_val1_flag=0;
      }
      
//        podao_bo[0]=sum_podao_y;	
//        podao_bo[1]=tuoluoyi_y;	 
//        podao_bo[2]=up_flag*500;
//        podao_bo[3]=down_flag*500;
//        vcan_sendware(&podao_bo,(uint32)(sizeof(podao_bo)));
      //printf("%d %d\n",up_flag,down_flag);
      
}