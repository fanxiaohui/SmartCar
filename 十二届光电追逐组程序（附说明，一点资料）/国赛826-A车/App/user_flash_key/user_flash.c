#include "common.h"
#include "include.h"
/***********************************************************************************************************************************/
uint32 Ajizhun_step[PARA_NUM];
uint32 AJIZHUN_ROW[PARA_NUM];            //中心权值行
uint32 AJIZHUN_UP[PARA_NUM];     //计算偏差的中线截取部分的上边界26
uint32 AJIZHUN_DOWN[PARA_NUM];         //计算偏差的中线截取部分的下边界35
double Aservo_step[PARA_NUM];  
double Aservo_a[PARA_NUM];        //满足二次函数
double Aservo_b[PARA_NUM];   
double Aservo_c[PARA_NUM];    
double Aservo_d[PARA_NUM]; 
double Aservo_e[PARA_NUM];    
double Aservo_f[PARA_NUM]; 
double Amotor_step[PARA_NUM];  
double Amotor_a[PARA_NUM];        //满足二次函数
double Amotor_b[PARA_NUM];   
double Amotor_c[PARA_NUM];    
double Amotor_d[PARA_NUM]; 
double Amotor_e[PARA_NUM];    
double Amotor_f[PARA_NUM]; 
 
int16  Aspeed_step[PARA_NUM];
int16  Aspeed_minl[PARA_NUM];
int16  Aspeed_maxl[PARA_NUM];
int16  Aspeed_minr[PARA_NUM];
int16  Aspeed_maxr[PARA_NUM];
int16  Aspeed_s[PARA_NUM];
int16  Aspeed_z[PARA_NUM];
int16 Aspeed_step2[PARA_NUM];
int16 Aspeed_poa[PARA_NUM];
int16 Aspeed_pob[PARA_NUM];
int16 Aspeed_za[PARA_NUM];
int16 Aspeed_zb[PARA_NUM];
int16 Aspeed_ha[PARA_NUM];
int16 Aspeed_hb[PARA_NUM];

double Adistance_step[PARA_NUM];  
double Adistance_a[PARA_NUM];        //满足二次函数
double Adistance_b[PARA_NUM];   
double Adistance_c[PARA_NUM];    
double Adistance_d[PARA_NUM]; 
double Adistance_e[PARA_NUM];    
double Adistance_f[PARA_NUM]; 
double Adistance2_step[PARA_NUM];  
double Adistance2_a[PARA_NUM];        //满足二次函数
double Adistance2_b[PARA_NUM];   
double Adistance2_c[PARA_NUM];    
double Adistance2_d[PARA_NUM]; 
double Adistance2_e[PARA_NUM];    
double Adistance2_f[PARA_NUM];
int16 Acnt_step[PARA_NUM];  
int16 Acnt_a[PARA_NUM];        //满足二次函数
int16 Acnt_b[PARA_NUM];   
int16 Acnt_c[PARA_NUM];    
int16 Acnt_d[PARA_NUM]; 
int16 Acnt_e[PARA_NUM];    
int16 Acnt_f[PARA_NUM];
double Aother1_step[PARA_NUM];  
double Aother1_a[PARA_NUM];        //满足二次函数
double Aother1_b[PARA_NUM];   
double Aother1_c[PARA_NUM];    
double Aother1_d[PARA_NUM]; 
double Aother1_e[PARA_NUM];    
double Aother1_f[PARA_NUM]; 
double Aother2_step[PARA_NUM];  
double Aother2_a[PARA_NUM];        //满足二次函数
double Aother2_b[PARA_NUM];   
double Aother2_c[PARA_NUM];    
double Aother2_d[PARA_NUM]; 
double Aother2_e[PARA_NUM];    
double Aother2_f[PARA_NUM]; 
double Aother3_step[PARA_NUM];  
double Aother3_a[PARA_NUM];        //满足二次函数
double Aother3_b[PARA_NUM];   
double Aother3_c[PARA_NUM];    
double Aother3_d[PARA_NUM]; 
double Aother3_e[PARA_NUM];    
double Aother3_f[PARA_NUM]; 
double Aother4_step[PARA_NUM];  
double Aother4_a[PARA_NUM];        //满足二次函数
double Aother4_b[PARA_NUM];   
double Aother4_c[PARA_NUM];    
double Aother4_d[PARA_NUM]; 
double Aother4_e[PARA_NUM];    
double Aother4_f[PARA_NUM]; 
double Aother5_step[PARA_NUM];  
double Aother5_a[PARA_NUM];        //满足二次函数
double Aother5_b[PARA_NUM];   
double Aother5_c[PARA_NUM];    
double Aother5_d[PARA_NUM]; 
double Aother5_e[PARA_NUM];    
double Aother5_f[PARA_NUM]; 
flash_data_st flash_data;
/***********************************************************************************************************************************/
uint8 read_flash(uint8 sd);
void flash_to_array();
void save_data(uint8 sd);
void erase_flash();
void write_flash(uint8 sd);
void shu_to_array(uint8 sd);
/***********************************************************************************************************************************/
uint8 read_flash(uint8 sd)
{
  uint8 * fp = (uint8 *)FLASH_ADDR;
  uint8 * p = (uint8 *)&flash_data;
  uint32 damaged_data_count=0;
  
  //读取Flash数据
  for(uint32 i=0; i<sizeof(flash_data); i++)
  {
    *(p + i) = *(fp + i);
    if(*(fp + i)==0xff) 
      damaged_data_count++;
  }
  //出错检查
#define A_CHOICE (sd-1)
  if(damaged_data_count < (sizeof(flash_data)>>1))  //如果错误数据在50%以内，则读取成功
  { 
    servo_m_step =flash_data.servo_m_step;
    SERVO_MID =flash_data.SERVO_MID;
    SERVO_L = flash_data.SERVO_L;
    SERVO_R = flash_data.SERVO_R;
    
    jizhun_step =flash_data.Ajizhun_step[A_CHOICE];
    JIZHUN_ROW =flash_data.AJIZHUN_ROW[A_CHOICE];          
    JIZHUN_UP =flash_data.AJIZHUN_UP[A_CHOICE];    
    JIZHUN_DOWN =flash_data.AJIZHUN_DOWN[A_CHOICE];        
    
    //舵机   pd 
    servo_step  = flash_data.Aservo_step[A_CHOICE];        //满足二次函数
    servo_a  = flash_data.Aservo_a[A_CHOICE];        //满足二次函数
    servo_b = flash_data.Aservo_b[A_CHOICE];   
    servo_c = flash_data.Aservo_c[A_CHOICE];    
    servo_d = flash_data.Aservo_d[A_CHOICE]; 
    servo_e = flash_data.Aservo_e[A_CHOICE]; 
    servo_f = flash_data.Aservo_f[A_CHOICE]; 
    
    motor_step  = flash_data.Amotor_step[A_CHOICE];        //满足二次函数
    motor_a  = flash_data.Amotor_a[A_CHOICE];        //满足二次函数
    motor_b = flash_data.Amotor_b[A_CHOICE];   
    motor_c = flash_data.Amotor_c[A_CHOICE];    
    motor_d = flash_data.Amotor_d[A_CHOICE]; 
    motor_e = flash_data.Amotor_e[A_CHOICE]; 
    motor_f = flash_data.Amotor_f[A_CHOICE]; 
    
    
    speed_step = flash_data.Aspeed_step[A_CHOICE];
    speed_minl = flash_data.Aspeed_minl[A_CHOICE];
    speed_maxl = flash_data.Aspeed_maxl[A_CHOICE];
    speed_minr = flash_data.Aspeed_minr[A_CHOICE];
    speed_maxr = flash_data.Aspeed_maxr[A_CHOICE];
    speed_s = flash_data.Aspeed_s[A_CHOICE];
    speed_z = flash_data.Aspeed_z[A_CHOICE];
    
    speed_step2= flash_data.Aspeed_step2[A_CHOICE];
    speed_poa=flash_data.Aspeed_poa[A_CHOICE];
    speed_pob=flash_data.Aspeed_pob[A_CHOICE];
    speed_za=flash_data.Aspeed_za[A_CHOICE];
    speed_zb=flash_data.Aspeed_zb[A_CHOICE];
    speed_ha=flash_data.Aspeed_ha[A_CHOICE];
    speed_hb=flash_data.Aspeed_hb[A_CHOICE];
    
    distance_step  = flash_data.Adistance_step[A_CHOICE];        //满足二次函数
    distance_a  = flash_data.Adistance_a[A_CHOICE];        //满足二次函数
    distance_b = flash_data.Adistance_b[A_CHOICE];   
    distance_c = flash_data.Adistance_c[A_CHOICE];    
    distance_d = flash_data.Adistance_d[A_CHOICE]; 
    distance_e = flash_data.Adistance_e[A_CHOICE]; 
    distance_f = flash_data.Adistance_f[A_CHOICE]; 
    distance2_step  = flash_data.Adistance2_step[A_CHOICE];        //满足二次函数
    distance2_a  = flash_data.Adistance2_a[A_CHOICE];        //满足二次函数
    distance2_b = flash_data.Adistance2_b[A_CHOICE];   
    distance2_c = flash_data.Adistance2_c[A_CHOICE];    
    distance2_d = flash_data.Adistance2_d[A_CHOICE]; 
    distance2_e = flash_data.Adistance2_e[A_CHOICE]; 
    distance2_f = flash_data.Adistance2_f[A_CHOICE];
    cnt_step  = flash_data.Acnt_step[A_CHOICE];        //满足二次函数
    cnt_a  = flash_data.Acnt_a[A_CHOICE];        //满足二次函数
    cnt_b = flash_data.Acnt_b[A_CHOICE];   
    cnt_c = flash_data.Acnt_c[A_CHOICE];    
    cnt_d = flash_data.Acnt_d[A_CHOICE]; 
    cnt_e = flash_data.Acnt_e[A_CHOICE]; 
    cnt_f = flash_data.Acnt_f[A_CHOICE]; 
    other1_step  = flash_data.Aother1_step[A_CHOICE];        //满足二次函数
    other1_a  = flash_data.Aother1_a[A_CHOICE];        //满足二次函数
    other1_b = flash_data.Aother1_b[A_CHOICE];   
    other1_c = flash_data.Aother1_c[A_CHOICE];    
    other1_d = flash_data.Aother1_d[A_CHOICE]; 
    other1_e = flash_data.Aother1_e[A_CHOICE]; 
    other1_f = flash_data.Aother1_f[A_CHOICE];
    other2_step  = flash_data.Aother2_step[A_CHOICE];        //满足二次函数
    other2_a  = flash_data.Aother2_a[A_CHOICE];        //满足二次函数
    other2_b = flash_data.Aother2_b[A_CHOICE];   
    other2_c = flash_data.Aother2_c[A_CHOICE];    
    other2_d = flash_data.Aother2_d[A_CHOICE]; 
    other2_e = flash_data.Aother2_e[A_CHOICE]; 
    other2_f = flash_data.Aother2_f[A_CHOICE];
    other3_step  = flash_data.Aother3_step[A_CHOICE];        //满足二次函数
    other3_a  = flash_data.Aother3_a[A_CHOICE];        //满足二次函数
    other3_b = flash_data.Aother3_b[A_CHOICE];   
    other3_c = flash_data.Aother3_c[A_CHOICE];    
    other3_d = flash_data.Aother3_d[A_CHOICE]; 
    other3_e = flash_data.Aother3_e[A_CHOICE]; 
    other3_f = flash_data.Aother3_f[A_CHOICE];
    other4_step  = flash_data.Aother4_step[A_CHOICE];        //满足二次函数
    other4_a  = flash_data.Aother4_a[A_CHOICE];        //满足二次函数
    other4_b = flash_data.Aother4_b[A_CHOICE];   
    other4_c = flash_data.Aother4_c[A_CHOICE];    
    other4_d = flash_data.Aother4_d[A_CHOICE]; 
    other4_e = flash_data.Aother4_e[A_CHOICE]; 
    other4_f = flash_data.Aother4_f[A_CHOICE];
    other5_step  = flash_data.Aother5_step[A_CHOICE];        //满足二次函数
    other5_a  = flash_data.Aother5_a[A_CHOICE];        //满足二次函数
    other5_b = flash_data.Aother5_b[A_CHOICE];   
    other5_c = flash_data.Aother5_c[A_CHOICE];    
    other5_d = flash_data.Aother5_d[A_CHOICE]; 
    other5_e = flash_data.Aother5_e[A_CHOICE]; 
    other5_f = flash_data.Aother5_f[A_CHOICE];
    flash_to_array();      //将flash 的值 赋予数
    return 1;
    
  } 
  else          //如果错误数据在50%以上，则读取失败
  {
    
    return 0;
    
  }
}
void flash_to_array()
{
  //    flash_data.servo_m_step=servo_m_step;
  //    flash_data.SERVO_MID = SERVO_MID;
  //    flash_data.SERVO_L = SERVO_L;
  //    flash_data.SERVO_R = SERVO_R;
  for(uint8 ii=0;ii<PARA_NUM;ii++)
  {
    Ajizhun_step[ii] =flash_data.Ajizhun_step[ii];
    AJIZHUN_ROW[ii] =flash_data.AJIZHUN_ROW[ii];          
    AJIZHUN_UP[ii] =flash_data.AJIZHUN_UP[ii];    
    AJIZHUN_DOWN[ii] =flash_data.AJIZHUN_DOWN[ii];  
    Aservo_step[ii]= flash_data.Aservo_step[ii];
    Aservo_a[ii]= flash_data.Aservo_a[ii];
    Aservo_b[ii] = flash_data.Aservo_b[ii];
    Aservo_c[ii] =  flash_data.Aservo_c[ii];
    Aservo_d[ii] = flash_data.Aservo_d[ii];
    Aservo_e[ii] =  flash_data.Aservo_e[ii];
    Aservo_f[ii] = flash_data.Aservo_f[ii];
    Amotor_step[ii]= flash_data.Amotor_step[ii];
    Amotor_a[ii]= flash_data.Amotor_a[ii];
    Amotor_b[ii] = flash_data.Amotor_b[ii];
    Amotor_c[ii] =  flash_data.Amotor_c[ii];
    Amotor_d[ii] = flash_data.Amotor_d[ii];
    Amotor_e[ii] =  flash_data.Amotor_e[ii];
    Amotor_f[ii] = flash_data.Amotor_f[ii];
 

    Aspeed_step[ii] = flash_data.Aspeed_step[ii];
    Aspeed_minl[ii] = flash_data.Aspeed_minl[ii];
    Aspeed_maxl[ii] = flash_data.Aspeed_maxl[ii];
    Aspeed_minr[ii] = flash_data.Aspeed_minr[ii];
    Aspeed_maxr[ii] = flash_data.Aspeed_maxr[ii];
    Aspeed_s[ii] = flash_data.Aspeed_s[ii];
    Aspeed_z[ii] = flash_data.Aspeed_z[ii];
    
     Aspeed_step2[ii] = flash_data.Aspeed_step2[ii];
    Aspeed_za[ii] = flash_data.Aspeed_za[ii];
    Aspeed_zb[ii] = flash_data.Aspeed_zb[ii];
    Aspeed_poa[ii] = flash_data.Aspeed_poa[ii];
    Aspeed_pob[ii] = flash_data.Aspeed_pob[ii];
    Aspeed_ha[ii] = flash_data.Aspeed_ha[ii];
    Aspeed_hb[ii] = flash_data.Aspeed_hb[ii];
    
    Adistance_step[ii]= flash_data.Adistance_step[ii];
    Adistance_a[ii]= flash_data.Adistance_a[ii];
    Adistance_b[ii] = flash_data.Adistance_b[ii];
    Adistance_c[ii] =  flash_data.Adistance_c[ii];
    Adistance_d[ii] = flash_data.Adistance_d[ii];
    Adistance_e[ii] =  flash_data.Adistance_e[ii];
    Adistance_f[ii] = flash_data.Adistance_f[ii];
    Adistance2_step[ii]= flash_data.Adistance2_step[ii];
    Adistance2_a[ii]= flash_data.Adistance2_a[ii];
    Adistance2_b[ii] = flash_data.Adistance2_b[ii];
    Adistance2_c[ii] =  flash_data.Adistance2_c[ii];
    Adistance2_d[ii] = flash_data.Adistance2_d[ii];
    Adistance2_e[ii] =  flash_data.Adistance2_e[ii];
    Adistance2_f[ii] = flash_data.Adistance2_f[ii];
    Acnt_step[ii]= flash_data.Acnt_step[ii];
    Acnt_a[ii]= flash_data.Acnt_a[ii];
    Acnt_b[ii] = flash_data.Acnt_b[ii];
    Acnt_c[ii] =  flash_data.Acnt_c[ii];
    Acnt_d[ii] = flash_data.Acnt_d[ii];
    Acnt_e[ii] =  flash_data.Acnt_e[ii];
    Acnt_f[ii] = flash_data.Acnt_f[ii];
    Aother1_step[ii]= flash_data.Aother1_step[ii];
    Aother1_a[ii]= flash_data.Aother1_a[ii];
    Aother1_b[ii] = flash_data.Aother1_b[ii];
    Aother1_c[ii] =  flash_data.Aother1_c[ii];
    Aother1_d[ii] = flash_data.Aother1_d[ii];
    Aother1_e[ii] =  flash_data.Aother1_e[ii];
    Aother1_f[ii] = flash_data.Aother1_f[ii];
    Aother2_step[ii]= flash_data.Aother2_step[ii];
    Aother2_a[ii]= flash_data.Aother2_a[ii];
    Aother2_b[ii] = flash_data.Aother2_b[ii];
    Aother2_c[ii] =  flash_data.Aother2_c[ii];
    Aother2_d[ii] = flash_data.Aother2_d[ii];
    Aother2_e[ii] =  flash_data.Aother2_e[ii];
    Aother2_f[ii] = flash_data.Aother2_f[ii];
    Aother3_step[ii]= flash_data.Aother3_step[ii];
    Aother3_a[ii]= flash_data.Aother3_a[ii];
    Aother3_b[ii] = flash_data.Aother3_b[ii];
    Aother3_c[ii] =  flash_data.Aother3_c[ii];
    Aother3_d[ii] = flash_data.Aother3_d[ii];
    Aother3_e[ii] =  flash_data.Aother3_e[ii];
    Aother3_f[ii] = flash_data.Aother3_f[ii];
    Aother4_step[ii]= flash_data.Aother4_step[ii];
    Aother4_a[ii]= flash_data.Aother4_a[ii];
    Aother4_b[ii] = flash_data.Aother4_b[ii];
    Aother4_c[ii] =  flash_data.Aother4_c[ii];
    Aother4_d[ii] = flash_data.Aother4_d[ii];
    Aother4_e[ii] =  flash_data.Aother4_e[ii];
    Aother4_f[ii] = flash_data.Aother4_f[ii];
    Aother5_step[ii]= flash_data.Aother5_step[ii];
    Aother5_a[ii]= flash_data.Aother5_a[ii];
    Aother5_b[ii] = flash_data.Aother5_b[ii];
    Aother5_c[ii] =  flash_data.Aother5_c[ii];
    Aother5_d[ii] = flash_data.Aother5_d[ii];
    Aother5_e[ii] =  flash_data.Aother5_e[ii];
    Aother5_f[ii] = flash_data.Aother5_f[ii];
   
  }
}
void save_data(uint8 sd)
{
  erase_flash();
  write_flash(sd);
}
void erase_flash()
{
  
  DisableInterrupts;
  flash_erase_sector(FLASH_SECTOR);                                     //擦扇区
  //EnableInterrupts;
  
}
void write_flash(uint8 sd)
{
  //更新结构体
  shu_to_array(sd);
  DisableInterrupts;
  flash_data.servo_m_step=servo_m_step;
  flash_data.SERVO_MID = SERVO_MID;
  flash_data.SERVO_L  = SERVO_L;
  flash_data.SERVO_R = SERVO_R; 
  
  for(uint8 ii=0;ii<PARA_NUM;ii++)
  {
    flash_data.Ajizhun_step[ii]=Ajizhun_step[ii];
    flash_data.AJIZHUN_ROW[ii]=AJIZHUN_ROW[ii];          
    flash_data.AJIZHUN_UP[ii]= AJIZHUN_UP[ii];    
    flash_data.AJIZHUN_DOWN[ii]=AJIZHUN_DOWN[ii]; 
    
    flash_data.Aservo_step[ii] = Aservo_step[ii];
    flash_data.Aservo_a[ii] = Aservo_a[ii];
    flash_data.Aservo_b[ii] = Aservo_b[ii];
    flash_data.Aservo_c[ii] =  Aservo_c[ii];
    flash_data.Aservo_d[ii] =  Aservo_d[ii];
    flash_data.Aservo_e[ii] =  Aservo_e[ii];
    flash_data.Aservo_f[ii] =  Aservo_f[ii];
    
    flash_data.Amotor_step[ii] = Amotor_step[ii];
    flash_data.Amotor_a[ii] = Amotor_a[ii];
    flash_data.Amotor_b[ii] = Amotor_b[ii];
    flash_data.Amotor_c[ii] =  Amotor_c[ii];
    flash_data.Amotor_d[ii] =  Amotor_d[ii];
    flash_data.Amotor_e[ii] =  Amotor_e[ii];
    flash_data.Amotor_f[ii] =  Amotor_f[ii];
    
 
    
    flash_data.Aspeed_step[ii] =  Aspeed_step[ii]; 
    flash_data.Aspeed_minl[ii] = Aspeed_minl[ii];
    flash_data.Aspeed_maxl[ii] = Aspeed_maxl[ii];
    flash_data.Aspeed_minr[ii] = Aspeed_minr[ii];
    flash_data.Aspeed_maxr[ii] = Aspeed_maxr[ii];
    flash_data.Aspeed_z[ii] = Aspeed_z[ii];
    flash_data.Aspeed_s[ii] = Aspeed_s[ii];
    
    flash_data.Aspeed_step2[A_CHOICE]=speed_step2;
    flash_data.Aspeed_poa[A_CHOICE]=speed_poa;
    flash_data.Aspeed_pob[A_CHOICE]=speed_pob;
    flash_data.Aspeed_za[A_CHOICE]=speed_za;
    flash_data.Aspeed_zb[A_CHOICE]=speed_zb;
    flash_data.Aspeed_ha[A_CHOICE]=speed_ha;
    flash_data.Aspeed_hb[A_CHOICE]=speed_hb;
    
    flash_data.Adistance_step[ii] = Adistance_step[ii];
    flash_data.Adistance_a[ii] = Adistance_a[ii];
    flash_data.Adistance_b[ii] = Adistance_b[ii];
    flash_data.Adistance_c[ii] =  Adistance_c[ii];
    flash_data.Adistance_d[ii] =  Adistance_d[ii];
    flash_data.Adistance_e[ii] =  Adistance_e[ii];
    flash_data.Adistance_f[ii] =  Adistance_f[ii];
    flash_data.Adistance2_step[ii] = Adistance2_step[ii];
    flash_data.Adistance2_a[ii] = Adistance2_a[ii];
    flash_data.Adistance2_b[ii] = Adistance2_b[ii];
    flash_data.Adistance2_c[ii] =  Adistance2_c[ii];
    flash_data.Adistance2_d[ii] =  Adistance2_d[ii];
    flash_data.Adistance2_e[ii] =  Adistance2_e[ii];
    flash_data.Adistance2_f[ii] =  Adistance2_f[ii];
    flash_data.Acnt_step[ii] = Acnt_step[ii];
    flash_data.Acnt_a[ii] = Acnt_a[ii];
    flash_data.Acnt_b[ii] = Acnt_b[ii];
    flash_data.Acnt_c[ii] =  Acnt_c[ii];
    flash_data.Acnt_d[ii] =  Acnt_d[ii];
    flash_data.Acnt_e[ii] =  Acnt_e[ii];
    flash_data.Acnt_f[ii] =  Acnt_f[ii];
    flash_data.Aother1_step[ii] = Aother1_step[ii];
    flash_data.Aother1_a[ii] = Aother1_a[ii];
    flash_data.Aother1_b[ii] = Aother1_b[ii];
    flash_data.Aother1_c[ii] =  Aother1_c[ii];
    flash_data.Aother1_d[ii] =  Aother1_d[ii];
    flash_data.Aother1_e[ii] =  Aother1_e[ii];
    flash_data.Aother1_f[ii] =  Aother1_f[ii];
    flash_data.Aother2_step[ii] = Aother2_step[ii];
    flash_data.Aother2_a[ii] = Aother2_a[ii];
    flash_data.Aother2_b[ii] = Aother2_b[ii];
    flash_data.Aother2_c[ii] =  Aother2_c[ii];
    flash_data.Aother2_d[ii] =  Aother2_d[ii];
    flash_data.Aother2_e[ii] =  Aother2_e[ii];
    flash_data.Aother2_f[ii] =  Aother2_f[ii];
    flash_data.Aother3_step[ii] = Aother3_step[ii];
    flash_data.Aother3_a[ii] = Aother3_a[ii];
    flash_data.Aother3_b[ii] = Aother3_b[ii];
    flash_data.Aother3_c[ii] =  Aother3_c[ii];
    flash_data.Aother3_d[ii] =  Aother3_d[ii];
    flash_data.Aother3_e[ii] =  Aother3_e[ii];
    flash_data.Aother3_f[ii] =  Aother3_f[ii];
    flash_data.Aother4_step[ii] = Aother4_step[ii];
    flash_data.Aother4_a[ii] = Aother4_a[ii];
    flash_data.Aother4_b[ii] = Aother4_b[ii];
    flash_data.Aother4_c[ii] =  Aother4_c[ii];
    flash_data.Aother4_d[ii] =  Aother4_d[ii];
    flash_data.Aother4_e[ii] =  Aother4_e[ii];
    flash_data.Aother4_f[ii] =  Aother4_f[ii];
    flash_data.Aother5_step[ii] = Aother5_step[ii];
    flash_data.Aother5_a[ii] = Aother5_a[ii];
    flash_data.Aother5_b[ii] = Aother5_b[ii];
    flash_data.Aother5_c[ii] =  Aother5_c[ii];
    flash_data.Aother5_d[ii] =  Aother5_d[ii];
    flash_data.Aother5_e[ii] =  Aother5_e[ii];
    flash_data.Aother5_f[ii] =  Aother5_f[ii];
  }     
  flash_write_buf(FLASH_SECTOR,0,sizeof(flash_data),(uint8*)&flash_data);    //写扇区
  //EnableInterrupts;
}
void shu_to_array(uint8 sd)
{
#define A_CHOICE (sd-1) 
  //调整参数
  Ajizhun_step[A_CHOICE]= jizhun_step;
  AJIZHUN_ROW[A_CHOICE]=JIZHUN_ROW;          
  AJIZHUN_UP[A_CHOICE]=JIZHUN_UP;    
  AJIZHUN_DOWN[A_CHOICE]=JIZHUN_DOWN;   
  Aservo_step[A_CHOICE]=servo_step;
  Aservo_a[A_CHOICE]=servo_a;
  Aservo_b[A_CHOICE]=servo_b;
  Aservo_c[A_CHOICE]=servo_c;
  Aservo_d[A_CHOICE]=servo_d;
  Aservo_e[A_CHOICE]=servo_e;
  Aservo_f[A_CHOICE]=servo_f;
  Amotor_step[A_CHOICE]=motor_step;
  Amotor_a[A_CHOICE]=motor_a;
  Amotor_b[A_CHOICE]=motor_b;
  Amotor_c[A_CHOICE]=motor_c;
  Amotor_d[A_CHOICE]=motor_d;
  Amotor_e[A_CHOICE]=motor_e;
  Amotor_f[A_CHOICE]=motor_f;
 
  
  Aspeed_step[A_CHOICE] = speed_step;
  Aspeed_minl[A_CHOICE] = speed_minl;
  Aspeed_maxl[A_CHOICE] = speed_maxl;
  Aspeed_minr[A_CHOICE] = speed_minr;
  Aspeed_maxr[A_CHOICE] = speed_maxr;
  Aspeed_s[A_CHOICE] = speed_s;
  Aspeed_z[A_CHOICE] = speed_z;
  
  Aspeed_step2[A_CHOICE]=speed_step2;
  Aspeed_poa[A_CHOICE]=speed_poa;
  Aspeed_pob[A_CHOICE]=speed_pob;
  Aspeed_za[A_CHOICE]=speed_za;
  Aspeed_zb[A_CHOICE]=speed_zb;
  Aspeed_ha[A_CHOICE]=speed_ha;
  Aspeed_hb[A_CHOICE]=speed_hb;
  
  Adistance_step[A_CHOICE]=distance_step;
  Adistance_a[A_CHOICE]=distance_a;
  Adistance_b[A_CHOICE]=distance_b;
  Adistance_c[A_CHOICE]=distance_c;
  Adistance_d[A_CHOICE]=distance_d;
  Adistance_e[A_CHOICE]=distance_e;
  Adistance_f[A_CHOICE]=distance_f;
  Adistance2_step[A_CHOICE]=distance2_step;
  Adistance2_a[A_CHOICE]=distance2_a;
  Adistance2_b[A_CHOICE]=distance2_b;
  Adistance2_c[A_CHOICE]=distance2_c;
  Adistance2_d[A_CHOICE]=distance2_d;
  Adistance2_e[A_CHOICE]=distance2_e;
  Adistance2_f[A_CHOICE]=distance2_f;
  Acnt_step[A_CHOICE]=cnt_step;
  Acnt_a[A_CHOICE]=cnt_a;
  Acnt_b[A_CHOICE]=cnt_b;
  Acnt_c[A_CHOICE]=cnt_c;
  Acnt_d[A_CHOICE]=cnt_d;
  Acnt_e[A_CHOICE]=cnt_e;
  Acnt_f[A_CHOICE]=cnt_f;
  Aother1_step[A_CHOICE]=other1_step;
  Aother1_a[A_CHOICE]=other1_a;
  Aother1_b[A_CHOICE]=other1_b;
  Aother1_c[A_CHOICE]=other1_c;
  Aother1_d[A_CHOICE]=other1_d;
  Aother1_e[A_CHOICE]=other1_e;
  Aother1_f[A_CHOICE]=other1_f;
  Aother2_step[A_CHOICE]=other2_step;
  Aother2_a[A_CHOICE]=other2_a;
  Aother2_b[A_CHOICE]=other2_b;
  Aother2_c[A_CHOICE]=other2_c;
  Aother2_d[A_CHOICE]=other2_d;
  Aother2_e[A_CHOICE]=other2_e;
  Aother2_f[A_CHOICE]=other2_f;
  Aother3_step[A_CHOICE]=other3_step;
  Aother3_a[A_CHOICE]=other3_a;
  Aother3_b[A_CHOICE]=other3_b;
  Aother3_c[A_CHOICE]=other3_c;
  Aother3_d[A_CHOICE]=other3_d;
  Aother3_e[A_CHOICE]=other3_e;
  Aother3_f[A_CHOICE]=other3_f;
  Aother4_step[A_CHOICE]=other4_step;
  Aother4_a[A_CHOICE]=other4_a;
  Aother4_b[A_CHOICE]=other4_b;
  Aother4_c[A_CHOICE]=other4_c;
  Aother4_d[A_CHOICE]=other4_d;
  Aother4_e[A_CHOICE]=other4_e;
  Aother4_f[A_CHOICE]=other4_f;
  Aother5_step[A_CHOICE]=other5_step;
  Aother5_a[A_CHOICE]=other5_a;
  Aother5_b[A_CHOICE]=other5_b;
  Aother5_c[A_CHOICE]=other5_c;
  Aother5_d[A_CHOICE]=other5_d;
  Aother5_e[A_CHOICE]=other5_e;
  Aother5_f[A_CHOICE]=other5_f;
}

//v