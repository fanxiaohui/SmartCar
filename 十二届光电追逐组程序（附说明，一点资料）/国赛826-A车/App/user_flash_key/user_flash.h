#ifndef _USER_FLASH_
#define _USER_FLASH_
/***********************************************************************************************************************************/
#define FLASH_SECTOR   (127)
#define FLASH_ADDR       (FLASH_SECTOR*4096)//  (FLASH_SECTOR*4096)
#define PARA_NUM 3
/***********************************************************************************************************************************/
typedef struct 
{
  uint32 servo_m_step;
  uint32  SERVO_MID;
  uint32  SERVO_L;
  uint32  SERVO_R;
  
  uint32 Ajizhun_step[PARA_NUM];
  uint32 AJIZHUN_ROW[PARA_NUM];            //中心权值行
  uint32 AJIZHUN_UP[PARA_NUM];     //计算偏差的中线截取部分的上边界26
  uint32 AJIZHUN_DOWN[PARA_NUM];         //计算偏差的中线截取部分的下边界35
  
  //舵机   pd 
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
  
}  flash_data_st;

/***********************************************************************************************************************************/
extern uint8 read_flash(uint8 sd);      //读取数据
extern  uint32 Ajizhun_step[PARA_NUM];
extern  uint32 AJIZHUN_ROW[PARA_NUM];            //中心权值行
extern  uint32 AJIZHUN_UP[PARA_NUM];     //计算偏差的中线截取部分的上边界26
extern  uint32 AJIZHUN_DOWN[PARA_NUM];         //计算偏差的中线截取部分的下边界35
extern double Aservo_step[PARA_NUM]; 
extern double Aservo_a[PARA_NUM];        //满足二次函数
extern double Aservo_b[PARA_NUM];   
extern double Aservo_c[PARA_NUM];    
extern double Aservo_d[PARA_NUM]; 
extern  double Aservo_e[PARA_NUM];    
extern  double Aservo_f[PARA_NUM]; 
extern double Amotor_step[PARA_NUM]; 
extern double Amotor_a[PARA_NUM];        //满足二次函数
extern double Amotor_b[PARA_NUM];   
extern double Amotor_c[PARA_NUM];    
extern double Amotor_d[PARA_NUM]; 
extern  double Amotor_e[PARA_NUM];    
extern  double Amotor_f[PARA_NUM];
extern    int16  Aspeed_step[PARA_NUM];
extern    int16  Aspeed_minl[PARA_NUM];
extern    int16  Aspeed_maxl[PARA_NUM];
extern    int16  Aspeed_minr[PARA_NUM];
extern    int16  Aspeed_maxr[PARA_NUM];
extern    int16  Aspeed_s[PARA_NUM];
extern    int16  Aspeed_z[PARA_NUM];
extern    int16 Aspeed_step2[PARA_NUM];
extern    int16 Aspeed_poa[PARA_NUM];
extern    int16 Aspeed_pob[PARA_NUM];
extern    int16 Aspeed_za[PARA_NUM];
extern    int16 Aspeed_zb[PARA_NUM];
extern    int16 Aspeed_ha[PARA_NUM];
extern    int16 Aspeed_hb[PARA_NUM];
extern  int16  Aspeed_z[PARA_NUM];
extern double Adistance_step[PARA_NUM]; 
extern double Adistance_a[PARA_NUM];        //满足二次函数
extern double Adistance_b[PARA_NUM];   
extern double Adistance_c[PARA_NUM];    
extern double Adistance_d[PARA_NUM]; 
extern  double Adistance_e[PARA_NUM];    
extern  double Adistance_f[PARA_NUM];
extern double Adistance2_step[PARA_NUM]; 
extern double Adistance2_a[PARA_NUM];        //满足二次函数
extern double Adistance2_b[PARA_NUM];   
extern double Adistance2_c[PARA_NUM];    
extern double Adistance2_d[PARA_NUM]; 
extern  double Adistance2_e[PARA_NUM];    
extern  double Adistance2_f[PARA_NUM];
extern int16 Acnt_step[PARA_NUM]; 
extern int16 Acnt_a[PARA_NUM];        //满足二次函数
extern int16 Acnt_b[PARA_NUM];   
extern int16 Acnt_c[PARA_NUM];    
extern int16 Acnt_d[PARA_NUM]; 
extern  int16 Acnt_e[PARA_NUM];    
extern  int16 Acnt_f[PARA_NUM];
extern double Aother1_step[PARA_NUM]; 
extern double Aother1_a[PARA_NUM];        //满足二次函数
extern double Aother1_b[PARA_NUM];   
extern double Aother1_c[PARA_NUM];    
extern double Aother1_d[PARA_NUM]; 
extern  double Aother1_e[PARA_NUM];    
extern  double Aother1_f[PARA_NUM]; 
extern double Aother2_step[PARA_NUM]; 
extern double Aother2_a[PARA_NUM];        //满足二次函数
extern double Aother2_b[PARA_NUM];   
extern double Aother2_c[PARA_NUM];    
extern double Aother2_d[PARA_NUM]; 
extern  double Aother2_e[PARA_NUM];    
extern  double Aother2_f[PARA_NUM]; 
extern double Aother3_step[PARA_NUM]; 
extern double Aother3_a[PARA_NUM];        //满足二次函数
extern double Aother3_b[PARA_NUM];   
extern double Aother3_c[PARA_NUM];    
extern double Aother3_d[PARA_NUM]; 
extern  double Aother3_e[PARA_NUM];    
extern  double Aother3_f[PARA_NUM]; 
extern double Aother4_step[PARA_NUM]; 
extern double Aother4_a[PARA_NUM];        //满足二次函数
extern double Aother4_b[PARA_NUM];   
extern double Aother4_c[PARA_NUM];    
extern double Aother4_d[PARA_NUM]; 
extern  double Aother4_e[PARA_NUM];    
extern  double Aother4_f[PARA_NUM]; 
extern double Aother5_step[PARA_NUM]; 
extern double Aother5_a[PARA_NUM];        //满足二次函数
extern double Aother5_b[PARA_NUM];   
extern double Aother5_c[PARA_NUM];    
extern double Aother5_d[PARA_NUM]; 
extern  double Aother5_e[PARA_NUM];    
extern  double Aother5_f[PARA_NUM]; 


#endif