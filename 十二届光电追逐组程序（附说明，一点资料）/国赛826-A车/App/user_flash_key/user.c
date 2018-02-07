#include "common.h"
#include "include.h"

/***********************************************************************************************************************************/

uint8  page_state;
#define FIRST_PAGE   (1u)
#define SERVO_PAGE   (2u)
#define SERVO_M_PAGE   (3u)  //电机
#define V_SET_PAGE    (4u)
#define JIZHUN_PAGE    (5u)
#define MOTOR_PAGE   (6u)

#define SECOND_PAGE     (1u)
#define CNT_PAGE       (2u)
#define V_SET2_PAGE    (3u)
#define DISTANCE_PAGE   (4u)
#define DISTANCE2_PAGE   (5u)

#define THIRD_PAGE     (1u)
#define OTHER1_PAGE       (2u)
#define OTHER2_PAGE    (3u)
#define OTHER3_PAGE   (4u)
#define OTHER4_PAGE   (5u)
#define OTHER5_PAGE   (6u)

/***********************************************************************************************************************************/

#define MOTOR_ADJUST     (1u)
#define MOTOR_UN_ADJUST  (0u)

#define SERVO_ADJUST     (1u)
#define SERVO_UN_ADJUST  (0u)



#define SERVO_M_ADJUST     (1u)
#define SERVO_M_UN_ADJUST  (0u)

#define V_SET_ADJUST     (1u)
#define V_SET_UN_ADJUST  (0u)

#define JIZHUN_ADJUST     (1u)
#define JIZHUN_UN_ADJUST  (0u)

#define V_SET2_ADJUST     (1u)
#define V_SET2_UN_ADJUST  (0u)

#define DISTANCE_ADJUST     (1u)
#define DISTANCE_UN_ADJUST  (0u)

#define DISTANCE2_ADJUST     (1u)
#define DISTANCE2_UN_ADJUST  (0u)

#define CNT_ADJUST     (1u)
#define CNT_UN_ADJUST  (0u)

#define OTHER1_ADJUST     (1u)
#define OTHER1_UN_ADJUST  (0u)
#define OTHER2_ADJUST     (1u)
#define OTHER2_UN_ADJUST  (0u)
#define OTHER3_ADJUST     (1u)
#define OTHER3_UN_ADJUST  (0u)
#define OTHER4_ADJUST     (1u)
#define OTHER4_UN_ADJUST  (0u)
#define OTHER5_ADJUST     (1u)
#define OTHER5_UN_ADJUST  (0u)

/***********************************************************************************************************************************/
double other1_step=1;
double other1_a=0;
double other1_b=0;
double other1_c=0;
double other1_d=0;
double other1_e=0;
double other1_f=0;
double other2_step=1;
double other2_a=0;
double other2_b=0;
double other2_c=0;
double other2_d=0;
double other2_e=0;
double other2_f=0;
double other3_step=1;
double other3_a=0;
double other3_b=0;
double other3_c=0;
double other3_d=0;
double other3_e=0;
double other3_f=0;
double other4_step=1;
double other4_a=0;
double other4_b=0;
double other4_c=0;
double other4_d=0;
double other4_e=0;
double other4_f=0;
double other5_step=1;
double other5_a=0;
double other5_b=0;
double other5_c=0;//超车完成起步距离
double other5_d=0;
double other5_e=0;
double other5_f=0;
/***********************************************************************************************************************************/
uint8 motor_state=0;
uint8 servo_state=0;
uint8 servo_m_state=0;
uint8 v_set_state=0;
uint8 jizhun_state=0;

uint8 v_set2_state=0;
uint8 distance_state=0;
uint8 distance2_state=0;
uint8 cnt_state=0;
uint8 other1_state=0;
uint8 other2_state=0;
uint8 other3_state=0;
uint8 other4_state=0;
uint8 other5_state=0;
/***********************************************************************************************************************************/
#define pagen_num    4

//FIRST_PAGE   第一个 页面
#define first_page_num 6
#define second_page_num 5
#define third_page_num 6

//SERVO_PAGE   舵机调节 页面
#define servo_page_num 7
uint8 *servo_sg[]  ={"servo:","step:","a:","b:","c:","d:","e:","f:"};
uint8 servo_msg_x[]={0, 2, 2,  2,  2 ,2,2,2};
uint8 servo_msg_y[] ={0, 1, 2,  3,  4,5,6,7};   
uint8 servo_pid_value_x[]={9, 9, 9, 9,9,9,9};
uint8 servo_pid_value_y[]={1, 2, 3, 4,5,6,7};
uint8 servo_up_down[][2]={{0,1},{0,2} ,{0,3} ,{0,4},{0,5},{0,6},{0,7}};
uint8 servo_l[][2]= {{0,1},{0,2} ,{0,3} ,{0,4},{0,5},{0,6},{0,7}};
uint8 servo_r[][2]={ {7,1},{7,2} ,{7,3} ,{7,4},{7,5},{7,6},{7,7}};  
uint8 pid_value_x[]={9,         9,       9,      9,  9, 9};
uint8 pid_value_y[]={1,         2,       3,      4,  5, 6};
double servo_pid_value[7]; 
double servo_step_up_down[]={0,0.01,0.1,1}; 

//motor_PAGE   电机调节 页面
#define motor_page_num 7
uint8 *motor_sg[]  ={"motor:","step:","a:","b:","c:","d:","e:","f:"};
uint8 motor_msg_x[]={0, 2, 2,  2,  2 ,2,2,2};
uint8 motor_msg_y[] ={0, 1, 2,  3,  4,5,6,7};   
uint8 motor_pid_value_x[]={9, 9, 9, 9,9,9,9};
uint8 motor_pid_value_y[]={1, 2, 3, 4,5,6,7};
uint8 motor_up_down[][2]={{0,1},{0,2} ,{0,3} ,{0,4},{0,5},{0,6},{0,7}};
uint8 motor_l[][2]= {{0,1},{0,2} ,{0,3} ,{0,4},{0,5},{0,6},{0,7}};
uint8 motor_r[][2]={ {7,1},{7,2} ,{7,3} ,{7,4},{7,5},{7,6},{7,7}}; 
double motor_pid_value[7]; 
double motor_step_up_down[]={0,0.01,0.1,1};



//中间  设定 窗口
#define servo_m_page_num 4
uint8 *servo_m_sg[]={"servo_m:","step","left","mid","right"};
uint8 servo_m_msg_x[]={0,         2,    2,  2 ,2 };
uint8 servo_m_msg_y[]={0,         1,    2,  3 , 4};   

uint16 servo_m_value[4]; 
uint8 servo_m_value_x[]={9,         9,       9,9};
uint8 servo_m_value_y[]={1,         2,       3,4};
uint8 servo_m_up_down[][2]={{0,1},{0,2} ,{0,3},{0,4}};
uint8 servo_m_step_up_down[]={0,1,2,10,20,100};

uint8 v_set_l[][2]= {{0,1},{0,2} ,{0,3} ,{0,4} ,{0,5},{0,6},{0,7}};
uint8 v_set_r[][2]={  {7,1},{7,2} ,{7,3} ,{7,4} ,{7,5},{7,6},{7,7}}; 
uint8 v_set_value_x[]={    9,     9,       9  ,9,9,9,9};
uint8 v_set_value_y[]={ 1,     2,       3,  4,5,6,7};

//V_SET_PAGE   速度设定  页面
#define v_set_page_num 7
uint8 *v_set_sg[]={"v_set:","step","minl","maxl","minr","maxr","s","z"};
uint8 v_set_msg_x[]={0,         2,    2,  2 ,2,2,2,2};
uint8 v_set_msg_y[]={0,         1,    2,  3 ,4,5,6,7};   

uint16 v_set_value[7]; 
uint8 v_set_up_down[][2]={{0,1},{0,2} ,{0,3},{0,4},{0,5},{0,6},{0,7}};
uint8 v_set_step_up_down[]={0,1,2,10,20,100}; 

//基准设定界面
//中间  设定 窗口
//中间  设定 窗口
#define jizhun_page_num 4
uint8 *jizhun_sg[]={"jizhun:","step","row","up","down"};
uint8 jizhun_msg_x[]={0,         2,    2,  2 ,2 };
uint8 jizhun_msg_y[]={0,         1,    2,  3 , 4};   

uint16 jizhun_value[4]; 
uint8 jizhun_value_x[]={9,         9,       9,9};
uint8 jizhun_value_y[]={1,         2,       3,4};
uint8 jizhun_up_down[][2]={{0,1},{0,2} ,{0,3},{0,4}};
uint8 jizhun_step_up_down[]={0,1,2,10,20,100};


/********************************双车参数*********************************************************/
//V_SET_PAGE   速度设定  页面
#define v_set_page_num2 7
uint8 *v_set_sg2[]={"v_set:","step","v_poa","v_pob","v_za","v_zb","v_ha","v_hb"};
uint16 v_set_value2[7]; 

//DISTANCE_PAGE   距离调节 页面
#define distance_page_num 7
uint8 *distance_sg[]  ={"distance:","step:","a:","b:","c:","d:","e:","f:"};
uint8 distance_msg_x[]={0, 2, 2,  2,  2 ,2,2,2};
uint8 distance_msg_y[] ={0, 1, 2,  3,  4,5,6,7};   
uint8 distance_pid_value_x[]={9, 9, 9, 9,9,9,9};
uint8 distance_pid_value_y[]={1, 2, 3, 4,5,6,7};
uint8 distance_up_down[][2]={{0,1},{0,2} ,{0,3} ,{0,4},{0,5},{0,6},{0,7}};
uint8 distance_l[][2]= {{0,1},{0,2} ,{0,3} ,{0,4},{0,5},{0,6},{0,7}};
uint8 distance_r[][2]={ {7,1},{7,2} ,{7,3} ,{7,4},{7,5},{7,6},{7,7}};  
double distance_pid_value[7]; 
double distance_step_up_down[]={0,0.01,0.1,1}; 

//DISTANCE2_PAGE   距离调节 页面
#define distance2_page_num 7
uint8 *distance2_sg[]  ={"distance2:","step:","a:","b:","c:","d:","e:","f:"};
uint8 distance2_msg_x[]={0, 2, 2,  2,  2 ,2,2,2};
uint8 distance2_msg_y[] ={0, 1, 2,  3,  4,5,6,7};   
uint8 distance2_pid_value_x[]={9, 9, 9, 9,9,9,9};
uint8 distance2_pid_value_y[]={1, 2, 3, 4,5,6,7};
uint8 distance2_up_down[][2]={{0,1},{0,2} ,{0,3} ,{0,4},{0,5},{0,6},{0,7}};
uint8 distance2_l[][2]= {{0,1},{0,2} ,{0,3} ,{0,4},{0,5},{0,6},{0,7}};
uint8 distance2_r[][2]={ {7,1},{7,2} ,{7,3} ,{7,4},{7,5},{7,6},{7,7}};  
double distance2_pid_value[7]; 
double distance2_step_up_down[]={0,0.01,0.1,1}; 



//CNT_PAGE   速度设定  页面
#define cnt_page_num 7

uint8 cnt_l[][2]= {{0,1},{0,2} ,{0,3} ,{0,4} ,{0,5},{0,6},{0,7}};
uint8 cnt_r[][2]={  {7,1},{7,2} ,{7,3} ,{7,4} ,{7,5},{7,6},{7,7}}; 
uint8 cnt_value_x[]={    9,     9,       9  ,9,9,9,9};
uint8 cnt_value_y[]={ 1,     2,       3,  4,5,6,7};

uint8 *cnt_sg[]={"cnt:","step","cnt_z","cnt_h","cnt_p","cnt_d","cnt_e","cnt_f"};
uint8 cnt_msg_x[]={0,         2,    2,  2 ,2,2,2,2};
uint8 cnt_msg_y[]={0,         1,    2,  3 ,4,5,6,7};   

uint16 cnt_value[7]; 
uint8 cnt_up_down[][2]={{0,1},{0,2} ,{0,3},{0,4},{0,5},{0,6},{0,7}};
uint8 cnt_step_up_down[]={0,1,2,10,20,100}; 


/********************************其他参数*********************************************************/
#define other1_page_num 7
uint8 *other1_sg[]  ={"other1:","step:","a:","b:","c:","d:","e:","f:"};
uint8 other1_msg_x[]={0, 2, 2,  2,  2 ,2,2,2};
uint8 other1_msg_y[] ={0, 1, 2,  3,  4,5,6,7};   
uint8 other1_pid_value_x[]={9, 9, 9, 9,9,9,9};
uint8 other1_pid_value_y[]={1, 2, 3, 4,5,6,7};
uint8 other1_up_down[][2]={{0,1},{0,2} ,{0,3} ,{0,4},{0,5},{0,6},{0,7}};
uint8 other1_l[][2]= {{0,1},{0,2} ,{0,3} ,{0,4},{0,5},{0,6},{0,7}};
uint8 other1_r[][2]={ {7,1},{7,2} ,{7,3} ,{7,4},{7,5},{7,6},{7,7}};  
double other1_pid_value[7]; 
double other1_step_up_down[]={0,0.01,0.1,1}; 

#define other2_page_num 7
uint8 *other2_sg[]  ={"other2:","step:","a:","b:","c:","d:","e:","f:"};
uint8 other2_msg_x[]={0, 2, 2,  2,  2 ,2,2,2};
uint8 other2_msg_y[] ={0, 1, 2,  3,  4,5,6,7};   
uint8 other2_pid_value_x[]={9, 9, 9, 9,9,9,9};
uint8 other2_pid_value_y[]={1, 2, 3, 4,5,6,7};
uint8 other2_up_down[][2]={{0,1},{0,2} ,{0,3} ,{0,4},{0,5},{0,6},{0,7}};
uint8 other2_l[][2]= {{0,1},{0,2} ,{0,3} ,{0,4},{0,5},{0,6},{0,7}};
uint8 other2_r[][2]={ {7,1},{7,2} ,{7,3} ,{7,4},{7,5},{7,6},{7,7}};  
double other2_pid_value[7]; 
double other2_step_up_down[]={0,0.01,0.1,1}; 

#define other3_page_num 7
uint8 *other3_sg[]  ={"other3:","step:","a:","b:","c:","d:","e:","f:"};
uint8 other3_msg_x[]={0, 2, 2,  2,  2 ,2,2,2};
uint8 other3_msg_y[] ={0, 1, 2,  3,  4,5,6,7};   
uint8 other3_pid_value_x[]={9, 9, 9, 9,9,9,9};
uint8 other3_pid_value_y[]={1, 2, 3, 4,5,6,7};
uint8 other3_up_down[][2]={{0,1},{0,2} ,{0,3} ,{0,4},{0,5},{0,6},{0,7}};
uint8 other3_l[][2]= {{0,1},{0,2} ,{0,3} ,{0,4},{0,5},{0,6},{0,7}};
uint8 other3_r[][2]={ {7,1},{7,2} ,{7,3} ,{7,4},{7,5},{7,6},{7,7}};  
double other3_pid_value[7]; 
double other3_step_up_down[]={0,0.01,0.1,1}; 

#define other4_page_num 7
uint8 *other4_sg[]  ={"other4:","step:","a:","b:","c:","d:","e:","f:"};
uint8 other4_msg_x[]={0, 2, 2,  2,  2 ,2,2,2};
uint8 other4_msg_y[] ={0, 1, 2,  3,  4,5,6,7};   
uint8 other4_pid_value_x[]={9, 9, 9, 9,9,9,9};
uint8 other4_pid_value_y[]={1, 2, 3, 4,5,6,7};
uint8 other4_up_down[][2]={{0,1},{0,2} ,{0,3} ,{0,4},{0,5},{0,6},{0,7}};
uint8 other4_l[][2]= {{0,1},{0,2} ,{0,3} ,{0,4},{0,5},{0,6},{0,7}};
uint8 other4_r[][2]={ {7,1},{7,2} ,{7,3} ,{7,4},{7,5},{7,6},{7,7}};  
double other4_pid_value[7]; 
double other4_step_up_down[]={0,0.01,0.1,1}; 

#define other5_page_num 7
uint8 *other5_sg[]  ={"other5:","step:","a:","b:","c:","d:","e:","f:"};
uint8 other5_msg_x[]={0, 2, 2,  2,  2 ,2,2,2};
uint8 other5_msg_y[] ={0, 1, 2,  3,  4,5,6,7};   
uint8 other5_pid_value_x[]={9, 9, 9, 9,9,9,9};
uint8 other5_pid_value_y[]={1, 2, 3, 4,5,6,7};
uint8 other5_up_down[][2]={{0,1},{0,2} ,{0,3} ,{0,4},{0,5},{0,6},{0,7}};
uint8 other5_l[][2]= {{0,1},{0,2} ,{0,3} ,{0,4},{0,5},{0,6},{0,7}};
uint8 other5_r[][2]={ {7,1},{7,2} ,{7,3} ,{7,4},{7,5},{7,6},{7,7}};  
double other5_pid_value[7]; 
double other5_step_up_down[]={0,0.01,0.1,1}; 







/***********************************************************************************************************************************/

void user_gpio_init();
void user_init();
void user_menu();
void lcd_double_show(Site_t site,double value);
void pagen_updown(uint8 key);
void pagen_show();
void first_page_show();
void first_page_up_down(uint8 key);
void second_page_show();
void second_page_up_down(uint8 key);
void third_page_show();
void third_page_up_down(uint8 key);
void servo_parameter_show();
void servo_page_up_down(uint8 key);
void servo_arrow_lr(uint8 key);
void servo_pid_adjust(uint8 key);
void motor_parameter_show();
void motor_page_up_down(uint8 key);
void motor_arrow_lr(uint8 key);
void motor_pid_adjust(uint8 key);
void servo_m_parameter_show();
void servo_m_page_up_down(uint8 key);
void servo_m_arrow_lr(uint8 key);
void servo_m_adjust(uint8 key);
void v_set_parameter_show();
void v_set_page_up_down(uint8 key);
void v_set_arrow_lr(uint8 key);
void v_set_adjust(uint8 key);
void jizhun_parameter_show();
void jizhun_page_up_down(uint8 key);
void jizhun_arrow_lr(uint8 key);
void jizhun_adjust(uint8 key);

/***********************双车*****************************/
void v_set_parameter_show2();
void v_set_page_up_down2(uint8 key);
void v_set_arrow_lr2(uint8 key);
void v_set_adjust2(uint8 key);
void distance_parameter_show();
void distance_page_up_down(uint8 key);
void distance_arrow_lr(uint8 key);
void distance_pid_adjust(uint8 key);
void distance2_parameter_show();
void distance2_page_up_down(uint8 key);
void distance2_arrow_lr(uint8 key);
void distance2_pid_adjust(uint8 key);
void cnt_parameter_show();
void cnt_page_up_down(uint8 key);
void cnt_arrow_lr(uint8 key);
void cnt_adjust(uint8 key);

/***********************其他****************************/
void other1_parameter_show();
void other1_page_up_down(uint8 key);
void other1_arrow_lr(uint8 key);
void other1_pid_adjust(uint8 key);
void other2_parameter_show();
void other2_page_up_down(uint8 key);
void other2_arrow_lr(uint8 key);
void other2_pid_adjust(uint8 key);
void other3_parameter_show();
void other3_page_up_down(uint8 key);
void other3_arrow_lr(uint8 key);
void other3_pid_adjust(uint8 key);
void other4_parameter_show();
void other4_page_up_down(uint8 key);
void other4_arrow_lr(uint8 key);
void other4_pid_adjust(uint8 key);
void other5_parameter_show();
void other5_page_up_down(uint8 key);
void other5_arrow_lr(uint8 key);
void other5_pid_adjust(uint8 key);
/***********************************************************************************************************************************/

void user_init()
{
  /*配置外设初始化*/
  user_gpio_init();              //按键系统
  uart_init( VCAN_PORT,115200);  //串口5 初始化函数
  LCD1_init();                   //液晶屏初始化函数
  car_init();                    //配置中断优先级
  ultrasonic_init();             //超声波初始化
  correspond_init();
  servo_init();                  //舵机初始化
  motor_init();                  //电机初始化
  IIC_init();
  InitMPU6050();
}

/***********************************************************************************************************************************/

void user_gpio_init()
{
  gpio_init(PTD9,GPI,0);
  port_init_NoALT(PTD9, PULLUP);
  gpio_init(PTD8,GPI,0);
  port_init_NoALT(PTD8, PULLUP);
  gpio_init(PTD1,GPI,0);
  port_init_NoALT(PTD1, PULLUP);
  gpio_init(PTD0,GPI,0);
  port_init_NoALT(PTD0, PULLUP);
  gpio_init(PTC19,GPI,0);
  port_init_NoALT(PTC19, PULLUP);
  gpio_init(PTC18,GPI,0);
  port_init_NoALT(PTC18, PULLUP);
  gpio_init(PTC17,GPI,0);
  port_init_NoALT(PTC17, PULLUP);
  gpio_init(PTC16,GPI,0);
  port_init_NoALT(PTC16, PULLUP);
  
  gpio_init(PTE24,GPO,0);
  //      port_init_NoALT(PTE24, PULLUP);
  gpio_init(PTE26,GPO,1);
  gpio_init(PTA17,GPO,1);
  
  
  //液晶屏按键  
  gpio_init (PTA24, GPI,0);     //up
  port_init_NoALT(PTA24, PULLUP);         //保持复用不变，仅仅改变配置选项   
  gpio_init (PTA26, GPI,0);
  port_init_NoALT(PTA26, PULLUP);         //保持复用不变，仅仅改变配置选项 
  gpio_init (PTA29, GPI,0);
  port_init_NoALT(PTA29, PULLUP);         //保持复用不变，仅仅改变配置选项 
  gpio_init (PTB8, GPI,0);
  port_init_NoALT(PTB8, PULLUP);         //保持复用不变，仅仅改变配置选项 
  gpio_init (PTB9,  GPI,0);     //middle
  port_init_NoALT(PTB9, PULLUP);         //保持复用不变，仅仅改变配置选项 
  //按键
  gpio_init (PTD15, GPI,0);     //up
  port_init_NoALT(PTD15, PULLUP);         //保持复用不变，仅仅改变配置选项   
  gpio_init (PTD14, GPI,0);
  port_init_NoALT(PTD14, PULLUP);         //保持复用不变，仅仅改变配置选项   
  gpio_init (PTD13, GPI,0);
  port_init_NoALT(PTD13, PULLUP);         //保持复用不变，仅仅改变配置选项   
  gpio_init (PTD12, GPI,0);
  port_init_NoALT(PTD12, PULLUP);         //保持复用不变，仅仅改变配置选项   
  gpio_init (PTD11,  GPI,0);     //middle
  port_init_NoALT(PTD11, PULLUP);         //保持复用不变，仅仅改变配置选项   
  gpio_init (PTD10,  GPI,0);     //middle
  port_init_NoALT(PTD10, PULLUP);         //保持复用不变，仅仅改变配置选项   
  
}

/***********************************************************************************************************************************/

void user_menu()
{
  uint8 sd=v_set_filter();
  uint8 read_ok=read_flash(sd); 
  Site_t sitexy={8*3,128-16*3};
  if(read_ok)
    LCD_str(sitexy,"read ok",WHITE,BLUE);
  else 
    LCD_str(sitexy,"read no",WHITE,BLUE);
  //选择 是否再次 读取！！！
  //再次 等待一个   右边
  uint8 next_key;
  while(1)
  {
    while ((next_key=once_check_key())==0);     //直到有按键按下
    if(next_key==LEFT)
    {
      read_ok=read_flash(sd);
      if(read_ok)
        LCD_str(sitexy,"read ok",WHITE,BLUE);
      else 
        LCD_str(sitexy,"read no",WHITE,BLUE);
    }
    else if(next_key==RIGHT)
    {
      break;
    }
  }
  arrow.x=1;
  arrow.y=1;
  servo_pid_value[0]=servo_step;
  servo_pid_value[1]=servo_a;
  servo_pid_value[2]=servo_b;
  servo_pid_value[3]=servo_c;
  servo_pid_value[4]=servo_d;
  servo_pid_value[5]=servo_e;
  servo_pid_value[6]=servo_f;
  motor_pid_value[0]=motor_step;
  motor_pid_value[1]=motor_a;
  motor_pid_value[2]=motor_b;
  motor_pid_value[3]=motor_c;
  motor_pid_value[4]=motor_d;
  motor_pid_value[5]=motor_e;
  motor_pid_value[6]=motor_f;
  servo_m_value[0]=servo_m_step;
  servo_m_value[1]=SERVO_L;
  servo_m_value[2]=SERVO_MID;
  servo_m_value[3]=SERVO_R;
  v_set_value[0]=speed_step;
  v_set_value[1]=speed_minl;
  v_set_value[2]=speed_maxl;
  v_set_value[3]=speed_minr;
  v_set_value[4]=speed_maxr;
  v_set_value[5]=speed_s;
  v_set_value[6]=speed_z;
  jizhun_value[0]=jizhun_step;
  jizhun_value[1]=JIZHUN_ROW;
  jizhun_value[2]=JIZHUN_UP;
  jizhun_value[3]=JIZHUN_DOWN;
  
  /****************双车***************/
  v_set_value2[0]=speed_step;
  v_set_value2[1]=speed_poa;
  v_set_value2[2]=speed_pob;
  v_set_value2[3]=speed_za;
  v_set_value2[4]=speed_zb;
  v_set_value2[5]=speed_ha;
  v_set_value2[6]=speed_hb;
  distance_pid_value[0]=distance_step;
  distance_pid_value[1]=distance_a;
  distance_pid_value[2]=distance_b;
  distance_pid_value[3]=distance_c;
  distance_pid_value[4]=distance_d;
  distance_pid_value[5]=distance_e;
  distance_pid_value[6]=distance_f;
  distance2_pid_value[0]=distance2_step;
  distance2_pid_value[1]=distance2_a;
  distance2_pid_value[2]=distance2_b;
  distance2_pid_value[3]=distance2_c;
  distance2_pid_value[4]=distance2_d;
  distance2_pid_value[5]=distance2_e;
  distance2_pid_value[6]=distance2_f;
  cnt_value[0]=cnt_step;
  cnt_value[1]=cnt_a;
  cnt_value[2]=cnt_b;
  cnt_value[3]=cnt_c;
  cnt_value[4]=cnt_d;
  cnt_value[5]=cnt_e;
  cnt_value[6]=cnt_f;
  other1_pid_value[0]=other1_step;
  other1_pid_value[1]=other1_a;
  other1_pid_value[2]=other1_b;
  other1_pid_value[3]=other1_c;
  other1_pid_value[4]=other1_d;
  other1_pid_value[5]=other1_e;
  other1_pid_value[6]=other1_f;
  other2_pid_value[0]=other2_step;
  other2_pid_value[1]=other2_a;
  other2_pid_value[2]=other2_b;
  other2_pid_value[3]=other2_c;
  other2_pid_value[4]=other2_d;
  other2_pid_value[5]=other2_e;
  other2_pid_value[6]=other2_f;
  other3_pid_value[0]=other3_step;
  other3_pid_value[1]=other3_a;
  other3_pid_value[2]=other3_b;
  other3_pid_value[3]=other3_c;
  other3_pid_value[4]=other3_d;
  other3_pid_value[5]=other3_e;
  other3_pid_value[6]=other3_f;
  other4_pid_value[0]=other4_step;
  other4_pid_value[1]=other4_a;
  other4_pid_value[2]=other4_b;
  other4_pid_value[3]=other4_c;
  other4_pid_value[4]=other4_d;
  other4_pid_value[5]=other4_e;
  other4_pid_value[6]=other4_f;
  other5_pid_value[0]=other5_step;
  other5_pid_value[1]=other5_a;
  other5_pid_value[2]=other5_b;
  other5_pid_value[3]=other5_c;
  other5_pid_value[4]=other5_d;
  other5_pid_value[5]=other5_e;
  other5_pid_value[6]=other5_f;
  
  
  uint8 key;
  page_state=FIRST_PAGE;
  
  
  uint8 change_page=1;
  
  while(1)
  {
    if(change_page)
    {
      LCD_clear(BLUE);    //先 清屏
      pagen_show();
    }
    
    while(change_page)
    {
      key=0;
      while((key=once_check_key())==0);                               //直到有按键按下
      if(arrow.y==1&&key==RIGHT)   //往右走
      {LCD_clear(BLUE);page_state=FIRST_PAGE;change_page=1;arrow.x=1;arrow.y=1; first_page_show();  break;}  
      else if(arrow.y==2&&key==RIGHT)   //往右走
      {LCD_clear(BLUE);page_state=SECOND_PAGE;change_page=2;arrow.x=1;arrow.y=1; second_page_show();  break;} 
      else if(arrow.y==3&&key==RIGHT)   //往右走
      {LCD_clear(BLUE);page_state=THIRD_PAGE;change_page=3;arrow.x=1;arrow.y=1; third_page_show();  break;}     
      else if(arrow.y==pagen_num&&key==RIGHT)   //往右走
      {save_data(sd); LCD_clear(BLACK); change_page=0; break;}   //退出之前保存数据   这样比较好 
      else if((key==UP) ||(key==DOWN))
      {
        pagen_updown(key);
      }
      
    }
    if(change_page==1)
    {
      while(1)
        
      {
        key=0;
        while ((key=once_check_key())==0);                               //直到有按键按下
        if(page_state==FIRST_PAGE&&arrow.y==first_page_num&&key==RIGHT)   //往右走
        {save_data(sd); LCD_clear(BLACK); change_page=0; break;}                       //退出之前保存数据   这样比较好
        else if(page_state==FIRST_PAGE&&key==LEFT)   //往右走
        {break;} 
        
        /****************************箭头 上下 左右*******************/   
        
        //首页
        else if(page_state==FIRST_PAGE&&(key==UP||key==DOWN))   //y 0-num_max箭头上下移动
          first_page_up_down(key);
        
        //舵机  
        else if(page_state==SERVO_PAGE&&servo_state==SERVO_UN_ADJUST&&(key==UP||key==DOWN))   //y 0-num_max箭头上下移动
          servo_page_up_down(key);
        
        else if(page_state==SERVO_PAGE&&((servo_state==SERVO_UN_ADJUST&&key==RIGHT)||(servo_state==SERVO_ADJUST&&key==LEFT)))
          servo_arrow_lr(key);
        //电机  
        else if(page_state==MOTOR_PAGE&&motor_state==MOTOR_UN_ADJUST&&(key==UP||key==DOWN))   //y 0-num_max箭头上下移动
          motor_page_up_down(key);
        
        else if(page_state==MOTOR_PAGE&&((motor_state==MOTOR_UN_ADJUST&&key==RIGHT)||(motor_state==MOTOR_ADJUST&&key==LEFT)))
          motor_arrow_lr(key);
        //中值
        else if(page_state==SERVO_M_PAGE&&servo_m_state==SERVO_M_UN_ADJUST&&(key==UP||key==DOWN))
          servo_m_page_up_down(key);
        else if(page_state==SERVO_M_PAGE&&((servo_m_state==SERVO_M_UN_ADJUST&&key==RIGHT)||(servo_m_state==SERVO_M_ADJUST&&key==LEFT)))
          servo_m_arrow_lr(key);
        
        //速度
        else if(page_state==V_SET_PAGE&&v_set_state==V_SET_UN_ADJUST&&(key==UP||key==DOWN))
          v_set_page_up_down(key);
        else if(page_state==V_SET_PAGE&&((v_set_state==V_SET_UN_ADJUST&&key==RIGHT)||(v_set_state==V_SET_ADJUST&&key==LEFT)))
          v_set_arrow_lr(key);
        // 基准
        else if(page_state==JIZHUN_PAGE&&jizhun_state==JIZHUN_UN_ADJUST&&(key==UP||key==DOWN))
          jizhun_page_up_down(key);
        else if(page_state==JIZHUN_PAGE&&((jizhun_state==JIZHUN_UN_ADJUST&&key==RIGHT)||(jizhun_state==JIZHUN_ADJUST&&key==LEFT)))
          jizhun_arrow_lr(key);
        
        
        
        /****************************调节模式 *******************/  
        
        //舵机
        else if(page_state==SERVO_PAGE&&servo_state==SERVO_ADJUST&&(key==UP||key==DOWN))    
          servo_pid_adjust(key);
        //电机
        else if(page_state==MOTOR_PAGE&&motor_state==MOTOR_ADJUST&&(key==UP||key==DOWN))    
          motor_pid_adjust(key); 
        
        //中值
        else if(page_state==SERVO_M_PAGE&&servo_m_state==SERVO_M_ADJUST&&(key==UP||key==DOWN))      
          servo_m_adjust(key);      
        //速度
        else if(page_state==V_SET_PAGE&&v_set_state==V_SET_ADJUST&&(key==UP||key==DOWN))      
          v_set_adjust(key);  
        //基准
        else if(page_state==JIZHUN_PAGE&&jizhun_state==JIZHUN_ADJUST&&(key==UP||key==DOWN))      
          jizhun_adjust(key); 
        
        
        
        /****************************换页模式*******************/   
        
        //首页到舵机页面
        else if(page_state==FIRST_PAGE&&arrow.y==1&&key==RIGHT)
        {LCD_clear(BLUE); servo_parameter_show(); page_state=SERVO_PAGE; arrow.x=1;arrow.y=1;}
        //舵机到首页
        else if(page_state==SERVO_PAGE&&servo_state==SERVO_UN_ADJUST&&key==LEFT)
        {LCD_clear(BLUE); first_page_show();     page_state=FIRST_PAGE; arrow.x=1;arrow.y=1;}
        //首页到中值页面 
        else if(page_state==FIRST_PAGE&&arrow.y==2&&key==RIGHT)
        {LCD_clear(BLUE);  servo_m_parameter_show(); page_state=SERVO_M_PAGE;  arrow.x=1;arrow.y=1;} 
        //中值到首页
        else if(page_state==SERVO_M_PAGE&&servo_m_state==SERVO_UN_ADJUST&&key==LEFT)
        { LCD_clear(BLUE);  first_page_show(); page_state=FIRST_PAGE;  arrow.x=1; arrow.y=1; }
        //首页到速度设定页面
        else if(page_state==FIRST_PAGE&&arrow.y==3&&key==RIGHT)
        {LCD_clear(BLUE);  v_set_parameter_show(); page_state=V_SET_PAGE;  arrow.x=1;arrow.y=1;}
        
        //速度设置页面到首页
        else if(page_state==V_SET_PAGE&&v_set_state==V_SET_UN_ADJUST&&key==LEFT)
        { LCD_clear(BLUE);  first_page_show(); page_state=FIRST_PAGE;  arrow.x=1; arrow.y=1; }
        //首页到基准页面 
        else if(page_state==FIRST_PAGE&&arrow.y==4&&key==RIGHT)
        {LCD_clear(BLUE);  jizhun_parameter_show(); page_state=JIZHUN_PAGE;  arrow.x=1;arrow.y=1;} 
        //基准到首页
        else if(page_state==JIZHUN_PAGE&&jizhun_state==SERVO_UN_ADJUST&&key==LEFT)
        { LCD_clear(BLUE);  first_page_show(); page_state=FIRST_PAGE;  arrow.x=1; arrow.y=1; }
        //首页到电机页面
        else if(page_state==FIRST_PAGE&&arrow.y==5&&key==RIGHT)
        {LCD_clear(BLUE); motor_parameter_show(); page_state=MOTOR_PAGE; arrow.x=1;arrow.y=1;}
        //电机到首页
        else if(page_state==MOTOR_PAGE&&key==LEFT)
        {LCD_clear(BLUE); first_page_show();     page_state=FIRST_PAGE; arrow.x=1;arrow.y=1;}
        
        
      }
    }
    else if(change_page==2)
    {
      while(1)
      {
        key=0;
        while ((key=once_check_key())==0);                               //直到有按键按下
        if(page_state==SECOND_PAGE&&arrow.y==second_page_num&&key==RIGHT)   //往右走
        {save_data(sd); LCD_clear(BLACK); change_page=0; break;}                       //退出之前保存数据   这样比较好
        else if(page_state==SECOND_PAGE&&key==LEFT)   //往右走
        {break;} 
        
        /****************************箭头 上下 左右*******************/   
        
        //首页
        else if(page_state==SECOND_PAGE&&(key==UP||key==DOWN))   //y 0-num_max箭头上下移动
          second_page_up_down(key);
        
        //次数 
        else if(page_state==CNT_PAGE&&cnt_state==CNT_UN_ADJUST&&(key==UP||key==DOWN))   //y 0-num_max箭头上下移动
          cnt_page_up_down(key);
        
        else if(page_state==CNT_PAGE&&((cnt_state==CNT_UN_ADJUST&&key==RIGHT)||(cnt_state==CNT_ADJUST&&key==LEFT)))
          cnt_arrow_lr(key);
        //速度
        else if(page_state==V_SET2_PAGE&&v_set2_state==V_SET2_UN_ADJUST&&(key==UP||key==DOWN))
          v_set_page_up_down2(key);
        else if(page_state==V_SET2_PAGE&&((v_set2_state==V_SET2_UN_ADJUST&&key==RIGHT)||(v_set2_state==V_SET2_ADJUST&&key==LEFT)))
          v_set_arrow_lr2(key);
        //距离  
        else if(page_state==DISTANCE_PAGE&&distance_state==DISTANCE_UN_ADJUST&&(key==UP||key==DOWN))   //y 0-num_max箭头上下移动
          distance_page_up_down(key);
        
        else if(page_state==DISTANCE_PAGE&&((distance_state==DISTANCE_UN_ADJUST&&key==RIGHT)||(distance_state==DISTANCE_ADJUST&&key==LEFT)))
          distance_arrow_lr(key);
        //距离  
        else if(page_state==DISTANCE2_PAGE&&distance2_state==DISTANCE2_UN_ADJUST&&(key==UP||key==DOWN))   //y 0-num_max箭头上下移动
          distance2_page_up_down(key);
        
        else if(page_state==DISTANCE2_PAGE&&((distance2_state==DISTANCE2_UN_ADJUST&&key==RIGHT)||(distance2_state==DISTANCE2_ADJUST&&key==LEFT)))
          distance2_arrow_lr(key);
        
        
        
        
        
        
        /****************************调节模式 *******************/  
        //基准
        else if(page_state==CNT_PAGE&&cnt_state==CNT_ADJUST&&(key==UP||key==DOWN))      
          cnt_adjust(key); 
        //速度
        else if(page_state==V_SET2_PAGE&&v_set2_state==V_SET2_ADJUST&&(key==UP||key==DOWN))      
          v_set_adjust2(key); 
        //距离
        else if(page_state==DISTANCE_PAGE&&distance_state==DISTANCE_ADJUST&&(key==UP||key==DOWN))    
          distance_pid_adjust(key);
        //距离
        else if(page_state==DISTANCE2_PAGE&&distance2_state==DISTANCE2_ADJUST&&(key==UP||key==DOWN))    
          distance2_pid_adjust(key);
        
        
        
        
        
        
        /****************************换页模式*******************/   
        
        //首页到次数页面
        else if(page_state==SECOND_PAGE&&arrow.y==1&&key==RIGHT)
        {LCD_clear(BLUE); cnt_parameter_show(); page_state=CNT_PAGE; arrow.x=1;arrow.y=1;}
        //次数到首页
        else if(page_state==CNT_PAGE&&cnt_state==CNT_UN_ADJUST&&key==LEFT)
        {LCD_clear(BLUE);second_page_show();     page_state=SECOND_PAGE; arrow.x=1;arrow.y=1;}
        
        //首页到速度设定页面
        else if(page_state==SECOND_PAGE&&arrow.y==2&&key==RIGHT)
        {LCD_clear(BLUE);  v_set_parameter_show2(); page_state=V_SET2_PAGE;  arrow.x=1;arrow.y=1;}
        
        //速度设置页面到首页
        else if(page_state==V_SET2_PAGE&&v_set_state==V_SET2_UN_ADJUST&&key==LEFT)
        { LCD_clear(BLUE);  second_page_show(); page_state=SECOND_PAGE;  arrow.x=1; arrow.y=1; }
        
        
        //首页到距离页面
        else if(page_state==SECOND_PAGE&&arrow.y==3&&key==RIGHT)
        {LCD_clear(BLUE); distance_parameter_show(); page_state=DISTANCE_PAGE; arrow.x=1;arrow.y=1;}
        //距离到首页
        else if(page_state==DISTANCE_PAGE&&distance_state==DISTANCE_UN_ADJUST&&key==LEFT)
        {LCD_clear(BLUE);second_page_show();     page_state=SECOND_PAGE; arrow.x=1;arrow.y=1;}
        //首页到距离页面
        else if(page_state==SECOND_PAGE&&arrow.y==4&&key==RIGHT)
        {LCD_clear(BLUE); distance2_parameter_show(); page_state=DISTANCE2_PAGE; arrow.x=1;arrow.y=1;}
        //距离到首页
        else if(page_state==DISTANCE2_PAGE&&distance2_state==DISTANCE2_UN_ADJUST&&key==LEFT)
        {LCD_clear(BLUE);second_page_show();     page_state=SECOND_PAGE; arrow.x=1;arrow.y=1;}
      }
    }
    else if(change_page==3)
    {
      while(1)
        
      {
        key=0;
        while ((key=once_check_key())==0);                               //直到有按键按下
        if(page_state==THIRD_PAGE&&arrow.y==third_page_num&&key==RIGHT)   //往右走
        {save_data(sd); LCD_clear(BLACK); change_page=0; break;}                       //退出之前保存数据   这样比较好
        else if(page_state==THIRD_PAGE&&key==LEFT)   //往右走
        {break;} 
        
        /****************************箭头 上下 左右*******************/   
        
        //首页
        else if(page_state==THIRD_PAGE&&(key==UP||key==DOWN))   //y 0-num_max箭头上下移动
          third_page_up_down(key);
        
        
        else if(page_state==OTHER1_PAGE&&other1_state==OTHER1_UN_ADJUST&&(key==UP||key==DOWN))   //y 0-num_max箭头上下移动
          other1_page_up_down(key); 
        else if(page_state==OTHER1_PAGE&&((other1_state==OTHER1_UN_ADJUST&&key==RIGHT)||(other1_state==OTHER1_ADJUST&&key==LEFT)))
          other1_arrow_lr(key);
        else if(page_state==OTHER2_PAGE&&other2_state==OTHER2_UN_ADJUST&&(key==UP||key==DOWN))   //y 0-num_max箭头上下移动
          other2_page_up_down(key);
        
        else if(page_state==OTHER2_PAGE&&((other2_state==OTHER2_UN_ADJUST&&key==RIGHT)||(other2_state==OTHER2_ADJUST&&key==LEFT)))
          other2_arrow_lr(key);
        else if(page_state==OTHER3_PAGE&&other3_state==OTHER3_UN_ADJUST&&(key==UP||key==DOWN))   //y 0-num_max箭头上下移动
          other3_page_up_down(key);
        
        else if(page_state==OTHER3_PAGE&&((other3_state==OTHER3_UN_ADJUST&&key==RIGHT)||(other3_state==OTHER3_ADJUST&&key==LEFT)))
          other3_arrow_lr(key);
        else if(page_state==OTHER4_PAGE&&other4_state==OTHER4_UN_ADJUST&&(key==UP||key==DOWN))   //y 0-num_max箭头上下移动
          other4_page_up_down(key);
        
        else if(page_state==OTHER4_PAGE&&((other4_state==OTHER4_UN_ADJUST&&key==RIGHT)||(other4_state==OTHER4_ADJUST&&key==LEFT)))
          other4_arrow_lr(key);
        else if(page_state==OTHER5_PAGE&&other5_state==OTHER5_UN_ADJUST&&(key==UP||key==DOWN))   //y 0-num_max箭头上下移动
          other5_page_up_down(key);
        
        else if(page_state==OTHER5_PAGE&&((other5_state==OTHER5_UN_ADJUST&&key==RIGHT)||(other5_state==OTHER5_ADJUST&&key==LEFT)))
          other5_arrow_lr(key);
        
        
        
        /****************************调节模式 *******************/  
        else if(page_state==OTHER1_PAGE&&other1_state==OTHER1_ADJUST&&(key==UP||key==DOWN))    
          other1_pid_adjust(key);
        else if(page_state==OTHER2_PAGE&&other2_state==OTHER2_ADJUST&&(key==UP||key==DOWN))    
          other2_pid_adjust(key);
        else if(page_state==OTHER3_PAGE&&other3_state==OTHER3_ADJUST&&(key==UP||key==DOWN))    
          other3_pid_adjust(key);
        else if(page_state==OTHER4_PAGE&&other4_state==OTHER4_ADJUST&&(key==UP||key==DOWN))    
          other4_pid_adjust(key);
        else if(page_state==OTHER5_PAGE&&other5_state==OTHER5_ADJUST&&(key==UP||key==DOWN))    
          other5_pid_adjust(key);
        
        
        
        
        /****************************换页模式*******************/   
        
        
        else if(page_state==THIRD_PAGE&&arrow.y==1&&key==RIGHT)
        {LCD_clear(BLUE); other1_parameter_show(); page_state=OTHER1_PAGE; arrow.x=1;arrow.y=1;}
        
        else if(page_state==OTHER1_PAGE&&other1_state==OTHER1_UN_ADJUST&&key==LEFT)
        {LCD_clear(BLUE); third_page_show();     page_state=THIRD_PAGE; arrow.x=1;arrow.y=1;}
        else if(page_state==THIRD_PAGE&&arrow.y==2&&key==RIGHT)
        {LCD_clear(BLUE); other2_parameter_show(); page_state=OTHER2_PAGE; arrow.x=1;arrow.y=1;}
        
        else if(page_state==OTHER2_PAGE&&other2_state==OTHER2_UN_ADJUST&&key==LEFT)
        {LCD_clear(BLUE); third_page_show();     page_state=THIRD_PAGE; arrow.x=1;arrow.y=1;}
        else if(page_state==THIRD_PAGE&&arrow.y==3&&key==RIGHT)
        {LCD_clear(BLUE); other3_parameter_show(); page_state=OTHER3_PAGE; arrow.x=1;arrow.y=1;}
        
        else if(page_state==OTHER3_PAGE&&other3_state==OTHER3_UN_ADJUST&&key==LEFT)
        {LCD_clear(BLUE); third_page_show();     page_state=THIRD_PAGE; arrow.x=1;arrow.y=1;}
        else if(page_state==THIRD_PAGE&&arrow.y==4&&key==RIGHT)
        {LCD_clear(BLUE); other4_parameter_show(); page_state=OTHER4_PAGE; arrow.x=1;arrow.y=1;}
        
        else if(page_state==OTHER4_PAGE&&other4_state==OTHER4_UN_ADJUST&&key==LEFT)
        {LCD_clear(BLUE); third_page_show();     page_state=THIRD_PAGE; arrow.x=1;arrow.y=1;}
        else if(page_state==THIRD_PAGE&&arrow.y==5&&key==RIGHT)
        {LCD_clear(BLUE); other5_parameter_show(); page_state=OTHER5_PAGE; arrow.x=1;arrow.y=1;}
        
        else if(page_state==OTHER5_PAGE&&other5_state==OTHER5_UN_ADJUST&&key==LEFT)
        {LCD_clear(BLUE); third_page_show();     page_state=THIRD_PAGE; arrow.x=1;arrow.y=1;}
//        else if(page_state==THIRD_PAGE&&arrow.y==6&&key==RIGHT)
//        {LCD_clear(BLUE); other5_parameter_show(); page_state=OTHER5_PAGE; arrow.x=1;arrow.y=1;}
//        
//        else if(page_state==OTHER5_PAGE&&other5_state==OTHER5_UN_ADJUST&&key==LEFT)
//        {LCD_clear(BLUE); third_page_show();     page_state=THIRD_PAGE; arrow.x=1;arrow.y=1;}
        
        
        
      }
    } 
    
    else
    {
      break;
    }
  }
  
}

/***********************************************************************************************************************************/

void lcd_double_show(Site_t site,double value)
{
  uint32 zhengshu,xiaoshu;
  zhengshu=(uint32)(value);
  xiaoshu=(uint32)((value-zhengshu)*100);
  LCD_num(site,zhengshu, WHITE,BLUE); 
  
  uint32 res=zhengshu;
  uint8 t=0;
  while( res )  /*得到数字长度t*/
  {
    res /= 10;
    t++;
  }
  site.x=site.x+t*8;
  LCD_char(site,'.', WHITE,BLUE);
  
  site.x=site.x+1*8;
  if(xiaoshu<10)
  {
    LCD_char(site,'0', WHITE,BLUE);
    site.x=site.x+1*8;
    LCD_num(site,xiaoshu, WHITE,BLUE);
  }
  else
    LCD_num(site,xiaoshu, WHITE,BLUE);
}

/***********************************************************************************************************************************/
void pagen_show()
{
  //第一页
  Site_t site_flag = {0,0};
  
  LCD_str(site_flag,"->", BLACK,BLUE);            
  
  Site_t site;
  site.x = 2*8;   site.y=0; 
  LCD_str(site,"frist_page", WHITE,BLUE);
  site.x = 2*8;   site.y=1*16; 
  LCD_str(site,"second_page", WHITE,BLUE);
  site.x = 2*8;   site.y=2*16;  
  LCD_str(site,"third_page", WHITE,BLUE);
  site.x = 2*8;   site.y=3*16;  
  LCD_str(site,"ready_go!", WHITE,BLUE);
}
/***********************************************************************************************************************************/
void pagen_updown(uint8 key)
{
  Site_t site_flag;
  site_flag.x=0;
  site_flag.y=(arrow.y-1)*16;
  LCD_str(site_flag,"  ", BLUE,BLUE); 
  
  if(key==UP)
  {
    arrow.y=arrow.y-1>=1?(arrow.y-1):pagen_num;
  }
  else
  {
    arrow.y=arrow.y+1<=pagen_num?(arrow.y+1):1;
  }
  
  site_flag.y=(arrow.y-1)*16;
  LCD_str(site_flag,"->", WHITE,BLUE); 
}
/***********************************************************************************************************************************/

void first_page_show()
{
  //第一页
  Site_t site_flag = {0,0};
  
  LCD_str(site_flag,"->", BLACK,BLUE);            
  
  Site_t site;
  site.x = 2*8;   site.y=0; 
  LCD_str(site,"servo_pid", WHITE,BLUE);
  site.x = 2*8;   site.y=1*16; 
  LCD_str(site,"servo_middle!", WHITE,BLUE);
  site.x = 2*8;   site.y=2*16; 
  LCD_str(site,"v_set", WHITE,BLUE);
  site.x = 2*8;   site.y=3*16; 
  LCD_str(site,"jizhun", WHITE,BLUE);
  site.x = 2*8;   site.y=4*16; 
  LCD_str(site,"motor_pid!", WHITE,BLUE);
  site.x = 2*8;   site.y=5*16; 
  LCD_str(site,"ready_go!", WHITE,BLUE);
  
}

/***********************************************************************************************************************************/

void first_page_up_down(uint8 key)
{
  Site_t site_flag;
  site_flag.x=0;
  site_flag.y=(arrow.y-1)*16;
  LCD_str(site_flag,"  ", BLUE,BLUE); 
  
  if(key==UP)
  {
    arrow.y=arrow.y-1>=1?(arrow.y-1):first_page_num;
  }
  else
  {
    arrow.y=arrow.y+1<=first_page_num?(arrow.y+1):1;
  }
  
  site_flag.y=(arrow.y-1)*16;
  LCD_str(site_flag,"->", WHITE,BLUE); 
  
}


void second_page_show()
{
  //第一页
  Site_t site_flag = {0,0};
  
  LCD_str(site_flag,"->", BLACK,BLUE);            
  
  Site_t site;
  site.x = 2*8;   site.y=0; 
  LCD_str(site,"cnt", WHITE,BLUE);
  site.x = 2*8;   site.y=1*16; 
  LCD_str(site,"v_set2", WHITE,BLUE);
  site.x = 2*8;   site.y=2*16; 
  LCD_str(site,"distance1", WHITE,BLUE);
  site.x = 2*8;   site.y=3*16; 
  LCD_str(site,"distance2", WHITE,BLUE);
  site.x = 2*8;   site.y=4*16; 
  LCD_str(site,"ready_go!", WHITE,BLUE);
}

/***********************************************************************************************************************************/

void second_page_up_down(uint8 key)
{
  Site_t site_flag;
  site_flag.x=0;
  site_flag.y=(arrow.y-1)*16;
  LCD_str(site_flag,"  ", BLUE,BLUE); 
  
  if(key==UP)
  {
    arrow.y=arrow.y-1>=1?(arrow.y-1):second_page_num;
  }
  else
  {
    arrow.y=arrow.y+1<=second_page_num?(arrow.y+1):1;
  }
  
  site_flag.y=(arrow.y-1)*16;
  LCD_str(site_flag,"->", WHITE,BLUE); 
  
}

/***********************************************************************************************************************************/
void third_page_show()
{
  //第一页
  Site_t site_flag = {0,0};
  
  LCD_str(site_flag,"->", BLACK,BLUE);            
  
  Site_t site;
  site.x = 2*8;   site.y=0; 
  LCD_str(site,"other1", WHITE,BLUE);
  site.x = 2*8;   site.y=1*16; 
  LCD_str(site,"other2", WHITE,BLUE);
  site.x = 2*8;   site.y=2*16; 
  LCD_str(site,"other3", WHITE,BLUE);
  site.x = 2*8;   site.y=3*16; 
  LCD_str(site,"other4", WHITE,BLUE);
  site.x = 2*8;   site.y=4*16; 
  LCD_str(site,"other5", WHITE,BLUE);
  site.x = 2*8;   site.y=5*16; 
  LCD_str(site,"ready_go!", WHITE,BLUE);
  
}

/***********************************************************************************************************************************/

void third_page_up_down(uint8 key)
{
  Site_t site_flag;
  site_flag.x=0;
  site_flag.y=(arrow.y-1)*16;
  LCD_str(site_flag,"  ", BLUE,BLUE); 
  
  if(key==UP)
  {
    arrow.y=arrow.y-1>=1?(arrow.y-1):third_page_num;
  }
  else
  {
    arrow.y=arrow.y+1<=third_page_num?(arrow.y+1):1;
  }
  
  site_flag.y=(arrow.y-1)*16;
  LCD_str(site_flag,"->", WHITE,BLUE); 
  
}
/***********************************************************************************************************************************/

void servo_parameter_show()
{
  
  Site_t site;
  //文字刷新
  for(uint8 i=0;i<8;i++)
  {
    site.x=servo_msg_x[i]*8;
    site.y=servo_msg_y[i]*16;
    LCD_str(site,servo_sg[i],WHITE ,BLUE);     //标题  
  }
  //数值刷新
  for(uint8 i=0;i<7;i++)
  {
    site.x=servo_pid_value_x[i]*8;
    site.y=servo_pid_value_y[i]*16;
    lcd_double_show(site,servo_pid_value[i]);
    //            //数值
  }
  //箭头刷新
  site.x=0; site.y=1*16;
  LCD_str(site,"->",WHITE ,BLUE);     //标题  
  //更新记录
  
}

/***********************************************************************************************************************************/

void servo_page_up_down(uint8 key)
{
  Site_t site_flag;
  site_flag.x=servo_up_down[arrow.y-1][0]*8;
  site_flag.y=servo_up_down[arrow.y-1][1]*16;                 
  LCD_str(site_flag,"  ", BLUE,BLUE);
  if(key==UP)
  {
    arrow.y=arrow.y-1>=1?(arrow.y-1):servo_page_num;
  }
  else
  {
    arrow.y=arrow.y+1<=servo_page_num?(arrow.y+1):1;
  }
  site_flag.y=servo_up_down[arrow.y-1][1]*16;
  LCD_str(site_flag,"->", WHITE,BLUE);    
  
}

/***********************************************************************************************************************************/

void servo_arrow_lr(uint8 key)
{
  Site_t site_flag;
  if(key==LEFT)
  {
    site_flag.x=servo_r[arrow.y-1][0]*8;
    site_flag.y=servo_r[arrow.y-1][1]*16;
    LCD_str(site_flag,"  ", BLUE,BLUE);
    
    site_flag.x=servo_l[arrow.y-1][0]*8;
    site_flag.y=servo_l[arrow.y-1][1]*16;
    LCD_str(site_flag,"->", WHITE,BLUE);
    servo_state=SERVO_UN_ADJUST;
    arrow.x=1;
  }
  else
  {
    site_flag.x=servo_l[arrow.y-1][0]*8;
    site_flag.y=servo_l[arrow.y-1][1]*16;
    LCD_str(site_flag,"  ", BLUE,BLUE);  
    
    site_flag.x=servo_r[arrow.y-1][0]*8;
    site_flag.y=servo_r[arrow.y-1][1]*16;
    LCD_str(site_flag,"->", WHITE,BLUE);
    servo_state=SERVO_ADJUST;
    arrow.x=2;
    
  }
  
}

/***********************************************************************************************************************************/

void servo_pid_adjust(uint8 key)
{
  
  if(arrow.y==1)      //motor_step
  { 
    
    if(key==UP)
    {
      for(uint8 i=0;i<3;i++)         //最后一个  不作考虑
      {
        if(fabsf(servo_step_up_down[i]-servo_pid_value[0])<0.001)
        {
          servo_pid_value[0]=servo_step_up_down[i+1];
          break;
        }      
      }  
    }
    else
    {
      for(uint8 i=3;i>0;i--)
      {
        if(fabsf(servo_step_up_down[i]-servo_pid_value[0])<0.001)
        {
          servo_pid_value[0]=servo_step_up_down[i-1];
          break;
        }
      }  
    }
  }
  else if(arrow.y<=servo_page_num)
  {
    if(key==UP)
      servo_pid_value[arrow.y-1]+=servo_pid_value[0];
    else
      servo_pid_value[arrow.y-1]-=servo_pid_value[0];
    
    servo_pid_value[arrow.y-1]= servo_pid_value[arrow.y-1]>0?servo_pid_value[arrow.y-1]:0;     //避免负数********
  }
  else
  {
    
  }
  servo_step=servo_pid_value[0];
  servo_a=servo_pid_value[1]; 
  servo_b=servo_pid_value[2]; 
  servo_c=servo_pid_value[3]; 
  servo_d=servo_pid_value[4];
  servo_e=servo_pid_value[5];
  servo_f=servo_pid_value[6];
  
  Site_t site;
  site.x=pid_value_x[arrow.y-1]*8;           //只与y有关
  site.y=pid_value_y[arrow.y-1]*16;
  for(uint8 i=servo_pid_value_x[0];i<16;i++)
  { 
    site.x=i*8;
    LCD_char(site,' ', BLUE,BLUE);
  }
  
  site.x=servo_pid_value_x[arrow.y-1]*8;           //只与y有关
  site.y=servo_pid_value_y[arrow.y-1]*16;
  lcd_double_show(site,servo_pid_value[arrow.y-1]); 
}

/***********************************************************************************************************************************/

void motor_parameter_show()
{
  
  Site_t site;
  //文字刷新
  for(uint8 i=0;i<8;i++)
  {
    site.x=motor_msg_x[i]*8;
    site.y=motor_msg_y[i]*16;
    LCD_str(site,motor_sg[i],WHITE ,BLUE);     //标题  
  }
  //数值刷新
  for(uint8 i=0;i<7;i++)
  {
    site.x=motor_pid_value_x[i]*8;
    site.y=motor_pid_value_y[i]*16;
    lcd_double_show(site,motor_pid_value[i]);
    //            //数值
  }
  //箭头刷新
  site.x=0; site.y=1*16;
  LCD_str(site,"->",WHITE ,BLUE);     //标题  
  //更新记录
  
}

/***********************************************************************************************************************************/

void motor_page_up_down(uint8 key)
{
  Site_t site_flag;
  site_flag.x=motor_up_down[arrow.y-1][0]*8;
  site_flag.y=motor_up_down[arrow.y-1][1]*16;                 
  LCD_str(site_flag,"  ", BLUE,BLUE);
  if(key==UP)
  {
    arrow.y=arrow.y-1>=1?(arrow.y-1):motor_page_num;
  }
  else
  {
    arrow.y=arrow.y+1<=motor_page_num?(arrow.y+1):1;
  }
  site_flag.y=motor_up_down[arrow.y-1][1]*16;
  LCD_str(site_flag,"->", WHITE,BLUE);    
  
}

/***********************************************************************************************************************************/

void motor_arrow_lr(uint8 key)
{
  Site_t site_flag;
  if(key==LEFT)
  {
    site_flag.x=motor_r[arrow.y-1][0]*8;
    site_flag.y=motor_r[arrow.y-1][1]*16;
    LCD_str(site_flag,"  ", BLUE,BLUE);
    
    site_flag.x=motor_l[arrow.y-1][0]*8;
    site_flag.y=motor_l[arrow.y-1][1]*16;
    LCD_str(site_flag,"->", WHITE,BLUE);
    motor_state=MOTOR_UN_ADJUST;
    arrow.x=1;
  }
  else
  {
    site_flag.x=motor_l[arrow.y-1][0]*8;
    site_flag.y=motor_l[arrow.y-1][1]*16;
    LCD_str(site_flag,"  ", BLUE,BLUE);  
    
    site_flag.x=motor_r[arrow.y-1][0]*8;
    site_flag.y=motor_r[arrow.y-1][1]*16;
    LCD_str(site_flag,"->", WHITE,BLUE);
    motor_state=MOTOR_ADJUST;
    arrow.x=2;
    
  }
  
}

/***********************************************************************************************************************************/

void motor_pid_adjust(uint8 key)
{
  
  if(arrow.y==1)      //motor_step
  { 
    
    if(key==UP)
    {
      for(uint8 i=0;i<3;i++)         //最后一个  不作考虑
      {
        if(fabsf(motor_step_up_down[i]-motor_pid_value[0])<0.001)
        {
          motor_pid_value[0]=motor_step_up_down[i+1];
          break;
        }      
      }  
    }
    else
    {
      for(uint8 i=3;i>0;i--)
      {
        if(fabsf(motor_step_up_down[i]-motor_pid_value[0])<0.001)
        {
          motor_pid_value[0]=motor_step_up_down[i-1];
          break;
        }
      }  
    }
  }
  else if(arrow.y<=motor_page_num)
  {
    if(key==UP)
      motor_pid_value[arrow.y-1]+=motor_pid_value[0];
    else
      motor_pid_value[arrow.y-1]-=motor_pid_value[0];
    
    motor_pid_value[arrow.y-1]= motor_pid_value[arrow.y-1]>0?motor_pid_value[arrow.y-1]:0;     //避免负数********
  }
  else
  {
    
  }
  motor_step=motor_pid_value[0];
  motor_a=motor_pid_value[1]; 
  motor_b=motor_pid_value[2]; 
  motor_c=motor_pid_value[3]; 
  motor_d=motor_pid_value[4];
  motor_e=motor_pid_value[5];
  motor_f=motor_pid_value[6];
  
  Site_t site;
  site.x=pid_value_x[arrow.y-1]*8;           //只与y有关
  site.y=pid_value_y[arrow.y-1]*16;
  for(uint8 i=motor_pid_value_x[0];i<16;i++)
  { 
    site.x=i*8;
    LCD_char(site,' ', BLUE,BLUE);
  }
  
  site.x=motor_pid_value_x[arrow.y-1]*8;           //只与y有关
  site.y=motor_pid_value_y[arrow.y-1]*16;
  lcd_double_show(site,motor_pid_value[arrow.y-1]); 
}


/***********************************************************************************************************************************/

void servo_m_parameter_show()
{
  Site_t site;
  //文字刷新
  for(uint8 i=0;i<5;i++)
  {
    site.x=servo_m_msg_x[i]*8;
    site.y=servo_m_msg_y[i]*16;
    LCD_str(site,servo_m_sg[i],WHITE ,BLUE);     //标题  
  }
  //数值刷新
  for(uint8 i=0;i<4;i++)
  {
    site.x=servo_m_value_x[i]*8;
    site.y=servo_m_value_y[i]*16;
    LCD_num(site,servo_m_value[i], WHITE,BLUE);
    //lcd_double_show(site,servo_m_value[i]);
    //            //数值
  }
  //箭头刷新
  site.x=0; site.y=1*16;
  LCD_str(site,"->",WHITE ,BLUE);     //标题  
  //更新记录
}


/***********************************************************************************************************************************/

void servo_m_page_up_down(uint8 key)
{
  Site_t site_flag;
  site_flag.x=servo_m_up_down[arrow.y-1][0]*8;
  site_flag.y=servo_m_up_down[arrow.y-1][1]*16;                 
  LCD_str(site_flag,"  ", BLUE,BLUE);
  if(key==UP)
  {
    arrow.y=arrow.y-1>=1?(arrow.y-1):servo_m_page_num;
  }
  else
  {
    arrow.y=arrow.y+1<=servo_m_page_num?(arrow.y+1):1;
  }
  site_flag.y=servo_m_up_down[arrow.y-1][1]*16;
  LCD_str(site_flag,"->", WHITE,BLUE); 
}

/***********************************************************************************************************************************/

void servo_m_arrow_lr(uint8 key)
{
  Site_t site_flag;
  if(key==LEFT)
  {
    site_flag.x=v_set_r[arrow.y-1][0]*8;
    site_flag.y=v_set_r[arrow.y-1][1]*16;
    LCD_str(site_flag,"  ", BLUE,BLUE);
    
    site_flag.x=v_set_l[arrow.y-1][0]*8;
    site_flag.y=v_set_l[arrow.y-1][1]*16;
    LCD_str(site_flag,"->", WHITE,BLUE);
    servo_m_state=SERVO_M_UN_ADJUST;
    arrow.x=1;
  }
  else
  {
    site_flag.x=v_set_l[arrow.y-1][0]*8;
    site_flag.y=v_set_l[arrow.y-1][1]*16;
    LCD_str(site_flag,"  ", BLUE,BLUE);  
    
    site_flag.x=v_set_r[arrow.y-1][0]*8;
    site_flag.y=v_set_r[arrow.y-1][1]*16;
    LCD_str(site_flag,"->", WHITE,BLUE);
    servo_m_state=SERVO_M_ADJUST;
    arrow.x=2;
    
  }
  
}

/***********************************************************************************************************************************/

void servo_m_adjust(uint8 key)
{
  if(arrow.y==1)      //motor_step
  { 
    
    if(key==UP)
    {
      for(uint8 i=0;i<5;i++)         //最后一个  不作考虑
      {
        if(ABS(servo_m_step_up_down[i]-servo_m_value[0])<0.01)
        {
          servo_m_value[0]=servo_m_step_up_down[i+1];
          break;
        }      
      }  
    }
    else
    {
      for(uint8 i=5;i>0;i--)
      {
        if(ABS(servo_m_step_up_down[i]-servo_m_value[0])<0.01)
        {
          servo_m_value[0]=servo_m_step_up_down[i-1];
          break;
        }
      }  
    }
  }
  else if(arrow.y<=servo_m_page_num)
  {
    if(key==UP)
      servo_m_value[arrow.y-1]+=servo_m_value[0];
    else
      servo_m_value[arrow.y-1]-=servo_m_value[0];
    
    servo_m_value[arrow.y-1]= servo_m_value[arrow.y-1]>0?servo_m_value[arrow.y-1]:0;     //避免负数********
  }
  else           //保存数据到flash中
  {
    //save_data();
  }
  servo_m_step=servo_m_value[0];
  SERVO_L =servo_m_value[1]; 
  SERVO_MID =servo_m_value[2]; 
  SERVO_R =servo_m_value[3];
  
  Site_t site;
  site.x=v_set_value_x[arrow.y-1]*8;           //只与y有关
  site.y=v_set_value_y[arrow.y-1]*16;
  for(uint8 i=v_set_value_x[0];i<16;i++)
  { 
    site.x=i*8;
    LCD_char(site,' ', BLUE,BLUE);
  }
  
  site.x=servo_m_value_x[arrow.y-1]*8;           //只与y有关
  site.y=servo_m_value_y[arrow.y-1]*16;
  //lcd_double_show(site,servo_m_value[arrow.y-1]);
  LCD_num(site,servo_m_value[arrow.y-1], WHITE,BLUE);
  
}

/***********************************************************************************************************************************/

void jizhun_parameter_show()
{
  Site_t site;
  //文字刷新
  for(uint8 i=0;i<5;i++)
  {
    site.x=jizhun_msg_x[i]*8;
    site.y=jizhun_msg_y[i]*16;
    LCD_str(site,jizhun_sg[i],WHITE ,BLUE);     //标题  
  }
  //数值刷新
  for(uint8 i=0;i<4;i++)
  {
    site.x=jizhun_value_x[i]*8;
    site.y=jizhun_value_y[i]*16;
    LCD_num(site,jizhun_value[i], WHITE,BLUE);
    //lcd_double_show(site,jizhun_value[i]);
    //            //数值
  }
  //箭头刷新
  site.x=0; site.y=1*16;
  LCD_str(site,"->",WHITE ,BLUE);     //标题  
  //更新记录
}

/***********************************************************************************************************************************/

void jizhun_page_up_down(uint8 key)
{
  Site_t site_flag;
  site_flag.x=jizhun_up_down[arrow.y-1][0]*8;
  site_flag.y=jizhun_up_down[arrow.y-1][1]*16;                 
  LCD_str(site_flag,"  ", BLUE,BLUE);
  if(key==UP)
  {
    arrow.y=arrow.y-1>=1?(arrow.y-1):jizhun_page_num;
  }
  else
  {
    arrow.y=arrow.y+1<=jizhun_page_num?(arrow.y+1):1;
  }
  site_flag.y=jizhun_up_down[arrow.y-1][1]*16;
  LCD_str(site_flag,"->", WHITE,BLUE); 
}

/***********************************************************************************************************************************/

void jizhun_arrow_lr(uint8 key)
{
  Site_t site_flag;
  if(key==LEFT)
  {
    site_flag.x=v_set_r[arrow.y-1][0]*8;
    site_flag.y=v_set_r[arrow.y-1][1]*16;
    LCD_str(site_flag,"  ", BLUE,BLUE);
    
    site_flag.x=v_set_l[arrow.y-1][0]*8;
    site_flag.y=v_set_l[arrow.y-1][1]*16;
    LCD_str(site_flag,"->", WHITE,BLUE);
    jizhun_state=JIZHUN_UN_ADJUST;
    arrow.x=1;
  }
  else
  {
    site_flag.x=v_set_l[arrow.y-1][0]*8;
    site_flag.y=v_set_l[arrow.y-1][1]*16;
    LCD_str(site_flag,"  ", BLUE,BLUE);  
    
    site_flag.x=v_set_r[arrow.y-1][0]*8;
    site_flag.y=v_set_r[arrow.y-1][1]*16;
    LCD_str(site_flag,"->", WHITE,BLUE);
    jizhun_state=JIZHUN_ADJUST;
    arrow.x=2;
    
  }
}

/***********************************************************************************************************************************/

void jizhun_adjust(uint8 key)
{
  if(arrow.y==1)      //motor_step
  { 
    
    if(key==UP)
    {
      for(uint8 i=0;i<5;i++)         //最后一个  不作考虑
      {
        if(ABS(jizhun_step_up_down[i]-jizhun_value[0])<0.01)
        {
          jizhun_value[0]=jizhun_step_up_down[i+1];
          break;
        }      
      }  
    }
    else
    {
      for(uint8 i=5;i>0;i--)
      {
        if(ABS(jizhun_step_up_down[i]-jizhun_value[0])<0.01)
        {
          jizhun_value[0]=jizhun_step_up_down[i-1];
          break;
        }
      }  
    }
  }
  else if(arrow.y<=jizhun_page_num)
  {
    if(key==UP)
      jizhun_value[arrow.y-1]+=jizhun_value[0];
    else
      jizhun_value[arrow.y-1]-=jizhun_value[0];
    
    jizhun_value[arrow.y-1]= jizhun_value[arrow.y-1]>0?jizhun_value[arrow.y-1]:0;     //避免负数********
  }
  else           //保存数据到flash中
  {
    //save_data();
  }
  jizhun_step=jizhun_value[0];
  JIZHUN_ROW =jizhun_value[1]; 
  JIZHUN_UP =jizhun_value[2]; 
  JIZHUN_DOWN =jizhun_value[3];
  
  Site_t site;
  site.x=v_set_value_x[arrow.y-1]*8;           //只与y有关
  site.y=v_set_value_y[arrow.y-1]*16;
  for(uint8 i=v_set_value_x[0];i<16;i++)
  { 
    site.x=i*8;
    LCD_char(site,' ', BLUE,BLUE);
  }
  
  site.x=jizhun_value_x[arrow.y-1]*8;           //只与y有关
  site.y=jizhun_value_y[arrow.y-1]*16;
  //lcd_double_show(site,jizhun_value[arrow.y-1]);
  LCD_num(site,jizhun_value[arrow.y-1], WHITE,BLUE);
  
}


/***********************************************************************************************************************************/

void v_set_parameter_show()
{
  Site_t site;
  //文字刷新
  for(uint8 i=0;i<v_set_page_num+1;i++)
  {
    site.x=v_set_msg_x[i]*8;
    site.y=v_set_msg_y[i]*16;
    LCD_str(site,v_set_sg[i],WHITE ,BLUE);     //标题  
  }
  //数值刷新
  for(uint8 i=0;i<v_set_page_num;i++)
  {
    site.x=v_set_value_x[i]*8;
    site.y=v_set_value_y[i]*16;
    //lcd_double_show(site,v_set_value[i]);
    LCD_num(site,v_set_value[i], WHITE,BLUE);
    //              //数值
  }
  //箭头刷新
  site.x=0; site.y=1*16;
  LCD_str(site,"->",WHITE ,BLUE);     //标题  
  //更新记录
}

/***********************************************************************************************************************************/

void v_set_page_up_down(uint8 key)
{
  Site_t site_flag;
  site_flag.x=v_set_up_down[arrow.y-1][0]*8;
  site_flag.y=v_set_up_down[arrow.y-1][1]*16;                 
  LCD_str(site_flag,"  ", BLUE,BLUE);
  if(key==UP)
  {
    arrow.y=arrow.y-1>=1?(arrow.y-1):v_set_page_num;
  }
  else
  {
    arrow.y=arrow.y+1<=v_set_page_num?(arrow.y+1):1;
  }
  site_flag.y=v_set_up_down[arrow.y-1][1]*16;
  LCD_str(site_flag,"->", WHITE,BLUE); 
}

/***********************************************************************************************************************************/

void v_set_arrow_lr(uint8 key)
{
  Site_t site_flag;
  if(key==LEFT)
  {
    site_flag.x=v_set_r[arrow.y-1][0]*8;
    site_flag.y=v_set_r[arrow.y-1][1]*16;
    LCD_str(site_flag,"  ", BLUE,BLUE);
    
    site_flag.x=v_set_l[arrow.y-1][0]*8;
    site_flag.y=v_set_l[arrow.y-1][1]*16;
    LCD_str(site_flag,"->", WHITE,BLUE);
    v_set_state=V_SET_UN_ADJUST;
    arrow.x=1;
  }
  else
  {
    site_flag.x=v_set_l[arrow.y-1][0]*8;
    site_flag.y=v_set_l[arrow.y-1][1]*16;
    LCD_str(site_flag,"  ", BLUE,BLUE);  
    
    site_flag.x=v_set_r[arrow.y-1][0]*8;
    site_flag.y=v_set_r[arrow.y-1][1]*16;
    LCD_str(site_flag,"->", WHITE,BLUE);
    v_set_state=V_SET_ADJUST;
    arrow.x=2;
    
  }
}

/***********************************************************************************************************************************/

void v_set_adjust(uint8 key)
{
  
  if(arrow.y==1)      //motor_step
  { 
    if(key==UP)
    {
      for(uint8 i=0;i<5;i++)         //最后一个  不作考虑
      {
        if(ABS(v_set_step_up_down[i]-v_set_value[0])<0.01)
        {
          v_set_value[0]=v_set_step_up_down[i+1];
          break;
        }
        
      }  
      
      
    }
    else
    {
      for(uint8 i=5;i>0;i--)
      {
        if(ABS(v_set_step_up_down[i]-v_set_value[0])<0.01)
        {
          v_set_value[0]=v_set_step_up_down[i-1];
          break;
        }
      }  
    }
  }
  else if(arrow.y<=v_set_page_num)
  {
    int16 linshi;
    if(key==UP)
    {
      linshi=v_set_value[arrow.y-1]+v_set_value[0]; 
    }
    else 
    {
      linshi=v_set_value[arrow.y-1]-v_set_value[0];
    }
    v_set_value[arrow.y-1]=MIN(MAX(linshi,50),2000);
    
  }
  //v_set_value[arrow.y-1]= v_set_value[arrow.y-1]>0?v_set_value[arrow.y-1]:0;     //避免负数********
  
  else           //保存数据到flash中
  {
    //save_data();
  }
  speed_step=v_set_value[0];
  speed_minl=v_set_value[1]; 
  speed_maxl=v_set_value[2]; 
  speed_minr=v_set_value[3];
  speed_maxr=v_set_value[4]; 
  speed_s=v_set_value[5]; 
  speed_z=v_set_value[6]; 
  Site_t site;
  site.x=v_set_value_x[arrow.y-1]*8;           //只与y有关
  site.y=v_set_value_y[arrow.y-1]*16;
  for(uint8 i=v_set_value_x[0];i<16;i++)
  { 
    site.x=i*8;
    LCD_char(site,' ', BLUE,BLUE);
  }
  
  site.x=v_set_value_x[arrow.y-1]*8;           //只与y有关
  site.y=v_set_value_y[arrow.y-1]*16;
  LCD_num(site,v_set_value[arrow.y-1], WHITE,BLUE);
}

/***********************************************************************************************************************************/

void v_set_parameter_show2()
{
  Site_t site;
  //文字刷新
  for(uint8 i=0;i<v_set_page_num2+1;i++)
  {
    site.x=v_set_msg_x[i]*8;
    site.y=v_set_msg_y[i]*16;
    LCD_str(site,v_set_sg2[i],WHITE ,BLUE);     //标题  
  }
  //数值刷新
  for(uint8 i=0;i<v_set_page_num2;i++)
  {
    site.x=v_set_value_x[i]*8;
    site.y=v_set_value_y[i]*16;
    //lcd_double_show(site,v_set_value[i]);
    LCD_num(site,v_set_value2[i], WHITE,BLUE);
    //              //数值
  }
  //箭头刷新
  site.x=0; site.y=1*16;
  LCD_str(site,"->",WHITE ,BLUE);     //标题  
  //更新记录
}

/***********************************************************************************************************************************/

void v_set_page_up_down2(uint8 key)
{
  Site_t site_flag;
  site_flag.x=v_set_up_down[arrow.y-1][0]*8;
  site_flag.y=v_set_up_down[arrow.y-1][1]*16;                 
  LCD_str(site_flag,"  ", BLUE,BLUE);
  if(key==UP)
  {
    arrow.y=arrow.y-1>=1?(arrow.y-1):v_set_page_num2;
  }
  else
  {
    arrow.y=arrow.y+1<=v_set_page_num2?(arrow.y+1):1;
  }
  site_flag.y=v_set_up_down[arrow.y-1][1]*16;
  LCD_str(site_flag,"->", WHITE,BLUE); 
}

/***********************************************************************************************************************************/

void v_set_arrow_lr2(uint8 key)
{
  Site_t site_flag;
  if(key==LEFT)
  {
    site_flag.x=v_set_r[arrow.y-1][0]*8;
    site_flag.y=v_set_r[arrow.y-1][1]*16;
    LCD_str(site_flag,"  ", BLUE,BLUE);
    
    site_flag.x=v_set_l[arrow.y-1][0]*8;
    site_flag.y=v_set_l[arrow.y-1][1]*16;
    LCD_str(site_flag,"->", WHITE,BLUE);
    v_set2_state=V_SET2_UN_ADJUST;
    arrow.x=1;
  }
  else
  {
    site_flag.x=v_set_l[arrow.y-1][0]*8;
    site_flag.y=v_set_l[arrow.y-1][1]*16;
    LCD_str(site_flag,"  ", BLUE,BLUE);  
    
    site_flag.x=v_set_r[arrow.y-1][0]*8;
    site_flag.y=v_set_r[arrow.y-1][1]*16;
    LCD_str(site_flag,"->", WHITE,BLUE);
    v_set2_state=V_SET2_ADJUST;
    arrow.x=2;
    
  }
}

/***********************************************************************************************************************************/

void v_set_adjust2(uint8 key)
{
  
  if(arrow.y==1)      //motor_step
  { 
    if(key==UP)
    {
      for(uint8 i=0;i<5;i++)         //最后一个  不作考虑
      {
        if(ABS(v_set_step_up_down[i]-v_set_value2[0])<0.01)
        {
          v_set_value2[0]=v_set_step_up_down[i+1];
          break;
        }
        
      }  
      
      
    }
    else
    {
      for(uint8 i=5;i>0;i--)
      {
        if(ABS(v_set_step_up_down[i]-v_set_value2[0])<0.01)
        {
          v_set_value2[0]=v_set_step_up_down[i-1];
          break;
        }
      }  
    }
  }
  else if(arrow.y<=v_set_page_num2)
  {
    int16 linshi;
    if(key==UP)
    {
      linshi=v_set_value2[arrow.y-1]+v_set_value2[0]; 
    }
    else 
    {
      linshi=v_set_value2[arrow.y-1]-v_set_value2[0];
    }
    v_set_value2[arrow.y-1]=MIN(MAX(linshi,50),2000);
    
  }
  //v_set_value[arrow.y-1]= v_set_value[arrow.y-1]>0?v_set_value[arrow.y-1]:0;     //避免负数********
  
  else           //保存数据到flash中
  {
    //save_data();
  }
  speed_step=v_set_value2[0];
  speed_poa=v_set_value2[1]; 
  speed_pob=v_set_value2[2]; 
  speed_za=v_set_value2[3];
  speed_zb=v_set_value2[4]; 
  speed_ha=v_set_value2[5]; 
  speed_hb=v_set_value2[6]; 
  Site_t site;
  site.x=v_set_value_x[arrow.y-1]*8;           //只与y有关
  site.y=v_set_value_y[arrow.y-1]*16;
  for(uint8 i=v_set_value_x[0];i<16;i++)
  { 
    site.x=i*8;
    LCD_char(site,' ', BLUE,BLUE);
  }
  
  site.x=v_set_value_x[arrow.y-1]*8;           //只与y有关
  site.y=v_set_value_y[arrow.y-1]*16;
  LCD_num(site,v_set_value2[arrow.y-1], WHITE,BLUE);
}

/***********************************************************************************************************************************/

void distance_parameter_show()
{
  
  Site_t site;
  //文字刷新
  for(uint8 i=0;i<8;i++)
  {
    site.x=distance_msg_x[i]*8;
    site.y=distance_msg_y[i]*16;
    LCD_str(site,distance_sg[i],WHITE ,BLUE);     //标题  
  }
  //数值刷新
  for(uint8 i=0;i<7;i++)
  {
    site.x=distance_pid_value_x[i]*8;
    site.y=distance_pid_value_y[i]*16;
    lcd_double_show(site,distance_pid_value[i]);
    //            //数值
  }
  //箭头刷新
  site.x=0; site.y=1*16;
  LCD_str(site,"->",WHITE ,BLUE);     //标题  
  //更新记录
  
}

/***********************************************************************************************************************************/

void distance_page_up_down(uint8 key)
{
  Site_t site_flag;
  site_flag.x=distance_up_down[arrow.y-1][0]*8;
  site_flag.y=distance_up_down[arrow.y-1][1]*16;                 
  LCD_str(site_flag,"  ", BLUE,BLUE);
  if(key==UP)
  {
    arrow.y=arrow.y-1>=1?(arrow.y-1):distance_page_num;
  }
  else
  {
    arrow.y=arrow.y+1<=distance_page_num?(arrow.y+1):1;
  }
  site_flag.y=distance_up_down[arrow.y-1][1]*16;
  LCD_str(site_flag,"->", WHITE,BLUE);    
  
}

/***********************************************************************************************************************************/

void distance_arrow_lr(uint8 key)
{
  Site_t site_flag;
  if(key==LEFT)
  {
    site_flag.x=distance_r[arrow.y-1][0]*8;
    site_flag.y=distance_r[arrow.y-1][1]*16;
    LCD_str(site_flag,"  ", BLUE,BLUE);
    
    site_flag.x=distance_l[arrow.y-1][0]*8;
    site_flag.y=distance_l[arrow.y-1][1]*16;
    LCD_str(site_flag,"->", WHITE,BLUE);
    distance_state=DISTANCE_UN_ADJUST;
    arrow.x=1;
  }
  else
  {
    site_flag.x=distance_l[arrow.y-1][0]*8;
    site_flag.y=distance_l[arrow.y-1][1]*16;
    LCD_str(site_flag,"  ", BLUE,BLUE);  
    
    site_flag.x=distance_r[arrow.y-1][0]*8;
    site_flag.y=distance_r[arrow.y-1][1]*16;
    LCD_str(site_flag,"->", WHITE,BLUE);
    distance_state=DISTANCE_ADJUST;
    arrow.x=2;
    
  }
  
}

/***********************************************************************************************************************************/

void distance_pid_adjust(uint8 key)
{
  
  if(arrow.y==1)      //motor_step
  { 
    
    if(key==UP)
    {
      for(uint8 i=0;i<3;i++)         //最后一个  不作考虑
      {
        if(fabsf(distance_step_up_down[i]-distance_pid_value[0])<0.001)
        {
          distance_pid_value[0]=distance_step_up_down[i+1];
          break;
        }      
      }  
    }
    else
    {
      for(uint8 i=3;i>0;i--)
      {
        if(fabsf(distance_step_up_down[i]-distance_pid_value[0])<0.001)
        {
          distance_pid_value[0]=distance_step_up_down[i-1];
          break;
        }
      }  
    }
  }
  else if(arrow.y<=distance_page_num)
  {
    if(key==UP)
      distance_pid_value[arrow.y-1]+=distance_pid_value[0];
    else
      distance_pid_value[arrow.y-1]-=distance_pid_value[0];
    
    distance_pid_value[arrow.y-1]= distance_pid_value[arrow.y-1]>0?distance_pid_value[arrow.y-1]:0;     //避免负数********
  }
  else
  {
    
  }
  distance_step=distance_pid_value[0];
  distance_a=distance_pid_value[1]; 
  distance_b=distance_pid_value[2]; 
  distance_c=distance_pid_value[3]; 
  distance_d=distance_pid_value[4];
  distance_e=distance_pid_value[5];
  distance_f=distance_pid_value[6];
  
  Site_t site;
  site.x=pid_value_x[arrow.y-1]*8;           //只与y有关
  site.y=pid_value_y[arrow.y-1]*16;
  for(uint8 i=distance_pid_value_x[0];i<16;i++)
  { 
    site.x=i*8;
    LCD_char(site,' ', BLUE,BLUE);
  }
  
  site.x=distance_pid_value_x[arrow.y-1]*8;           //只与y有关
  site.y=distance_pid_value_y[arrow.y-1]*16;
  lcd_double_show(site,distance_pid_value[arrow.y-1]); 
}

/***********************************************************************************************************************************/

void distance2_parameter_show()
{
  
  Site_t site;
  //文字刷新
  for(uint8 i=0;i<8;i++)
  {
    site.x=distance2_msg_x[i]*8;
    site.y=distance2_msg_y[i]*16;
    LCD_str(site,distance2_sg[i],WHITE ,BLUE);     //标题  
  }
  //数值刷新
  for(uint8 i=0;i<7;i++)
  {
    site.x=distance2_pid_value_x[i]*8;
    site.y=distance2_pid_value_y[i]*16;
    lcd_double_show(site,distance2_pid_value[i]);
    //            //数值
  }
  //箭头刷新
  site.x=0; site.y=1*16;
  LCD_str(site,"->",WHITE ,BLUE);     //标题  
  //更新记录
  
}

/***********************************************************************************************************************************/

void distance2_page_up_down(uint8 key)
{
  Site_t site_flag;
  site_flag.x=distance2_up_down[arrow.y-1][0]*8;
  site_flag.y=distance2_up_down[arrow.y-1][1]*16;                 
  LCD_str(site_flag,"  ", BLUE,BLUE);
  if(key==UP)
  {
    arrow.y=arrow.y-1>=1?(arrow.y-1):distance2_page_num;
  }
  else
  {
    arrow.y=arrow.y+1<=distance2_page_num?(arrow.y+1):1;
  }
  site_flag.y=distance2_up_down[arrow.y-1][1]*16;
  LCD_str(site_flag,"->", WHITE,BLUE);    
  
}

/***********************************************************************************************************************************/

void distance2_arrow_lr(uint8 key)
{
  Site_t site_flag;
  if(key==LEFT)
  {
    site_flag.x=distance2_r[arrow.y-1][0]*8;
    site_flag.y=distance2_r[arrow.y-1][1]*16;
    LCD_str(site_flag,"  ", BLUE,BLUE);
    
    site_flag.x=distance2_l[arrow.y-1][0]*8;
    site_flag.y=distance2_l[arrow.y-1][1]*16;
    LCD_str(site_flag,"->", WHITE,BLUE);
    distance2_state=DISTANCE2_UN_ADJUST;
    arrow.x=1;
  }
  else
  {
    site_flag.x=distance2_l[arrow.y-1][0]*8;
    site_flag.y=distance2_l[arrow.y-1][1]*16;
    LCD_str(site_flag,"  ", BLUE,BLUE);  
    
    site_flag.x=distance2_r[arrow.y-1][0]*8;
    site_flag.y=distance2_r[arrow.y-1][1]*16;
    LCD_str(site_flag,"->", WHITE,BLUE);
    distance2_state=DISTANCE2_ADJUST;
    arrow.x=2;
    
  }
  
}

/***********************************************************************************************************************************/

void distance2_pid_adjust(uint8 key)
{
  
  if(arrow.y==1)      //motor_step
  { 
    
    if(key==UP)
    {
      for(uint8 i=0;i<3;i++)         //最后一个  不作考虑
      {
        if(fabsf(distance2_step_up_down[i]-distance2_pid_value[0])<0.001)
        {
          distance2_pid_value[0]=distance2_step_up_down[i+1];
          break;
        }      
      }  
    }
    else
    {
      for(uint8 i=3;i>0;i--)
      {
        if(fabsf(distance2_step_up_down[i]-distance2_pid_value[0])<0.001)
        {
          distance2_pid_value[0]=distance2_step_up_down[i-1];
          break;
        }
      }  
    }
  }
  else if(arrow.y<=distance2_page_num)
  {
    if(key==UP)
      distance2_pid_value[arrow.y-1]+=distance2_pid_value[0];
    else
      distance2_pid_value[arrow.y-1]-=distance2_pid_value[0];
    
    distance2_pid_value[arrow.y-1]= distance2_pid_value[arrow.y-1]>0?distance2_pid_value[arrow.y-1]:0;     //避免负数********
  }
  else
  {
    
  }
  distance2_step=distance2_pid_value[0];
  distance2_a=distance2_pid_value[1]; 
  distance2_b=distance2_pid_value[2]; 
  distance2_c=distance2_pid_value[3]; 
  distance2_d=distance2_pid_value[4];
  distance2_e=distance2_pid_value[5];
  distance2_f=distance2_pid_value[6];
  
  Site_t site;
  site.x=pid_value_x[arrow.y-1]*8;           //只与y有关
  site.y=pid_value_y[arrow.y-1]*16;
  for(uint8 i=distance2_pid_value_x[0];i<16;i++)
  { 
    site.x=i*8;
    LCD_char(site,' ', BLUE,BLUE);
  }
  
  site.x=distance2_pid_value_x[arrow.y-1]*8;           //只与y有关
  site.y=distance2_pid_value_y[arrow.y-1]*16;
  lcd_double_show(site,distance2_pid_value[arrow.y-1]); 
}


/***********************************************************************************************************************************/

void cnt_parameter_show()
{
  Site_t site;
  //文字刷新
  for(uint8 i=0;i<cnt_page_num+1;i++)
  {
    site.x=cnt_msg_x[i]*8;
    site.y=cnt_msg_y[i]*16;
    LCD_str(site,cnt_sg[i],WHITE ,BLUE);     //标题  
  }
  //数值刷新
  for(uint8 i=0;i<cnt_page_num;i++)
  {
    site.x=cnt_value_x[i]*8;
    site.y=cnt_value_y[i]*16;
    //lcd_double_show(site,cnt_value[i]);
    LCD_num(site,cnt_value[i], WHITE,BLUE);
    //              //数值
  }
  //箭头刷新
  site.x=0; site.y=1*16;
  LCD_str(site,"->",WHITE ,BLUE);     //标题  
  //更新记录
}

/***********************************************************************************************************************************/

void cnt_page_up_down(uint8 key)
{
  Site_t site_flag;
  site_flag.x=cnt_up_down[arrow.y-1][0]*8;
  site_flag.y=cnt_up_down[arrow.y-1][1]*16;                 
  LCD_str(site_flag,"  ", BLUE,BLUE);
  if(key==UP)
  {
    arrow.y=arrow.y-1>=1?(arrow.y-1):cnt_page_num;
  }
  else
  {
    arrow.y=arrow.y+1<=cnt_page_num?(arrow.y+1):1;
  }
  site_flag.y=cnt_up_down[arrow.y-1][1]*16;
  LCD_str(site_flag,"->", WHITE,BLUE); 
}

/***********************************************************************************************************************************/

void cnt_arrow_lr(uint8 key)
{
  Site_t site_flag;
  if(key==LEFT)
  {
    site_flag.x=cnt_r[arrow.y-1][0]*8;
    site_flag.y=cnt_r[arrow.y-1][1]*16;
    LCD_str(site_flag,"  ", BLUE,BLUE);
    
    site_flag.x=cnt_l[arrow.y-1][0]*8;
    site_flag.y=cnt_l[arrow.y-1][1]*16;
    LCD_str(site_flag,"->", WHITE,BLUE);
    cnt_state=CNT_UN_ADJUST;
    arrow.x=1;
  }
  else
  {
    site_flag.x=cnt_l[arrow.y-1][0]*8;
    site_flag.y=cnt_l[arrow.y-1][1]*16;
    LCD_str(site_flag,"  ", BLUE,BLUE);  
    
    site_flag.x=cnt_r[arrow.y-1][0]*8;
    site_flag.y=cnt_r[arrow.y-1][1]*16;
    LCD_str(site_flag,"->", WHITE,BLUE);
    cnt_state=CNT_ADJUST;
    arrow.x=2;
    
  }
}

/***********************************************************************************************************************************/

void cnt_adjust(uint8 key)
{
  
  if(arrow.y==1)      //motor_step
  { 
    if(key==UP)
    {
      for(uint8 i=0;i<5;i++)         //最后一个  不作考虑
      {
        if(ABS(cnt_step_up_down[i]-cnt_value[0])<0.01)
        {
          cnt_value[0]=cnt_step_up_down[i+1];
          break;
        }
        
      }  
      
      
    }
    else
    {
      for(uint8 i=5;i>0;i--)
      {
        if(ABS(cnt_step_up_down[i]-cnt_value[0])<0.01)
        {
          cnt_value[0]=cnt_step_up_down[i-1];
          break;
        }
      }  
    }
  }
  else if(arrow.y<=cnt_page_num)
  {
    int16 linshi;
    if(key==UP)
    {
      linshi=cnt_value[arrow.y-1]+cnt_value[0]; 
    }
    else 
    {
      linshi=cnt_value[arrow.y-1]-cnt_value[0];
    }
    cnt_value[arrow.y-1]=MIN(MAX(linshi,0),2000);
    
  }
  //cnt_value[arrow.y-1]= cnt_value[arrow.y-1]>0?cnt_value[arrow.y-1]:0;     //避免负数********
  
  else           //保存数据到flash中
  {
    //save_data();
  }
  cnt_step=cnt_value[0];
  cnt_a=cnt_value[1]; 
  cnt_b=cnt_value[2]; 
  cnt_c=cnt_value[3];
  cnt_d=cnt_value[4]; 
  cnt_e=cnt_value[5]; 
  cnt_f=cnt_value[6]; 
  Site_t site;
  site.x=cnt_value_x[arrow.y-1]*8;           //只与y有关
  site.y=cnt_value_y[arrow.y-1]*16;
  for(uint8 i=cnt_value_x[0];i<16;i++)
  { 
    site.x=i*8;
    LCD_char(site,' ', BLUE,BLUE);
  }
  
  site.x=cnt_value_x[arrow.y-1]*8;           //只与y有关
  site.y=cnt_value_y[arrow.y-1]*16;
  LCD_num(site,cnt_value[arrow.y-1], WHITE,BLUE);
}

/***********************************************************************************************************************************/

void other1_parameter_show()
{
  
  Site_t site;
  //文字刷新
  for(uint8 i=0;i<8;i++)
  {
    site.x=other1_msg_x[i]*8;
    site.y=other1_msg_y[i]*16;
    LCD_str(site,other1_sg[i],WHITE ,BLUE);     //标题  
  }
  //数值刷新
  for(uint8 i=0;i<7;i++)
  {
    site.x=other1_pid_value_x[i]*8;
    site.y=other1_pid_value_y[i]*16;
    lcd_double_show(site,other1_pid_value[i]);
    //            //数值
  }
  //箭头刷新
  site.x=0; site.y=1*16;
  LCD_str(site,"->",WHITE ,BLUE);     //标题  
  //更新记录
  
}

/***********************************************************************************************************************************/

void other1_page_up_down(uint8 key)
{
  Site_t site_flag;
  site_flag.x=other1_up_down[arrow.y-1][0]*8;
  site_flag.y=other1_up_down[arrow.y-1][1]*16;                 
  LCD_str(site_flag,"  ", BLUE,BLUE);
  if(key==UP)
  {
    arrow.y=arrow.y-1>=1?(arrow.y-1):other1_page_num;
  }
  else
  {
    arrow.y=arrow.y+1<=other1_page_num?(arrow.y+1):1;
  }
  site_flag.y=other1_up_down[arrow.y-1][1]*16;
  LCD_str(site_flag,"->", WHITE,BLUE);    
  
}

/***********************************************************************************************************************************/

void other1_arrow_lr(uint8 key)
{
  Site_t site_flag;
  if(key==LEFT)
  {
    site_flag.x=other1_r[arrow.y-1][0]*8;
    site_flag.y=other1_r[arrow.y-1][1]*16;
    LCD_str(site_flag,"  ", BLUE,BLUE);
    
    site_flag.x=other1_l[arrow.y-1][0]*8;
    site_flag.y=other1_l[arrow.y-1][1]*16;
    LCD_str(site_flag,"->", WHITE,BLUE);
    other1_state=SERVO_UN_ADJUST;
    arrow.x=1;
  }
  else
  {
    site_flag.x=other1_l[arrow.y-1][0]*8;
    site_flag.y=other1_l[arrow.y-1][1]*16;
    LCD_str(site_flag,"  ", BLUE,BLUE);  
    
    site_flag.x=other1_r[arrow.y-1][0]*8;
    site_flag.y=other1_r[arrow.y-1][1]*16;
    LCD_str(site_flag,"->", WHITE,BLUE);
    other1_state=SERVO_ADJUST;
    arrow.x=2;
    
  }
  
}

/***********************************************************************************************************************************/

void other1_pid_adjust(uint8 key)
{
  
  if(arrow.y==1)      //motor_step
  { 
    
    if(key==UP)
    {
      for(uint8 i=0;i<3;i++)         //最后一个  不作考虑
      {
        if(fabsf(other1_step_up_down[i]-other1_pid_value[0])<0.001)
        {
          other1_pid_value[0]=other1_step_up_down[i+1];
          break;
        }      
      }  
    }
    else
    {
      for(uint8 i=3;i>0;i--)
      {
        if(fabsf(other1_step_up_down[i]-other1_pid_value[0])<0.001)
        {
          other1_pid_value[0]=other1_step_up_down[i-1];
          break;
        }
      }  
    }
  }
  else if(arrow.y<=other1_page_num)
  {
    if(key==UP)
      other1_pid_value[arrow.y-1]+=other1_pid_value[0];
    else
      other1_pid_value[arrow.y-1]-=other1_pid_value[0];
    
    other1_pid_value[arrow.y-1]= other1_pid_value[arrow.y-1]>0?other1_pid_value[arrow.y-1]:0;     //避免负数********
  }
  else
  {
    
  }
  other1_step=other1_pid_value[0];
  other1_a=other1_pid_value[1]; 
  other1_b=other1_pid_value[2]; 
  other1_c=other1_pid_value[3]; 
  other1_d=other1_pid_value[4];
  other1_e=other1_pid_value[5];
  other1_f=other1_pid_value[6];
  
  Site_t site;
  site.x=pid_value_x[arrow.y-1]*8;           //只与y有关
  site.y=pid_value_y[arrow.y-1]*16;
  for(uint8 i=other1_pid_value_x[0];i<16;i++)
  { 
    site.x=i*8;
    LCD_char(site,' ', BLUE,BLUE);
  }
  
  site.x=other1_pid_value_x[arrow.y-1]*8;           //只与y有关
  site.y=other1_pid_value_y[arrow.y-1]*16;
  lcd_double_show(site,other1_pid_value[arrow.y-1]); 
}

/***********************************************************************************************************************************/

void other2_parameter_show()
{
  
  Site_t site;
  //文字刷新
  for(uint8 i=0;i<8;i++)
  {
    site.x=other2_msg_x[i]*8;
    site.y=other2_msg_y[i]*16;
    LCD_str(site,other2_sg[i],WHITE ,BLUE);     //标题  
  }
  //数值刷新
  for(uint8 i=0;i<7;i++)
  {
    site.x=other2_pid_value_x[i]*8;
    site.y=other2_pid_value_y[i]*16;
    lcd_double_show(site,other2_pid_value[i]);
    //            //数值
  }
  //箭头刷新
  site.x=0; site.y=1*16;
  LCD_str(site,"->",WHITE ,BLUE);     //标题  
  //更新记录
  
}

/***********************************************************************************************************************************/

void other2_page_up_down(uint8 key)
{
  Site_t site_flag;
  site_flag.x=other2_up_down[arrow.y-1][0]*8;
  site_flag.y=other2_up_down[arrow.y-1][1]*16;                 
  LCD_str(site_flag,"  ", BLUE,BLUE);
  if(key==UP)
  {
    arrow.y=arrow.y-1>=1?(arrow.y-1):other2_page_num;
  }
  else
  {
    arrow.y=arrow.y+1<=other2_page_num?(arrow.y+1):1;
  }
  site_flag.y=other2_up_down[arrow.y-1][1]*16;
  LCD_str(site_flag,"->", WHITE,BLUE);    
  
}

/***********************************************************************************************************************************/

void other2_arrow_lr(uint8 key)
{
  Site_t site_flag;
  if(key==LEFT)
  {
    site_flag.x=other2_r[arrow.y-1][0]*8;
    site_flag.y=other2_r[arrow.y-1][1]*16;
    LCD_str(site_flag,"  ", BLUE,BLUE);
    
    site_flag.x=other2_l[arrow.y-1][0]*8;
    site_flag.y=other2_l[arrow.y-1][1]*16;
    LCD_str(site_flag,"->", WHITE,BLUE);
    other2_state=SERVO_UN_ADJUST;
    arrow.x=1;
  }
  else
  {
    site_flag.x=other2_l[arrow.y-1][0]*8;
    site_flag.y=other2_l[arrow.y-1][1]*16;
    LCD_str(site_flag,"  ", BLUE,BLUE);  
    
    site_flag.x=other2_r[arrow.y-1][0]*8;
    site_flag.y=other2_r[arrow.y-1][1]*16;
    LCD_str(site_flag,"->", WHITE,BLUE);
    other2_state=SERVO_ADJUST;
    arrow.x=2;
    
  }
  
}

/***********************************************************************************************************************************/

void other2_pid_adjust(uint8 key)
{
  
  if(arrow.y==1)      //motor_step
  { 
    
    if(key==UP)
    {
      for(uint8 i=0;i<3;i++)         //最后一个  不作考虑
      {
        if(fabsf(other2_step_up_down[i]-other2_pid_value[0])<0.001)
        {
          other2_pid_value[0]=other2_step_up_down[i+1];
          break;
        }      
      }  
    }
    else
    {
      for(uint8 i=3;i>0;i--)
      {
        if(fabsf(other2_step_up_down[i]-other2_pid_value[0])<0.001)
        {
          other2_pid_value[0]=other2_step_up_down[i-1];
          break;
        }
      }  
    }
  }
  else if(arrow.y<=other2_page_num)
  {
    if(key==UP)
      other2_pid_value[arrow.y-1]+=other2_pid_value[0];
    else
      other2_pid_value[arrow.y-1]-=other2_pid_value[0];
    
    other2_pid_value[arrow.y-1]= other2_pid_value[arrow.y-1]>0?other2_pid_value[arrow.y-1]:0;     //避免负数********
  }
  else
  {
    
  }
  other2_step=other2_pid_value[0];
  other2_a=other2_pid_value[1]; 
  other2_b=other2_pid_value[2]; 
  other2_c=other2_pid_value[3]; 
  other2_d=other2_pid_value[4];
  other2_e=other2_pid_value[5];
  other2_f=other2_pid_value[6];
  
  Site_t site;
  site.x=pid_value_x[arrow.y-1]*8;           //只与y有关
  site.y=pid_value_y[arrow.y-1]*16;
  for(uint8 i=other2_pid_value_x[0];i<16;i++)
  { 
    site.x=i*8;
    LCD_char(site,' ', BLUE,BLUE);
  }
  
  site.x=other2_pid_value_x[arrow.y-1]*8;           //只与y有关
  site.y=other2_pid_value_y[arrow.y-1]*16;
  lcd_double_show(site,other2_pid_value[arrow.y-1]); 
}

/***********************************************************************************************************************************/

void other3_parameter_show()
{
  
  Site_t site;
  //文字刷新
  for(uint8 i=0;i<8;i++)
  {
    site.x=other3_msg_x[i]*8;
    site.y=other3_msg_y[i]*16;
    LCD_str(site,other3_sg[i],WHITE ,BLUE);     //标题  
  }
  //数值刷新
  for(uint8 i=0;i<7;i++)
  {
    site.x=other3_pid_value_x[i]*8;
    site.y=other3_pid_value_y[i]*16;
    lcd_double_show(site,other3_pid_value[i]);
    //            //数值
  }
  //箭头刷新
  site.x=0; site.y=1*16;
  LCD_str(site,"->",WHITE ,BLUE);     //标题  
  //更新记录
  
}

/***********************************************************************************************************************************/

void other3_page_up_down(uint8 key)
{
  Site_t site_flag;
  site_flag.x=other3_up_down[arrow.y-1][0]*8;
  site_flag.y=other3_up_down[arrow.y-1][1]*16;                 
  LCD_str(site_flag,"  ", BLUE,BLUE);
  if(key==UP)
  {
    arrow.y=arrow.y-1>=1?(arrow.y-1):other3_page_num;
  }
  else
  {
    arrow.y=arrow.y+1<=other3_page_num?(arrow.y+1):1;
  }
  site_flag.y=other3_up_down[arrow.y-1][1]*16;
  LCD_str(site_flag,"->", WHITE,BLUE);    
  
}

/***********************************************************************************************************************************/

void other3_arrow_lr(uint8 key)
{
  Site_t site_flag;
  if(key==LEFT)
  {
    site_flag.x=other3_r[arrow.y-1][0]*8;
    site_flag.y=other3_r[arrow.y-1][1]*16;
    LCD_str(site_flag,"  ", BLUE,BLUE);
    
    site_flag.x=other3_l[arrow.y-1][0]*8;
    site_flag.y=other3_l[arrow.y-1][1]*16;
    LCD_str(site_flag,"->", WHITE,BLUE);
    other3_state=SERVO_UN_ADJUST;
    arrow.x=1;
  }
  else
  {
    site_flag.x=other3_l[arrow.y-1][0]*8;
    site_flag.y=other3_l[arrow.y-1][1]*16;
    LCD_str(site_flag,"  ", BLUE,BLUE);  
    
    site_flag.x=other3_r[arrow.y-1][0]*8;
    site_flag.y=other3_r[arrow.y-1][1]*16;
    LCD_str(site_flag,"->", WHITE,BLUE);
    other3_state=SERVO_ADJUST;
    arrow.x=2;
    
  }
  
}

/***********************************************************************************************************************************/

void other3_pid_adjust(uint8 key)
{
  
  if(arrow.y==1)      //motor_step
  { 
    
    if(key==UP)
    {
      for(uint8 i=0;i<3;i++)         //最后一个  不作考虑
      {
        if(fabsf(other3_step_up_down[i]-other3_pid_value[0])<0.001)
        {
          other3_pid_value[0]=other3_step_up_down[i+1];
          break;
        }      
      }  
    }
    else
    {
      for(uint8 i=3;i>0;i--)
      {
        if(fabsf(other3_step_up_down[i]-other3_pid_value[0])<0.001)
        {
          other3_pid_value[0]=other3_step_up_down[i-1];
          break;
        }
      }  
    }
  }
  else if(arrow.y<=other3_page_num)
  {
    if(key==UP)
      other3_pid_value[arrow.y-1]+=other3_pid_value[0];
    else
      other3_pid_value[arrow.y-1]-=other3_pid_value[0];
    
    other3_pid_value[arrow.y-1]= other3_pid_value[arrow.y-1]>0?other3_pid_value[arrow.y-1]:0;     //避免负数********
  }
  else
  {
    
  }
  other3_step=other3_pid_value[0];
  other3_a=other3_pid_value[1]; 
  other3_b=other3_pid_value[2]; 
  other3_c=other3_pid_value[3]; 
  other3_d=other3_pid_value[4];
  other3_e=other3_pid_value[5];
  other3_f=other3_pid_value[6];
  
  Site_t site;
  site.x=pid_value_x[arrow.y-1]*8;           //只与y有关
  site.y=pid_value_y[arrow.y-1]*16;
  for(uint8 i=other3_pid_value_x[0];i<16;i++)
  { 
    site.x=i*8;
    LCD_char(site,' ', BLUE,BLUE);
  }
  
  site.x=other3_pid_value_x[arrow.y-1]*8;           //只与y有关
  site.y=other3_pid_value_y[arrow.y-1]*16;
  lcd_double_show(site,other3_pid_value[arrow.y-1]); 
}

/***********************************************************************************************************************************/

void other4_parameter_show()
{
  
  Site_t site;
  //文字刷新
  for(uint8 i=0;i<8;i++)
  {
    site.x=other4_msg_x[i]*8;
    site.y=other4_msg_y[i]*16;
    LCD_str(site,other4_sg[i],WHITE ,BLUE);     //标题  
  }
  //数值刷新
  for(uint8 i=0;i<7;i++)
  {
    site.x=other4_pid_value_x[i]*8;
    site.y=other4_pid_value_y[i]*16;
    lcd_double_show(site,other4_pid_value[i]);
    //            //数值
  }
  //箭头刷新
  site.x=0; site.y=1*16;
  LCD_str(site,"->",WHITE ,BLUE);     //标题  
  //更新记录
  
}

/***********************************************************************************************************************************/

void other4_page_up_down(uint8 key)
{
  Site_t site_flag;
  site_flag.x=other4_up_down[arrow.y-1][0]*8;
  site_flag.y=other4_up_down[arrow.y-1][1]*16;                 
  LCD_str(site_flag,"  ", BLUE,BLUE);
  if(key==UP)
  {
    arrow.y=arrow.y-1>=1?(arrow.y-1):other4_page_num;
  }
  else
  {
    arrow.y=arrow.y+1<=other4_page_num?(arrow.y+1):1;
  }
  site_flag.y=other4_up_down[arrow.y-1][1]*16;
  LCD_str(site_flag,"->", WHITE,BLUE);    
  
}

/***********************************************************************************************************************************/

void other4_arrow_lr(uint8 key)
{
  Site_t site_flag;
  if(key==LEFT)
  {
    site_flag.x=other4_r[arrow.y-1][0]*8;
    site_flag.y=other4_r[arrow.y-1][1]*16;
    LCD_str(site_flag,"  ", BLUE,BLUE);
    
    site_flag.x=other4_l[arrow.y-1][0]*8;
    site_flag.y=other4_l[arrow.y-1][1]*16;
    LCD_str(site_flag,"->", WHITE,BLUE);
    other4_state=SERVO_UN_ADJUST;
    arrow.x=1;
  }
  else
  {
    site_flag.x=other4_l[arrow.y-1][0]*8;
    site_flag.y=other4_l[arrow.y-1][1]*16;
    LCD_str(site_flag,"  ", BLUE,BLUE);  
    
    site_flag.x=other4_r[arrow.y-1][0]*8;
    site_flag.y=other4_r[arrow.y-1][1]*16;
    LCD_str(site_flag,"->", WHITE,BLUE);
    other4_state=SERVO_ADJUST;
    arrow.x=2;
    
  }
  
}

/***********************************************************************************************************************************/

void other4_pid_adjust(uint8 key)
{
  
  if(arrow.y==1)      //motor_step
  { 
    
    if(key==UP)
    {
      for(uint8 i=0;i<3;i++)         //最后一个  不作考虑
      {
        if(fabsf(other4_step_up_down[i]-other4_pid_value[0])<0.001)
        {
          other4_pid_value[0]=other4_step_up_down[i+1];
          break;
        }      
      }  
    }
    else
    {
      for(uint8 i=3;i>0;i--)
      {
        if(fabsf(other4_step_up_down[i]-other4_pid_value[0])<0.001)
        {
          other4_pid_value[0]=other4_step_up_down[i-1];
          break;
        }
      }  
    }
  }
  else if(arrow.y<=other4_page_num)
  {
    if(key==UP)
      other4_pid_value[arrow.y-1]+=other4_pid_value[0];
    else
      other4_pid_value[arrow.y-1]-=other4_pid_value[0];
    
    other4_pid_value[arrow.y-1]= other4_pid_value[arrow.y-1]>0?other4_pid_value[arrow.y-1]:0;     //避免负数********
  }
  else
  {
    
  }
  other4_step=other4_pid_value[0];
  other4_a=other4_pid_value[1]; 
  other4_b=other4_pid_value[2]; 
  other4_c=other4_pid_value[3]; 
  other4_d=other4_pid_value[4];
  other4_e=other4_pid_value[5];
  other4_f=other4_pid_value[6];
  
  Site_t site;
  site.x=pid_value_x[arrow.y-1]*8;           //只与y有关
  site.y=pid_value_y[arrow.y-1]*16;
  for(uint8 i=other4_pid_value_x[0];i<16;i++)
  { 
    site.x=i*8;
    LCD_char(site,' ', BLUE,BLUE);
  }
  
  site.x=other4_pid_value_x[arrow.y-1]*8;           //只与y有关
  site.y=other4_pid_value_y[arrow.y-1]*16;
  lcd_double_show(site,other4_pid_value[arrow.y-1]); 
}


/***********************************************************************************************************************************/

void other5_parameter_show()
{
  
  Site_t site;
  //文字刷新
  for(uint8 i=0;i<8;i++)
  {
    site.x=other5_msg_x[i]*8;
    site.y=other5_msg_y[i]*16;
    LCD_str(site,other5_sg[i],WHITE ,BLUE);     //标题  
  }
  //数值刷新
  for(uint8 i=0;i<7;i++)
  {
    site.x=other5_pid_value_x[i]*8;
    site.y=other5_pid_value_y[i]*16;
    lcd_double_show(site,other5_pid_value[i]);
    //            //数值
  }
  //箭头刷新
  site.x=0; site.y=1*16;
  LCD_str(site,"->",WHITE ,BLUE);     //标题  
  //更新记录
  
}

/***********************************************************************************************************************************/

void other5_page_up_down(uint8 key)
{
  Site_t site_flag;
  site_flag.x=other5_up_down[arrow.y-1][0]*8;
  site_flag.y=other5_up_down[arrow.y-1][1]*16;                 
  LCD_str(site_flag,"  ", BLUE,BLUE);
  if(key==UP)
  {
    arrow.y=arrow.y-1>=1?(arrow.y-1):other5_page_num;
  }
  else
  {
    arrow.y=arrow.y+1<=other5_page_num?(arrow.y+1):1;
  }
  site_flag.y=other5_up_down[arrow.y-1][1]*16;
  LCD_str(site_flag,"->", WHITE,BLUE);    
  
}

/***********************************************************************************************************************************/

void other5_arrow_lr(uint8 key)
{
  Site_t site_flag;
  if(key==LEFT)
  {
    site_flag.x=other5_r[arrow.y-1][0]*8;
    site_flag.y=other5_r[arrow.y-1][1]*16;
    LCD_str(site_flag,"  ", BLUE,BLUE);
    
    site_flag.x=other5_l[arrow.y-1][0]*8;
    site_flag.y=other5_l[arrow.y-1][1]*16;
    LCD_str(site_flag,"->", WHITE,BLUE);
    other5_state=SERVO_UN_ADJUST;
    arrow.x=1;
  }
  else
  {
    site_flag.x=other5_l[arrow.y-1][0]*8;
    site_flag.y=other5_l[arrow.y-1][1]*16;
    LCD_str(site_flag,"  ", BLUE,BLUE);  
    
    site_flag.x=other5_r[arrow.y-1][0]*8;
    site_flag.y=other5_r[arrow.y-1][1]*16;
    LCD_str(site_flag,"->", WHITE,BLUE);
    other5_state=SERVO_ADJUST;
    arrow.x=2;
    
  }
  
}

/***********************************************************************************************************************************/

void other5_pid_adjust(uint8 key)
{
  
  if(arrow.y==1)      //motor_step
  { 
    
    if(key==UP)
    {
      for(uint8 i=0;i<3;i++)         //最后一个  不作考虑
      {
        if(fabsf(other5_step_up_down[i]-other5_pid_value[0])<0.001)
        {
          other5_pid_value[0]=other5_step_up_down[i+1];
          break;
        }      
      }  
    }
    else
    {
      for(uint8 i=3;i>0;i--)
      {
        if(fabsf(other5_step_up_down[i]-other5_pid_value[0])<0.001)
        {
          other5_pid_value[0]=other5_step_up_down[i-1];
          break;
        }
      }  
    }
  }
  else if(arrow.y<=other5_page_num)
  {
    if(key==UP)
      other5_pid_value[arrow.y-1]+=other5_pid_value[0];
    else
      other5_pid_value[arrow.y-1]-=other5_pid_value[0];
    
    other5_pid_value[arrow.y-1]= other5_pid_value[arrow.y-1]>0?other5_pid_value[arrow.y-1]:0;     //避免负数********
  }
  else
  {
    
  }
  other5_step=other5_pid_value[0];
  other5_a=other5_pid_value[1]; 
  other5_b=other5_pid_value[2]; 
  other5_c=other5_pid_value[3]; 
  other5_d=other5_pid_value[4];
  other5_e=other5_pid_value[5];
  other5_f=other5_pid_value[6];
  
  Site_t site;
  site.x=pid_value_x[arrow.y-1]*8;           //只与y有关
  site.y=pid_value_y[arrow.y-1]*16;
  for(uint8 i=other5_pid_value_x[0];i<16;i++)
  { 
    site.x=i*8;
    LCD_char(site,' ', BLUE,BLUE);
  }
  
  site.x=other5_pid_value_x[arrow.y-1]*8;           //只与y有关
  site.y=other5_pid_value_y[arrow.y-1]*16;
  lcd_double_show(site,other5_pid_value[arrow.y-1]); 
}











