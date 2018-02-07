#include "include.h"



#define SCL_INIT        gpio_init (SEEKFREE_SCL, GPO,0)
#define SDA_INIT        gpio_init (SEEKFREE_SDA, GPO,0)

#define SCL_PULL_INIT   port_init_NoALT (SEEKFREE_SCL, PULLUP)//ODO
#define SDA_PULL_INIT   port_init_NoALT (SEEKFREE_SDA, PULLUP)

#define SDA             gpio_get (SEEKFREE_SDA)
#define SDA0()          gpio_set (SEEKFREE_SDA, 0)	//IO口输出低电平
#define SDA1()          gpio_set (SEEKFREE_SDA, 1)	//IO口输出高电平  
#define SCL0()          gpio_set (SEEKFREE_SCL, 0)	//IO口输出低电平
#define SCL1()          gpio_set (SEEKFREE_SCL, 1)	//IO口输出高电平
#define DIR_OUT()       gpio_ddr (SEEKFREE_SDA, GPO);   //输出方向
#define DIR_IN()        gpio_ddr (SEEKFREE_SDA, GPI);   //输入方向



void mma8451_delay()
{
  uint8 i;   
  for(i=0;i<10;i++) 	   
    asm("nop");
}

//内部数据定义
unsigned char IIC_ad_main; //器件从地址	    
unsigned char IIC_ad_sub;  //器件子地址	   
unsigned char *IIC_buf;    //发送|接收数据缓冲区	    
unsigned char IIC_num;     //发送|接收数据个数	     

#define ack 1      //主应答
#define no_ack 0   //从应答	 

void IIC_start(void)
{
  SCL0();
  SDA1();
  mma8451_delay();
  SCL1();
  mma8451_delay();
  SDA0();
  mma8451_delay();
  SCL0();
}
//************************************************
//送停止位 SDA=0->1
void IIC_stop(void)
{
  SCL0();
  mma8451_delay();
  SDA0();
  mma8451_delay();
  SCL1();
  mma8451_delay();
  SDA1();
  mma8451_delay();
  SCL0();
}
//************************************************
//主应答(包含ack:SDA=0和no_ack:SDA=0)
void IIC_ack_main(unsigned char ack_main)
{
  SCL0();
  if(ack_main) SDA0(); //ack主应答
  else SDA1();         //no_ack无需应答
  mma8451_delay();
  SCL1();
  mma8451_delay();
  SCL0();
}
//*************************************************
//字节发送程序
//发送c(可以是数据也可是地址)，送完后接收从应答
//不考虑从应答位
void send_ch(unsigned char c)
{
  unsigned char i;
  for(i=0;i<8;i++)
  {
    SCL0();
    if((c<<i) & 0x80)SDA1(); //判断发送位
    else SDA0();
    mma8451_delay();
    SCL1();
    mma8451_delay();
    SCL0();
  }
  mma8451_delay();
  SDA1();             //发送完8bit，释放总线准备接收应答位
  mma8451_delay();
  SCL1();
  mma8451_delay();    //sda上数据即是从应答位              
  SCL0();             //不考虑从应答位|但要控制好时序
}
//**************************************************
//字节接收程序
//接收器件传来的数据，此程序应配合|主应答函数|IIC_ack_main()使用
//return: uint8型1字节
unsigned char read_ch(void)
{
  unsigned char i;
  unsigned char c;
  c=0;
  SCL0();
  mma8451_delay();
  SDA1();             //置数据线为输入方式
  DIR_IN();
  for(i=0;i<8;i++)
  {
    mma8451_delay();
    SCL0();         //置时钟线为低，准备接收数据位
    mma8451_delay();
    SCL1();         //置时钟线为高，使数据线上数据有效
    mma8451_delay();
    c<<=1;
    if(SDA) c+=1;   //读数据位，将接收的数据存c
  }
  SCL0();
  DIR_OUT();
  return c;
}
//***************************************************
//向无子地址器件发送单字节数据
void send_to_ch(unsigned char ad_main,unsigned char c)
{
  IIC_start();
  send_ch(ad_main);   //发送器件地址
  send_ch(c);         //发送数据c
  IIC_stop();
}
//***************************************************
//向有子地址器件发送多字节数据
void send_to_nch(unsigned char ad_main,unsigned char ad_sub,unsigned char *buf,unsigned char num)
{
  unsigned char i;
  IIC_start();
  send_ch(ad_main);   //发送器件地址
  send_ch(ad_sub);    //发送器件子地址
  for(i=0;i<num;i++)
  {
    send_ch(*buf);  //发送数据*buf
    buf++;
  }
  IIC_stop();
}
//***************************************************
//从无子地址器件读单字节数据
//function:器件地址，所读数据存在接收缓冲区当前字节
void read_from_ch(unsigned char ad_main,unsigned char *buf)
{
  IIC_start();
  send_ch(ad_main);           //发送器件地址
  *buf=read_ch();
  IIC_ack_main(no_ack);       //无需应答<no_ack=0>
  IIC_stop();
}
//***************************************************
//从有子地址器件读多个字节数据
//function:
void read_from_nch(unsigned char ad_main,unsigned char ad_sub,unsigned char *buf,unsigned char num)
{
  unsigned char i;
  IIC_start();
  send_ch(ad_main);
  send_ch(ad_sub);
  for(i=0;i<num-1;i++)
  {
    *buf=read_ch();
    IIC_ack_main(ack);      //主应答<ack=1>
    buf++;
  }
  *buf=read_ch();
  buf++;                      //本次指针调整无意义，目的是操作后buf指向下一地址
  IIC_ack_main(no_ack);       //无需应答<no_ack=0>
  IIC_stop();
}

void IIC_init(void)
{
  
  SCL_INIT;     
  SDA_INIT;
  SCL_PULL_INIT;
  SDA_PULL_INIT;
  
}