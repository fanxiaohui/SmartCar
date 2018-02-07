#ifndef _SEEKFREE_IIC_h
#define _SEEKFREE_IIC_h


#define SEEKFREE_SCL    PTE6                          //定义SCL引脚
#define SEEKFREE_SDA    PTE7                         //定义SDA引脚


void IIC_start(void);
void IIC_stop(void);
void IIC_ack_main(unsigned char ack_main);
void send_ch(unsigned char c);
unsigned char read_ch(void);
void send_to_ch(unsigned char ad_main,unsigned char c);
void send_to_nch(unsigned char ad_main,unsigned char ad_sub,unsigned char *buf,unsigned char num);
void read_from_ch(unsigned char ad_main,unsigned char *buf);
void read_from_nch(unsigned char ad_main,unsigned char ad_sub,unsigned char *buf,unsigned char num);
void IIC_init(void);











#endif