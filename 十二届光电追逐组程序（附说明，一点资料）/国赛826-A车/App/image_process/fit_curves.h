#ifndef _FIT_CURVES_H_
#define _FIT_CURVES_H_

/*------------------------------宏定义枚举-----------------------------*/



/*----------------------------------------------------------------------*/

/*------------------------------变量声明区------------------------------*/

extern float b2,b1,b0;
extern float a1,a0;

/*----------------------------------------------------------------------*/

/*------------------------------函数声明区------------------------------*/

extern void second_order_int(uint8 c,int16 x[],int16 y[]);
extern void first_order(uint8 c,float x[],float y[]);
extern void first_order_int(uint8 c,int16 x[],int16 y[]);
extern void second_order(uint8 c,float x[],float y[]);
extern void second_order_double(uint8 c,int16 x[],int16 y[]);
extern void abc(uint8 i);

/*----------------------------------------------------------------------*/






#endif


