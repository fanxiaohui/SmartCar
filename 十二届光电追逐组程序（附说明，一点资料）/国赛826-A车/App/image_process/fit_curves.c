#include "include.h"

/***********************************************************************************************************************************/

float b2,b1,b0;
float a1,a0;

/***********************************************************************************************************************************/

///////////////整型数组/////////////////////////
float sum_1(int16 *p,uint8 m) //求和函数 //之后给了例子
{
      int16 i;
      float sum=0.0;
      for(i=0;i<m;i++)
      {
            sum+=*(p+i);
      }
      return sum;
}

float sum_2(int16 *p,uint8 m) //求平方和
{
      int16 i;
      float sum=0.0;
      for(i=0;i<m;i++)
      {
            sum+=(*(p+i))*(*(p+i));
      }
      return sum;
}

float sum_3(int16 *p,int16 *q,uint8 m)  //求乘积和
{
      int16 i;
      float sum=0.0;
      for(i=0;i<m;i++)
      {
            sum+=(*(p+i))*(*(q+i));
      }
      return sum;
}

float sum_4(int16 *p,int16 *q,uint8 m) //求乘积
{
      int16 i;
      float sum=0.0;
      for(i=0;i<m;i++)
      {
            sum+=( (*(p+i))*(*(p+i)))*(*(q+i));
      }
      return sum;
      
}

float sum_5(int16 *p,uint8 m)
{
      int16 i;
      float sum=0.0;
      for(i=0;i<m;i++)
      {
            sum+=pow(*(p+i),3);  
      }
      return sum;
}

float sum_6(int16 *p,uint8 m)
{
      int16 i;
      float sum=0.0;
      for(i=0;i<m;i++)
            sum+=pow(*(p+i),4);
      return sum;
}

/***********************************************************************************************************************************/

///////////////浮点数组///////////////////////
float sum_11(float *p,uint8 m) //求和函数 //之后给了例子
{
      int16 i;
      float sum=0.0;
      for(i=0;i<m;i++)
      {
            sum+=*(p+i);
      }
      return sum;
}

float sum_22(float *p,uint8 m) //求平方和
{
      int16 i;
      float sum=0.0;
      for(i=0;i<m;i++)
      {
            sum+=(*(p+i))*(*(p+i));
      }
      return sum;
}

float sum_33(float *p,float *q,uint8 m)  //求乘积和
{
      int16 i;
      float sum=0.0;
      for(i=0;i<m;i++)
      {
            sum+=(*(p+i))*(*(q+i));
      }
      return sum;
}

float sum_44(float *p,float *q,uint8 m) //求乘积
{
      int16 i;
      float sum=0.0;
      for(i=0;i<m;i++)
      {
            sum+=( (*(p+i))*(*(p+i)))*(*(q+i));
      }
      return sum;
      
}

float sum_55(float *p,uint8 m)
{
      int16 i;
      float sum=0.0;
      for(i=0;i<m;i++)
      {
            sum+=pow(*(p+i),3);  
      }
      return sum;
}

float sum_66(float *p,uint8 m)
{
      int16 i;
      float sum=0.0;
      for(i=0;i<m;i++)
            sum+=pow(*(p+i),4);
      return sum;
}

/***********************************************************************************************************************************/

///////////////double数组////////////////////
double sum_111(int16 *p,uint8 m) //求和函数 //之后给了例子
{
      int16 i;
      double sum=0.0;
      for(i=0;i<m;i++)
      {
            sum+=*(p+i);
      }
      return sum;
}

double sum_222(int16 *p,uint8 m) //求平方和
{
      int16 i;
      double sum=0.0;
      for(i=0;i<m;i++)
      {
            sum+=(*(p+i))*(*(p+i));
      }
      return sum;
}

double sum_333(int16 *p,int16 *q,uint8 m)  //求乘积和
{
      int16 i;
      double sum=0.0;
      for(i=0;i<m;i++)
      {
            sum+=(*(p+i))*(*(q+i));
      }
      return sum;
}

double sum_444(int16 *p,int16 *q,uint8 m) //求乘积
{
      int16 i;
      double sum=0.0;
      for(i=0;i<m;i++)
      {
            sum+=( (*(p+i))*(*(p+i)))*(*(q+i));
      }
      return sum;
      
}

double sum_555(int16 *p,uint8 m)
{
      int16 i;
      double sum=0.0;
      for(i=0;i<m;i++)
      {
            sum+=pow(*(p+i),3);  
      }
      return sum;
}

double sum_666(int16 *p,uint8 m)
{
      int16 i;
      double sum=0.0;
      for(i=0;i<m;i++)
            sum+=pow(*(p+i),4);
      return sum;
}

/***********************************************************************************************************************************/

void first_order(uint8 c,float x[],float y[])
{
      
      float a;
      float sum_x,sum_x2,sum_x3,sum_x4,sum_y,sum_xy,sum_x2y;
      
      sum_x=sum_11(x,c);
      sum_x2=sum_22(x,c);
      sum_x3=sum_55(x,c);
      sum_x4=sum_66(x,c);
      sum_y=sum_11(y,c);
      sum_xy=sum_33(x,y,c);
      sum_x2y=sum_44(x,y,c);
      
      a =c*sum_x2-sum_x*sum_x;
      a1 =(c*sum_xy-sum_x*sum_y)/a;
      a0 =(sum_y*sum_x2-sum_xy*sum_x)/a;
      
      
}

/***********************************************************************************************************************************/

void first_order_int(uint8 c,int16 x[],int16 y[])
{
      
      float a;
      float sum_x,sum_x2,sum_x3,sum_x4,sum_y,sum_xy,sum_x2y;
      
      sum_x=sum_1(x,c);
      sum_x2=sum_2(x,c);
      sum_x3=sum_5(x,c);
      sum_x4=sum_6(x,c);
      sum_y=sum_1(y,c);
      sum_xy=sum_3(x,y,c);
      sum_x2y=sum_4(x,y,c);
      
      a =c*sum_x2-sum_x*sum_x;
      a1 =(c*sum_xy-sum_x*sum_y)/a;
      a0 =(sum_y*sum_x2-sum_xy*sum_x)/a;
      
      
}

/***********************************************************************************************************************************/

void second_order_int(uint8 c,int16 x[],int16 y[])
{
      
      float k=0,k1=0,k2=0;//,b2,b1,b0; //只是求导数的话，不需要求b0;
      float sum_x=0,sum_x2=0,sum_x3=0,sum_x4=0,sum_y=0,sum_xy=0,sum_x2y=0;
      
      sum_x=sum_1(x,c);
      sum_x2=sum_2(x,c);
      sum_x3=sum_5(x,c);
      sum_x4=sum_6(x,c);
      sum_y=sum_1(y,c);
      sum_xy=sum_3(x,y,c);
      sum_x2y=sum_4(x,y,c);
      
      k =sum_x/c;
      k1 =sum_x2/c;
      k2 =(sum_x3-k1*sum_x)/(sum_x2-k*sum_x);
      
      b2 =(sum_x2y-k1*sum_y-k2*(sum_xy-k*sum_y))/((sum_x4-k1*sum_x2)-k2*(sum_x3-k*sum_x2));
      b1 =(sum_xy-k*sum_y-(sum_x3-k*sum_x2)*b2)/(sum_x2-k*sum_x);
      b0 =(sum_y-sum_x2*b2-sum_x*b1)/c;   
}

/***********************************************************************************************************************************/

void second_order(uint8 c,float x[],float y[])
{
      
      float k=0,k1=0,k2=0;//,b2,b1,b0; //只是求导数的话，不需要求b0;
      float sum_x=0,sum_x2=0,sum_x3=0,sum_x4=0,sum_y=0,sum_xy=0,sum_x2y=0;
      
      sum_x=sum_11(x,c);
      sum_x2=sum_22(x,c);
      sum_x3=sum_55(x,c);
      sum_x4=sum_66(x,c);
      sum_y=sum_11(y,c);
      sum_xy=sum_33(x,y,c);
      sum_x2y=sum_44(x,y,c);
      
      k =sum_x/c;
      k1 =sum_x2/c;
      k2 =(sum_x3-k1*sum_x)/(sum_x2-k*sum_x);
      
      b2 =(sum_x2y-k1*sum_y-k2*(sum_xy-k*sum_y))/((sum_x4-k1*sum_x2)-k2*(sum_x3-k*sum_x2));
      b1 =(sum_xy-k*sum_y-(sum_x3-k*sum_x2)*b2)/(sum_x2-k*sum_x);
      b0 =(sum_y-sum_x2*b2-sum_x*b1)/c;   
}

/***********************************************************************************************************************************/

void second_order_double(uint8 c,int16 x[],int16 y[])
{
      float k=0,k1=0,k2=0;//,b2,b1,b0; //只是求导数的话，不需要求b0;
      double sum_x=0,sum_x2=0,sum_x3=0,sum_x4=0,sum_y=0,sum_xy=0,sum_x2y=0;
      
      sum_x=sum_111(x,c);
      sum_x2=sum_222(x,c);
      sum_x3=sum_555(x,c);
      sum_x4=sum_666(x,c);
      sum_y=sum_111(y,c);
      sum_xy=sum_333(x,y,c);
      sum_x2y=sum_444(x,y,c);
      
      k =sum_x/c;
      k1 =sum_x2/c;
      k2 =(sum_x3-k1*sum_x)/(sum_x2-k*sum_x);
      
      b2 =(sum_x2y-k1*sum_y-k2*(sum_xy-k*sum_y))/((sum_x4-k1*sum_x2)-k2*(sum_x3-k*sum_x2));
      b1 =(sum_xy-k*sum_y-(sum_x3-k*sum_x2)*b2)/(sum_x2-k*sum_x);
      b0 =(sum_y-sum_x2*b2-sum_x*b1)/c;   
}

/***********************************************************************************************************************************/

void abc(uint8 i)
{
      switch(i)
      {
      case 16: b2=1.5890707E-1;    b1=-1.37754163E+1;     b0=3.8162619011E+2;break;
      case 17: b2=1.99466467E-1;   b1=-1.80526504E+1;     b0=4.86422882E+2;break;
      case 18: b2=1.8311911822E-1; b1=-1.7722421644E+1;   b0=5.0654376222E+2;break;
      case 19: b2=1.94644615E-1;   b1=-1.94948597E+1;     b0=5.60022522E+2;break;
      case 20: b2=1.63336292E-1;   b1=-1.6358034133E+1;   b0=4.78971679E+2;break;
      case 21: b2=1.84433192E-1;   b1=-1.82139167E+1;     b0=5.34169921E+2;break;
      case 22: b2=1.2911608811E-1; b1=-1.32688284E+1;     b0=4.0890393E+2;break;
      
      }
}

/***********************************************************************************************************************************/


