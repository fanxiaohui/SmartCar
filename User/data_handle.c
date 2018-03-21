#include "common.h"
#include "include.h"
#include "Image_Process.h"




	






/*
void zhongxian(uint8 *img,uint8 *zhongxian)
{
  int i,j,left_back,right_back,left_flag,right_flag,n;
  for(i=CAMERA_H-1;i>0;i--)              //提取中线
       {
          for(j=CAMERA_W/2;j<CAMERA_W;j++)
          {
            if((*(img+i)+j)==0xff&&(*(img+i)+j+1)==0x00)
              {
                   left_back=j;
                   left_flag=1;
                   //left_back_old=left_back;
                  break;
              }
              else left_flag=0;
          }
          for(j=CAMERA_W/2;j>0;j--)
          {
              if((*(img+i)+j)==0xff&&(*(img+i)+j-1)==0x00)
                  {
                    right_back=j;
                    //right_back_old=right_back;
                    right_flag=1;
                    break;                                                                                                                                        
                  }
              else right_flag=0;
          }
          //if(right_flag==1&&left_flag==1)
          //{
          n=(right_back+left_back)/2;
           img[i][n]=0;
           imgzhongxian[i][j]=0;
          // }
       }

}
*/
float piancha(uint8 *img)
{
    int i,j;
    int flag1=80;
    int flag2=80;
    int flag3=80;
    float num;
    i=39;
    for(j=0;j<CAMERA_W;j++)
    {
      if((*(img+i)+j)==0)
      {
        flag1=j;
        break;
      }
    }
    i=40;
    for(j=0;j<CAMERA_W;j++)
    {
      if((*(img+i)+j)==0)
      {
        flag2=j;
        break;
      }
    }
     i=41;
    for(j=0;j<CAMERA_W;j++)
    {
      if((*(img+i)+j)==0)
      {
        flag3=j;
        break;
      }
    }
    num=((flag1+flag2+flag3)-240)/3;
    return num;
}