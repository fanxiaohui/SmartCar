#include "include.h"
#include "common.h"

/***********************************************************************************************************************************/

Site_t site     = {0, 0};                      //显示图像左上角位置
Size_t imgsize  = {COL, ROW};                  //图像大小
Size_t imgsize2  = {CAMERA_W,CAMERA_H};        //抽取后图像大小
Size_t imgsize_erzhi = {CAMERA_W,CAMERA_H};
Size_t size;                                   //显示区域图像大小

/***********************************************************************************************************************************/

void LCD1_init();                  //液晶屏初始化函数

void LCD_CAMERA_show();            //摄像头采集原图像显示函数
void LCD_CAMERA_erzhiSHOW();       //二值化图像显示函数
void LCD_CAMERA_BARREL_show();     //图像桶形失真矫正显示函数

/***********************************************************************************************************************************/

void LCD1_init()     //液晶屏初始化函数
{
      LCD_init();
      //LCD_str            (site,"Cam init ing",FCOLOUR,BCOLOUR);
      size.H = LCD_H;
      size.W = LCD_W;
      //LCD_str            (site,"Cam init OK!",FCOLOUR,BCOLOUR);
}

/***********************************************************************************************************************************/

void LCD_CAMERA_BARREL_show()
{
      barrel_rectification();     //桶形失真矫正函数
      site.x=0;
      site.y=0;
      LCD_Img_gray_Z(site, size, (uint8 *) img_barrel,imgsize2);
      site.x=64;
      site.y=64;
      LCD_cross(site,128,RED);
}

/***********************************************************************************************************************************/

void LCD_CAMERA_show()
{
      site.x=0;
      site.y=0;
      LCD_Img_gray_Z(site, size, (uint8 *)img,imgsize);
          site.x=64;
      site.y=64;
      LCD_cross(site,128,RED);
}

/***********************************************************************************************************************************/

void LCD_CAMERA_erzhiSHOW()
{
      edge_add();
      midline_add();
      jizhun_add();
      site.x=0;
      site.y=0;
      LCD_Img_gray_Z(site, size, (uint8 *)img_data,imgsize_erzhi);
}

/***********************************************************************************************************************************/




