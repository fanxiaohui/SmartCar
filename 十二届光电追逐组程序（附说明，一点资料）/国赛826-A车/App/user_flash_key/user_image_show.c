#include "common.h"
#include "include.h"
#include "get_cr_init.h"
#include  "user.h"   


extern int16 mpu_gyro_x,mpu_gyro_y,mpu_gyro_z;
extern uint8 img_threshold;


extern uint8 img_replace[188][120];
#define LINK_NUM 50
extern uint8 line_x[LINK_NUM];
extern uint8 line_y[LINK_NUM];

extern uint32 time;

extern uint8 fact_end_H;
uint8 dot_sum;

extern void add_center();

#define  BINARY_SHOW            (!(gpio_get (PTC16)))
#define  BINARY_LINE_SHOW       (!(gpio_get (PTC17)))
#define  MIDDDLE_STATE_SHOW     (!(gpio_get (PTC18)))

//灰度二进制图像显示  取消！！！
//#define  GRAY_BINARY_SHOW     (!(gpio_get (PTD0 )))

//#define  ADJUST_LINE_SHOW     (!(gpio_get (PTD4)))                          

void img_state_show();

void gray_binary_show();       //灰度  二值  显示

void middle_state_show();      //显示补线效果和一些参数

extern float steer_weight[5];
extern uint8 refer_line[5];
extern float steer_error_now;

//斜率 偏差 控制
extern float center_k;
extern int16 img_offset;

extern uint8 shangpojieduan;
void road_type_show();

void road_type_show()
{
//    Site_t site1={128-8*6,128-16*1};
//     Site_t site2={128-8*6,128-16*3};
//      Site_t site3={128-8*6,128-16*5};
    
 // LCD_num(site1,(uint32)fabs(steer_error_now), BLUE,RED); 
   Site_t site2={128-8*6,128-16*3};
  LCD_num(site2,shangpojieduan, BLUE,RED); 
//      LCD_str(site1,"  ",RED,RED);
//       LCD_str(site2,"  ",RED,RED);
//       LCD_str(site3,"  ",RED,RED);
//      if(thispic_start)
//      {
//        
//        
//      }
//       if(obstacle_flag)
//      {
//        LCD_num(site2,1, BLUE,RED);
//      }
//      if(up_hill_flag)
//      {
//        LCD_num(site3,2, BLUE,RED);
//      }
    
      
}
#define DOT_NUM 5
  //抽取 40行 到20行的  点进行判定！！！
 extern  uint8 refer_h[DOT_NUM];
 extern  float weight_h[DOT_NUM];
 
void  offset_show()
{
  
  uint8 location_x,location_y;
    for(uint8 i=0;i<DOT_NUM;i++)
    {
      Site_t site={128-8*6,128-16*(i+2)};
      LCD_str(site,"  ",RED,RED);
      LCD_num(site,refer_h[i], BLUE,RED);
      site.x=128-8*3;
       LCD_str(site,"  ",RED,RED);
      LCD_num(site,img_center[refer_h[i]], BLUE,RED); 
      
    }
   
  
  
    Site_t site1={128-8*4,128-16*1};
      LCD_str(site1,"  ",RED,RED);
    LCD_num(site1,(uint8)(fabs(steer_error_now)), BLUE,RED);
    
    Site_t site2={30,60};
    LCD_num(site2,(uint32)(ABS(mpu_gyro_y)), YELLOW,BLACK); 
    
    Site_t site3={10,20};
    LCD_num(site3,(uint32)(ABS(img_threshold)), YELLOW,BLACK); 
}


void img_state_show()
{
  
     Site_t site={0,0};
     Size_t size={128,128};
     Size_t imgsize={80,60};
      
   if(BINARY_SHOW&&BINARY_LINE_SHOW)        //显示二值图像和补线效果
     {
       add_center();
       LCD_Img_gray_Z(site,size,*img_data,imgsize);  
       
     }
   else if(!(gpio_get (PTC17)))
         {
             add_center();
       LCD_Img_gray_Z(site,size,*img_data,imgsize); 
         }
   else if(BINARY_SHOW&&!BINARY_LINE_SHOW)   //只显示二值图像
     {
       int8 i,j;
  
  for(i=row_line_start;i>row_line_end;i--)
    {
      img_data[i][img_center[i]]=0;
    }
  if(col_line_ok)
  {
     if(col_line_start>col_line_end)
         {
         for(j=col_line_start;j>=col_line_end;j--)
           { 
            img_data[img_center_col[j]][j]=0;
          }
         }
     if(col_line_start<col_line_end)
         {
          for(j=col_line_start;j<=col_line_end;j++)
            { 
            img_data[img_center_col[j]][j]=0;
            }
         }
   }
       LCD_Img_gray_Z(site,size,*img_data,imgsize); 
     }
//   else if(GRAY_BINARY_SHOW)
//    {
//          gray_binary_show();
//    } 
   
   if(MIDDDLE_STATE_SHOW)
   {
      // middle_state_show();
       offset_show();
    } 
//    //灰度图像取消 显示
//    if(!(gpio_get (PTD6)))
//     {
//        Site_t site={0,0};
//       Size_t size={128,128};
//        Size_t imgsize={186,120};
//        LCD_Img_gray_Z(site,size,*image,imgsize); 
//                   
//     }
   road_type_show();
     offset_show();
}


void gray_binary_show()
{
  //显示灰度图像     显示二值化图像     显示获取图像所需时间    显示阈值
  //image               img_data             time                 img_threshold
#define IMG_GRAY        0
#define IMG_BINARY      1
#define IMG_TIME        2
#define IMG_THRESHOLD   3
              //横向为16   纵向为8
uint8 site_x[]={0,   0    ,  8*13 , 8*13   };  
uint8 site_y[]={0,  16*4  ,  16*2  ,16*5    };

uint8 size_x[]={8*12, 8*12};
uint8 size_y[]={16*4, 16*4};
  
Site_t gray_site     = {site_x[IMG_GRAY], site_y[IMG_GRAY]};                           //显示图像左上角位置
Site_t binary_site     = {site_x[IMG_BINARY], site_y[IMG_BINARY]};                      
Site_t time_site     = {site_x[IMG_TIME], site_y[IMG_TIME]}; 
Site_t threshold_site   = {site_x[IMG_THRESHOLD], site_y[IMG_THRESHOLD]}; 


Size_t gray_show_size  = {size_x[IMG_GRAY], size_y[IMG_GRAY]};                   //显示区域图像大小 
Size_t binary_show_size  = {size_x[IMG_BINARY], size_y[IMG_BINARY]};             

Size_t gray_size={186,120};                 //图像大小
Size_t binary_size={80,60};


LCD_Img_gray_Z(gray_site,gray_show_size,*image,gray_size);             //灰度图像显示

LCD_Img_gray_Z(binary_site,binary_show_size,*img_data,binary_size);             //二值图像显示

          
LCD_str(time_site,"   ",RED,RED);

LCD_num(time_site,time, BLUE,RED);        //显示时间
time_site.y-=1*16;      
LCD_str(time_site,"T",BLUE,RED);

LCD_str(threshold_site,"   ",RED,RED);
LCD_num(threshold_site,img_threshold, BLUE,RED);        //显示阈值
threshold_site.y-=1*16;      
LCD_str(threshold_site,"Y_Z",BLUE,RED);


}

 void middle_state_show()
 {
     Site_t site={128-8*4,128-16*2};
   
     
     LCD_str(site,"    ",RED,RED);
   
   // uint8 piancha=(uint8)(ABS(steer_error_now));
     
     dot_sum=ABS(IMG_H-fact_end_H);
     
     dot_sum=ABS(row_line_start-row_line_end)+ABS(col_line_start-col_line_end);
     
     LCD_num(site,dot_sum, BLUE,RED);        //显示时间
     
  
 }





