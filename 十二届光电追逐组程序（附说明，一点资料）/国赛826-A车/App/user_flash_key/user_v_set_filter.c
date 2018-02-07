#include "common.h"
#include "include.h"

Site_t arrow;    //记录箭头的位置

uint8 v_set_filter();
void v_set_page_show();       //速度选择  显示



uint8 v_set_filter()
{
  arrow.x=1;
  arrow.y=1;
  
  uint8 key=0;
  v_set_page_show();       //速度选择  显示
  
  while(1)
  {
    key=0;
    while ((key=once_check_key())==0);     //直到有按键按下
    if(key==UP)
    {
      if(arrow.y==1)
         arrow.y=SD_PAGE_NUM;
      else if(arrow.y>1)
         arrow.y--;
     
    }
    else if(key==DOWN)
    {
      if(arrow.y<SD_PAGE_NUM)
         arrow.y++;
      else if(arrow.y==SD_PAGE_NUM)
         arrow.y=1;
    }
    if(key==LEFT)
    {
    }
    else if(key==RIGHT)
    {
   //     LCD_clear(RED);
      return arrow.y;
    }
    //更新
    LCD_clear(RED);
   v_set_page_show();       //速度选择  显示
    
  }
  
}

void v_set_page_show()       //速度选择  显示
{
    //第一页
  Site_t site_flag = {(arrow.x-1)*8,(arrow.y-1)*16};
  
  LCD_str(site_flag,"->", BLACK,RED);            

  Site_t site;
  site.x = 2*8;   site.y=0; 
  LCD_str(site,"v_fast", BLUE,RED);
  site.x = 2*8;   site.y=1*16; 
  LCD_str(site,"v_mid", BLUE,RED);
  
  site.x = 2*8;   site.y=2*16; 
  LCD_str(site,"v_slow", BLUE,RED);
  
}