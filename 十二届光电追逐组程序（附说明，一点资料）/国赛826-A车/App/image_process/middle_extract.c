#include "include.h"
#include "common.h"

/***********************************************************************************************************************************/

double wide=180;          //逆透视图像赛道宽度变量
int16 midlinebu_row=15;    //中线补充最大宽度变量

uint8 midline_cnt;         //中线个数
int16 mid_temp_row;        //中线当前搜索行
int16 mid_temp_row;        //中线当前搜索行
int16 mid_down_pot[2];     //中线搜索图像外下方点（用于底部图像连线）

int16 midline[CAMERA_H];             //原始图像中心线
Site_xy inv_left_edge[CAMERA_H];     //逆透视图像左边沿坐标数组
Site_xy inv_right_edge[CAMERA_H];    //逆透视图像右边沿坐标数组
Site_xy inv_midline[CAMERA_H];       //逆透视图像中线坐标数组

uint8 midline_down_num;                      //横向提取边沿的实际个数
Site_xy midline_down_inv[MIDDLE_DOWN_NUM];   //横向提取边沿的逆透视坐标
int16 midline_down_x[MIDDLE_DOWN_NUM];       //横向提取边沿的原始坐标
int16 midline_down_y[MIDDLE_DOWN_NUM];       //横向提取边沿的原始坐标

Site_xy1 mid_end;
Site_xy1 mid_end_jizhun;
Site_xy  inv_mid_end;
uint8 mid_end_jizhun_flag;

double  loop_range_col[CAMERA_H];   //障碍物搜索范围数组
/***********************************************************************************************************************************/

void midline_extract();            //提取中线主函数
void midline_search();             //逆透视获取中线函数（最小二乘法）用于普通赛道
void midline_search_over();        //逆透视获取中线函数（最小二乘法）用于靠边停车
void midline_search_loop();        //逆透视获取中线函数（最小二乘法）用于环型赛道
void midline_bu();                 //中线补断点函数（用于中间离散中线）
void midline_bu_end();             //中线补断点函数（用于补弯道末尾中线）

void midline_jizhun_search(int16 i);   //基准中线搜索函数
void middle_down();     //提取横向中线函数
void edge_edge_lvbo();  //边沿滤波函数
void midline_lvbo();    //中线滤波函数

void tingche();         //出界停车函数
void tingche2();        //出界停车函数2

void edge_add();        //边沿添加函数
void midline_add();     //中线添加函数
void jizhun_add();      //基准线添加函数

void get_inv_edge();    //获取逆透视边沿坐标函数
void bar_range_loop();
Site_xy get_inv_img(int16 xxx ,int16 yyy);          //逆透视（xxx为图像行，yyy为列）
Site_xy1 get_invinv_img(double xxx,double yyy);   //反逆透视（xxx为实际行，yyy为列）
/***********************************************************************************************************************************/
void bar_range_loop() 
{
  uint8 x;
  Site_xy xy_1,xy_2;
  Site_xy1 xy1;
  for( x=ROW_START ; x<=ROW_END ; x++ )
  {
    xy_1=get_inv_img( x , 79 );
    xy_2=get_inv_img( x , 80 );
    inv_distance[x]=(xy_1.x+xy_2.x)/2;
  }
  
  for( x=ROW_START ; x<=ROW_END ; x++ )
  {
    xy1=get_invinv_img( inv_distance[x] , ROAD_LOOP_WIDE/2 );
    loop_range_col[x]=abs(xy1.y-79.5 );
  }
}
/***********************************************************************************************************************************/

void midline_extract()                //提取中线主函数
{
      midline_cnt=0;        //中线个数置  0
      mid_temp_row=-1;      //中线当前搜索行置最近行
      mid_down_pot[0]=-1;   //中线搜索图像外下方点置  -1
      mid_down_pot[1]=-1;   //中线搜索图像外下方点置  -1
      
      if( car_L_pull_over_flag || car_R_pull_over_flag )    //当处于超车时
      {
            midline_search_over();
            midline_bu();
      }
      else
      {
            if( LOOP_TEMP )    //当处于环形赛道时
            {
                  int16 i;
                  if( LOOP_IN==0 )  //当还未进入环道
                  {
                        if( loop_l_r_flag==1 )   //当环道为左转
                        {
                              for( i=ROW_END ; i>=ROW_START ; i-- )
                              {
                                    if( i>=LOOP_IN_ROW )      //当在入口行下方时
                                    {
                                          if( left_edge[i]!=-1 && right_edge[i]!=-1 )
                                          {
                                                midline[i]=(left_edge[i]+right_edge[i])/2;
                                                midline_cnt++;
                                                mid_temp_row=i;
                                          }
                                          else
                                          {
                                                if( left_edge[i]!=-1 )
                                                {
                                                      midline[i]=left_edge[i]+loop_range_col[i];
                                                      if( midline[i]>COL_END )
                                                      {
                                                            midline[i]=COL_END;
                                                      }
                                                      midline_cnt++;
                                                      mid_temp_row=i;
                                                }
                                                else
                                                {
                                                      midline[i]=loop_range_col[i];
                                                      midline_cnt++;
                                                      mid_temp_row=i;
                                                }
                                          }
                                    }
                                    else      //当在入口行上方时
                                    {
                                          if( left_edge[i]!=-1 )
                                          {
                                                midline[i]=left_edge[i]+loop_in_k*bar_range_col[i];
                                                if( midline[i]>COL_END )
                                                {
                                                      midline[i]=COL_END;
                                                }
                                                midline_cnt++;
                                                mid_temp_row=i;
                                          }
                                          else
                                          {
                                                midline[i]=loop_in_k_1*bar_range_col[i];
                                                midline_cnt++;
                                                mid_temp_row=i;
                                          }
                                    }
                              }
                        }
                        else   //当环道为右转
                        {
                              for( i=ROW_END ; i>=ROW_START ; i-- )
                              {
                                    if( i>=LOOP_IN_ROW )      //当在入口行下方时
                                    {
                                          if( left_edge[i]!=-1 && right_edge[i]!=-1 )
                                          {
                                                midline[i]=(left_edge[i]+right_edge[i])/2;
                                                midline_cnt++;
                                                mid_temp_row=i;
                                          }
                                          else
                                          {
                                                if( right_edge[i]!=-1 )
                                                {
                                                      midline[i]=right_edge[i]-loop_range_col[i];
                                                      if( midline[i]<COL_START )
                                                      {
                                                            midline[i]=COL_START;
                                                      }
                                                      midline_cnt++;
                                                      mid_temp_row=i;
                                                }
                                                else
                                                {
                                                      midline[i]=COL_END-loop_range_col[i];
                                                      midline_cnt++;
                                                      mid_temp_row=i;
                                                }
                                          }
                                    }
                                    else      //当在入口行上方时
                                    {
                                          if( right_edge[i]!=-1 )
                                          {
                                                midline[i]=right_edge[i]-loop_in_k*bar_range_col[i];
                                                if( midline[i]<COL_START )
                                                {
                                                      midline[i]=COL_START;
                                                }
                                                midline_cnt++;
                                                mid_temp_row=i;
                                          }
                                          else
                                          {
                                                midline[i]=COL_END-loop_in_k_1*bar_range_col[i];
                                                midline_cnt++;
                                                mid_temp_row=i;
                                          }
                                    }
                              }
                        }
                        midline_bu();
                  }
                  else  //当进入环道
                  {
                        if( LOOP_OUT==0 )   //还在环道内
                        {
                              if( out_row[0]==-1 )    //当未搜索得到出环口
                              {
                                    if( loop_l_r_flag==1 )   //当环道为左转
                                    {
                                          for( i=ROW_START ; i<=ROW_END ; i++ )
                                          {
                                                if( left_edge[i]!=-1 )
                                                {
                                                      for( i=i+1 ; i<=ROW_END ; i++ )
                                                      {
                                                            if( left_edge[i]==-1 )
                                                            {
                                                                  midline[i]=COL_START;
                                                                  midline_cnt++;
                                                            }
                                                      }
                                                }
                                          }
                                    }
                                    else   //当环道为右转
                                    {
                                          for( i=ROW_START ; i<=ROW_END ; i++ )
                                          {
                                                if( right_edge[i]!=-1 )
                                                {
                                                      for( i=i+1 ; i<=ROW_END ; i++ )
                                                      {
                                                            if( right_edge[i]==-1 )
                                                            {
                                                                  midline[i]=COL_END;
                                                                  midline_cnt++;
                                                            }
                                                      }
                                                }
                                          }
                                    }
                                    midline_search_loop();
                                    midline_bu();
                              }
                              else    //当搜索得到出环口
                              {
                                    midline_search_loop();
                                    midline_bu();
                                    if( loop_l_r_flag==1 )   //当环道为左转
                                    {
                                          if( out_row[0]>30 )
                                          {
                                                if( mid_temp_row!=-1 )
                                                {
                                                      for( i=mid_temp_row-1 ; i>=ROW_START ; i-- )
                                                      {
                                                            if( left_edge[i]!=-1 )
                                                            {
                                                                  midline[i]=left_edge[i]+bar_range_col[i];
                                                                  if( midline[i]>COL_END )
                                                                  {
                                                                        midline[i]=COL_END;
                                                                  }
                                                                  midline_cnt++;
                                                            }
                                                            else
                                                            {
                                                                  midline[i]=loop_in_k_1*bar_range_col[i];
                                                                  midline_cnt++;
                                                            }
                                                      }
                                                }
                                                else
                                                {
                                                      for( i=ROW_END ; i>=ROW_START ; i-- )
                                                      {
                                                            if( left_edge[i]!=-1 )
                                                            {
                                                                  midline[i]=left_edge[i]+bar_range_col[i];
                                                                  if( midline[i]>COL_END )
                                                                  {
                                                                        midline[i]=COL_END;
                                                                  }
                                                                  midline_cnt++;
                                                            }
                                                            else
                                                            {
                                                                  midline[i]=loop_in_k_1*bar_range_col[i];
                                                                  midline_cnt++;
                                                            }
                                                      }
                                                }
                                          }
                                          else
                                          {
                                                if( mid_temp_row!=-1 && midline[mid_temp_row]<60 )
                                                {
                                                      for( i=mid_temp_row-1 ; i>=ROW_START ; i-- )
                                                      {
                                                            if( left_edge[i]!=-1 )
                                                            {
                                                                  midline[i]=left_edge[i]+bar_range_col[i];
                                                                  if( midline[i]>COL_END )
                                                                  {
                                                                        midline[i]=COL_END;
                                                                  }
                                                                  midline_cnt++;
                                                            }
                                                            else
                                                            {
                                                                  midline[i]=loop_in_k_1*bar_range_col[i];
                                                                  midline_cnt++;
                                                            }
                                                      }
                                                }
                                          }
                                    }
                                    else   //当环道为右转
                                    {
                                          if( out_row[0]>30 )
                                          {
                                                if( mid_temp_row!=-1 )
                                                {
                                                      for( i=mid_temp_row-1 ; i>=ROW_START ; i-- )
                                                      {
                                                            if( right_edge[i]!=-1 )
                                                            {
                                                                  midline[i]=right_edge[i]-bar_range_col[i];
                                                                  if( midline[i]<COL_START )
                                                                  {
                                                                        midline[i]=COL_START;
                                                                  }
                                                                  midline_cnt++;
                                                            }
                                                            else
                                                            {
                                                                  midline[i]=COL_END-loop_in_k_1*bar_range_col[i];
                                                                  midline_cnt++;
                                                            }
                                                      }
                                                }
                                                else
                                                {
                                                      for( i=ROW_END ; i>=ROW_START ; i-- )
                                                      {
                                                            if( right_edge[i]!=-1 )
                                                            {
                                                                  midline[i]=right_edge[i]-bar_range_col[i];
                                                                  if( midline[i]<COL_START )
                                                                  {
                                                                        midline[i]=COL_START;
                                                                  }
                                                                  midline_cnt++;
                                                            }
                                                            else
                                                            {
                                                                  midline[i]=COL_END-loop_in_k_1*bar_range_col[i];
                                                                  midline_cnt++;
                                                            }
                                                      }
                                                }
                                          }
                                          else
                                          {
                                                if( mid_temp_row!=-1 && midline[mid_temp_row]>100 )
                                                {
                                                      for( i=mid_temp_row-1 ; i>=ROW_START ; i-- )
                                                      {
                                                            if( right_edge[i]!=-1 )
                                                            {
                                                                  midline[i]=right_edge[i]-bar_range_col[i];
                                                                  if( midline[i]<COL_START )
                                                                  {
                                                                        midline[i]=COL_START;
                                                                  }
                                                                  midline_cnt++;
                                                            }
                                                            else
                                                            {
                                                                  midline[i]=COL_END-loop_in_k_1*bar_range_col[i];
                                                                  midline_cnt++;
                                                            }
                                                      }
                                                }
                                          }
                                    }
                              }
                        }
                        else   //将要脱离环道
                        {
                              midline_search();
                              if( midline_cnt>=3 )
                              {
                                    midline_bu();
                              }
                        }
                  }
                  
                  if( mid_temp_row!=-1 )
                  {
                        if( loop_l_r_flag==1 )   //当环道为左转
                        {
                              for( i=ROW_END ; i>=mid_temp_row ; i-- )
                              {
                                    if( midline[i]==-1 )
                                    {
                                          midline[i]=COL_START;
                                          midline_cnt++;
                                    }
                              }
                        }
                        else   //当环道为右转
                        {
                              for( i=ROW_END ; i>=mid_temp_row ; i-- )
                              {
                                    if( midline[i]==-1 )
                                    {
                                          midline[i]=COL_END;
                                          midline_cnt++;
                                    }
                              }
                        }
                  }
            }
            else    //当处于普通赛道时
            {
                  midline_search();
                  if( midline_cnt>=3 )
                  {
                        midline_bu();
                        midline_bu_end();
                  }
            }
      }
}

/***********************************************************************************************************************************/

void midline_search()                 //逆透视获取中线函数（最小二乘法）用于普通赛道
{
      double xxx,yyy;         //逆透视中线坐标
      double x_angle,y_angle; //离边沿的竖直、水平偏移
      double xx1,yy1,xx2,yy2,xx3,yy3;  //边沿逆透视坐标
      double k;               //逆透视边沿计算斜率
      double uSrcImg,vSrcImg; //反逆透视坐标
      int16 mid_x,mid_y;       //反逆透视的行列值
      int16 row_mid=-1;        //中线末尾行值
      
      wide=ROAD_WIDE/2;
      
      if( left_edge[ROW_START]!=-1 && right_edge[ROW_START]!=-1 )
      {
            midline[ROW_START]=(left_edge[ROW_START]+right_edge[ROW_START])/2;
            midline_cnt++;
      }
      if( left_edge[ROW_START+1]!=-1 && right_edge[ROW_START+1]!=-1 )
      {
            midline[ROW_START+1]=(left_edge[ROW_START+1]+right_edge[ROW_START+1])/2;
            midline_cnt++;
      }
      if( left_edge[ROW_END]!=-1 && right_edge[ROW_END]!=-1 )
      {
            midline[ROW_END]=(left_edge[ROW_END]+right_edge[ROW_END])/2;
            mid_temp_row=ROW_END;
            midline_cnt++;
      }
      if( left_edge[ROW_END-1]!=-1 && right_edge[ROW_END-1]!=-1 )
      {
            midline[ROW_END-1]=(left_edge[ROW_END-1]+right_edge[ROW_END-1])/2;
            mid_temp_row=ROW_END-1;
            midline_cnt++;
      }
      for( int16 i=ROW_END-ROW_INTERVAL ; i>= ROW_START+ROW_INTERVAL ; i-- )
      {
            wide=(1.0*ROAD_WIDE+(inv_distance[i])/20)/2;
            
            if( left_edge[i]!=-1 )
            {
                  if( right_edge[i]!=-1 )   //如果左右边沿都有
                  {
                        midline[i]=(left_edge[i]+right_edge[i])/2;
                        midline_cnt++;
                        row_mid=i;
                        mid_temp_row=-1;
                  }
                  else   //如果只有左边沿，做逆透视
                  {
                        if( inv_left_edge[i+ROW_INTERVAL].x!=-1 && inv_left_edge[i].x!=-1 && inv_left_edge[i-ROW_INTERVAL].x!=-1 )
                        {
                              xx1=inv_left_edge[i+ROW_INTERVAL].x;
                              yy1=inv_left_edge[i+ROW_INTERVAL].y;
                              xx2=inv_left_edge[i].x;
                              yy2=inv_left_edge[i].y;
                              xx3=inv_left_edge[i-ROW_INTERVAL].x;
                              yy3=inv_left_edge[i-ROW_INTERVAL].y;
                              
                              /*最小二乘法获取边沿法线斜率，平移赛道宽度一半*/
                              k = (-1.0)*(xx1*yy1+xx2*yy2+xx3*yy3-(xx1+xx2+xx3)*(yy1+yy2+yy3)/3)/(xx1*xx1+xx2*xx2+xx3*xx3-(xx1+xx2+xx3)*(xx1+xx2+xx3)/3);
                              
                              if((k==0)|| (isnan(k)==1) || (isnan(k)==-1))  //水平
                              {
                                    xxx=xx2;
                                    yyy=yy2+wide;
                              }
                              else if((isinf(k)==1) || (isinf(k)==-1))     //垂直
                              {
                                    xxx=xx2-wide;
                                    yyy=yy2;
                              }
                              else
                              {
                                    y_angle=wide/sqrtf(1+k*k);   //离边沿的水平偏移
                                    x_angle=y_angle*fabsf(k);        //离边沿的竖直偏移
                                    if(k>0)
                                    {
                                          xxx=xx2+x_angle;
                                          yyy=yy2+y_angle;
                                    }
                                    if(k<0)
                                    {
                                          xxx=xx2-x_angle;
                                          yyy=yy2+y_angle;
                                    }
                              }
                              
                              /*反逆透视*/
                              uSrcImg =(INV_B*INV_F - INV_C*INV_E + INV_E*yyy - INV_B*xxx - INV_F*INV_H*yyy + INV_C*INV_H*xxx)/(INV_A*INV_E - INV_B*INV_D + INV_D*INV_H*yyy - INV_E*INV_G*yyy - INV_A*INV_H*xxx + INV_B*INV_G*xxx);
                              vSrcImg =-(INV_A*INV_F - INV_C*INV_D + INV_D*yyy - INV_A*xxx - INV_F*INV_G*yyy + INV_C*INV_G*xxx)/(INV_A*INV_E - INV_B*INV_D + INV_D*INV_H*yyy - INV_E*INV_G*yyy - INV_A*INV_H*xxx + INV_B*INV_G*xxx);
                              
                              mid_x=round(vSrcImg-1);
                              mid_y=round(uSrcImg-1);
                              if( mid_temp_row==-1 )
                              {
                                    if( mid_x>=ROW_START && mid_x>=i-20 )
                                    {
                                          if( mid_x<=ROW_END )
                                          {
                                                if( mid_y>=COL_START && mid_y<=COL_END )
                                                {
                                                      midline[mid_x]=mid_y;
                                                      midline_cnt++;
                                                      mid_temp_row=mid_x;
                                                }
                                          }
                                          else
                                          {
                                                if( mid_y>=COL_START && mid_y<=COL_END && mid_x<=ROW_END+10 )
                                                {
                                                      if( mid_down_pot[0]==-1 )
                                                      {
                                                            mid_down_pot[0]=mid_x;
                                                            mid_down_pot[1]=mid_y;
                                                      }
                                                      else
                                                      {
                                                            if( mid_x<=mid_down_pot[0] )
                                                            {
                                                                  mid_down_pot[0]=mid_x;
                                                                  mid_down_pot[1]=mid_y;
                                                            }
                                                      }
                                                }
                                          }
                                    }
                              }
                              else
                              {
                                    if( mid_x>=ROW_START && mid_x<=ROW_END && mid_x>=mid_temp_row-10 && mid_x<=mid_temp_row+3 )
                                    {
                                          if( mid_y>=COL_START && mid_y<=COL_END )
                                          {
                                                if( mid_y>=midline[mid_temp_row]-3 )
                                                {
                                                      midline[mid_x]=mid_y;
                                                      midline_cnt++;
                                                      mid_temp_row=mid_x;
                                                }
                                          }
                                          else
                                          {
                                                if( mid_y<=COL_END+20 )
                                                {
                                                      if( mid_y>=midline[mid_temp_row]-3 )
                                                      {
                                                            midline[mid_x]=COL_END;
                                                            midline_cnt++;
                                                            mid_temp_row=mid_x;
                                                      }
                                                }
                                          }
                                    }
                              }
                        }
                  }
            }
            else
            {
                  if( right_edge[i]!=-1 )   //如果只有右边沿，做逆透视
                  {
                        if(inv_right_edge[i+ROW_INTERVAL].x!=-1 && inv_right_edge[i].x!=-1 && inv_right_edge[i-ROW_INTERVAL].x!=-1 )
                        {
                              xx1=inv_right_edge[i+ROW_INTERVAL].x;
                              yy1=inv_right_edge[i+ROW_INTERVAL].y;
                              xx2=inv_right_edge[i].x;
                              yy2=inv_right_edge[i].y;
                              xx3=inv_right_edge[i-ROW_INTERVAL].x;
                              yy3=inv_right_edge[i-ROW_INTERVAL].y;
                              
                              /*最小二乘法获取边沿法线斜率，平移赛道宽度一半*/
                              k=(-1.0)*(xx1*yy1+xx2*yy2+xx3*yy3-(xx1+xx2+xx3)*(yy1+yy2+yy3)/3)/(xx1*xx1+xx2*xx2+xx3*xx3-(xx1+xx2+xx3)*(xx1+xx2+xx3)/3);
                              
                              if( k==0 || (isnan(k)==1) || (isnan(k)==-1) )  //水平
                              {
                                    xxx=xx2;
                                    yyy=yy2-wide;
                              }
                              else if( (isinf(k)==1) || (isinf(k)==-1) )  //垂直
                              {
                                    xxx=xx2-wide;
                                    yyy=yy2;
                              }        
                              else
                              {
                                    y_angle=wide/sqrtf(1+k*k); //离边沿的水平偏移
                                    x_angle=y_angle*fabsf(k);      //离边沿的竖直偏移
                                    if(k>0)
                                    {
                                          xxx=xx2-x_angle;
                                          yyy=yy2-y_angle;
                                    }
                                    if(k<0)
                                    {
                                          xxx=xx2+x_angle;
                                          yyy=yy2-y_angle;
                                    }
                              }
                              
                              /*反逆透视*/
                              uSrcImg =(INV_B*INV_F - INV_C*INV_E + INV_E*yyy - INV_B*xxx - INV_F*INV_H*yyy + INV_C*INV_H*xxx)/(INV_A*INV_E - INV_B*INV_D + INV_D*INV_H*yyy - INV_E*INV_G*yyy - INV_A*INV_H*xxx + INV_B*INV_G*xxx);
                              vSrcImg =-(INV_A*INV_F - INV_C*INV_D + INV_D*yyy - INV_A*xxx - INV_F*INV_G*yyy + INV_C*INV_G*xxx)/(INV_A*INV_E - INV_B*INV_D + INV_D*INV_H*yyy - INV_E*INV_G*yyy - INV_A*INV_H*xxx + INV_B*INV_G*xxx);
                              
                              mid_x=round(vSrcImg-1);
                              mid_y=round(uSrcImg-1);
                              if( mid_temp_row==-1 )
                              {
                                    if( mid_x>=ROW_START && mid_x<=ROW_END && mid_x>=i-20 )
                                    {
                                          if( mid_x<=ROW_END )
                                          {
                                                if( mid_y>=COL_START && mid_y<=COL_END )
                                                {
                                                      midline[mid_x]=mid_y;
                                                      midline_cnt++;
                                                      mid_temp_row=mid_x;
                                                }
                                          }
                                          else
                                          {
                                                if( mid_y>=COL_START && mid_y<=COL_END && mid_x<=ROW_END+10 )
                                                {
                                                      if( mid_down_pot[0]==-1 )
                                                      {
                                                            mid_down_pot[0]=mid_x;
                                                            mid_down_pot[1]=mid_y;
                                                      }
                                                      else
                                                      {
                                                            if( mid_x<=mid_down_pot[0] )
                                                            {
                                                                  mid_down_pot[0]=mid_x;
                                                                  mid_down_pot[1]=mid_y;
                                                            }
                                                      }
                                                }
                                          }
                                    }
                              }
                              else
                              {
                                    if( mid_x>=ROW_START && mid_x<=ROW_END && mid_x>=mid_temp_row-10 && mid_x<=mid_temp_row+3 )
                                    {
                                          if( mid_y>=COL_START && mid_y<=COL_END )
                                          {
                                                if( mid_y<=midline[mid_temp_row]+3 )
                                                {
                                                      midline[mid_x]=mid_y;
                                                      midline_cnt++;
                                                      mid_temp_row=mid_x;
                                                }
                                          }
                                          else
                                          {
                                                if( mid_y>=COL_START-20 )
                                                {
                                                      if( mid_y<=midline[mid_temp_row]+3 )
                                                      {
                                                            midline[mid_x]=COL_START;
                                                            midline_cnt++;
                                                            mid_temp_row=mid_x;
                                                      }
                                                }
                                          }
                                    }
                              }
                        }
                  }
            }
      }
      
      if( row_mid!=-1 )
      {
            if( mid_temp_row!=-1 )
            {
                  if( row_mid<mid_temp_row )
                  {
                        mid_temp_row=row_mid;
                  }
            }
            else
            {
                  mid_temp_row=row_mid;
            }
            
      }
}

/***********************************************************************************************************************************/

void midline_search_over()            //逆透视获取中线函数（最小二乘法）用于靠边停车
{
      double xxx,yyy;         //逆透视中线坐标
      double x_angle,y_angle; //离边沿的竖直、水平偏移
      double xx1,yy1,xx2,yy2,xx3,yy3;  //边沿逆透视坐标
      double k;               //逆透视边沿计算斜率
      double uSrcImg,vSrcImg; //反逆透视坐标
      int16 mid_x,mid_y;       //反逆透视的行列值
      wide=ROAD_WIDE/4;
      if(speedout_val_cross_flag && car_stop_flag && !car_flag)
      {
         wide=ROAD_WIDE/4*1.2;
      }
      if(car_L_pull_over_flag)    //当为超车左侧停车状态时
      {
            for( int16 i=ROW_END-ROW_INTERVAL ; i>= ROW_START+ROW_INTERVAL ; i-- )
            {
                  if( inv_left_edge[i+ROW_INTERVAL].x!=-1 && inv_left_edge[i].x!=-1 && inv_left_edge[i-ROW_INTERVAL].x!=-1 )
                  {
                        xx1=inv_left_edge[i+ROW_INTERVAL].x;
                        yy1=inv_left_edge[i+ROW_INTERVAL].y;
                        xx2=inv_left_edge[i].x;
                        yy2=inv_left_edge[i].y;
                        xx3=inv_left_edge[i-ROW_INTERVAL].x;
                        yy3=inv_left_edge[i-ROW_INTERVAL].y;
                        
                        /*最小二乘法获取边沿法线斜率，平移赛道宽度一半*/
                        k = (-1.0)*(xx1*yy1+xx2*yy2+xx3*yy3-(xx1+xx2+xx3)*(yy1+yy2+yy3)/3)/(xx1*xx1+xx2*xx2+xx3*xx3-(xx1+xx2+xx3)*(xx1+xx2+xx3)/3);
                        
                        if((k==0)|| (isnan(k)==1) || (isnan(k)==-1))  //水平
                        {
                              xxx=xx2;
                              yyy=yy2+wide;
                        }
                        else if((isinf(k)==1) || (isinf(k)==-1))     //垂直
                        {
                              xxx=xx2-wide;
                              yyy=yy2;
                        }
                        else
                        {
                              y_angle=wide/sqrtf(1+k*k);   //离边沿的水平偏移
                              x_angle=y_angle*fabsf(k);        //离边沿的竖直偏移
                              if(k>0)
                              {
                                    xxx=xx2+x_angle;
                                    yyy=yy2+y_angle;
                              }
                              if(k<0)
                              {
                                    xxx=xx2-x_angle;
                                    yyy=yy2+y_angle;
                              }
                        }
                        /*反逆透视*/
                        uSrcImg =(INV_B*INV_F - INV_C*INV_E + INV_E*yyy - INV_B*xxx - INV_F*INV_H*yyy + INV_C*INV_H*xxx)/(INV_A*INV_E - INV_B*INV_D + INV_D*INV_H*yyy - INV_E*INV_G*yyy - INV_A*INV_H*xxx + INV_B*INV_G*xxx);
                        vSrcImg =-(INV_A*INV_F - INV_C*INV_D + INV_D*yyy - INV_A*xxx - INV_F*INV_G*yyy + INV_C*INV_G*xxx)/(INV_A*INV_E - INV_B*INV_D + INV_D*INV_H*yyy - INV_E*INV_G*yyy - INV_A*INV_H*xxx + INV_B*INV_G*xxx);
                        
                        mid_x=round(vSrcImg-1);
                        mid_y=round(uSrcImg-1);
                        if( mid_temp_row==-1 )
                        {
                              if( mid_x>=ROW_START && mid_x>=i-20 )
                              {
                                    if( mid_x<=ROW_END )
                                    {
                                          if( mid_y>=COL_START && mid_y<=COL_END )
                                          {
                                                midline[mid_x]=mid_y;
                                                midline_cnt++;
                                                mid_temp_row=mid_x;
                                          }
                                    }
                                    else
                                    {
                                          if( mid_y>=COL_START && mid_y<=COL_END && mid_x<=ROW_END+10 )
                                          {
                                                if( mid_down_pot[0]==-1 )
                                                {
                                                      mid_down_pot[0]=mid_x;
                                                      mid_down_pot[1]=mid_y;
                                                }
                                                else
                                                {
                                                      if( mid_x<=mid_down_pot[0] )
                                                      {
                                                            mid_down_pot[0]=mid_x;
                                                            mid_down_pot[1]=mid_y;
                                                      }
                                                }
                                          }
                                    }
                              }
                        }
                        else
                        {
                              if( mid_x>=ROW_START && mid_x<=ROW_END && mid_x>=mid_temp_row-10 && mid_x<=mid_temp_row+3 )
                              {
                                    if( mid_y>=COL_START && mid_y<=COL_END )
                                    {
                                          if( mid_y>=midline[mid_temp_row]-3 )
                                          {
                                                midline[mid_x]=mid_y;
                                                midline_cnt++;
                                                mid_temp_row=mid_x;
                                          }
                                    }
                                    else
                                    {
                                          if( mid_y<=COL_END+20 )
                                          {
                                                if( mid_y>=midline[mid_temp_row]-3 )
                                                {
                                                      midline[mid_x]=COL_END;
                                                      midline_cnt++;
                                                      mid_temp_row=mid_x;
                                                }
                                          }
                                    }
                              }
                        }
                  }
            }
      }
      else
      {
            if(car_R_pull_over_flag)    //当为超车右侧停车状态时
            {
                  for( int16 i=ROW_END-ROW_INTERVAL ; i>= ROW_START+ROW_INTERVAL ; i-- )
                  {
                        if(inv_right_edge[i+2].x!=-1 && inv_right_edge[i].x!=-1 && inv_right_edge[i-2].x!=-1 )
                        {
                              xx1=inv_right_edge[i+ROW_INTERVAL].x;
                              yy1=inv_right_edge[i+ROW_INTERVAL].y;
                              xx2=inv_right_edge[i].x;
                              yy2=inv_right_edge[i].y;
                              xx3=inv_right_edge[i-ROW_INTERVAL].x;
                              yy3=inv_right_edge[i-ROW_INTERVAL].y;
                              
                              /*最小二乘法获取边沿法线斜率，平移赛道宽度一半*/
                              k=(-1.0)*(xx1*yy1+xx2*yy2+xx3*yy3-(xx1+xx2+xx3)*(yy1+yy2+yy3)/3)/(xx1*xx1+xx2*xx2+xx3*xx3-(xx1+xx2+xx3)*(xx1+xx2+xx3)/3);
                              
                              if( k==0 || (isnan(k)==1) || (isnan(k)==-1) )  //水平
                              {
                                    xxx=xx2;
                                    yyy=yy2-wide;
                              }
                              else if( (isinf(k)==1) || (isinf(k)==-1) )  //垂直
                              {
                                    xxx=xx2-wide;
                                    yyy=yy2;
                              }        
                              else
                              {
                                    y_angle=wide/sqrtf(1+k*k); //离边沿的水平偏移
                                    x_angle=y_angle*fabsf(k);      //离边沿的竖直偏移
                                    if(k>0)
                                    {
                                          xxx=xx2-x_angle;
                                          yyy=yy2-y_angle;
                                    }
                                    if(k<0)
                                    {
                                          xxx=xx2+x_angle;
                                          yyy=yy2-y_angle;
                                    }
                              }
                              
                              /*反逆透视*/
                              uSrcImg =(INV_B*INV_F - INV_C*INV_E + INV_E*yyy - INV_B*xxx - INV_F*INV_H*yyy + INV_C*INV_H*xxx)/(INV_A*INV_E - INV_B*INV_D + INV_D*INV_H*yyy - INV_E*INV_G*yyy - INV_A*INV_H*xxx + INV_B*INV_G*xxx);
                              vSrcImg =-(INV_A*INV_F - INV_C*INV_D + INV_D*yyy - INV_A*xxx - INV_F*INV_G*yyy + INV_C*INV_G*xxx)/(INV_A*INV_E - INV_B*INV_D + INV_D*INV_H*yyy - INV_E*INV_G*yyy - INV_A*INV_H*xxx + INV_B*INV_G*xxx);
                              
                              mid_x=round(vSrcImg-1);
                              mid_y=round(uSrcImg-1);
                              if( mid_temp_row==-1 )
                              {
                                    if( mid_x>=ROW_START && mid_x<=ROW_END && mid_x>=i-20 )
                                    {
                                          if( mid_x<=ROW_END )
                                          {
                                                if( mid_y>=COL_START && mid_y<=COL_END )
                                                {
                                                      midline[mid_x]=mid_y;
                                                      midline_cnt++;
                                                      mid_temp_row=mid_x;
                                                }
                                          }
                                          else
                                          {
                                                if( mid_y>=COL_START && mid_y<=COL_END && mid_x<=ROW_END+10 )
                                                {
                                                      if( mid_down_pot[0]==-1 )
                                                      {
                                                            mid_down_pot[0]=mid_x;
                                                            mid_down_pot[1]=mid_y;
                                                      }
                                                      else
                                                      {
                                                            if( mid_x<=mid_down_pot[0] )
                                                            {
                                                                  mid_down_pot[0]=mid_x;
                                                                  mid_down_pot[1]=mid_y;
                                                            }
                                                      }
                                                }
                                          }
                                    }
                              }
                              else
                              {
                                    if( mid_x>=ROW_START && mid_x<=ROW_END && mid_x>=mid_temp_row-10 && mid_x<=mid_temp_row+3 )
                                    {
                                          if( mid_y>=COL_START && mid_y<=COL_END )
                                          {
                                                if( mid_y<=midline[mid_temp_row]+3 )
                                                {
                                                      midline[mid_x]=mid_y;
                                                      midline_cnt++;
                                                      mid_temp_row=mid_x;
                                                }
                                          }
                                          else
                                          {
                                                if( mid_y>=COL_START-20 )
                                                {
                                                      if( mid_y<=midline[mid_temp_row]+3 )
                                                      {
                                                            midline[mid_x]=COL_START;
                                                            midline_cnt++;
                                                            mid_temp_row=mid_x;
                                                      }
                                                }
                                          }
                                    }
                              }
                        }
                  }
            }
      }
}

/***********************************************************************************************************************************/

void midline_search_loop()            //逆透视获取中线函数（最小二乘法）用于环型赛道
{
      double xxx,yyy;         //逆透视中线坐标
      double x_angle,y_angle; //离边沿的竖直、水平偏移
      double xx1,yy1,xx2,yy2,xx3,yy3;  //边沿逆透视坐标
      double k;               //逆透视边沿计算斜率
      double uSrcImg,vSrcImg; //反逆透视坐标
      int16 mid_x,mid_y;       //反逆透视的行列值
      
      wide=ROAD_WIDE/2;
      
      
      if( loop_l_r_flag==1 )   //当环道为左转
      {
            for( int16 i=ROW_END-ROW_INTERVAL ; i>= ROW_START+ROW_INTERVAL ; i-- )
            {
                  wide=(1.0*ROAD_WIDE+(inv_distance[i])/20)/2;
                  
                  if( inv_left_edge[i+ROW_INTERVAL].x!=-1 && inv_left_edge[i].x!=-1 && inv_left_edge[i-ROW_INTERVAL].x!=-1 )
                  {
                        xx1=inv_left_edge[i+ROW_INTERVAL].x;
                        yy1=inv_left_edge[i+ROW_INTERVAL].y;
                        xx2=inv_left_edge[i].x;
                        yy2=inv_left_edge[i].y;
                        xx3=inv_left_edge[i-ROW_INTERVAL].x;
                        yy3=inv_left_edge[i-ROW_INTERVAL].y;
                        
                        /*最小二乘法获取边沿法线斜率，平移赛道宽度一半*/
                        k=(-1.0)*(xx1*yy1+xx2*yy2+xx3*yy3-(xx1+xx2+xx3)*(yy1+yy2+yy3)/3)/(xx1*xx1+xx2*xx2+xx3*xx3-(xx1+xx2+xx3)*(xx1+xx2+xx3)/3);
                        
                        if( (k==0)|| (isnan(k)==1) || (isnan(k)==-1) )  //水平
                        {
                              xxx=xx2;
                              yyy=yy2+wide;
                        }
                        else
                        {
                              if( (isinf(k)==1) || (isinf(k)==-1) )  //垂直
                              {
                                    xxx=xx2-wide;
                                    yyy=yy2;
                              }
                              else
                              {
                                    y_angle=wide/sqrtf(1+k*k); //离边沿的水平偏移
                                    x_angle=y_angle*fabsf(k);  //离边沿的竖直偏移
                                    if(k>0)
                                    {
                                          xxx=xx2+x_angle;
                                          yyy=yy2+y_angle;
                                    }
                                    if(k<0)
                                    {
                                          xxx=xx2-x_angle;
                                          yyy=yy2+y_angle;
                                    }
                              }
                        }
                        
                        /*反逆透视*/
                        uSrcImg =(INV_B*INV_F - INV_C*INV_E + INV_E*yyy - INV_B*xxx - INV_F*INV_H*yyy + INV_C*INV_H*xxx)/(INV_A*INV_E - INV_B*INV_D + INV_D*INV_H*yyy - INV_E*INV_G*yyy - INV_A*INV_H*xxx + INV_B*INV_G*xxx);
                        vSrcImg =-(INV_A*INV_F - INV_C*INV_D + INV_D*yyy - INV_A*xxx - INV_F*INV_G*yyy + INV_C*INV_G*xxx)/(INV_A*INV_E - INV_B*INV_D + INV_D*INV_H*yyy - INV_E*INV_G*yyy - INV_A*INV_H*xxx + INV_B*INV_G*xxx);
                        
                        mid_x=round(vSrcImg-1);
                        mid_y=round(uSrcImg-1);
                        if( midline[mid_x]==-1 )
                        {
                              if( mid_temp_row==-1 )
                              {
                                    if( mid_x>=ROW_START && mid_x>=i-20 )
                                    {
                                          if( mid_x<=ROW_END )
                                          {
                                                if( mid_y>=COL_START && mid_y<=COL_END )
                                                {
                                                      midline[mid_x]=mid_y;
                                                      midline_cnt++;
                                                      mid_temp_row=mid_x;
                                                }
                                          }
                                          else
                                          {
                                                if( mid_y>=COL_START && mid_y<=COL_END && mid_x<=ROW_END+10 )
                                                {
                                                      if( mid_down_pot[0]==-1 )
                                                      {
                                                            mid_down_pot[0]=mid_x;
                                                            mid_down_pot[1]=mid_y;
                                                      }
                                                      else
                                                      {
                                                            if( mid_x<=mid_down_pot[0] )
                                                            {
                                                                  mid_down_pot[0]=mid_x;
                                                                  mid_down_pot[1]=mid_y;
                                                            }
                                                      }
                                                }
                                          }
                                    }
                              }
                              else
                              {
                                    if( mid_x>=ROW_START && mid_x<=ROW_END && mid_x<=mid_temp_row+3 )
                                    {
                                          if( mid_y>=COL_START && mid_y<=COL_END )
                                          {
                                                midline[mid_x]=mid_y;
                                                midline_cnt++;
                                                mid_temp_row=mid_x;
                                          }
                                          else
                                          {
                                                if( mid_y<=COL_END+20 )
                                                {
                                                      midline[mid_x]=COL_END;
                                                      midline_cnt++;
                                                      mid_temp_row=mid_x;
                                                }
                                          }
                                    }
                              }
                        }
                  }
            }
      }
      else   //当环道为右转
      {
            for( int16 i=ROW_END-ROW_INTERVAL ; i>= ROW_START+ROW_INTERVAL ; i-- )
            {
                  wide=(1.0*ROAD_WIDE+(inv_distance[i])/20)/2;
                  
                  if(inv_right_edge[i+ROW_INTERVAL].x!=-1 && inv_right_edge[i].x!=-1 && inv_right_edge[i-ROW_INTERVAL].x!=-1 )
                  {
                        xx1=inv_right_edge[i+ROW_INTERVAL].x;
                        yy1=inv_right_edge[i+ROW_INTERVAL].y;
                        xx2=inv_right_edge[i].x;
                        yy2=inv_right_edge[i].y;
                        xx3=inv_right_edge[i-ROW_INTERVAL].x;
                        yy3=inv_right_edge[i-ROW_INTERVAL].y;
                        
                        /*最小二乘法获取边沿法线斜率，平移赛道宽度一半*/
                        k=(-1.0)*(xx1*yy1+xx2*yy2+xx3*yy3-(xx1+xx2+xx3)*(yy1+yy2+yy3)/3)/(xx1*xx1+xx2*xx2+xx3*xx3-(xx1+xx2+xx3)*(xx1+xx2+xx3)/3);
                        
                        if( k==0 || (isnan(k)==1) || (isnan(k)==-1) )  //水平
                        {
                              xxx=xx2;
                              yyy=yy2-wide;
                        }
                        else if( (isinf(k)==1) || (isinf(k)==-1) )  //垂直
                        {
                              xxx=xx2-wide;
                              yyy=yy2;
                        }        
                        else
                        {
                              y_angle=wide/sqrtf(1+k*k); //离边沿的水平偏移
                              x_angle=y_angle*fabsf(k);  //离边沿的竖直偏移
                              if(k>0)
                              {
                                    xxx=xx2-x_angle;
                                    yyy=yy2-y_angle;
                              }
                              if(k<0)
                              {
                                    xxx=xx2+x_angle;
                                    yyy=yy2-y_angle;
                              }
                        }
                        
                        /*反逆透视*/
                        uSrcImg =(INV_B*INV_F - INV_C*INV_E + INV_E*yyy - INV_B*xxx - INV_F*INV_H*yyy + INV_C*INV_H*xxx)/(INV_A*INV_E - INV_B*INV_D + INV_D*INV_H*yyy - INV_E*INV_G*yyy - INV_A*INV_H*xxx + INV_B*INV_G*xxx);
                        vSrcImg =-(INV_A*INV_F - INV_C*INV_D + INV_D*yyy - INV_A*xxx - INV_F*INV_G*yyy + INV_C*INV_G*xxx)/(INV_A*INV_E - INV_B*INV_D + INV_D*INV_H*yyy - INV_E*INV_G*yyy - INV_A*INV_H*xxx + INV_B*INV_G*xxx);
                        
                        mid_x=round(vSrcImg-1);
                        mid_y=round(uSrcImg-1);
                        if( mid_x>=ROW_START && mid_x<=ROW_END && mid_y>=COL_START && mid_y<=COL_END && mid_x>=i-30 )
                        {
                              if( midline[mid_x]==-1 && mid_x<mid_temp_row+3 )
                              {
                                    midline[mid_x]=mid_y;
                                    midline_cnt++;
                                    mid_temp_row=mid_x;
                              }
                        }
                        if( midline[mid_x]==-1 )
                        {
                              if( mid_temp_row==-1 )
                              {
                                    if( mid_x>=ROW_START && mid_x>=i-20 )
                                    {
                                          if( mid_x<=ROW_END )
                                          {
                                                if( mid_y>=COL_START && mid_y<=COL_END )
                                                {
                                                      midline[mid_x]=mid_y;
                                                      midline_cnt++;
                                                      mid_temp_row=mid_x;
                                                }
                                          }
                                          else
                                          {
                                                if( mid_y>=COL_START && mid_y<=COL_END && mid_x<=ROW_END+10 )
                                                {
                                                      if( mid_down_pot[0]==-1 )
                                                      {
                                                            mid_down_pot[0]=mid_x;
                                                            mid_down_pot[1]=mid_y;
                                                      }
                                                      else
                                                      {
                                                            if( mid_x<=mid_down_pot[0] )
                                                            {
                                                                  mid_down_pot[0]=mid_x;
                                                                  mid_down_pot[1]=mid_y;
                                                            }
                                                      }
                                                }
                                          }
                                    }
                              }
                              else
                              {
                                    if( mid_x>=ROW_START && mid_x<=ROW_END && mid_x<=mid_temp_row+3 )
                                    {
                                          if( mid_y>=COL_START && mid_y<=COL_END )
                                          {
                                                midline[mid_x]=mid_y;
                                                midline_cnt++;
                                                mid_temp_row=mid_x;
                                          }
                                          else
                                          {
                                                if( mid_y>=COL_START-20 )
                                                {
                                                      midline[mid_x]=COL_START;
                                                      midline_cnt++;
                                                      mid_temp_row=mid_x;
                                                }
                                          }
                                    }
                              }
                        }
                  }
            }
      }
}

/***********************************************************************************************************************************/

void midline_bu()                     //中线补断点函数（用于中间离散中线）
{
      int16 x,i,xx;    //循环变量
      int16 mid_down=-1,mid_up=-1,mid;   //中线上下离散点行值
      double mid_row,mid_col,mid_start; //斜率计算变量
      if( S_FLAG==0 && LOOP_TEMP==0 )
      {
            midlinebu_row=MID_ADD;             //设定补线范围
      }
      else
      {
            if( L_s_temp==1 || R_s_temp==1 )
            {
                  midlinebu_row=MID_ADD+120;             //设定补线范围
            }
            else
            {
                  if( LOOP_TEMP==1 )
                  {
                        midlinebu_row=MID_ADD+120;             //设定补线范围
                  }
            }
      }
      
      for( x=ROW_END ; x>=mid_temp_row ; x-- )   //搜索中线，完成补线
      {
            if( midline[x]!=-1 )
            {
                  mid_down=x;
            }
            else
            {
                  if( mid_down!=-1 )
                  {
                        for( i=x-1 ; i>=mid_temp_row ; i-- )   //向上搜索中线
                        {
                              if( midline[i]!=-1 )
                              {
                                    mid_up=i;
                                    break;
                              }
                        }
                        if( mid_up==-1 )
                        {
                              break;   //搜索不到中线，则结束补线
                        }
                        else
                        {
                              if( inv_distance[mid_up]-inv_distance[mid_down]<midlinebu_row )  //当搜索得到上中线，且在范围内，进行补线
                              {
                                    mid_row=mid_down-mid_up;
                                    mid_col=midline[mid_down]-midline[mid_up];
                                    mid_col=mid_col/mid_row;
                                    mid_start=midline[mid_down];
                                    for( xx=mid_down-1 ; xx>mid_up ; xx-- )
                                    {
                                          mid_start=mid_start-mid_col;
                                          mid=round(mid_start);
                                          midline[xx]=mid;
                                          midline_cnt++;
                                    }
                                    mid_down=mid_up;
                                    x=mid_up+1;
                                    mid_up=-1;
                              }
                              else   //当搜索搜索得到上中线，不在范围内，放弃补线
                              {
                                    x=mid_up+1;
                                    mid_down=mid_up;
                                    mid_up=-1;
                              }
                        }
                  }
            }
      }
      
      mid_down=-1;
      for( x=ROW_END ; x>ROW_END-25 ; x-- )   //搜索中线，完成底部补线
      {
            if( midline[x]!=-1 )
            {
                  if( x==ROW_END )
                  {
                        break;
                  }
                  else
                  {
                        if( mid_down_pot[0]==-1 )    //未搜索到图像外下方点
                        {
                              mid_down=x;
                              mid_up=0;
                              for( x=mid_down-1 ; x>=mid_down-6 ; x-- )   //搜索中线，完成底部补线
                              {
                                    if( midline[x]!=-1 )
                                    {
                                          mid_up++;
                                    }
                                    else
                                    {
                                          break;
                                    }
                              }
                              if( mid_up>=6 )
                              {
                                    mid_row=6;
                                    mid_col=midline[mid_down]-midline[mid_down-6];
                                    mid_col=mid_col/mid_row;
                                    if( fabsf(mid_col)>1 )
                                    {
                                          if( mid_col<0 )
                                          {
                                                mid_col=-1;
                                          }
                                          else
                                          {
                                                mid_col=1;
                                          }
                                    }
                                    mid_start=midline[mid_down-3];
                                    for( xx=mid_down-2 ; xx<=ROW_END ; xx++ )
                                    {
                                          mid_start=mid_start+mid_col;
                                          mid=round(mid_start);
                                          if( mid<COL_START || mid>COL_END )
                                          {
                                                if( mid<COL_START )
                                                {
                                                      for( i=xx ; i<=ROW_END ; i++ )
                                                      {
                                                            midline[xx]=COL_START;
                                                            midline_cnt++;
                                                      }
                                                }
                                                else
                                                {
                                                      for( i=xx ; i<=ROW_END ; i++ )
                                                      {
                                                            midline[xx]=COL_END;
                                                            midline_cnt++;
                                                      }
                                                }
                                                break;
                                          }
                                          midline[xx]=mid;
                                          midline_cnt++;
                                    }
                                    break;
                              }
                        }
                        else    //搜索到图像外下方点
                        {
                              mid_row=mid_down_pot[0]-x;
                              mid_col=mid_down_pot[1]-midline[x];
                              mid_col=mid_col/mid_row;
                              mid_start=midline[x];
                              for( xx=x+1 ; xx<=ROW_END ; xx++ )
                              {
                                    mid_start=mid_start+mid_col;
                                    mid=round(mid_start);
                                    if( mid<COL_START || mid>COL_END )
                                    {
                                          if( mid<COL_START )
                                          {
                                                for( i=xx ; i<=ROW_END ; i++ )
                                                {
                                                      midline[xx]=COL_START;
                                                      midline_cnt++;
                                                }
                                          }
                                          else
                                          {
                                                for( i=xx ; i<=ROW_END ; i++ )
                                                {
                                                      midline[xx]=COL_END;
                                                      midline_cnt++;
                                                }
                                          }
                                          break;
                                    }
                                    midline[xx]=mid;
                                    midline_cnt++;
                              }
                              break;
                        }
                  }
            }
      }
      
      if( mid_temp_row!=-1 )
      {
            mid_down=mid_temp_row;
            for( x=mid_temp_row+1 ; x<=ROW_END ; x++ )   //搜索中线，完成中间丢失中线赋值
            {
                  if( midline[x]==-1 )
                  {
                        if( midline[mid_down]<=50 && left_edge[x]==-1 )
                        {
                              midline[x]=COL_START;
                              mid_down=x;
                        }
                        if( midline[x-1]>=110 && right_edge[x]==-1 )
                        {
                              midline[x]=COL_END;
                              mid_down=x;
                        }
                  }
                  else
                  {
                        mid_down=x;
                  }
            }
      }
}

/***********************************************************************************************************************************/

void midline_bu_end()                 //中线补断点函数（用于补弯道末尾中线）
{
      mid_end_jizhun_flag=0;
      if(R_TURN)
      {
            for(int16 j= COL_END ;j>= COL_END-20;j--)
            {
                  midline_jizhun_search(j);
                  if( mid_end_jizhun_flag)
                  {
                        break;
                  }
            }
            inv_mid_end=get_inv_img(mid_end_jizhun.x ,mid_end_jizhun.y);
            inv_mid_end.x=inv_mid_end.x-((1.0*ROAD_WIDE+(inv_distance[mid_end_jizhun.x])/20)/2);
            mid_end=get_invinv_img(inv_mid_end.x,inv_mid_end.y);
            
      }
      else  if(L_TURN)
      {
            for(int16 j= 0 ;j<= 20;j++)
            {
                  midline_jizhun_search(j);
                  if( mid_end_jizhun_flag)
                  {
                        break;
                  }
            }
            inv_mid_end=get_inv_img(mid_end_jizhun.x ,mid_end_jizhun.y);
            inv_mid_end.x=inv_mid_end.x-((1.0*ROAD_WIDE+(inv_distance[mid_end_jizhun.x])/20)/2);
            mid_end=get_invinv_img(inv_mid_end.x,inv_mid_end.y);
      }
      if( mid_end_jizhun_flag)
      {
            double line_k,line_b;
            int16 mid_x,mid_y;
            line_k=(1.0)*(midline[mid_temp_row]-mid_end.y)/(mid_temp_row-mid_end.x);
            line_b=(-1.0)*(midline[mid_temp_row]-mid_end.y)/(mid_temp_row-mid_end.x)*mid_end.x + mid_end.y;
            for(int16 i = mid_temp_row;i>= mid_end.x;i--)
            {
                  mid_x=i;
                  mid_y =round( line_k *mid_x + line_b);
                  if(( mid_x >=ROW_START) && (mid_x <= ROW_END) && (mid_y>=COL_START) && (mid_y <= COL_END))
                  {
                        midline[mid_x]=mid_y;
                        //mid_temp_row=mid_x;            
                  }
            }
      }
}

/***********************************************************************************************************************************/

void midline_jizhun_search(int16 i)   //基准中线搜索函数
{
      //找基准
      for(int16 ii=10;ii<CAMERA_H;ii++)//找到最右边那个点
      {
            if(img_data[ii][i]==0&&img_data[ii-1][i]==0&&img_data[ii+1][i]==255&&img_data[ii+10][i]==255)
            {
                  mid_end_jizhun.x=ii;
                  mid_end_jizhun.y=i;
                  mid_end_jizhun_flag=1;
                  break;
            }
      }
}

/***********************************************************************************************************************************/

void middle_down()                    //提取横向中线函数
{
      int16 down_row,down_col,down_edge_num,xx;
      float x,y;
      midline_down_num=0;         //横向提取边沿个数置  0
      
      if( L_TURN==1 &&  R_end_row!=-1 )     //当判断赛道为左转，并且右边沿结束行存在，开始横向搜索中心线
      {
            down_edge_num=0;                     //经过下降边沿个数置  0
            down_row=R_end_row-1;                //给起始行赋初值
            for( xx=R_end_row ; xx<ROW_END ;xx++ )
            {
                  if( right_edge[xx+1]-right_edge[xx]>=END_COL )    //确定最后起始行
                  {
                        down_row++;
                  }
                  else
                  {
                        break;
                  }
            }
            down_col=right_edge[down_row+1];      //给起始列赋初值
            for( midline_down_num=0 ;  midline_down_num<MIDDLE_DOWN_NUM ; midline_down_num++ )
            {
                  down_col=down_col-DOWN_COL;     //向左移动几列
                  if( down_row>ROW_END || down_col<COL_START )        //超过图像范围，停止搜索
                  {
                        midline_down_num--;
                        
                        break;
                  }
                  if( down_edge_num<R_LINK_NUM )  //当前方存在下降边沿点
                  {
                        if( down_col>right_down_edge[down_edge_num] )   //未到达前方下降边沿点
                        {
                              midline_down_inv[midline_down_num].y = (INV_A*down_col+INV_B*down_row+INV_C)/(INV_G*down_col+INV_H*down_row+1);    //求横向边沿逆透视坐标
                              midline_down_inv[midline_down_num].x = (INV_D*down_col+INV_E*down_row+INV_F)/(INV_G*down_col+INV_H*down_row+1);    //求横向边沿逆透视坐标
                              midline_down_inv[midline_down_num].x=midline_down_inv[midline_down_num].x-wide/2;
                              
                              x=midline_down_inv[midline_down_num].x;
                              y=midline_down_inv[midline_down_num].y;
                              
                              midline_down_y[midline_down_num]=(INV_B*INV_F-INV_C*INV_E+INV_E*y-INV_B*x-INV_F*INV_H*y+INV_C*INV_H*x)/(INV_A*INV_E-INV_B*INV_D+INV_D*INV_H*y-INV_E*INV_G*y-INV_A*INV_H*x+INV_B*INV_G*x)-1;     //求横向边沿原始坐标
                              midline_down_x[midline_down_num]=-(INV_A*INV_F-INV_C*INV_D+INV_D*y-INV_A*x-INV_F*INV_G*y+INV_C*INV_G*x)/(INV_A*INV_E-INV_B*INV_D+INV_D*INV_H*y-INV_E*INV_G*y-INV_A*INV_H*x+INV_B*INV_G*x)-1;    //求横向边沿原始坐标
                              
                              if( midline_down_x[midline_down_num]<ROW_START || midline_down_x[midline_down_num]>ROW_END || midline_down_y[midline_down_num]<COL_START || midline_down_y[midline_down_num]>COL_END )
                              {
                                    midline_down_x[midline_down_num]=-1;
                                    midline_down_y[midline_down_num]=-1;
                                    midline_down_num--;
                              }
                        }
                        else   //到达前方下降边沿点
                        {
                              down_row++;        //行值加一
                              down_edge_num++;   //经过下降边沿个数加一
                              
                              midline_down_inv[midline_down_num].y = (INV_A*down_col+INV_B*down_row+INV_C)/(INV_G*down_col+INV_H*down_row+1);    //求横向边沿逆透视坐标
                              midline_down_inv[midline_down_num].x = (INV_D*down_col+INV_E*down_row+INV_F)/(INV_G*down_col+INV_H*down_row+1);    //求横向边沿逆透视坐标
                              midline_down_inv[midline_down_num].x=midline_down_inv[midline_down_num].x-wide/2;
                              
                              x=midline_down_inv[midline_down_num].x;
                              y=midline_down_inv[midline_down_num].y;
                              
                              midline_down_y[midline_down_num]=(INV_B*INV_F-INV_C*INV_E+INV_E*y-INV_B*x-INV_F*INV_H*y+INV_C*INV_H*x)/(INV_A*INV_E-INV_B*INV_D+INV_D*INV_H*y-INV_E*INV_G*y-INV_A*INV_H*x+INV_B*INV_G*x)-1;     //求横向边沿原始坐标
                              midline_down_x[midline_down_num]=-(INV_A*INV_F-INV_C*INV_D+INV_D*y-INV_A*x-INV_F*INV_G*y+INV_C*INV_G*x)/(INV_A*INV_E-INV_B*INV_D+INV_D*INV_H*y-INV_E*INV_G*y-INV_A*INV_H*x+INV_B*INV_G*x)-1;    //求横向边沿原始坐标
                              
                              if( midline_down_x[midline_down_num]<ROW_START || midline_down_x[midline_down_num]>ROW_END || midline_down_y[midline_down_num]<COL_START || midline_down_y[midline_down_num]>COL_END )
                              {
                                    midline_down_x[midline_down_num]=-1;
                                    midline_down_y[midline_down_num]=-1;
                                    midline_down_num--;
                              }
                        }
                  }
                  else                            //当前方不存在下降边沿点
                  {
                        midline_down_inv[midline_down_num].y = (INV_A*down_col+INV_B*down_row+INV_C)/(INV_G*down_col+INV_H*down_row+1);    //求横向边沿逆透视坐标
                        midline_down_inv[midline_down_num].x = (INV_D*down_col+INV_E*down_row+INV_F)/(INV_G*down_col+INV_H*down_row+1);    //求横向边沿逆透视坐标
                        midline_down_inv[midline_down_num].x=midline_down_inv[midline_down_num].x-wide/2;
                        
                        x=midline_down_inv[midline_down_num].x;
                        y=midline_down_inv[midline_down_num].y;
                        
                        midline_down_y[midline_down_num]=(INV_B*INV_F-INV_C*INV_E+INV_E*y-INV_B*x-INV_F*INV_H*y+INV_C*INV_H*x)/(INV_A*INV_E-INV_B*INV_D+INV_D*INV_H*y-INV_E*INV_G*y-INV_A*INV_H*x+INV_B*INV_G*x)-1;     //求横向边沿原始坐标
                        midline_down_x[midline_down_num]=-(INV_A*INV_F-INV_C*INV_D+INV_D*y-INV_A*x-INV_F*INV_G*y+INV_C*INV_G*x)/(INV_A*INV_E-INV_B*INV_D+INV_D*INV_H*y-INV_E*INV_G*y-INV_A*INV_H*x+INV_B*INV_G*x)-1;    //求横向边沿原始坐标
                        
                        if( midline_down_x[midline_down_num]<ROW_START || midline_down_x[midline_down_num]>ROW_END || midline_down_y[midline_down_num]<COL_START || midline_down_y[midline_down_num]>COL_END )
                        {
                              midline_down_x[midline_down_num]=-1;
                              midline_down_y[midline_down_num]=-1;
                              midline_down_num--;
                        }
                  }
            }
            midline_down_num++;    //得出实际横向中心线个数
      }
      
      if( R_TURN==1 &&  L_end_row!=-1 )     //当判断赛道为右转，并且左边沿结束行存在，开始横向搜索中心线
      {
            down_edge_num=0;                     //经过下降边沿个数置  0
            down_row=L_end_row-1;                //给起始行赋初值
            for( xx=L_end_row ; xx<ROW_END ; xx++ )
            {
                  if( left_edge[xx]-left_edge[xx+1]>=END_COL )    //确定最后起始行
                  {
                        down_row++;
                  }
                  else
                  {
                        break;
                  }
            }
            down_col=left_edge[down_row+1];      //给起始列赋初值
            for( midline_down_num=0 ;  midline_down_num<MIDDLE_DOWN_NUM ; midline_down_num++ )
            {
                  down_col=down_col+DOWN_COL;     //向右移动几列
                  if( down_row>ROW_END || down_col>COL_END )        //超过图像范围，停止搜索
                  {
                        midline_down_num--;
                        
                        break;
                  }
                  if( down_edge_num<R_LINK_NUM )  //当前方存在下降边沿点
                  {
                        if( down_col<left_down_edge[down_edge_num] )   //未到达前方下降边沿点
                        {
                              midline_down_inv[midline_down_num].y = (INV_A*down_col+INV_B*down_row+INV_C)/(INV_G*down_col+INV_H*down_row+1);    //求横向边沿逆透视坐标
                              midline_down_inv[midline_down_num].x = (INV_D*down_col+INV_E*down_row+INV_F)/(INV_G*down_col+INV_H*down_row+1);    //求横向边沿逆透视坐标
                              midline_down_inv[midline_down_num].x=midline_down_inv[midline_down_num].x-wide/2;
                              
                              x=midline_down_inv[midline_down_num].x;
                              y=midline_down_inv[midline_down_num].y;
                              
                              midline_down_y[midline_down_num]=(INV_B*INV_F-INV_C*INV_E+INV_E*y-INV_B*x-INV_F*INV_H*y+INV_C*INV_H*x)/(INV_A*INV_E-INV_B*INV_D+INV_D*INV_H*y-INV_E*INV_G*y-INV_A*INV_H*x+INV_B*INV_G*x)-1;     //求横向边沿原始坐标
                              midline_down_x[midline_down_num]=-(INV_A*INV_F-INV_C*INV_D+INV_D*y-INV_A*x-INV_F*INV_G*y+INV_C*INV_G*x)/(INV_A*INV_E-INV_B*INV_D+INV_D*INV_H*y-INV_E*INV_G*y-INV_A*INV_H*x+INV_B*INV_G*x)-1;    //求横向边沿原始坐标
                              
                              if( midline_down_x[midline_down_num]<ROW_START || midline_down_x[midline_down_num]>ROW_END || midline_down_y[midline_down_num]<COL_START || midline_down_y[midline_down_num]>COL_END )
                              {
                                    midline_down_x[midline_down_num]=-1;
                                    midline_down_y[midline_down_num]=-1;
                                    midline_down_num--;
                              }
                        }
                        else   //到达前方下降边沿点
                        {
                              down_row++;        //行值加一
                              down_edge_num++;   //经过下降边沿个数加一
                              
                              midline_down_inv[midline_down_num].y = (INV_A*down_col+INV_B*down_row+INV_C)/(INV_G*down_col+INV_H*down_row+1);    //求横向边沿逆透视坐标
                              midline_down_inv[midline_down_num].x = (INV_D*down_col+INV_E*down_row+INV_F)/(INV_G*down_col+INV_H*down_row+1);    //求横向边沿逆透视坐标
                              midline_down_inv[midline_down_num].x=midline_down_inv[midline_down_num].x-wide/2;
                              
                              x=midline_down_inv[midline_down_num].x;
                              y=midline_down_inv[midline_down_num].y;
                              
                              midline_down_y[midline_down_num]=(INV_B*INV_F-INV_C*INV_E+INV_E*y-INV_B*x-INV_F*INV_H*y+INV_C*INV_H*x)/(INV_A*INV_E-INV_B*INV_D+INV_D*INV_H*y-INV_E*INV_G*y-INV_A*INV_H*x+INV_B*INV_G*x)-1;     //求横向边沿原始坐标
                              midline_down_x[midline_down_num]=-(INV_A*INV_F-INV_C*INV_D+INV_D*y-INV_A*x-INV_F*INV_G*y+INV_C*INV_G*x)/(INV_A*INV_E-INV_B*INV_D+INV_D*INV_H*y-INV_E*INV_G*y-INV_A*INV_H*x+INV_B*INV_G*x)-1;    //求横向边沿原始坐标
                              
                              if( midline_down_x[midline_down_num]<ROW_START || midline_down_x[midline_down_num]>ROW_END || midline_down_y[midline_down_num]<COL_START || midline_down_y[midline_down_num]>COL_END )
                              {
                                    midline_down_x[midline_down_num]=-1;
                                    midline_down_y[midline_down_num]=-1;
                                    midline_down_num--;
                              }
                        }
                  }
                  else                            //当前方不存在下降边沿点
                  {
                        midline_down_inv[midline_down_num].y = (INV_A*down_col+INV_B*down_row+INV_C)/(INV_G*down_col+INV_H*down_row+1);    //求横向边沿逆透视坐标
                        midline_down_inv[midline_down_num].x = (INV_D*down_col+INV_E*down_row+INV_F)/(INV_G*down_col+INV_H*down_row+1);    //求横向边沿逆透视坐标
                        midline_down_inv[midline_down_num].x=midline_down_inv[midline_down_num].x-wide/2;
                        
                        x=midline_down_inv[midline_down_num].x;
                        y=midline_down_inv[midline_down_num].y;
                        
                        midline_down_y[midline_down_num]=(INV_B*INV_F-INV_C*INV_E+INV_E*y-INV_B*x-INV_F*INV_H*y+INV_C*INV_H*x)/(INV_A*INV_E-INV_B*INV_D+INV_D*INV_H*y-INV_E*INV_G*y-INV_A*INV_H*x+INV_B*INV_G*x)-1;     //求横向边沿原始坐标
                        midline_down_x[midline_down_num]=-(INV_A*INV_F-INV_C*INV_D+INV_D*y-INV_A*x-INV_F*INV_G*y+INV_C*INV_G*x)/(INV_A*INV_E-INV_B*INV_D+INV_D*INV_H*y-INV_E*INV_G*y-INV_A*INV_H*x+INV_B*INV_G*x)-1;    //求横向边沿原始坐标
                        
                        if( midline_down_x[midline_down_num]<ROW_START || midline_down_x[midline_down_num]>ROW_END || midline_down_y[midline_down_num]<COL_START || midline_down_y[midline_down_num]>COL_END )
                        {
                              midline_down_x[midline_down_num]=-1;
                              midline_down_y[midline_down_num]=-1;
                              midline_down_num--;
                        }
                  }
            }
            midline_down_num++;    //得出实际横向中心线个数
      }
}

/***********************************************************************************************************************************/

void edge_edge_lvbo()                 //边沿滤波函数
{
      for(int16 i=ROW_END-10;i>=ROW_START+10;i--)
      {
            if(L_TURN && (left_edge[i+1]==-1)&& (left_edge[i+2]==-1) && (left_edge[i+3]==-1) &&(left_edge[i+10]==-1)&&((left_edge[i-1]==-1)||(left_edge[i-2]==-1)||(left_edge[i-3]==-1)||(left_edge[i-10]==-1)))
            {
                  left_edge[i]=-1;
            }
            if(R_TURN && (right_edge[i+1]==-1) && (right_edge[i+2]==-1) && (right_edge[i+3]==-1) && (right_edge[i+10]==-1)&&((right_edge[i-1]==-1)||(right_edge[i-2]==-1)||(right_edge[i-3]==-1)||(right_edge[i-10]==-1)))
            {
                  right_edge[i]=-1;
            }
      }
}

/***********************************************************************************************************************************/

void midline_lvbo()                   //中线滤波函数
{
      int16 row_next, row_now,row_last;
      uint8 row_now_flag,row_next_flag,row_last_flag;
      int16 ii,jj;
      row_next=-1;
      row_now=-1; 
      row_last=-1;
      ii=ROW_START-1;
      if(/*( s_flag==0)&&*/(LOOP_TEMP==0))
      {
            for(ii=ROW_END-1;ii>=ROW_START+1;ii--)
            {
                  row_now_flag=0;
                  if(midline[ii]!=-1)
                  {
                        row_now_flag=1;
                        row_now=ii;
                        row_last_flag=0;
                        for(jj=row_now-1;jj>=row_now-10;jj--)
                        { 
                              if(midline[jj]!=-1)
                              {
                                    row_last = jj;
                                    row_last_flag=1;
                                    break;
                              }
                        }
                        if(row_last_flag==1)
                        {
                              row_next_flag=0;
                              for(jj=row_now+1;jj<=row_now+10;jj++)
                              { 
                                    if(midline[jj]!=-1)
                                    {
                                          row_next = jj;
                                          row_next_flag=1;
                                          break;
                                    }
                              }
                        }    
                  }
                  
                  if((row_now_flag) && (row_next_flag) && (row_last_flag))//连线(只有一个点的时候没法连)
                  {
                        if((((midline[row_next]-midline[row_now])*(midline[row_now]-midline[row_last])<=0)&&(abs(midline[row_next]+midline[row_last]-2*midline[row_now])>=40)) || (((midline[row_next]-midline[row_now])*(midline[row_now]-midline[row_last])>=0)&&(abs(midline[row_next]+midline[row_last]-2*midline[row_now])>=120)))
                        {
                              midline[row_now]=-1;
                        }
                  }
            }
      }
}

/***********************************************************************************************************************************/

void tingche()                        //出界停车函数
{
      int16 i1=0;
      for(int16 k =CAMERA_H-3;k>CAMERA_H-6;k--)
      {
            for(int16 j=0;j<CAMERA_W;j++)  
            { 
                  if(img_data[k][j]==DOT_B)
                        i1++;
            }
      }
      if(i1 >= CAMERA_W*3-5)
      {
            car_status = CAR_STOPP;
            QAQ_STOP=1;
      }
}

/***********************************************************************************************************************************/

void tingche2()                       //出界停车函数2
{
      int16 i1=0;
      for( int16 k=CAMERA_H-3 ; k>CAMERA_H-6 ; k-- )
      {
            for( int16 j=0 ; j<CAMERA_W ; j++ )  
            { 
                  if( img_barrel[k][j]<=TINGCHE_HUDU )
                  {
                        i1++;
                  }
            }
      }
      if( i1>=CAMERA_W*3-10 )
      {
            car_status=CAR_STOPP;
            QAQ_STOP=1;
      }
}

/***********************************************************************************************************************************/

void edge_add()                       //边沿添加函数
{
      int16 x,y;
      for( x=ROW_END ; x>=ROW_START ; x-- )
      {
            if( left_edge[x]!=-1 && left_edge[x]<=COL_END-5 )            //过滤掉标志位-1的点
            {
                  img_data[x][left_edge[x]+4]=DOT_B;
                  img_data[x][left_edge[x]+5]=DOT_B;
            }
      }
      for( x=ROW_END ; x>=ROW_START ; x-- )
      {
            if( right_edge[x]!=-1 && right_edge[x]>=COL_START+5 )        //过滤掉标志位-1的点
            {
                  img_data[x][right_edge[x]-4]=DOT_B;
                  img_data[x][right_edge[x]-5]=DOT_B;
            }
      }
      
      if( L_down_cross[0]!=-1 && L_down_cross[1]<=COL_END-14 )
      {
            img_data[L_down_cross[0]][L_down_cross[1]+8]=DOT_B;
            img_data[L_down_cross[0]][L_down_cross[1]+10]=DOT_B;
            img_data[L_down_cross[0]][L_down_cross[1]+12]=DOT_B;
            img_data[L_down_cross[0]][L_down_cross[1]+14]=DOT_B;
      }
      if( L_up_cross[0]!=-1 && L_up_cross[1]<=COL_END-14 )
      {
            img_data[L_up_cross[0]][L_up_cross[1]+8]=DOT_B;
            img_data[L_up_cross[0]][L_up_cross[1]+10]=DOT_B;
            img_data[L_up_cross[0]][L_up_cross[1]+12]=DOT_B;
            img_data[L_up_cross[0]][L_up_cross[1]+14]=DOT_B;
      }
      if( R_down_cross[0]!=-1 && R_down_cross[1]>=COL_START+14 )
      {
            img_data[R_down_cross[0]][R_down_cross[1]-8]=DOT_B;
            img_data[R_down_cross[0]][R_down_cross[1]-10]=DOT_B;
            img_data[R_down_cross[0]][R_down_cross[1]-12]=DOT_B;
            img_data[R_down_cross[0]][R_down_cross[1]-14]=DOT_B;
            
      }
      if( R_up_cross[0]!=-1 && R_up_cross[1]>=COL_START+14 )
      {
            img_data[R_up_cross[0]][R_up_cross[1]-8]=DOT_B;
            img_data[R_up_cross[0]][R_up_cross[1]-10]=DOT_B;
            img_data[R_up_cross[0]][R_up_cross[1]-12]=DOT_B;
            img_data[R_up_cross[0]][R_up_cross[1]-14]=DOT_B;
      }
//      if( R_TURN==1 || L_TURN==1 )
//      {
//            if( L_LINK_NUM>0 )
//            {
//                  y=0;
//                  for( x=L_end_row ; x<=L_end_row+L_LINK_NUM-1 ; x++ )
//                  {
//                        if( left_down_edge[y]>=COL_START+5 )
//                        {
//                              img_data[x][left_down_edge[y]-4]=DOT_B;
//                              img_data[x][left_down_edge[y]-5]=DOT_B;
//                              
//                              y++;
//                        }
//                  }
//            }
//            if( R_LINK_NUM>0 )
//            {
//                  y=0;
//                  for( x=R_end_row ; x<=R_end_row+R_LINK_NUM-1 ; x++ )
//                  {
//                        if( right_down_edge[y]<=COL_END-5 )
//                        {
//                              img_data[x][right_down_edge[y]+4]=DOT_B;
//                              img_data[x][right_down_edge[y]+5]=DOT_B;
//                              
//                              y++;
//                        }
//                  }
//            }
//      }
}

/***********************************************************************************************************************************/

void midline_add()                    //中线添加函数
{
      int16 a,x;
      for(x=ROW_END;x>=ROW_START;x--)
      {
            
            if( midline[x]!=-1 )
            {
                  if(  midline[x]==COL_END )
                  {
                        a=midline[x];
                        img_data[x][a-1]=DOT_B;
                        img_data[x][a]=DOT_B;
                  }
                  else
                  {
                        a=midline[x];
                        img_data[x][a]=DOT_B;
                        img_data[x][a+1]=DOT_B;
                  }
            }
      }
      //      for( x=0 ; x<midline_down_num ; x++ )
      //      {
      //            img_data[midline_down_x[x]][midline_down_y[x]]=DOT_B;
      //            img_data[midline_down_x[x]][midline_down_y[x]+1]=DOT_B;
      //      }
}

/***********************************************************************************************************************************/

void jizhun_add()                     //基准线添加函数
{
      int16 x;
      for(x=JIZHUN_DOWN;x>=JIZHUN_UP;x=x-2)
      {
            img_data[x][CAMERA_W/2]=DOT_B;
            img_data[x][(CAMERA_W/2-1)]=DOT_B;
      }
}

/***********************************************************************************************************************************/

void get_inv_edge()                   //获取逆透视边沿函数
{
//      for(uint8 ii=0;ii<CAMERA_H;ii++)
//      {
//            if( left_edge[ii]!=-1 )
//            {
//                  inv_left_edge[ii]=get_inv_img( ii , left_edge[ii] );
//            }
//            else
//            {
//                  inv_left_edge[ii].y = -1;
//                  inv_left_edge[ii].x = -1;
//            }
//            if( right_edge[ii]!=-1 )
//            {
//                  inv_right_edge[ii]=get_inv_img( ii , right_edge[ii] );
//            }
//            else
//            {
//                  inv_right_edge[ii].y = -1;
//                  inv_right_edge[ii].x = -1;
//            }
//      }
      
      
      
      if( LOOP_TEMP==0 && car_L_pull_over_flag==0 && car_R_pull_over_flag==0 )   //当不为环道和超车状态时
      {
            for(uint8 ii=0;ii<CAMERA_H;ii++)
            {
                  if( left_edge[ii]!=-1 && right_edge[ii]==-1 )
                  {
                        inv_left_edge[ii]=get_inv_img( ii , left_edge[ii] );
                        inv_right_edge[ii].y = -1;
                        inv_right_edge[ii].x = -1;
                  }
                  if( left_edge[ii]==-1 && right_edge[ii]!=-1 )
                  {
                        inv_left_edge[ii].y = -1;
                        inv_left_edge[ii].x = -1;
                        inv_right_edge[ii]=get_inv_img( ii , right_edge[ii] );
                  }
                  if( left_edge[ii]!=-1 && right_edge[ii]!=-1 )
                  {
                        inv_left_edge[ii].y = -1;
                        inv_left_edge[ii].x = -1;
                        inv_right_edge[ii].y = -1;
                        inv_right_edge[ii].x = -1;
                  }
                  if( left_edge[ii]==-1 && right_edge[ii]==-1 )
                  {
                        inv_left_edge[ii].y = -1;
                        inv_left_edge[ii].x = -1;
                        inv_right_edge[ii].y = -1;
                        inv_right_edge[ii].x = -1;
                  }
            }
      }
      else   //当为环道和超车状态时
      {
            if( LOOP_TEMP==1 )    //当为环道时
            {
                  if( LOOP_IN==1 )   //当进入环道
                  {
                        if( LOOP_OUT==0 )
                        {
                              if( loop_l_r_flag==1 )   //当环道为左转
                              {
                                    for(uint8 ii=0;ii<CAMERA_H;ii++)
                                    {
                                          if( left_edge[ii]!=-1 )
                                          {
                                                inv_left_edge[ii]=get_inv_img( ii , left_edge[ii] );
                                          }
                                          else
                                          {
                                                inv_left_edge[ii].y = -1;
                                                inv_left_edge[ii].x = -1;
                                          }
                                    }
                                    if( LOOP_OUT==1 )   //将要脱离环道
                                    {
                                          for(uint8 ii=0;ii<CAMERA_H;ii++)
                                          {
                                                if( right_edge[ii]!=-1 )
                                                {
                                                      inv_right_edge[ii]=get_inv_img( ii , right_edge[ii] );
                                                }
                                                else
                                                {
                                                      inv_right_edge[ii].y = -1;
                                                      inv_right_edge[ii].x = -1;
                                                }
                                          }
                                    }
                              }
                              else   //当环道为右转
                              {
                                    for(uint8 ii=0;ii<CAMERA_H;ii++)
                                    {
                                          if( right_edge[ii]!=-1 )
                                          {
                                                inv_right_edge[ii]=get_inv_img( ii , right_edge[ii] );
                                          }
                                          else
                                          {
                                                inv_right_edge[ii].y = -1;
                                                inv_right_edge[ii].x = -1;
                                          }
                                    }
                                    if( LOOP_OUT==1 )   //将要脱离环道
                                    {
                                          for(uint8 ii=0;ii<CAMERA_H;ii++)
                                          {
                                                for(uint8 ii=0;ii<CAMERA_H;ii++)
                                                {
                                                      if( left_edge[ii]!=-1 )
                                                      {
                                                            inv_left_edge[ii]=get_inv_img( ii , left_edge[ii] );
                                                      }
                                                      else
                                                      {
                                                            inv_left_edge[ii].y = -1;
                                                            inv_left_edge[ii].x = -1;
                                                      }
                                                }
                                          }
                                    }
                              }
                        }
                        else
                        {
                              for(uint8 ii=0;ii<CAMERA_H;ii++)
                              {
                                    if( left_edge[ii]!=-1 && right_edge[ii]==-1 )
                                    {
                                          inv_left_edge[ii]=get_inv_img( ii , left_edge[ii] );
                                          inv_right_edge[ii].y = -1;
                                          inv_right_edge[ii].x = -1;
                                    }
                                    if( left_edge[ii]==-1 && right_edge[ii]!=-1 )
                                    {
                                          inv_left_edge[ii].y = -1;
                                          inv_left_edge[ii].x = -1;
                                          inv_right_edge[ii]=get_inv_img( ii , right_edge[ii] );
                                    }
                                    if( left_edge[ii]!=-1 && right_edge[ii]!=-1 )
                                    {
                                          inv_left_edge[ii].y = -1;
                                          inv_left_edge[ii].x = -1;
                                          inv_right_edge[ii].y = -1;
                                          inv_right_edge[ii].x = -1;
                                    }
                                    if( left_edge[ii]==-1 && right_edge[ii]==-1 )
                                    {
                                          inv_left_edge[ii].y = -1;
                                          inv_left_edge[ii].x = -1;
                                          inv_right_edge[ii].y = -1;
                                          inv_right_edge[ii].x = -1;
                                    }
                              }
                        }
                  }
            }
            else    //当为超车状态时
            {
                  if( car_L_pull_over_flag==1 )    //当为超车左侧停车状态时
                  {
                        for(uint8 ii=0;ii<CAMERA_H;ii++)
                        {
                              if( left_edge[ii]!=-1 )
                              {
                                    inv_left_edge[ii]=get_inv_img( ii , left_edge[ii] );
                              }
                              else
                              {
                                    inv_left_edge[ii].y = -1;
                                    inv_left_edge[ii].x = -1;
                              }
                        }
                  }
                  else
                  {
                        if( car_R_pull_over_flag==1 )    //当为超车右侧停车状态时
                        {
                              for(uint8 ii=0;ii<CAMERA_H;ii++)
                              {
                                    if( right_edge[ii]!=-1 )
                                    {
                                          inv_right_edge[ii]=get_inv_img( ii , right_edge[ii] );
                                    }
                                    else
                                    {
                                          inv_right_edge[ii].y = -1;
                                          inv_right_edge[ii].x = -1;
                                    }
                              }
                        }
                  }
            }
      }
}

/***********************************************************************************************************************************/

Site_xy get_inv_img(int16 xxx ,int16 yyy)        //逆透视（xxx为图像行，yyy为列）
{
      Site_xy temp;
      xxx++;
      yyy++;
      temp.y = (INV_A*yyy+INV_B*xxx+INV_C)/(INV_G*yyy+INV_H*xxx+1);
      temp.x = (INV_D*yyy+INV_E*xxx+INV_F)/(INV_G*yyy+INV_H*xxx+1);
      return temp;
}

/***********************************************************************************************************************************/

Site_xy1 get_invinv_img(double xxx,double yyy) //反逆透视（xxx为实际行，yyy为列）
{
      Site_xy1 temp;
      double uSrcImg,vSrcImg;//反逆透视坐标
      uSrcImg =(INV_B*INV_F - INV_C*INV_E + INV_E*yyy - INV_B*xxx - INV_F*INV_H*yyy + INV_C*INV_H*xxx)/(INV_A*INV_E - INV_B*INV_D + INV_D*INV_H*yyy - INV_E*INV_G*yyy - INV_A*INV_H*xxx + INV_B*INV_G*xxx);
      vSrcImg =-(INV_A*INV_F - INV_C*INV_D + INV_D*yyy - INV_A*xxx - INV_F*INV_G*yyy + INV_C*INV_G*xxx)/(INV_A*INV_E - INV_B*INV_D + INV_D*INV_H*yyy - INV_E*INV_G*yyy - INV_A*INV_H*xxx + INV_B*INV_G*xxx);
      
      temp.x=round(vSrcImg-1);
      temp.y=round(uSrcImg-1);
      return temp;
}

/***********************************************************************************************************************************/



