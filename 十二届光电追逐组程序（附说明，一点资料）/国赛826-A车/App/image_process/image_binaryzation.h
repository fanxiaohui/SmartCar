#ifndef _IMAGE_BINARYZATION_H_
#define _IMAGE_BINARYZATION_H_

/*------------------------------宏定义区--------------------------------*/

#define ERZHI                50
#define IMG_THRESHOLD_ADD    0
#define HEIGHT_STRAT         0
#define HEIGHT_END           60

/*----------------------------------------------------------------------*/

/*------------------------------变量声明区------------------------------*/

typedef struct 
{
      uint16  x;       //个数
      uint16  y;       //灰度值
}image_pixel;          //灰度梯度统计结构体
//为了消除黑影     行之间取值的时候   除去前3行  117行为最近处 30为最远处

extern uint8 img_data[CAMERA_H][ CAMERA_W ];

extern uint8 img_threshold;
extern uint8 img_thresholdbu;
extern uint8 gray_start;//灰度最小值
extern uint8 gray_end;//灰度最大值
extern uint16 gray_wide;
extern double deltaTmp;//最大方差
extern double gray_mean;//灰度平均值
extern uint8 erzhi_flag;
extern double u0;//第一类的平均灰度 背景
extern double u1; //第二类的平均灰度  前景

/*----------------------------------------------------------------------*/

/*------------------------------函数声明区------------------------------*/

extern uint8 binary_process(uint8 location,uint8 height_start,uint8 height_end);     //图像二值化求域值算法
extern void img_binary();                                                           //图像二值化处理函数
extern void bubble_sort( uint16 n);                                                 //冒泡排序函数

/*----------------------------------------------------------------------*/



#endif




