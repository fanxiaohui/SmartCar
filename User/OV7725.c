#include "OV7725.h"

uint8 imgbuff[CAMERA_SIZE];                         //定义存储接收原始图像的一维数组
uint8 img[CAMERA_H][CAMERA_W];                      //定义存储转化后的图像的二维数组


//中断函数声明
void PORTA_IRQHandler();          //PA29中断
void DMA0_IRQHandler();           //PA27触发DMA中断

/*
*  @brief      OV7725初始化，图像存储到一维数组imgbuff处
*  @param      image             图像地址
*/
void OV7725_Init()
{
	camera_init(imgbuff);

	//配置中断复位函数
	set_vector_handler(PORTA_VECTORn, PORTA_IRQHandler);    //设置LPTMR的中断复位函数为 PORTA_IRQHandler
	set_vector_handler(DMA0_VECTORn, DMA0_IRQHandler);      //设置LPTMR的中断复位函数为 PORTA_IRQHandler
}


/*
*  @brief      获取图像，并对图像进行解压缩，存储到二维数组img处
*  @param      image             图像地址
*/
void Get_img()
{

	camera_get_img();                                       //摄像头获取图像

	img_extract(img, imgbuff, OV7725_ImgSize);              //图像解压
}

///*
//*  @brief      缩放解压图像显示到液晶，默认显示二维图像
//*  @param      image             图像地址
//*/
//void Img_Show_LCD(uint8 *image)
//{
//	Site_t site = { 0, 0 };                              //显示图像左上角位置
//	Size_t imgsize = { 160, 60 };                       //图像大小
//	Size_t size = { 128, 128 };                      //显示区域图像大小
//
//	LCD_Img_gray_Z(site, size, (uint8*)image, imgsize);
//}

///*
//*  @brief      缩放解压图像显示到上位机，默认显示二维图像
//*  @param      image             图像地址
//*/
//void Img_Show_UART(uint8 *image)
//{
//	vcan_sendimg(image, OV7725_ImgSize);
//}

/*!
*  @brief      PORTA中断服务函数
*  @since      v5.0
*/
void PORTA_IRQHandler()
{
	uint8  n;    //引脚号
	uint32 flag;

	while (!PORTA_ISFR);
	flag = PORTA_ISFR;
	PORTA_ISFR = ~0;                                   //清中断标志位

	n = 29;                                             //场中断
	if (flag & (1 << n))                                 //PTA29触发中断
	{
		camera_vsync();
	}
#if ( CAMERA_USE_HREF == 1 )                            //使用行中断
	n = 28;
	if (flag & (1 << n))                                 //PTA28触发中断
	{
		camera_href();
	}
#endif

}

/*!
*  @brief      DMA0中断服务函数
*  @since      v5.0
*/
void DMA0_IRQHandler()
{
	camera_dma();
}
