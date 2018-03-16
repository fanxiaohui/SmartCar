#include "Image_Process.h"

/************************************************************************/
/*                     特殊路况标记                                     */
uint8                  cross_flag;              //十字标志
uint8                  S_flag;                  //连续S弯标志
uint8                  L_Sharp_turn_flag;       //左传急弯标志
uint8                  R_Sharp_turn_flag;       //右传急弯标志
uint8                  Big_Turn_flag;           //大弯缓弯标志
uint8                  L_Loop_flag;             //左侧环岛标志
uint8                  R_Loop_flag;             //右侧环岛标志
uint8                  Barrier_flag;            //障碍标志
uint8                  Stop_flag;               //停止标志
/************************************************************************/

uint8  zhongxian[CAMERA_H][CAMERA_W];                  //定义储存中线的数组
uint8  bianyan[CAMERA_H];                              //定义储存边沿信息数据
uint8  bianyan_flag = 0;                                //定义边沿flag
uint8  midline1_L,midline2_L,midline3_L;

/*
 *  @brief      路况标识初始化函数
 *  @param
*/
void Flag_Init(void)
{
	cross_flag = 0;
	S_flag = 0;
	L_Sharp_turn_flag = 0;
	R_Sharp_turn_flag = 0;
	Big_Turn_flag = 0;
	L_Loop_flag = 0;
	R_Loop_flag = 0;
	Barrier_flag = 0;
	Stop_flag = 0;
}

/*
*  @brief      蜂鸣器初始化函数
*  @param
*/
void Buzzer_Init(void)
{
	gpio_init(Buzzer, GPO, 0);
}

/*
*  @brief      蜂鸣器响一声函数
*  @param
*/
void Buzzer_NO(void)
{
	gpio_set(Buzzer, 1);     //开
	DELAY_MS(100);
	gpio_set(Buzzer, 0);
}


/*
*  @brief      获取中线函数，并且提取边沿信息
*  @param      
               隐藏调用全局数组：img[CAMERA_H][CAMERA_W]; imgbuff[CAMERA_SIZE];
			   隐藏输出全局数组：zhongxian[CAMERA_H][CAMERA_W]; bianyan[CAMERA_H];
			   隐藏输出全局变量：bianyan_flag
*/
void Get_MiddleLine(uint8 midline1_H, uint8 midline2_H, uint8 midline3_H)
{
	int i, j, n, left_back, right_back;
	uint8 right_flag, left_flag;
	/************************************************************************/
	/*                          中线数组赋初值（全白）                      */
	  for (i = 0; i < CAMERA_H; i++)
		 for (j = 0; j < CAMERA_W; j++)
			zhongxian[i][j] = 255;
	/************************************************************************/

   /************************************************************************/
   /*                            提取中线                                  */
	  for (i = CAMERA_H - 1; i > CAMERA_H - 4; i--)          //提取前三行中线
	  {
		  for (j = CAMERA_W / 2; j < CAMERA_W; j++)
		  {
			  if (img[i][j] == 0xff && img[i][j + 1] == 0x00)
			  {
				  right_back = j;
				  right_flag = 10;
				  break;
			  }
			  else
			  {
				  right_flag = 0;
				  right_back = CAMERA_W - 5;
			  }
		  }
		  for (j = CAMERA_W / 2; j > 0; j--)
		  {
			  if (img[i][j] == 0xff && img[i][j - 1] == 0x00)
			  {
				  left_back = j;
				  left_flag = 1;
				  break;
			  }
			  else
			  {
				  left_flag = 0;
				  left_back = 5;
			  }
		  }
	  }

	  for (i = CAMERA_H - 4; i > 0; i--)              //提取中线及其他信息
	  {
		  for (j = right_back - 10; j < (MIN((right_back + 10), CAMERA_W)); j++)
		  {
			  if (img[i][j] == 0xff && img[i][j + 1] == 0x00)
			  {
				  right_back = j;
				  right_flag = 10;
				  break;
			  }
			  else right_flag = 0;
		  }
		  for (j = left_back + 10; j > (MAX((left_back - 10), 0)); j--)
		  {
			  if (img[i][j] == 0xff && img[i][j - 1] == 0x00)
			  {
				  left_back = j;
				  left_flag = 1;
				  break;
			  }
			  else left_flag = 0;
		  }
                  
          /*提取前瞻行*/
		  if (i == midline1_H)
		  {
			  midline1_L = n;
		  }
		  if (i == midline2_H)
		  {
			  midline2_L = n;
		  }
          if(i==midline3_H)
		  {
			 midline3_L = n;
		  }

		  n = (right_back + left_back) / 2;
		  bianyan_flag = left_flag + right_flag;

		  img[i][n] = 0;
		  zhongxian[i][n] = 0;
		  bianyan[i] = bianyan_flag;
	  }
  /************************************************************************/
}

/*
*  @brief      获取路面特殊路况标志函数
*  @param
*/
void Get_Road_flag(void)
{
	uint8 Sharp_left_number = 0, Sharp_right_number = 0;
	uint8 cross_number = 0;
	/************************************************************************/
	/*         ********判断急弯情况*******                                 */
	/************************************************************************/
	if (bianyan_flag == 1)                 //如果边沿信息和为1，说明右边丢了边沿
	{
		Sharp_right_number++;
		if (Sharp_right_number >= 20)
		{
			R_Sharp_turn_flag = 1;        //如果连续20行右边丢边沿，则判断此时为右急弯
			Buzzer_NO();
		}
	}
	else Sharp_right_number = 0;

	if (bianyan_flag == 10)                 //如果边沿信息和为10，说明左边丢了边沿
	{
		Sharp_left_number++;
		if (Sharp_left_number >= 20)
		{
			L_Sharp_turn_flag = 1;        //如果连续20行左边丢边沿，则判断此时为左急弯
			Buzzer_NO();
		}
	}
	else Sharp_left_number = 0;
	/**********急弯判断结束********/


	/************************************************************************/
	/*                             判断是否为十字                           */

	/************************************************************************/
	if (bianyan_flag == 0)                 //如果边沿信息和为0，说明两边丢了边沿
	{
		cross_number++;
		if (cross_number >= 10)
		{
			cross_flag = 1;                  //如果连续10行两边丢边沿，则判断此时为十字
			Buzzer_NO();
		}
	}
	else cross_number = 0;

	/************************************************************************/
	/*                          判断连续s弯道                               */
	/*仅根据单边沿数据判断连续s弯道如左边沿
	将某边沿列数存为一个一维数组，当获取完一副图像时对该数组进行隔行扫描
	隔行扫描的目的：1、节省时间；2、能体现出显著差异
	先扫描离车近的若干行数据如35-55行。
	从55行开始，比较52行左边沿列数与55的大小，不管大还是小，都记录一个大小标志位
	继续比较。。直到出现大小标志位不同，表明此处是s弯道突出处。
	找出下一个突出处。
	以这两处突出处
	/************************************************************************/

	/************************************************************************/
	/*                       环岛判断                                       */

	/* ****** 先判断左环岛******/
	/*将左边沿信息也存一个一维数组，在每次获取完一幅图像后扫描该数组。
	从第一行开始搜索该一维数组
	如果左边沿处于连续丢失三行的情况 记录一个标志位a
	当存在标志位a时则从当前行继续向后搜索，
	当搜索到左边沿不丢失时，记录一个标志位b
	当存在标志位b时，从当前行继续向后搜索。
	当ab标志位都存在时，从当前行继续向后搜索，若左边沿又连续三行地出现，则判断当前已经处于左环岛。

	/************************************************************************/

}



