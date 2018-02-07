#ifndef _LCD_MENU_H_
#define _LCD_MENU_H_

/*------------------------------宏定义枚举-----------------------------*/



/*----------------------------------------------------------------------*/

/*------------------------------变量声明区------------------------------*/

extern Site_t site;
extern Size_t imgsize;
extern Size_t size;

extern int8 m_menu,sub_menu;

/*----------------------------------------------------------------------*/

/*------------------------------函数声明区------------------------------*/

void HCI_lcd_open();
void HCI_lcd_close();
void HCI_lcd_work();

void HCI_lcd_11();
void HCI_lcd_12();
void HCI_lcd_13();
void HCI_lcd_14();

void HCI_lcd_21();
void HCI_lcd_22();
void HCI_lcd_23();
void HCI_lcd_24();

void HCI_lcd_31();
void HCI_lcd_32();
void HCI_lcd_33();
void HCI_lcd_34();

/*----------------------------------------------------------------------*/






#endif



