#ifndef _OLED_H
#define _OLED_H
#include "stdbool.h"   
#include "oled.h"
#include "include.h"

#define byte uint8
#define word uint16

/*
#define change_page_last      (bool)PTC6_IN 
#define change_page_next (bool)PTC7_IN
#define change_line (bool)PTC8_IN
#define go_now      (bool)PTC9_IN
#define Add_1       (bool)PTC17_IN  
#define Sub_1       (bool)PTC16_IN
*/



//void pre_show(void);
//void redraw();
//void KeyScan(void); 
void LCD_Init(void);
void LCD_CLS(void);
void write_6_8_char(byte x,byte y,byte ch);
void write_6_8_string(byte x,byte y,byte ch[]);
void write_6_8_number(unsigned char x,unsigned char y, float number);
void write_8_16_char(byte x,byte y,byte ch);
void write_8_16_string(byte x,byte y,byte ch[]);
void write_8_16_number(unsigned char x,unsigned char y, float number);
void LCD_PutPixel(byte x,byte y);
void LCD_Rectangle(byte x1,byte y1,byte x2,byte y2,byte gif);
void Draw_BMP(byte x0,byte y0,byte x1,byte y1,byte bmp[]); 
void LCD_Fill(byte dat);
void LCD_DLY_ms(word ms);
void change_value(unsigned char page,unsigned char m,float i);
void LCD_Set_Pos(byte x, byte y);
void LCD_WrCmd(byte cmd);
void LCD_WrDat(byte data);
/***************功能描述：显示8*16一组整型变量	显示的坐标（x,y），y为页范围0～3****************/
void write_6_8_int(uint16 x,uint16 y,char data);

//void dra_pixel1(void);
//void dra_pixel2(void);
#endif

