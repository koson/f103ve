#ifndef _OLED_H
#define _OLED_H

#include "stm32f10x.h"
#define OLED_GPIO    GPIOB
#define OLED_RCCEN   RCC_APB2Periph_GPIOB
#define OLED_SCL     GPIO_Pin_12      //D0
#define OLED_SDA     GPIO_Pin_13      //D1
#define OLED_RST     GPIO_Pin_15      //RST
#define OLED_DC      GPIO_Pin_14      //DC

#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据

#define OLED_D0_OL		GPIO_ResetBits(OLED_GPIO, OLED_SCL)	//D0 IO口输出低电平
#define OLED_D0_OH		GPIO_SetBits(OLED_GPIO, OLED_SCL)  	//D0 IO口输出高电平

#define OLED_D1_OL		GPIO_ResetBits(OLED_GPIO, OLED_SDA)	//D1 IO口输出低电平
#define OLED_D1_OH		GPIO_SetBits(OLED_GPIO, OLED_SDA) 	//D1 IO口输出高电平

#define OLED_RST_OL		GPIO_ResetBits(OLED_GPIO, OLED_RST)	//RST IO口输出低电平
#define OLED_RST_OH		GPIO_SetBits(OLED_GPIO, OLED_RST) 	//RST IO口输出高电平

#define OLED_DC_OL		GPIO_ResetBits(OLED_GPIO, OLED_DC)	//DC IO口输出低电平
#define OLED_DC_OH		GPIO_SetBits(OLED_GPIO, OLED_DC) 	//DC IO口输出高电平

#define OLED_RST_Clr()  GPIO_ResetBits(OLED_GPIO, OLED_RST) //RST IO口输出低电平
#define OLED_RST_Set()  GPIO_SetBits(OLED_GPIO, OLED_RST) 	//RST IO口输出高电平

#define OLED_RS_Clr()   GPIO_ResetBits(OLED_GPIO, OLED_DC)	//DC IO口输出低电平
#define OLED_RS_Set()   GPIO_SetBits(OLED_GPIO, OLED_DC) 	  //DC IO口输出高电平

#define OLED_SCLK_Clr() GPIO_ResetBits(OLED_GPIO, OLED_SCL)	//D0 IO口输出低电平
#define OLED_SCLK_Set() GPIO_SetBits(OLED_GPIO, OLED_SCL)  	//D0 IO口输出高电平

#define OLED_SDIN_Clr() GPIO_ResetBits(OLED_GPIO, OLED_SDA)	//D1 IO口输出低电平
#define OLED_SDIN_Set() GPIO_SetBits(OLED_GPIO, OLED_SDA) 	//D1 IO口输出高电平

void setgpio(void);
void OLED_Init(void);
void OLED_CLS(void);
void OLED_4num(unsigned char x,unsigned char y,int number);
void OLED_3num(unsigned char x,unsigned char y,unsigned int number);
void OLED_Num(unsigned char x,unsigned char y,unsigned char asc);
void OLED_P6x8Str(unsigned char x,unsigned char y,unsigned char ch[]);
void OLED_P8x16Str(unsigned char x,unsigned char y,unsigned char ch[]);
void OLED_P14x16Str(unsigned char x,unsigned char y,unsigned char ch[]);
void OLED_Print(unsigned char x, unsigned char y, unsigned char ch[]);
void OLED_PutPixel(unsigned char x,unsigned char y);
void OLED_Rectangle(int16_t acc_x,int16_t acc_y);
void Draw_Logo(void);
void Draw_BMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char bmp[]); 
void OLED_Fill(unsigned char dat);
void Dis_String(unsigned char y, unsigned char x, unsigned char ch[]);
void Dis_Char(unsigned char y,unsigned char x,unsigned char asc);
void Dis_Num(unsigned char y, unsigned char x, unsigned int num,unsigned char N);
void Dis_Float(unsigned char Y,unsigned char X,double real,unsigned char N);
void Dis_Float2(unsigned char Y,unsigned char X,double real,unsigned char N1,unsigned char N2);
void OLED_P6x8Num_8bit(unsigned char x,unsigned char y,unsigned char Number); 
void OLED_Num5(unsigned char x,unsigned char y,unsigned int number);
#endif
