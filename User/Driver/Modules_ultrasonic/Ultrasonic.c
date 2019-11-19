#include "include.h"

#define	ECHO_PORT      GPIOB		  //超声波模块ECHO回响信号输出端口---飞控CH7输入PWM 
#define	ECHO_PIN       GPIO_Pin_0	//ECHO--CH7（PB0） 

#define	TRIG_PORT      GPIOB		  //超声波模块TRIG触发信号输入端口---飞控CH8输出PWM
#define	TRIG_PIN       GPIO_Pin_1 //TRIG--CH8（PB1）


float US100_Alt;
float US100_Alt_delta,US100_Alt_old;
unsigned int g_Hight=0,g_HightOld=0;
float g_Alt_Hight=0,g_Alt_HightOld=0;
float g_HightControl=0,g_HightControlold=0,hight_increment=0;
unsigned char RcvIndex,GLengthHigh, GLengthLow; 
float g_hight_Kp=0.8,g_hight_Ki=0.015,g_hight_Kd=10;  
float hight_error=0,hight_errorold=0,hight_erroroldd,cao;

/*====================================================================================================*/
/*====================================================================================================*
**函数 : Ultrasonic_Config
**功能 : 超声波配置
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void Ultrasonic_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;	       
  /* config the extiline(PB0) clock and AFIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);
    
  GPIO_InitStructure.GPIO_Pin = TRIG_PIN;					    
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//设为推挽输出模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         
  GPIO_Init(TRIG_PORT, &GPIO_InitStructure);	        //初始化外设GPIO 	
	GPIO_ResetBits(TRIG_PORT,TRIG_PIN);
}
/*====================================================================================================*/
/*====================================================================================================*
**函数 : Ultrasonic_Pulsing
**功能 : 启动超声波
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void Ultrasonic_Pulsing(void)
{
  GPIO_SetBits(TRIG_PORT,TRIG_PIN);		  //送>10US的高电平
	delay_us(20);
  GPIO_ResetBits(TRIG_PORT,TRIG_PIN);
}

