#include "board_config.h"

led_Fsm LED;
LEDBuf_t LEDBuf;


void LED_GPIO_Config(void)
{		
	/*定义一个GPIO_InitTypeDef类型的结构体*/
	GPIO_InitTypeDef GPIO_InitStructure;

	/*开启GPIOB的外设时钟*/
	RCC_APB2PeriphClockCmd( RCC_GPIO_LED, ENABLE); 
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOE, ENABLE); 
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	//设置LED使用到得管脚
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);

	/*选择要控制的GPIOC引脚*/															   
  GPIO_InitStructure.GPIO_Pin = LED_R | LED_G | LED_B;	

	/*设置引脚模式为通用推挽输出*/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

	/*设置引脚速率为50MHz */   
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

	/*调用库函数，初始化GPIOB*/
  GPIO_Init(GPIO_LED, &GPIO_InitStructure);		  

	/* 关闭所有led灯	*/
	GPIO_SetBits(GPIO_LED, LED_R | LED_G | LED_B);	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6| GPIO_Pin_10 | GPIO_Pin_15;	//添加板上的LED灯
	
  GPIO_Init(GPIOE, &GPIO_InitStructure);	
  GPIO_SetBits(GPIOE, GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_10 | GPIO_Pin_15);	
}

void LED_SHOW(void)
{
   LED_ALLON();
	 delay(900);
	 LED_ALLOFF();
	 delay(16000);
	 LED_ALLON();
	 delay(900);
	 LED_ALLOFF();
	 delay(16000);
	 LED_ALLON();
	 delay(900);
	 LED_ALLOFF();
	 delay(16000);
	 LED_ALLON();
}

void LEDReflash(void)
{
	if(LEDBuf.bits.R)
		Ledr_on;
	else
		Ledr_off;

	if(LEDBuf.bits.G)
		Ledg_on;
	else
		Ledg_off;

	if(LEDBuf.bits.B)
		Ledb_on;
	else
		Ledb_off;
}
void LED_Fsm(void)
{
 	switch(LED.event)
 	{
	  case Ht_ARMED:       
			if(++LED.cnt >= 120)  
				LED.cnt=0;
			if(LED.cnt<10 || (LED.cnt>20 && LED.cnt<30)) 
				LEDBuf.byte =LG|LB;
			else
				LEDBuf.byte =0;
		break;
			
		case Ht_DISARMED:       
			if(++LED.cnt >= 60)
				LED.cnt=0;
		  if(LED.cnt<=20)
				LEDBuf.byte =LR;
			else if(LED.cnt<=40)
				LEDBuf.byte =LG;
			else  
				LEDBuf.byte =LB;
		break;	
			
		case Ht_CALIBRATA:           
        LEDBuf.byte =LB;
	  break;
		
 		case Ht_CALIBRATM_X:         
        LEDBuf.byte =LR;
 		break;
		
		case Ht_CALIBRATM_Y:         
        LEDBuf.byte =LG;
 		break;
		
		case Ht_CALIBRATM_Z:         
        LEDBuf.byte =LB;
 		break;
		
		case Ht_CALIBRATG:         
			  LEDBuf.byte =LR;
		break;
 	}
	LEDReflash();
}
/*====================================================================================================*/
/*====================================================================================================*
**函数 : Hto_LED_Reflash
**功能 : LED状态机
**输入 :  
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void FailSafeLEDAlarm(void)
{
	if(flag.ARMED)
		LED.event=Ht_ARMED;//解锁了
	
	if(!flag.ARMED)
		LED.event=Ht_DISARMED;//没解锁
	
	if(flag.calibratingA)
		LED.event=Ht_CALIBRATA;
	
	if(flag.calibratingG)		
 		 LED.event=Ht_CALIBRATG;
	
	if(flag.calibratingM){
		switch(flag.calibratingM)
 	  {
			case 1:LED.event=Ht_CALIBRATM_X; break;
			case 2:LED.event=Ht_CALIBRATM_Y; break;
			case 3:LED.event=Ht_CALIBRATM_Z; break;
		}
	}
	LED_Fsm();
}

