/*
*************************************************************************************************************************
*		        版本：超声波定高版
*        
*               超声波接线： VCC---5V
*							 ECHO -> CH7（PB0）
*							 TRIG -> CH8（PB1）
*						 
*				遥控器CH5通控制飞行模式：CH5接两挡开关
*							 低档：手动控制
*							 高档：超声波定高模式     
*						PS:只能在手动下解锁
*
*				电调油门行程统一校准步骤，校准最好不要上桨
*							 1)关闭飞行器电源
*							 2)打开遥控器，将油门推到最大；
*							 3)打开飞行器电源，此时会听到“滴滴”的声音，显示屏会显示“电调校准”；
*							 4)此时立即将遥控器油门摇杆推到最小，听到“滴滴--滴”的音乐声；
*							 5)推动油门摇杆电机会转动，代表电调校准完毕，需要重启飞控进入正常模式；
*   		
************************************************************************************************************************
*/
#include "board_config.h"

int main(void)
{
	/* 系统初始化 */
	System_Init(); 		
	/* 参数加载函数 */
	ParamLoad();	
	/* 开启定时器 */
	TIM_ON;		
	while(1)
	{
		/* 任务处理 */
		Task();
	}			
}
