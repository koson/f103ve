#include "stm32f10x_it.h"
#include "board_config.h"
#include "include.h"
void TIM5_IRQHandler(void)		    //2.5ms中断一次
{	
	if(TIM5->SR & TIM_IT_Update)	{    
    TIM5->SR = ~TIM_FLAG_Update;
 
		realtime++;

 		testtime=TIM5->CNT;
		Time_slice();//2us		
		RDAU();//62us对接收的遥控通道值进行处理，包含解锁、校准等动作解析	
		AHRS_Geteuler();//1650us，其中地磁占用310us		
		Calculate_Target();	//5us
		Altitute_calculate();//180+200us			
		CONTROL(Target);//65us
 	  testtime=TIM5->CNT-testtime;  
	}
}

u8 TxBuffer[256];
u8 TxCounter=0;
u8 count=0; 
u8 Rxcounter=0,count_rx=0;
u8 Rx_Buf[256];	//串口接收缓存

void USART1_IRQHandler(void)
{
	u8 com_data;
	
	if( USART_GetITStatus(USART1,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);//清除中断标志

		com_data = USART1->DR;
		Rx_Buf[Rxcounter++]=com_data;
	}
	
	//发送（进入移位）中断，不使用，其实没有打开发送中断的。
	if( USART_GetITStatus(USART1,USART_IT_TXE ) )
	{
		USART_ClearITPendingBit(USART1,USART_IT_TXE);//清除发送中断标志
	}
}

extern u8 Flag_Uart_Send;
//串口1  DMA方式发送中断  
void DMA1_Channel4_IRQHandler(void)  
{   
  //清除标志位  
   DMA_ClearFlag(DMA1_FLAG_TC4);   
	 Flag_Uart_Send=0;                    
} 


/*
*********************************************************************************************************
*	Cortex-M3 内核异常中断服务程序
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*	函 数 名: NMI_Handler
*	功能说明: 不可屏蔽中断服务程序。
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/  
void NMI_Handler(void)
{
}

/*
*********************************************************************************************************
*	函 数 名: HardFault_Handler
*	功能说明: 硬件失效中断服务程序。
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/ 
void HardFault_Handler(void)
{
  /* 当硬件失效异常发生时进入死循环 */
  while (1)
  {
  }
}

/*
*********************************************************************************************************
*	函 数 名: MemManage_Handler
*	功能说明: 内存管理异常中断服务程序。
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/   
void MemManage_Handler(void)
{
  /* 当内存管理异常发生时进入死循环 */
  while (1)
  {
  }
}

/*
*********************************************************************************************************
*	函 数 名: BusFault_Handler
*	功能说明: 总线访问异常中断服务程序。
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/    
void BusFault_Handler(void)
{
  /* 当总线异常时进入死循环 */
  while (1)
  {
  }
}

/*
*********************************************************************************************************
*	函 数 名: UsageFault_Handler
*	功能说明: 未定义的指令或非法状态中断服务程序。
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/   
void UsageFault_Handler(void)
{
  /* 当用法异常时进入死循环 */
  while (1)
  {
  }
}

/*
*********************************************************************************************************
*	函 数 名: SVC_Handler
*	功能说明: 通过SWI指令的系统服务调用中断服务程序。
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/   
void SVC_Handler(void)
{
}

/*
*********************************************************************************************************
*	函 数 名: DebugMon_Handler
*	功能说明: 调试监视器中断服务程序。
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/   
void DebugMon_Handler(void)
{
}

/*
*********************************************************************************************************
*	函 数 名: PendSV_Handler
*	功能说明: 可挂起的系统服务调用中断服务程序。
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/     
void PendSV_Handler(void)
{
}

/*
*********************************************************************************************************
*	函 数 名: SysTick_Handler
*	功能说明: 系统嘀嗒定时器中断服务程序。
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/     
// extern void SysTick_ISR(void);	/* 声明调用外部的函数 */
// void SysTick_Handler(void)
// {
// 	SysTick_ISR();	/* 这个函数在bsp_timer.c中 */
// }

/*
*********************************************************************************************************
*	STM32F10x内部外设中断服务程序
*	用户在此添加用到外设中断服务函数。有效的中断服务函数名请参考启动文件(startup_stm32f10x_xx.s)
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*	函 数 名: USB_LP_CAN1_RX0_IRQHandler
*	功能说明: 低优先级USB中断服务程序。
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/ 


