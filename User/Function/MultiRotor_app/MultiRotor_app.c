#include "include.h"
#include "MultiRotor_app.h"

//应用程序
uint8 RXBUF[32];

extern uint8 rxdata,rxdata1;
fp32 Battery_Voltage;
Flag_t flag;
u8 NRFRXOK,RXstate;
extern u16 Moto_duty[MOTOR_NUM];
extern u8 timetoconver;		
extern u8 GpsFlag;
extern float Longitude_val;
extern u8 Location_Sta;

u16 tempnum;
	
struct KALMAN_Data{
			double x_last;
			double p_last;
}Kalman_Data;


void System_Init(void)
{
	delay_init(72);
	Nvic_Init();
	I2C_INIT();
	LED_GPIO_Config();
	USART1_Config();
	TIM5_Config();
	PWM_OUT_Config();
	PWM_IN_Config();
	ECS_Calibrate();
	Ultrasonic_Config();
	LED_SHOW();
	FLASH_Unlock();
	EE_Init();	
	flag.MpuExist = InitMPU6050();
}


unsigned int testT,testT_old;
unsigned int DISARMED_count,RC_count;

void Task(void)
{	
	if(flag.Loop_100Hz){
		flag.Loop_100Hz=0;
		#ifdef ANO 
			ANO_DT_Data_Exchange();					//使用匿名地面站发送与接收数据	
		#endif 	
	}
		
	if(flag.Loop_20Hz){
			flag.Loop_20Hz=0;
	}

	if(flag.Loop_200Hz){
			flag.Loop_200Hz=0;
			usart_data_RX();
			FailSafeLEDAlarm();			
	}
		
	if(flag.Loop_40Hz){
			flag.Loop_40Hz=0;
	}

	if(flag.Loop_10Hz)
		{
		  flag.Loop_10Hz=0;
			EE_SAVE_Attitude_PID();		
		}
}

/*====================================================================================================*/
/*====================================================================================================*
**函数 : Time_slice
**功能 : 时间
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void Time_slice(void)
{
  static u16 tick[6]={0,0,0,0,0,0};
	
	tick[0]++;tick[1]++;tick[2]++;tick[3]++;tick[4]++;tick[5]++;
	
	if(tick[0]>=2){
		tick[0] = 0;
		flag.Loop_200Hz = 1;
	}
  if(tick[1]>=4){
		tick[1] = 0;
		flag.Loop_100Hz = 1;
    timetoconver=1;
	}	
	  if(tick[2]>=10){
		tick[2] = 0;
		flag.Loop_40Hz = 1;
	}	
	  if(tick[3]>=15){
		tick[3] = 0;
		flag.Loop_27Hz = 1;
	}			
  if(tick[4] >= 40)	{
		tick[4] = 0;
		flag.Loop_10Hz = 1;
	}
	if(tick[5] >= 20)	{
	  tick[5] = 0;
	  flag.Loop_20Hz = 1;
  }
}

