#include "MultiRotor_rc.h"
#include "include.h"

//遥控器操作
RC_GETDATA RC_Data;
rcReadRawData rcReadRawFunc = RC_Data_Refine;
extern u16 Moto_duty[MOTOR_NUM];

void RDAU(void)
{
	RC_directive();
	rcReadRawFunc();
}

/*====================================================================================================*/
/*====================================================================================================*
**函数 : RC_directive
**功能 : 遥控指令
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void RC_directive(void)
{
  u8 stTmp = 0,keyTemp=0,i;
	static u8  rcSticks,keySticks;
	static u8  rcDelayCommand,keyDelayCommand;
  static u16 seltLockCommend;	
/**************************以下处理遥控控制通道数据******************************/	
	for (i = 0; i < 4; i++) {
			stTmp >>= 2;
			if (RC_Data.rc_data[i] > RC_MINCHECK)
					stTmp |= 0x80;  // check for MIN
			if (RC_Data.rc_data[i] < RC_MAXCHECK)
					stTmp |= 0x40;  // check for MAX
	}

	if (stTmp == rcSticks) {
			if (rcDelayCommand < 250)//250
					rcDelayCommand++;
	} else
			rcDelayCommand = 0;
	rcSticks = stTmp;
	
	if (rcDelayCommand == 250) {//150
		if (flag.ARMED){
			 if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_CE)   //上锁 左下+中位
				  flag.ARMED=0;
		}
		else{
					if ((rcSticks == THR_LO + YAW_HI + PIT_CE + ROL_CE) && flag.FlightMode==MANUAL_High)    //解锁 右下+中位 ,处于手动模式 
					{
						 flag.ARMED=1;Pressure_groud=Pressure;//解锁即记下当前的气压值作为地面气压值。
					}
					if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_HI)    //加速度矫正 左下+右
							{flag.calibratingA = 1;flag.calibratingG = 1;}
					if ((rcSticks == THR_HI + YAW_LO + PIT_LO + ROL_HI) && flag.calibratingM_pre)//左上+右上指南针矫正 
							flag.calibratingM = 1; 
					if (rcSticks == THR_HI + YAW_HI + PIT_LO + ROL_LO)//右上+左上    
							flag.calibratingM_pre = 1;
					else flag.calibratingM_pre = 0;	
    }
	}
	//解锁之后一段时间油门保持最低  则自动上锁
	if (flag.ARMED){
	   if (rcSticks == THR_LO + YAW_CE + PIT_CE + ROL_CE) {
		    if (seltLockCommend < AUTODISARMDE_TIME)
					 seltLockCommend++;
				else 
					 flag.ARMED=0;
		 }
		 else 
        seltLockCommend = 0;			 
	}

/**************************以下处理遥控开关数据******************************/
    for (i = 0; i < 4; i++) {
            keyTemp >>= 2;
            if (RC_Data.rc_data[i+4] > RC_MINCHECK)
                    keyTemp |= 0x80;  // check for MIN
            if (RC_Data.rc_data[i+4] < RC_MAXCHECK)
                    keyTemp |= 0x40;  // check for MAX
            }
    if (keyTemp == keySticks) {
            if (keyDelayCommand < 200)//250
                    keyDelayCommand++;
            else keyDelayCommand = 0;
    } else
            keyDelayCommand = 0;
    keySticks = keyTemp;

    if (keyDelayCommand==120) {
       // if (flag.ARMED) {
            if ((keySticks&0X03)==CH5_LO) {//CH5处于低位时，正常模式
                flag.FlightMode=MANUAL_High;
							  flag.HUDMode=STABILIZE_MODE;
							//keephigh=0;
            }
            if ((keySticks&0X03)==CH5_HI) {//CH5处于高位时，超声波定高模式
              //gps_flag=1;
							  flag.FlightMode=ULTRASONIC_High;//ULTRASONIC_High;ACC_High;// 
							  flag.HUDMode=ALTHOLD_MODE;
			}
        //}
    }
}
/*====================================================================================================*/
/*====================================================================================================*
**函数 : RcData_Refine
**功能 : 提炼遥控数据
**输入 : None
**输出 : None
**备注 : 无
**====================================================================================================*/
/*====================================================================================================*/
void RC_Data_Refine(void)
{
  u8 chan,a;	

	u16 rcDataMax[8], rcDataMin[8];
	static int16_t rcDataCache[8][4], rcDataMean[8];
	static uint8_t rcValuesIndex = 0;

	rcValuesIndex++;
	for (chan = 0; chan < 8; chan++) {
		  //滑动平均值滤波，4次
		  if(RC_Pwm_In[chan]>2800 || RC_Pwm_In[chan]<800)  RC_Pwm_In[chan] = RC_Pwm_In_his[chan];
			rcDataCache[chan][rcValuesIndex % 4] = RC_Pwm_In[chan] ;		
		  RC_Pwm_In_his[chan] = RC_Pwm_In[chan];
			
			rcDataMean[chan] = 0;
		  rcDataMax[chan]  = 0;
		  rcDataMin[chan]  = 25000;
		
			for (a = 0; a < 4; a++) {
				  // 记录缓存中最大值 && 最小值
				  if(rcDataCache[chan][a] > rcDataMax[chan])  rcDataMax[chan] = rcDataCache[chan][a];     
					if(rcDataCache[chan][a] < rcDataMin[chan])	rcDataMin[chan] = rcDataCache[chan][a]; 
				  // 求和
					rcDataMean[chan] += rcDataCache[chan][a];  
      }
			// 剔除缓存中 最大值 && 最小值 
			rcDataMean[chan] = (rcDataMean[chan] - (rcDataMax[chan] + rcDataMin[chan])) / 2;
	} 

	 RC_Data.ROLL     = RC_Data.rc_data[0] = rcDataMean[0];
	 RC_Data.PITCH    = RC_Data.rc_data[1] = rcDataMean[1];
	 RC_Data.THROTTLE = RC_Data.rc_data[3] = rcDataMean[2]; 
	 RC_Data.YAW      = RC_Data.rc_data[2] = rcDataMean[3];
	 RC_Data.rc_data[4] = rcDataMean[4];
	 RC_Data.rc_data[5] = rcDataMean[5];
	 RC_Data.rc_data[6] = rcDataMean[6];
}

/*====================================================================================================*/
/*====================================================================================================**
**函数 : ECS_Calibrate
**功能 : 电调油门行程设置
**输入 : None
**输出 : None
**备注 :
		//             油门行程设置               //
	  //     —————————            —————————     //
	  //    |    |    |          |         |    //
	  //    |    |    |          |    |    |    //
	  //    |         |          |    |    |    //
	  //    |         |          |         |    //
	  //     —————————            —————————     //
	  //   油门推到最高         右侧摇杆推中间  //	
**====================================================================================================*/
/*====================================================================================================*/
void ECS_Calibrate(void)
{
	static vs8 ECS_Calibrate_FLAG=0;
	static vs16 time=2000;
	while(time--)
	  RDAU();
  while(RC_Data.THROTTLE >= 1850 || ECS_Calibrate_FLAG)
  {   
		LED_ALLON();//白色LED常亮
		ECS_Calibrate_FLAG=1;//油门行程标志位置1
    RDAU();	
			
		#ifdef QUADROTOR 
     Moto_duty[0] = Moto_duty[1] = Moto_duty[2] = Moto_duty[3] = RC_Data.THROTTLE-1000;
		#elif defined HEXRCOPTER
      Moto_duty[0] = Moto_duty[1] = Moto_duty[2] = Moto_duty[3] = Moto_duty[4] = Moto_duty[5] = RC_Data.THROTTLE-1000;
		#endif 
	  moto_PwmRflash(&Moto_duty[0]);//马达输出刷新，直接写PWM输出寄存器
  }
}
