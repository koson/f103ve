#include "include.h"

uint16_t VirtAddVarTab[NumbOfVar] ={0xAA00, 0xAA01, 0xAA02, 0xAA03, 0xAA04, 0xAA05, 0xAA06, 0xAA07, 0xAA08, 0xAA09, 0xAA0A, 0xAA0B, 0xAA0C, 0xAA0D, 0xAA0E, 0xAA0F,
																		0xAA10, 0xAA11, 0xAA12, 0xAA13, 0xAA14, 0xAA15, 0xAA16, 0xAA17, 0xAA18, 0xAA19, 0xAA1A, 0xAA1B, 0xAA1C, 0xAA1D, 0xAA1E, 0xAA1F,
																		0xAA20, 0xAA21, 0xAA22, 0xAA23, 0xAA24, 0xAA25, 0xAA26, 0xAA27, 0xAA28, 0xAA29, 0xAA2A, 0xAA2B, 0xAA2C, 0xAA2D, 0xAA2E, 0xAA2F,
																		0xAA30, 0xAA31, 0xAA32, 0xAA33, 0xAA34, 0xAA35, 0xAA36, 0xAA37, 0xAA38};

/*====================================================================================================*/
/*====================================================================================================*
**函数 : EE_READ_ACC_OFFSET
**功能 : 读取加速度零偏
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
static void EE_READ_ACC_OFFSET(void)
{
	EE_ReadVariable(VirtAddVarTab[0], &sensor.acc.quiet.x);
	EE_ReadVariable(VirtAddVarTab[1], &sensor.acc.quiet.y);
	EE_ReadVariable(VirtAddVarTab[2], &sensor.acc.quiet.z);
}

/*====================================================================================================*/
/*====================================================================================================*
**函数 : EE_SAVE_ACC_OFFSET
**功能 : 保存加速度零偏
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void EE_SAVE_ACC_OFFSET(void)
{
  EE_WriteVariable(VirtAddVarTab[0],sensor.acc.quiet.x);
  EE_WriteVariable(VirtAddVarTab[1],sensor.acc.quiet.y);
	EE_WriteVariable(VirtAddVarTab[2],sensor.acc.quiet.z);
}	
/*====================================================================================================*/
/*====================================================================================================*
**函数 : EE_READ_MAG_OFFSET
**功能 : 读取磁力计零偏
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
static void EE_READ_MAG_OFFSET(void)
{
	EE_ReadVariable(VirtAddVarTab[3], &HMC58X3_limit[0]);
	EE_ReadVariable(VirtAddVarTab[4], &HMC58X3_limit[1]);
	EE_ReadVariable(VirtAddVarTab[5], &HMC58X3_limit[2]);
	EE_ReadVariable(VirtAddVarTab[6], &HMC58X3_limit[3]);
	EE_ReadVariable(VirtAddVarTab[7], &HMC58X3_limit[4]);
	EE_ReadVariable(VirtAddVarTab[8], &HMC58X3_limit[5]);
}
/*====================================================================================================*/
/*====================================================================================================*
**函数 : EE_SAVE_MAG_OFFSET
**功能 : 保存磁力计零偏
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void EE_SAVE_MAG_OFFSET(void)
{
	u8 cy;
	for(cy=0;cy<6;cy++)
    EE_WriteVariable(VirtAddVarTab[3+cy],*(mag_limt+cy));
}
/*====================================================================================================*/
/*====================================================================================================*
**函数 : EE_SAVE_Attitude_PID
**功能 : 保存姿态PID参数
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void EE_SAVE_Attitude_PID(void)
{
	u16 _temp;
	
	if(flag.ParamSave){
		flag.ParamSave=0;
		_temp = ctrl.pitch.core.kp * 100;
		EE_WriteVariable(VirtAddVarTab[EE_PID_CORE_PITCH_P],_temp);
		_temp = ctrl.pitch.core.ki * 1000;
		EE_WriteVariable(VirtAddVarTab[EE_PID_CORE_PITCH_I],_temp);
		_temp = ctrl.pitch.core.kd * 100;
		EE_WriteVariable(VirtAddVarTab[EE_PID_CORE_PITCH_D],_temp);
		_temp = ctrl.roll.core.kp * 100;
		EE_WriteVariable(VirtAddVarTab[EE_PID_CORE_ROLL_P],_temp);
		_temp = ctrl.roll.core.ki * 1000;
		EE_WriteVariable(VirtAddVarTab[EE_PID_CORE_ROLL_I],_temp);
		_temp = ctrl.roll.core.kd * 100;
		EE_WriteVariable(VirtAddVarTab[EE_PID_CORE_ROLL_D],_temp);
		_temp = ctrl.yaw.core.kp * 100;
		EE_WriteVariable(VirtAddVarTab[EE_PID_CORE_YAW_P],_temp);
		_temp = ctrl.yaw.core.ki * 1000;
		EE_WriteVariable(VirtAddVarTab[EE_PID_CORE_YAW_I],_temp);
		_temp = ctrl.yaw.core.kd * 100;
		EE_WriteVariable(VirtAddVarTab[EE_PID_CORE_YAW_D],_temp);
		
		_temp = ultra_wz_speed_pid.kp * 100;
		EE_WriteVariable(VirtAddVarTab[EE_PID_ULTRA_SPEED_Z_P],_temp);		
		_temp = ultra_wz_speed_pid.ki * 1000;
		EE_WriteVariable(VirtAddVarTab[EE_PID_ULTRA_SPEED_Z_I],_temp);			
		_temp = ultra_wz_speed_pid.kd * 100;
		EE_WriteVariable(VirtAddVarTab[EE_PID_ULTRA_SPEED_Z_D],_temp);
		
		_temp = ultra_pid.kp * 100;
		EE_WriteVariable(VirtAddVarTab[EE_PID_ULTRA_P],_temp);		
		_temp = ultra_pid.ki * 1000;
		EE_WriteVariable(VirtAddVarTab[EE_PID_ULTRA_I],_temp);			
		_temp = ultra_pid.kd * 100;
		EE_WriteVariable(VirtAddVarTab[EE_PID_ULTRA_D],_temp);			

		_temp = baro_pid.kp * 100;
		EE_WriteVariable(VirtAddVarTab[EE_PID_BARO_P],_temp);		
		_temp = baro_pid.ki * 1000;
		EE_WriteVariable(VirtAddVarTab[EE_PID_BARO_I],_temp);			
		_temp = baro_pid.kd * 100;
		EE_WriteVariable(VirtAddVarTab[EE_PID_BARO_D],_temp);

		_temp = baro_wz_speed_pid.kp * 100;
		EE_WriteVariable(VirtAddVarTab[EE_PID_BARO_SPEED_Z_P],_temp);		
		_temp =  baro_wz_speed_pid.ki * 1000;
		EE_WriteVariable(VirtAddVarTab[EE_PID_BARO_SPEED_Z_I],_temp);			
		_temp =  baro_wz_speed_pid.kd * 100;
		EE_WriteVariable(VirtAddVarTab[EE_PID_BARO_SPEED_Z_D],_temp);
	}
}	

/*====================================================================================================*/
/*====================================================================================================*
**函数 : EE_READ_Attitude_PID
**功能 : 读取姿态PID参数
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
static void EE_READ_Attitude_PID(void)
{
	int16_t _temp;
	EE_ReadVariable(VirtAddVarTab[EE_PID_CORE_PITCH_P],&_temp);
	
	//  参数为0  说明已经清空flash  则使用默认参数
	if(_temp==0)  return;
	ctrl.pitch.core.kp =(float)_temp / 100;	
	EE_ReadVariable(VirtAddVarTab[EE_PID_CORE_PITCH_I],&_temp);
	ctrl.pitch.core.ki =(float)_temp / 1000;
	EE_ReadVariable(VirtAddVarTab[EE_PID_CORE_PITCH_D],&_temp);
	ctrl.pitch.core.kd = (float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_PID_CORE_ROLL_P],&_temp);
	ctrl.roll.core.kp =(float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_PID_CORE_ROLL_I],&_temp);
	ctrl.roll.core.ki = (float)_temp / 1000;	
	EE_ReadVariable(VirtAddVarTab[EE_PID_CORE_ROLL_D],&_temp);
	ctrl.roll.core.kd = (float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_PID_CORE_YAW_P],&_temp);
	ctrl.yaw.core.kp =(float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_PID_CORE_YAW_I],&_temp);
	ctrl.yaw.core.ki =(float)_temp / 1000;
	EE_ReadVariable(VirtAddVarTab[EE_PID_CORE_YAW_D], &_temp);
	ctrl.yaw.core.kd =(float)_temp / 100;
	
	EE_ReadVariable(VirtAddVarTab[EE_PID_ULTRA_SPEED_Z_P],&_temp);
	ultra_wz_speed_pid.kp =(float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_PID_ULTRA_SPEED_Z_I],&_temp);
	ultra_wz_speed_pid.ki =(float)_temp / 1000;
	EE_ReadVariable(VirtAddVarTab[EE_PID_ULTRA_SPEED_Z_D], &_temp);
	ultra_wz_speed_pid.kd =(float)_temp / 100;

	EE_ReadVariable(VirtAddVarTab[EE_PID_ULTRA_P],&_temp);
	ultra_pid.kp =(float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_PID_ULTRA_I],&_temp);
	ultra_pid.ki =(float)_temp / 1000;
	EE_ReadVariable(VirtAddVarTab[EE_PID_ULTRA_D], &_temp);
	ultra_pid.kd =(float)_temp / 100;
	
	EE_ReadVariable(VirtAddVarTab[EE_PID_BARO_P],&_temp);
	baro_pid.kp =(float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_PID_BARO_I],&_temp);
	baro_pid.ki =(float)_temp / 1000;
	EE_ReadVariable(VirtAddVarTab[EE_PID_BARO_D], &_temp);
	baro_pid.kd =(float)_temp / 100;
	
	EE_ReadVariable(VirtAddVarTab[EE_PID_BARO_SPEED_Z_P],&_temp);
	baro_wz_speed_pid.kp =(float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_PID_BARO_SPEED_Z_I],&_temp);
	baro_wz_speed_pid.ki =(float)_temp / 1000;
	EE_ReadVariable(VirtAddVarTab[EE_PID_BARO_SPEED_Z_D], &_temp);
	baro_wz_speed_pid.kd =(float)_temp / 100;	
}

//**************************************************************************
//参数加载
//**************************************************************************
void ParamLoad(void)
{
	ctrl.pitch.shell.kp = 1.5;   //shell:外环
	
	ctrl.pitch.shell.ki = 0.01;  
	
	ctrl.pitch.core.kp = 1.5;    //core:内环
	ctrl.pitch.core.ki = 0.01;  
	ctrl.pitch.core.kd = 0.10;  
	
	//The data of roll
	ctrl.roll.shell.kp = 1.5;  
	ctrl.roll.shell.ki = 0.01;  


	ctrl.roll.core.kp = 1.5; 
	ctrl.roll.core.ki = 0.01;
	ctrl.roll.core.kd = 0.10;
	
	//The data of yaw
	ctrl.yaw.shell.kp = 1.5;
	ctrl.yaw.shell.kd = 0.01;
	
	ctrl.yaw.core.kp = 1.85;
	ctrl.yaw.core.ki = 0;
	ctrl.yaw.core.kd = 0.05;
	
	//limit for the max increment
	ctrl.pitch.shell.increment_max = 20;
	ctrl.roll.shell.increment_max = 20;
	
	ctrl.ctrlRate = 0;
	
	WZ_Speed_PID_Init();//高度控制PID初始化
	Ultra_PID_Init();//超声波PID
	Baro_PID_Init();//气压计PID
	
    EE_READ_ACC_OFFSET();   //读取加速度零偏
	EE_READ_MAG_OFFSET();   //读取磁力计零偏
	EE_READ_Attitude_PID(); //读取内环PID参数
	Gyro_OFFSET();          //采集陀螺仪零偏，,所以上电时不静止就悲剧了，除非起飞前重新校准
}
