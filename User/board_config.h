#ifndef __BOARD_CONFIG_H
#define	__BOARD_CONFIG_H

#include "include.h"


#define QUADROTOR 

#ifdef QUADROTOR 
		#define MOTOR_NUM 4
#elif defined HEXRCOPTER
		#define MOTOR_NUM 6
#endif 

#define ANO//便使用匿名地面站

/*-------------向量表偏移量----------------------*/
/*-------------重要  不要动----------------------*/
#define FLASH_EXCURSION  0x20000
#define pro_FALG_ADD 0x0801FFF0

/*----------------电机怠速----------------------*/
#define IDLING 180

/*--------------遥控控制方式选择----------------*/
#define RC_CONTROL_USE_NRF24l01

/*---------------陀螺仪采集---------------------*/
#define GYRO_GATHER   700 //原来是100

/*----------------油门检查----------------------*/
#define RC_MINCHECK   1200
#define RC_MAXCHECK   1800

/*-------------当前遥控的RC值情况-----------------*/
#define MINRCVALUE 1110 
#define MIDRCVALUE 1520
#define MAXRCVALUE 1930


/*-------------自动解除武装时间-----------------*/
#define AUTODISARMDE_TIME 2000  //

#define MANUAL_High 1
#define ULTRASONIC_High 2//当前处于哪个控高模式
#define ATMOSPHERE_High 3
#define AUTO_High 4
#define ERROR_High 5
#define ACC_High 6

#define RC_MODE 0x01
#define PC_MODE 0x02

#define STABILIZE_MODE 0x01
#define ALTHOLD_MODE   0x02
#define POSHOLD_MODE   0x03
#define AUTO_MODE      0x04
#define LAND_MODE      0x05
#define CIRCLE_MODE    0x06
#define RTL_MODE       0x07

#define SecondHigh_Factor 0.02//与下面的参数一起用于高度控制互补滤波
#define MainHigh_Factor 0.98

#define Ultrasonic_MAX_Height 2500  //单位是mm
#define Ultrasonic_MIN_Height 200  //单位是mm
#define Baro_MAX_Height 8000  //单位是mm,8米，可以根据实际修改
#define Baro_MIN_Height 1000  //单位是mm,1米，再低其实不太可靠，可以根据实际修改
#define TT 0.0025//控制周期2.5ms，与定时器5中断时间对应
#define CTRL_HEIGHT 1       //0失能，1使能控高功能
#define TAKE_OFF_THR 550 //根据个人飞机实际情况，设置起飞离地油门，用于定高模式下稍推油起飞。
#define MAX_PWM				100			///%	最大PWM输出为100%油门
#define MAX_THR       80 			///%	油门通道最大占比80%，留20%给控制量
typedef void (*rcReadRawData)(void);        

#endif /* __BOARD_CONFIG_H */
