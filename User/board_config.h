#ifndef __BOARD_CONFIG_H
#define	__BOARD_CONFIG_H

#include "include.h"


#define QUADROTOR 

#ifdef QUADROTOR 
		#define MOTOR_NUM 4
#elif defined HEXRCOPTER
		#define MOTOR_NUM 6
#endif 

#define ANO//��ʹ����������վ

/*-------------������ƫ����----------------------*/
/*-------------��Ҫ  ��Ҫ��----------------------*/
#define FLASH_EXCURSION  0x20000
#define pro_FALG_ADD 0x0801FFF0

/*----------------�������----------------------*/
#define IDLING 180

/*--------------ң�ؿ��Ʒ�ʽѡ��----------------*/
#define RC_CONTROL_USE_NRF24l01

/*---------------�����ǲɼ�---------------------*/
#define GYRO_GATHER   700 //ԭ����100

/*----------------���ż��----------------------*/
#define RC_MINCHECK   1200
#define RC_MAXCHECK   1800

/*-------------��ǰң�ص�RCֵ���-----------------*/
#define MINRCVALUE 1110 
#define MIDRCVALUE 1520
#define MAXRCVALUE 1930


/*-------------�Զ������װʱ��-----------------*/
#define AUTODISARMDE_TIME 2000  //

#define MANUAL_High 1
#define ULTRASONIC_High 2//��ǰ�����ĸ��ظ�ģʽ
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

#define SecondHigh_Factor 0.02//������Ĳ���һ�����ڸ߶ȿ��ƻ����˲�
#define MainHigh_Factor 0.98

#define Ultrasonic_MAX_Height 2500  //��λ��mm
#define Ultrasonic_MIN_Height 200  //��λ��mm
#define Baro_MAX_Height 8000  //��λ��mm,8�ף����Ը���ʵ���޸�
#define Baro_MIN_Height 1000  //��λ��mm,1�ף��ٵ���ʵ��̫�ɿ������Ը���ʵ���޸�
#define TT 0.0025//��������2.5ms���붨ʱ��5�ж�ʱ���Ӧ
#define CTRL_HEIGHT 1       //0ʧ�ܣ�1ʹ�ܿظ߹���
#define TAKE_OFF_THR 550 //���ݸ��˷ɻ�ʵ��������������������ţ����ڶ���ģʽ����������ɡ�
#define MAX_PWM				100			///%	���PWM���Ϊ100%����
#define MAX_THR       80 			///%	����ͨ�����ռ��80%����20%��������
typedef void (*rcReadRawData)(void);        

#endif /* __BOARD_CONFIG_H */
