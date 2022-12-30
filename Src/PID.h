#ifndef PID_H
#define PID_H
#include "stm32f4xx.h"

extern float g_Yaw, g_Pitch, g_Roll;     //eular

#define OUTTER_LOOP_KP 0 //0.257 * 0.83 0.255
#define OUTTER_LOOP_KI 0
#define OUTTER_LOOP_KD 0

#define INNER_LOOP_KP 0.03f
#define INNER_LOOP_KI 0
#define INNER_LOOP_KD 0

#define SUM_ERRO_MAX 900
#define SUM_ERRO_MIN -900

#define PID_IMAX 30
#define PID_IMIN -30
//	??
#define THROTTLE_MAX (unsigned short)3600	//2ms	-	top position of throttle
#define THROTTLE_MIN (unsigned short)1620	//0.9ms	-	bottom position of throttle


typedef struct {
    float InnerLast;			//Luu gia tri cu cua vong lap ben trong de tao ra su khac biet nguoc
    float OutterLast;			//Luu gia tri cu cua vong lap ben ngoai de tao ra su khac biet nguoc
    float *Feedback;			//Du lieu hoi, du lieu goc thoi gian thuc
    float *Gyro;				///Van toc goc
    float Error;				//Su khac biet
    float p;					//???(??????)
    float i;					//???(?????)
    float d;					//???(??????)
    short output;				//PID??, ????PWM?, 2??
    __IO uint16_t *Channel1;	//PWM??, ??1
    __IO uint16_t *Channel2;	//PWM??, ??2
} pid_st, *pid_pst;

void pid_SingleAxis(pid_pst package, float setPoint);

#endif