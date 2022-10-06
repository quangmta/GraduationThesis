/*
 * PID.h
 *
 *  Created on: Mar 21, 2022
 *      Author: nguye
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct
{
 float Kp;
 float Ki;
 float Kd;
} PidParameter;
#include "main.h"
int16_t PID_Speed_Calc(PidParameter PID,float speed,float setPoint);
int16_t PWM_Calc(float frequency,float pressure);
float  PID_Pressure_Calc (PidParameter PID,float pressure, float setPoint );

#endif /* INC_PID_H_ */
