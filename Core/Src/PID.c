/*
 * PID.c
 *
 *  Created on: Mar 21, 2022
 *      Author: nguye
 */
#include "PID.h"
#include "main.h"
#include "math.h"
uint32_t timerPID_pres=0;
float last_error_pres=0;
float integrated_error_pres=0;
extern UART_HandleTypeDef huart1;
uint32_t timerPID_pres_speed=0;
float last_error_pres_speed=0;
float integrated_error_pres_speed=0;
float delta = 80;

//int16_t PWM_Calc(float frequency,float pressure)
//{
//	int16_t pwm;
//	float coeff_freq = COEFFICIENT_FREQ;
//	if(frequency>-600 && frequency<600)
//	{
//		if (frequency > 0)
//			coeff_freq = COEFFICIENT_FREQ*(2.2-frequency*0.002);
//		else
//			coeff_freq = COEFFICIENT_FREQ*(2.2+frequency*0.002);
//	}
//	pwm = (int)(frequency*coeff_freq);
//	
//	if (pwm>PWM_MAX) pwm=PWM_MAX;
//	else if(pwm<-PWM_MAX) pwm=-PWM_MAX;
//	return pwm;
//}
int16_t PID_Speed_Calc(PidParameter PID,float speed,float setPoint)
{
  int16_t pidOut=0;
	float pTerm = 0, iTerm = 0,dTerm = 0;
	float dt = (float) (HAL_GetTick() - timerPID_pres_speed);
	timerPID_pres_speed= HAL_GetTick();
	float error = setPoint - speed;	
	pTerm =   PID.Kp  * error;	
	integrated_error_pres_speed += error * dt;
	iTerm =   PID.Ki * integrated_error_pres_speed/1000.0;
	if ( dt != 0) {
		dTerm =  1000*PID.Kd * (error-last_error_pres_speed)/dt;
	}
	pidOut = (int16_t)(pTerm + iTerm + dTerm);
	last_error_pres_speed = error;	
	if (pidOut>PWM_MAX) pidOut=PWM_MAX;
	else if(pidOut<-PWM_MAX) pidOut=-PWM_MAX;
	return pidOut;
}
float  PID_Pressure_Calc (PidParameter PID,float pressure, float setPoint )
{
	float pidOut=0;
	if(pressure> PRESSURE_MAX ||pressure< PRESSURE_MIN)
	{
		pidOut=0;
		last_error_pres=0;
		integrated_error_pres=0;
	}
	else
	{
		float pTerm = 0, iTerm = 0,dTerm = 0;
		float dt = (float) (HAL_GetTick() - timerPID_pres);
		timerPID_pres= HAL_GetTick();
		float error = setPoint - pressure;
		pTerm =   PID.Kp  * error;
		if((error<20 && error>-20) || integrated_error_pres >400 || integrated_error_pres <-400)
			integrated_error_pres += error * dt;
		iTerm =   PID.Ki * integrated_error_pres/1000.0;
		if ( dt != 0) {
			dTerm =  1000*PID.Kd * (error-last_error_pres)/dt;
		}		
		pidOut = pTerm + iTerm + dTerm;
		if (error>0) pidOut+=delta;
		else pidOut-=delta;
		last_error_pres = error;
		if (pidOut>FREQUENCY_MAX) pidOut=FREQUENCY_MAX;
		else if(pidOut<-FREQUENCY_MAX) pidOut=-FREQUENCY_MAX;		
		if(error>-0.5 && error<0.5)
			pidOut = 0;
	}
	return pidOut;
}

