/*
 * pressureReceive.c
 *
 *  Created on: Mar 26, 2022
 *      Author: nguye
 */

#include "pressureReceive.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"

float char2float(char*buffer)
{
	char* dec = strchr(buffer,'.');
	if (dec == NULL)
	{
		return (float)atoi(buffer);
	}
	else
	{
		char num1[10],num2[10];
		uint8_t i=0;
		while(buffer[i]!='.')
		{
		  num1[i]=buffer[i];
		  i++;
		}
		num1[i]='\0';
		i=0;
		while(dec[i+1]!='\0')
		{
		  num2[i]=dec[i+1];
		  i++;
		}
		num2[i]='\0';
		if(atoi(num1)>=0)
			return (float)atoi(num1)+(float)atoi(num2)/pow(10,i);
		else
			return (float)atoi(num1)-(float)atoi(num2)/pow(10,i);
	}
}
PressureParameter pressureCalcParam(char* buffer,
	float offset)
{
    PressureParameter pressureParam;
    pressureParam.flag = buffer[0];
    pressureParam.Offset = offset;

    if (buffer[0] == 'a' || buffer[0] == 'b')
    {
        char num[30];
        uint8_t num_index = 0;
        while (buffer[num_index] != '\0')
        {
            num[num_index] = buffer[num_index + 1];
            num_index++;
        }
        pressureParam.Amplitude = atof(num);
    }
    else if (buffer[0] == 'c' || buffer[0] == 'd')
    {
        char num1[10], num2[10];
        uint8_t i = 0;
        char* dec = strchr(buffer, ',');
        while (buffer[i + 1] != ',')
        {
            num1[i] = buffer[i + 1];
            i++;
        }
        num1[i] = '\0';
        i = 0;
        while (dec[i + 1] != '\0')
        {
            num2[i] = dec[i + 1];
            i++;
        }
        num2[i] = '\0';
        pressureParam.Amplitude = atof(num1);
        pressureParam.Period = atof(num2);
    }
    else if (buffer[0] == 'e')
    {
        char num1[10], num2[10], num3[10], num4[10];
        uint8_t i = 0, index = 0;

        while (buffer[i + 1] != ',')
        {
            num1[i] = buffer[i + 1];
            i++;
        }
        num1[i] = '\0';

        index = i + 2;
        i = 0;
        while (buffer[index] != ',')
        {
            num2[i] = buffer[index];
            i++;
            index++;
        }
        num2[i] = '\0';

        index = index + 1;
        i = 0;
        while (buffer[index] != ',')
        {
            num3[i] = buffer[index];
            i++;
            index++;
        }
        num3[i] = '\0';

        index = index + 1;
        i = 0;
        while (buffer[index] != '\0')
        {
            num4[i] = buffer[index];
            i++;
            index++;
        }
        num4[i] = '\0';
        pressureParam.Amplitude = atof(num1);
        pressureParam.Period = atof(num2);
        pressureParam.t1 = atof(num3);
        pressureParam.t2 = atof(num4);
    }
    return pressureParam;
}
float pressureReceive(float pressure, uint32_t timeStart, PressureParameter pressureParam)
{
	float pressureCalc = pressure;
	if(pressureParam.flag == 'a')
	{
		pressureCalc = pressureParam.Amplitude+PRESSURE_NORMAL;
	}
	else if(pressureParam.flag == 'b')
	{
		pressureCalc = pressureParam.Offset + pressureParam.Amplitude*(HAL_GetTick()-timeStart)/1000;
	}
	else if(pressureParam.flag == 'c')
	{
		pressureCalc = pressureParam.Offset + pressureParam.Amplitude*sinf(2*PI/pressureParam.Period*(HAL_GetTick()-timeStart)/1000);
	}
	else if(pressureParam.flag == 'd')
	{
		float t = (HAL_GetTick()-timeStart-((uint16_t)((HAL_GetTick()-timeStart)/(pressureParam.Period*1000)))*pressureParam.Period*1000)/1000;
		if(t<=0.25*pressureParam.Period)
			pressureCalc = pressureParam.Offset + pressureParam.Amplitude/(pressureParam.Period/4)*t;
		else if (t<=0.75*pressureParam.Period)
			pressureCalc = pressureParam.Offset + pressureParam.Amplitude - pressureParam.Amplitude/(pressureParam.Period/4)*(t-0.25*pressureParam.Period);
		else
			pressureCalc = pressureParam.Offset - pressureParam.Amplitude + pressureParam.Amplitude/(pressureParam.Period/4)*(t-0.75*pressureParam.Period);
	}
	else if(pressureParam.flag == 'e')
	{
		float t = (HAL_GetTick()-timeStart-((uint16_t)((HAL_GetTick()-timeStart)/(pressureParam.Period*1000)))*pressureParam.Period*1000)/1000;
		if(t<pressureParam.t1)
		{
			pressureCalc = pressureParam.Offset + pressureParam.Amplitude/(pressureParam.t1)*t;
		}
		else if(t<pressureParam.t1+pressureParam.t2)
		{
			pressureCalc = pressureParam.Offset + pressureParam.Amplitude;
		}
		else if(t<2*pressureParam.t1+pressureParam.t2)
		{
			pressureCalc = pressureParam.Offset + pressureParam.Amplitude - pressureParam.Amplitude/(pressureParam.t1)*(t-pressureParam.t1-pressureParam.t2);
		}
		else 
		{
			pressureCalc = pressureParam.Offset;
		}		
	}
	if (pressureCalc > PRESSURE_MAX) pressureCalc = PRESSURE_MAX;
	else if (pressureCalc < PRESSURE_MIN) pressureCalc = PRESSURE_MIN;
	return pressureCalc;
}
