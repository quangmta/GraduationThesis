/*
 * pressureReceive.h
 *
 *  Created on: Mar 26, 2022
 *      Author: nguye
 */

#ifndef INC_PRESSURERECEIVE_H_
#define INC_PRESSURERECEIVE_H_
#include "main.h"

typedef struct
{
	uint8_t flag;
	float Offset;
	float Amplitude;
	float Period;
	float t1;
	float t2;
} PressureParameter;
float char2float(char*buffer);
PressureParameter pressureCalcParam(char* buffer,float offset);
float pressureReceive(float pressure, uint32_t timeStart, PressureParameter pressureParam);

#endif /* INC_PRESSURERECEIVE_H_ */
