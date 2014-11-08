/*
 * main.h
 *
 *  Created on: 10 jul 2012
 *      Author: BenjaminVe
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "FreeRTOS.h"
#include "task.h"

typedef struct { //Battery information
	int16_t mV;
	int8_t percent;
} battstate_t;

extern xTaskHandle hTimeTask;
extern xTaskHandle hDRIVETask;
extern xTaskHandle hSLAMTask;
extern xTaskHandle hGuiTask;
extern xTaskHandle hDebugTask;

typedef uint8_t u_int8_t;
typedef uint16_t u_int16_t;
typedef uint32_t u_int32_t;

extern u_int32_t systemTick;

extern battstate_t battery;

#define TRUE 1
#define FALSE 0

// Function prototypes

#endif /* MAIN_H_ */
