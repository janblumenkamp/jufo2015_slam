#include "stm32f4xx.h"
#include "slam.h"
#include "slamdefs.h"
#include "main.h"
#include "queue.h"

/* Spin...3 ESCs or BREAK to abort...:)

Piccolo Laser Distance Scanner

Copyright (c) 2009-2011 Neato Robotics, Inc.

All Rights Reserved



Loader	V2.5.15295

CPU	F2802x/c001

Serial	KSH42612AA-0070252

LastCal	[5371726C]

Runtime	V2.6.15295

*/

#ifndef XV11_H
#define XV11_H


//Source: http://eliaselectronics.com/stm32f4-tutorials/stm32f4-usart-tutorial/

#define XV11_SPEED_PWM_INIT		180 //MOSFET switches 5V to motor. 8bit PWM: (5/255)*170 ~~ 3.3V (optimal voltage, motor turns approx. with 300RPM)
#define XV11_SPEED_IREG_INIT	180.0 //Should be (XV11_SPEED_PWM_INIT).0
#define XV11_SPEED_RPM_TO		300.0 //5Hz; Regulate speed to this value
#define XV11_SPEED_LIM			20.0	//RPM_TO +- Limit: Good results of XV11, otherwise set state to XV11_STARTING
#define XV11_STATE_ON_CNT		250		//Before the state switches to "XV11_ON", the RPM has to be stable (not unter/above XV11_SPEED_LIM) for n iterations / n*4 measurements
#define XV11_SPEED_MIN			50.0 //Lowest possible speed value
#define XV11_SPEED_DIV_I		64.0 //I-Regulator Divisor (the higher the value the inert the regulator)

#define XV11_PACKAGE_LENGTH 22 //In bytes

#define XV11_VAR_NODATA			0 //Value of distance of the measuement is not usable

enum XV11_STATE {
	XV11_OFF,
	XV11_STARTING, XV11_ON,
	XV11_GETSTATE
};

typedef struct {
	u8 state;
	float speed;
	int16_t dist_polar[360];
} XV11_t;

extern volatile XV11_t xv11;

extern QueueHandle_t xQueueLidar;

extern int8_t xv11_state(u8 state);

extern void xv11_init(void);

#endif // XV11_H
