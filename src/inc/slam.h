#ifndef SLAM_H
#define SLAM_H

#include "slamdefs.h"
#include "main.h"

//Motor information
typedef struct {
	int8_t speed_l_is;
	int8_t speed_r_is;
	int8_t speed_l_to;
	int8_t speed_r_to;
	uint8_t driver_standby;
	int32_t enc_l;
	int32_t enc_r;
} mot_t;

extern slam_t slam;

extern mot_t motor;

extern void slam_LCD_DispMap(int16_t x0, int16_t y0, slam_t *slam);

#endif // SLAM_H
