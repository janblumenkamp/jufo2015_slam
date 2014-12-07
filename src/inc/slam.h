#ifndef SLAM_H
#define SLAM_H

#include "main.h"
#include "slamdefs.h"
#include "slam.h"
#include "xv11.h"
#include "FreeRTOS.h"
#include "semphr.h"

//Motor information
typedef struct {
	int8_t speed_l_is;
	int8_t speed_r_is;
	int16_t speed_l_ms; //Speed in m/s
	int16_t speed_r_ms; //"
	int8_t speed_l_to;
	int8_t speed_r_to;
	uint8_t driver_standby;
	int32_t enc_l;
	int32_t enc_r;
} mot_t;

extern slam_t slam;

extern mot_t motor;

extern SemaphoreHandle_t lidarSync; //Snychronize SLAM Task with Lidar!

extern void slam_LCD_DispMap(int16_t x0, int16_t y0, float scale, slam_t *slam);

extern void slam_LCD_DispMapProcessed(int16_t x0, int16_t y0, slam_t *slam);

extern void slam_processLaserscan(slam_t *slam, XV11_t *xv11, float speed_ms);

#endif // SLAM_H
