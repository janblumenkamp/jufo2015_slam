#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
#include "utils.h"
#include "debug.h"
#include "printf.h"
#include "xv11.h"
#include "main.h"
#include "comm.h"
#include "comm_api.h"
#include "navigation_api.h"
#include "navigation.h"

#include "gui.h"
#include "slam.h"
#include "slamdefs.h"

#include "SSD1963.h"
#include "SSD1963_api.h"

#include "stdio.h"
#include <math.h>
#include <stdlib.h>

slam_t slam; //slam container structure
mot_t motor; //Motor information (encoder etc.)

//slam_coordinates_t lidar_lastPosition; //Stores the position of the robot at the beginning of the next lidar scan to calculate the dist the robot has driven.

///////SLAM Task
SemaphoreHandle_t lidarSync; //Snychronize SLAM Task with Lidar!


portTASK_FUNCTION( vSLAMTask, pvParameters ) {
//	portTickType xLastWakeTime;

	foutf(&debugOS, "xTask SLAM started.\r\n");

	//xLastWakeTime = xTaskGetTickCount();
	lidarSync = xSemaphoreCreateBinary();

	nav_initWaypointStack();

	motor.driver_standby = 0;

	comm_readMotorData(&motor); //Important as start value of slam struct .odo_[dir]_old!!
	slam_init(&slam, 1000, 1000, 0, 90, &motor.enc_l, &motor.enc_r);

	//while(xv11_state(XV11_GETSTATE) != XV11_ON);
	uint32_t timer_slam = 0;

	for(;;)
	{
		if(xSemaphoreTake(lidarSync, 0xffff) == pdTRUE)
		{
			foutf(&debug, "Semaphore taken after %ims\n\r", (int)(systemTick - timer_slam));

			timer_slam = systemTick; //Timer for executing task with max. 5Hz

			slam_processLaserscan(&slam, (XV11_t *) &xv11, (motor.speed_l_ms + motor.speed_r_ms) / 2);

			//lidar_lastPosition = slam.robot_pos.coord;

			if(mapping)
			{
				comm_readMotorData(&motor);
				int16_t slam_updateVar = abs(motor.speed_l_is - motor.speed_r_is); //Difference of speed. The smaller, the straighter drives the robot.

				slam_processMovement(&slam);

				int best = 0;
				best = slam_monteCarloSearch(&slam, 100, 10, 1300);

				/*if(slam_updateVar <= 10)
					slam_updateVar = 6 - slam_updateVar/2;
				else
					slam_updateVar = 1;*/
				if(slam_updateVar < 10)
					slam_updateVar = 10 - slam_updateVar;
				else
					slam_updateVar = 1;

				slam_map_update(&slam, slam_updateVar, 200);

				//foutf(debug, "time: %i, quality: %i, pos x: %i, pos y: %i, psi: %i\n\r", (int)(systemTick - timer_slam), best, (int)slam.robot_pos.coord.x, (int)slam.robot_pos.coord.y, (int)slam.robot_pos.psi);
			}
			else
			{
				slam_map_update(&slam, 100, 160);
				motor.speed_l_to = 0;
				motor.speed_r_to = 0;
			}
		}
		//slam_line(&slam, 10, 10, 100, 180, 10, 255);

		//if((systemTick - timer_slam) < 200)
		//	vTaskDelayUntil( &xLastWakeTime, ( (200 - (systemTick - timer_slam)) / portTICK_RATE_MS ) );
		//vTaskDelayUntil( &xLastWakeTime, ( 5000 / portTICK_RATE_MS ) );
	}
}


//////////////////////////////////////////////////////////////////////////
/// \brief slam_processLaserscan
///			Compensates the movement of the roboter in the laser scan
///			(If the robot moves with 0.3m/s and the lidar turns with 5Hz, the
///			robot already moves 0.3m/s / 5Hz = 0.06m = 6cm in one scan!)
/// \param slam
///			Slam container structure
/// \param speed_ms
///			Robot speed in m/s
///
void slam_processLaserscan(slam_t *slam, XV11_t *xv11, float speed_ms)
{
	float distCorrection_mm = 0;

	for(int16_t i = 0; i < LASERSCAN_POINTS; i++)
	{
		//distCorrection_mm += cosf((i + 180) * M_PI/180) * ((speed_ms / (xv11->speed / 60) / LASERSCAN_POINTS) * 1000);

		int16_t i_sensor = (i + 270);
		if(i_sensor >= LASERSCAN_POINTS)
			i_sensor -= LASERSCAN_POINTS;

		if(xv11->dist_polar[i_sensor] > 0)
			slam->sensordata.lidar[i] = xv11->dist_polar[i_sensor] + (int)distCorrection_mm; //The dist correction value in the driving direction of the robot for one ray. (slam->sensordata.xv11->speed / 60): Speed in RPM, s now we get the turning frequency. Conversion only works for 360° Lidars!
		else
			slam->sensordata.lidar[i] = LASERSCAN_NODATA;
	}
}

/////////////////////////////////////////////////////////////////
/// \brief slam_LCD_DispMap
///		Displays the slam map
/// \param x0
///		X start coordinate
/// \param y0
///		Y start coordinate
/// \param slam
///		slam container structure

void slam_LCD_DispMap(int16_t x0, int16_t y0, slam_t *slam)
{
	u8 mapval = 0;

	LCD_SetArea(x0,
				y0,
				x0 + (MAP_SIZE_X_MM / MAP_RESOLUTION_MM) - 1,
				y0 + (MAP_SIZE_Y_MM / MAP_RESOLUTION_MM) - 1);

	LCD_WriteCommand(CMD_WR_MEMSTART);

	Clr_Cs;

	for (int16_t y = (MAP_SIZE_Y_MM / MAP_RESOLUTION_MM) - 1; y >= 0; y--)
	{
		for (int16_t x = 0; x < (MAP_SIZE_X_MM / MAP_RESOLUTION_MM); x++)
		{
			mapval = slam->map.px[x][y][slam->robot_pos.coord.z];
			LCD_WriteData(0xffff - RGB565CONVERT(mapval, mapval, mapval));
		}
	}

	Set_Cs;
}
/////////////////////////////////////////////////////////////////
/// \brief slam_LCD_DispMapProcessed
///		Displays the slam map (processed)
/// \param x0
///		X start coordinate
/// \param y0
///		Y start coordinate
/// \param slam
///		slam container structure

void slam_LCD_DispMapProcessed(int16_t x0, int16_t y0, slam_t *slam)
{
	LCD_SetArea(x0,
				y0,
				x0 + (MAP_SIZE_X_MM / MAP_RESOLUTION_MM) - 1,
				y0 + (MAP_SIZE_Y_MM / MAP_RESOLUTION_MM) - 1);

	LCD_WriteCommand(CMD_WR_MEMSTART);

	Clr_Cs;

	for (int16_t y = (MAP_SIZE_Y_MM / MAP_RESOLUTION_MM) - 1; y >= 0; y--)
	{
		for (int16_t x = 0; x < (MAP_SIZE_X_MM / MAP_RESOLUTION_MM); x++)
		{
			if(slam->map.px[x][y][slam->robot_pos.coord.z] > 120 && slam->map.px[x][y][slam->robot_pos.coord.z] != 127)
			{
				LCD_WriteData(0);
			}
			else
			{
				LCD_WriteData(0xffff);
			}
		}
	}

	Set_Cs;
}
