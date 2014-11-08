#include "FreeRTOS.h"
#include "task.h"

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

#include <math.h>
#include <stdlib.h>

slam_t slam; //slam container structure
mot_t motor; //Motor information (encoder etc.)

///////SLAM Task


portTASK_FUNCTION( vSLAMTask, pvParameters ) {
	portTickType xLastWakeTime;

	#if(configDEBUG_MESSAGES == 1)
		printf("xTask SLAM started.\r\n");
	#endif

	xLastWakeTime = xTaskGetTickCount();

	nav_initWaypointStack();

	motor.driver_standby = 0;

	comm_readMotorData(&motor); //Important as start value of slam struct .odo_[dir]_old!!
	slam_init(&slam, 1000, 1000, 0, 90, (XV11_t *) &xv11, &motor.enc_l, &motor.enc_r);

	//while(xv11_state(XV11_GETSTATE) != XV11_ON);

	for(;;)
	{
		uint32_t timer_slam = systemTick; //Timer for executing task with max. 5Hz

		if(mapping)
		{
			comm_readMotorData(&motor);
			slam_processMovement(&slam);
			int best = 0;
			best = slam_monteCarloSearch(&slam, 50, 10, 1000);
			slam_map_update(&slam, 3, 300);
			printf("time: %i, quality: %i, pos x: %i, pos y: %i, psi: %i\n", (int)(systemTick - timer_slam), best, (int)slam.robot_pos.coord.x, (int)slam.robot_pos.coord.y, (int)slam.robot_pos.psi);
		}
		else
		{
			slam_map_update(&slam, 100, 300);
			motor.speed_l_to = 0;
			motor.speed_r_to = 0;
		}

		if((systemTick - timer_slam) < 200)
			vTaskDelayUntil( &xLastWakeTime, ( (200 - (systemTick - timer_slam)) / portTICK_RATE_MS ) );
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
