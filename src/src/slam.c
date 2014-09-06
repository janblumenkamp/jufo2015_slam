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

#include "gui.h"
#include "slam.h"
#include "slamdefs.h"

#include "SSD1963.h"
#include "SSD1963_api.h"

slam_t slam; //slam container structure

/*ts_map_t map;
ts_state_t state;
ts_robot_parameters_t robot_parameters;
ts_laser_parameters_t laser_parameters;
ts_position_t robot_position;
ts_sensor_data_t sensor_data;
ts_scan_t laserscan;*/

portTASK_FUNCTION( vSLAMTask, pvParameters ) {
	portTickType xLastWakeTime;

	#if(configDEBUG_MESSAGES == 1)
		printf("xTask SLAM started.\r\n");
	#endif

	xLastWakeTime = xTaskGetTickCount();

	slam_init(&slam, 0, 0, 0, 0, xv11.dist_polar, NULL, NULL);

	/*ts_map_init(&map);
	ts_state_init(&state, &map, &robot_parameters, &laser_parameters, &robot_position, 1.0, 1.0, TS_HOLE_WIDTH, TS_DIRECTION_FORWARD);

	laserscan.nb_points = 360;

	while(xv11_state(XV11_GETSTATE) != XV11_ON);

	printf("Lidar spinning...\n");

	for(u16 i = 0; i < 360; i++)
		sensor_data.d[i] = xv11.dist_polar[i];

	printf("Buid scan...\n");
	ts_build_scan(&sensor_data, &laserscan, &state, 360);

	printf("done!\nSend laser processed data...\n");
	printf("nbpoints: %i\n", laserscan.nb_points);

	for(u8 i = 0; i < laserscan.nb_points; i++)
		printf("%i\n", laserscan.value[i]);

	printf("done!\nUpdate map...\n");

	ts_map_update(&laserscan, &map, &robot_position, TS_MAP_QUALITY, TS_HOLE_WIDTH);

	printf("done!");*/

	for(;;)
	{
		//ts_iterative_map_building(&sensor_data, &state);

		vTaskDelayUntil( &xLastWakeTime, ( 1000 / portTICK_RATE_MS ) );
	}
}


void slam_LCD_DispMap(int16_t x0, int16_t y0, slam_t *slam)
{
	u8 mapval = 0;

	LCD_SetArea(x0,
				y0,
				x0 + (MAP_SIZE_X_MM / MAP_RESOLUTION_MM) - 1,
				y0 + (MAP_SIZE_Y_MM / MAP_RESOLUTION_MM) - 1);

	LCD_WriteCommand(CMD_WR_MEMSTART);

	Clr_Cs;

	for (u16 y = 0; y < (MAP_SIZE_Y_MM / MAP_RESOLUTION_MM); y++)
		for (u16 x = 0; x < (MAP_SIZE_X_MM / MAP_RESOLUTION_MM); x++)
		{
			mapval = IS_OBSTACLE + slam->map.px[x][y][slam->robot_pos.coord.z];
			LCD_WriteData(0xffff - RGB565CONVERT(mapval, mapval, mapval));
		}

	Set_Cs;
}
