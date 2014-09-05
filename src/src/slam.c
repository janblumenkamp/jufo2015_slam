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
#include "CoreSLAM.h"

#include "SSD1963.h"
#include "SSD1963_api.h"

ts_map_t map;
ts_state_t state;
ts_robot_parameters_t robot_parameters;
ts_laser_parameters_t laser_parameters;
ts_position_t robot_position;
ts_sensor_data_t sensor_data;
ts_scan_t laserscan;

portTASK_FUNCTION( vSLAMTask, pvParameters ) {
	portTickType xLastWakeTime;

	#if(configDEBUG_MESSAGES == 1)
		printf("xTask SLAM started.\r\n");
	#endif

	xLastWakeTime = xTaskGetTickCount();

	laser_parameters.angle_max = 359;
	laser_parameters.angle_min = 0;
	laser_parameters.detection_margin = 0;
	laser_parameters.distance_no_detection = 0;
	laser_parameters.offset = 0;
	laser_parameters.scan_size = 359;

	robot_position.theta = 0;
	robot_position.x = 50;
	robot_position.y = 50;

	robot_parameters.inc = 360;
	robot_parameters.R = 50;
	robot_parameters.r = 50;
	robot_parameters.ratio = 20;

	ts_map_init(&map);
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

	printf("done!");

	for(;;)
	{
		//ts_iterative_map_building(&sensor_data, &state);

		vTaskDelayUntil( &xLastWakeTime, ( 1000 / portTICK_RATE_MS ) );
	}
}


void LCD_DispMap(int16_t x0, int16_t y0, int16_t width, int16_t height, ts_map_t *map)
{
	u16 x1;
	u16 y1;

	u16 mapval = 0;

	x1 = width-1 + x0;

	y1 = height-1 + y0;

	LCD_SetArea(x0,y0,x1,y1);
	LCD_WriteCommand(CMD_WR_MEMSTART);

	Clr_Cs;

	for (u16 i = 0; i < (width*height); i++)
	{
		mapval = map->map[i] >> 8;
		LCD_WriteData(0xffff - RGB565CONVERT(mapval, mapval, mapval));
	}

	Set_Cs;

//==============================
}
