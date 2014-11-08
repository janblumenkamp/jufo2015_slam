#include "FreeRTOS.h"
#include "task.h"

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
#include "printf.h"
#include "xv11.h"
#include "main.h"
#include "comm.h"
#include "comm_api.h"
#include "navigation.h"

#include "gui.h"

#include <stdlib.h>

///////SLAM Task
portTASK_FUNCTION( vDRIVETask, pvParameters ) {
	portTickType xLastWakeTime;

	#if(configDEBUG_MESSAGES == 1)
		printf("xTask DRIVE started.\r\n");
	#endif

	xLastWakeTime = xTaskGetTickCount();

	for(;;)
	{
		if(mapping)
		{
			/*int reg = (260 - xv11.dist_polar[140])/10;

			int speed = (20 + reg); //Geschwindigkeit auf MAXSPEED begrenzen
			if(speed > 20)
				speed = 20;
			if(speed < 0)
			  speed  = 0;
			motor.speed_l_to = speed;

			speed = (20 - reg);
			if(speed > 20)
			  speed = 20;
			if(speed < -20)
			  speed  = -20;
			motor.speed_r_to = speed;*/
			navigate(&slam, &motor);

			comm_setMotor(&motor);
		}

		vTaskDelayUntil( &xLastWakeTime, ( 200 / portTICK_RATE_MS ) );
	}
}
