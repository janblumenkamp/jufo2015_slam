#include "FreeRTOS.h"
#include "task.h"

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
#include "xv11.h"
#include "main.h"
#include "comm.h"
#include "comm_api.h"
#include "navigation.h"

#include "gui.h"
#include "outf.h"

#include <stdlib.h>
#include <stdio.h>

///////SLAM Task
portTASK_FUNCTION( vDRIVETask, pvParameters ) {
	portTickType xLastWakeTime;

	foutf(&debugOS, "xTask DRIVE started.\n");

	xLastWakeTime = xTaskGetTickCount();

	for(;;)
	{
		//foutf(&debugOS, "Watermark drive: %i\n", uxTaskGetStackHighWaterMark( NULL ));

		if(mapping)
		{
			navigate(&slam, &motor);

			comm_setMotor(&motor);
		}

		vTaskDelayUntil( &xLastWakeTime, ( 200 / portTICK_RATE_MS ) );
	}
}
