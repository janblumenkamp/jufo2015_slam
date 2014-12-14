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

SemaphoreHandle_t driveSync; //Snychronize DRIVE Task with SLAM Task!

portTASK_FUNCTION( vDRIVETask, pvParameters )
{
//	portTickType xLastWakeTime = xTaskGetTickCount();

	driveSync = xSemaphoreCreateBinary();

	foutf(&debugOS, "xTask DRIVE started.\n");

	for(;;)
	{
		//foutf(&debugOS, "Watermark drive: %i\n", uxTaskGetStackHighWaterMark( NULL ));

		if(xSemaphoreTake(driveSync, portMAX_DELAY) == pdTRUE) //After SLAM Task calculated new position, process the navigation tasks and calculate speed etc.
		{
			navigate(&slam, &motor);
			comm_setMotor(&motor);
		}
	}
}
