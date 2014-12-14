#ifndef DRIVE_H
#define DRIVE_H

#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"

extern SemaphoreHandle_t driveSync; //Snychronize DRIVE Task with SLAM Task!

#endif // DRIVE_H
