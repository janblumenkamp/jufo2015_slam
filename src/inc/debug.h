/*
 * debug.h
 *
 *  Created on: Feb 9, 2011
 *      Author: James
 */

#ifndef DEBUG_H_
#define DEBUG_H_

#include "main.h"
#include "slamdefs.h"
#include "queue.h"

extern QueueHandle_t xQueueTXUSART2;

extern void pcui_sendMsg(char *id, u_int32_t length, char *msg);

extern void pcui_sendMap(slam_t *slam);

extern void vDebugPrintResetType(void);

void vUSART2_Init(void);


#endif /* DEBUG_H_ */
