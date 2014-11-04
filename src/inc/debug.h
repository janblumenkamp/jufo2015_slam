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

extern void pcui_sendMsg(char *id, u_int32_t length, char *msg);

extern void pcui_sendMap(slam_t *slam);

extern void vDebugPrintResetType( void );


#endif /* DEBUG_H_ */
