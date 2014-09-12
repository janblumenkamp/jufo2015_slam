/////////////////////////////////////////////////////////////////////////////////
/// Communication with slave
/////////////////////////////////////////////////////////////////////////////////

#ifndef COMM_H
#define COMM_H

#include "main.h"
#include "slam.h"

#define COMM_REGSIZE 53 //In bytes. Size of the register.

//Handles all queries. To call as often as possible!
extern void comm_handler(void);

//Reads out motor data (speed and encoders)
extern u_int8_t comm_readMotorData(mot_t *mot);

//sets motor speed
extern u_int8_t comm_setMotor(mot_t *mot);

#endif // COMM_H
