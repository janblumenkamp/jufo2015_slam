//////////////////////////////////////////////////////////////////////////////////////
/// comm.c - Interface to the Subcontroller
///
///
////////////////////////////////////////////////////////////////////////////////

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
#include "main.h"
#include "comm.h"
#include "comm_api.h"
#include "slam.h"
#include "printf.h"

static volatile uint8_t comm_reg[COMM_REGSIZE];

//////////////////////////////////////////////////////////////////////
/// \brief comm_handler
///		handles the queries from the slave (to call as often as possible)

uint8_t comm_handler_sm = 0;

void comm_handler(void)
{

}

//////////////////////////////////////////////////////////////////////////
/// \brief comm_readMotorData
///		Reads the motor data (speed and encoder values) and saves it into
///		mot
/// \param mot
/// \return
///		1 if successfully accessed slave, otherwise 0

u_int8_t comm_readMotorData(mot_t *mot)
{
	comm_msg_t speedmsg;
	//Read 10 registers (Encoder left, Encoder right, speed left, speed right)
	speedmsg.reg = COMM_MOT_ENC_L_LSB_0;
	speedmsg.batch_write = 0;
	speedmsg.batch = 10;
	u_int8_t speedmsg_receive[10];
	if(comm_bidirectionalPackage(&speedmsg, &speedmsg_receive[0], 3))
	{
		//succeed!
		mot->enc_l = (speedmsg_receive[3] << 24) | (speedmsg_receive[2] << 16) | (speedmsg_receive[1] << 8) | speedmsg_receive[0];
		mot->enc_r = (speedmsg_receive[7] << 24) | (speedmsg_receive[6] << 16) | (speedmsg_receive[5] << 8) | speedmsg_receive[4];
		mot->speed_l_is = (int8_t) speedmsg_receive[8];
		mot->speed_r_is = (int8_t) speedmsg_receive[9];

		return 1;
	}
	else
	{
		return 0;
	}
}

//////////////////////////////////////////////////////////////////////////
/// \brief comm_setMotor
///		sets motor speed to the values saved in the given motor struct.
///		Also (un)activates motordriver (standby).
/// \param mot
/// \return
///		1 if successfully accessed slave, otherwise 0

u_int8_t comm_setMotor(mot_t *mot)
{
	comm_msg_t speedmsg;
	//Write 3 registers (speed left, speed right, driver active)
	u_int8_t speedmsg_send[3];
	speedmsg_send[0] = (u_int8_t)mot->speed_l_to;
	speedmsg_send[1] = (u_int8_t)mot->speed_r_to;
	speedmsg_send[2] = mot->driver_standby;

	speedmsg.reg = COMM_MOT_SPEED_L_TO;
	speedmsg.batch_write = 1;
	speedmsg.batch = 3;
	speedmsg.data = &speedmsg_send[0];

	if(comm_bidirectionalPackage(&speedmsg, NULL, 3))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
