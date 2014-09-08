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

static volatile uint8_t comm_reg[COMM_REGSIZE];

//////////////////////////////////////////////////////////////////////
/// \brief comm_handler
///		handles the queries to the slave (to call as often as possible)

uint8_t comm_handler_sm = 0;

void comm_handler(void)
{

}
