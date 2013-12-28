#ifndef ATLANTRONIC_CAN_MOTOR_H
#define ATLANTRONIC_CAN_MOTOR_H

#include <stdint.h>
#include "atlantronic_canopen.h"

struct atlantronic_can_motor
{
	struct canopen_node node;
	int32_t speedCmd;  //! commande de vitesse en rpm
	uint32_t pos;      //! position actuelle
	//float v;           //! vitesse actuelle
};

void atlantronic_can_motor_callback(struct atlantronic_canopen* canopen, struct canopen_node* node, struct can_msg msg, int type);

#endif
