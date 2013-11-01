#ifndef ATLANTRONIC_CAN_MOTOR_H
#define ATLANTRONIC_CAN_MOTOR_H

#include <stdint.h>

struct atlantronic_can_motor
{
	uint8_t nodeid; //! id
	float posCmd;    //! commande de position
	float speedCmd;  //! commande de vitesse
	uint32_t pos;    //! position actuelle
	float v;         //! vitesse actuelle
};

int atlantronic_can_motor_connect(struct atlantronic_can_motor* motor);

#endif
