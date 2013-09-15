#ifndef ATLANTRONIC_CAN_MOTOR_MIP_H
#define ATLANTRONIC_CAN_MOTOR_MIP_H

#include <stdint.h>
#include "atlantronic_can.h"

#define MIP_MOTOR_ENCODER_RESOLUTION       1024

struct atlantronic_can_motor_mip
{
	struct atlantronic_can_bus* can;
	int id;
	uint32_t state;        //!< state
	int32_t speedCmd;      //! commande de vitesse en rpm
	float positionOffset;  //!< offset sur la position
	float raw_pos;     //! position actuelle
	float raw_v;       //! vitesse actuelle
	float pos;         //! position actuelle
	float v;           //! vitesse actuelle
	float dtSync;      //!< temps passe depuis le dernier sync
	float outputGain;  //!< gain sur la sortie pour avoir la position et la vitesse en unités utilisables (rd/s ou mm/s)
};

void atlantronic_can_motor_mip_init(struct atlantronic_can_motor_mip* s, int id, float outputGain, float offset, struct atlantronic_can_bus* can);

void atlantronic_can_motor_mip_rx(struct atlantronic_can_motor_mip* s, struct can_msg msg);

void atlantronic_can_motor_mip_update(struct atlantronic_can_motor_mip* s, float dt);

#endif
