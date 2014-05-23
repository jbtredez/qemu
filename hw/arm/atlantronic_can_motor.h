#ifndef ATLANTRONIC_CAN_MOTOR_H
#define ATLANTRONIC_CAN_MOTOR_H

#include <stdint.h>
#include "atlantronic_canopen.h"

#define MOTOR_ENCODER_RESOLUTION         3000

enum
{
	MOTOR_CMD_SPEED,
	MOTOR_CMD_POSITION,
};

struct atlantronic_can_motor
{
	struct canopen_node node;
	uint32_t statusWord;   //!< status word
	int32_t speedCmd;      //! commande de vitesse en rpm
	int32_t posCmd;        //!< commande de position
	int32_t kinematicsMode; //!< mode (MOTOR_CMD_SPEED ou MOTOR_CMD_POSITION)
	float positionOffset;  //!< offset sur la position
	float raw_pos;     //! position actuelle
	float raw_v;       //! vitesse actuelle
	float pos;         //! position actuelle
	float v;           //! vitesse actuelle
	float dtSync;      //!< temps passe depuis le dernier sync
	float outputGain;  //!< gain sur la sortie pour avoir la position et la vitesse en unités utilisables (rd/s ou mm/s)
};

void atlantronic_can_motor_init(struct atlantronic_can_motor* s, float outputGain, float offset);

void atlantronic_can_motor_callback(struct atlantronic_canopen* canopen, struct canopen_node* node, struct can_msg msg, int type);

void atlantronic_can_motor_update(struct atlantronic_can_motor* s, float dt);

#endif
