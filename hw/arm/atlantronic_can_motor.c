#include <stdio.h>
#include <string.h>

#include "atlantronic_can_motor.h"
#include "qemu/osdep.h"

#define CAN_MOTOR_CMD_DI   0x08      //!< enable
#define CAN_MOTOR_CMD_EN   0x0f      //!< disable
#define CAN_MOTOR_CMD_M    0x3c      //!< debut du mouvement
#define CAN_MOTOR_CMD_LCC  0x80      //!< limitation courant continu
#define CAN_MOTOR_CMD_LPC  0x81      //!< limitation courant max
#define CAN_MOTOR_CMD_V    0x93      //!< commande de vitesse
#define CAN_MOTOR_CMD_LA   0xb4      //!< commande de position

void atlantronic_can_motor_init(struct atlantronic_can_motor* s, float outputGain)
{
	s->raw_pos = 0;
	s->raw_v = 0;
	s->pos = 0;
	s->v = 0;
	s->speedCmd = 0;
	s->outputGain = outputGain;
}

void atlantronic_can_motor_update(struct atlantronic_can_motor* motor, float dt)
{
	motor->raw_v = (MOTOR_ENCODER_RESOLUTION * motor->speedCmd) / 60;
	motor->raw_pos += motor->raw_v * dt;
	motor->dtSync += dt;
	motor->pos = motor->outputGain * motor->raw_pos;
	motor->v = motor->outputGain * motor->raw_v;
}

void atlantronic_can_motor_callback(struct atlantronic_canopen* canopen, struct canopen_node* node, struct can_msg msg, int type)
{
	struct atlantronic_can_motor* motor = container_of(node, struct atlantronic_can_motor, node);
	struct can_msg rx_msg;
	uint32_t pos_int;

	switch(type)
	{
		case CANOPEN_NMT:
			if( msg.data[0] == 1)
			{
				// on passe en "switch on disable"
				rx_msg.id = 0x80 * CANOPEN_RX_PDO1 + node->nodeid;
				rx_msg.data[0] = 0x60;
				rx_msg.data[1] = 0;
				rx_msg.size = 2;
				atlantronic_canopen_write_bus(canopen, rx_msg);
			}
			break;
		case CANOPEN_SYNC:
			// envoi message
			pos_int = (uint32_t) motor->raw_pos;
			rx_msg.id = 0x80 * CANOPEN_RX_PDO3 + node->nodeid;
			rx_msg.data[0] = pos_int & 0xff;
			rx_msg.data[1] = (pos_int >> 8) & 0xff;
			rx_msg.data[2] = (pos_int >> 16) & 0xff;
			rx_msg.data[3] = (pos_int >> 24) & 0xff;
			rx_msg.data[4] = 0;
			rx_msg.data[5] = 0;
			rx_msg.data[6] = (motor->dtSync*1000);
			rx_msg.size = 7;
			motor->dtSync = 0;
			atlantronic_canopen_write_bus(canopen, rx_msg);
			break;
		case CANOPEN_TX_PDO2:
			switch(msg.data[0])
			{
				case CAN_MOTOR_CMD_EN:
					// enable
					// on passe en "op enable"
					rx_msg.id = 0x80 * CANOPEN_RX_PDO1 + node->nodeid;
					rx_msg.data[0] = 0x27;
					rx_msg.data[1] = 0;
					rx_msg.size = 2;
					atlantronic_canopen_write_bus(canopen, rx_msg);
					break;
				case CAN_MOTOR_CMD_V:
					memcpy(&motor->speedCmd, &msg.data[1], 4);
					break;
			}
			break;
		case CANOPEN_SDO_REQ:
			// on repond ok a tout les sdo
			rx_msg.id = 0x80 * CANOPEN_SDO_RES + node->nodeid;
			rx_msg.data[0] = 0x60;
			rx_msg.data[1] = msg.data[1];
			rx_msg.data[2] = msg.data[2];
			rx_msg.data[3] = msg.data[3];
			rx_msg.size = 4;

			atlantronic_canopen_write_bus(canopen, rx_msg);
			break;
		default:
			break;
	}
}
