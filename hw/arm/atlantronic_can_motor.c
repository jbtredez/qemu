#include "atlantronic_can_motor.h"
#include "atlantronic_canopen.h"

void atlantronic_can_motor_callback(void* can_interface, void* opaque, struct can_msg msg, int type);

void atlantronic_can_motor_callback(void* can_interface, void* opaque, struct can_msg msg, int type)
{
	struct atlantronic_can_motor* motor = opaque;
	struct can_msg rx_msg;

	switch(type)
	{
		case CANOPEN_SDO_REQ:
			// on repond ok a tout les sdo
			rx_msg.id = 0x80 * CANOPEN_SDO_RES + motor->nodeid;
			rx_msg.data[0] = 0x60;
			rx_msg.data[1] = msg.data[1];
			rx_msg.data[2] = msg.data[2];
			rx_msg.data[3] = msg.data[3];
			rx_msg.size = 4;

			atlantronic_can_rx(can_interface, rx_msg);
			break;
		default:
			break;
	}
}

int atlantronic_can_motor_connect(struct atlantronic_can_motor* motor)
{
	return atlantronic_canopen_register_node(motor->nodeid, (void*)motor, atlantronic_can_motor_callback);
}
