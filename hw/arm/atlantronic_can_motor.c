#include "atlantronic_can_motor.h"
#include "atlantronic_canopen.h"

void atlantronic_can_motor_callback(void* motor, struct can_msg msg, int type);

void atlantronic_can_motor_callback(void* motor, struct can_msg msg, int type)
{
	(void) motor;
	(void) msg;
	(void) type;
}

int atlantronic_can_motor_connect(struct atlantronic_can_motor* motor)
{
	return atlantronic_canopen_register_node(motor->nodeid, (void*)motor, atlantronic_can_motor_callback);
}
