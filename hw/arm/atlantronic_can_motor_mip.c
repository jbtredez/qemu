#include <stdio.h>
#include <string.h>

#include "atlantronic_can_motor_mip.h"
#include "qemu/osdep.h"

#define CAN_MIP_CMD_SYNC                   0x20
#define CAN_MIP_CMD_SPEED                  0x40
#define CAN_MIP_CMD_RAZ                    0x60
#define CAN_MIP_CMD_DISABLE                0x70
#define CAN_MIP_CMD_ENABLE                 0x80

#define CAN_MIP_MOTOR_STATE_TRAJ_PTS_FULL         0x01
#define CAN_MIP_MOTOR_STATE_IN_MOTION             0x02
#define CAN_MIP_MOTOR_STATE_POWERED               0x04
#define CAN_MIP_MOTOR_STATE_POSITION_UNKNOWN      0x08
#define CAN_MIP_MOTOR_STATE_ERR_RANGE             0x10
#define CAN_MIP_MOTOR_STATE_ERR_TRAJ              0x20
#define CAN_MIP_MOTOR_STATE_ERR_TEMPERATURE       0x40
#define CAN_MIP_MOTOR_STATE_ERR_CMD_VS_MES        0x80
#define CAN_MIP_MOTOR_STATE_ERROR                 0xf0

void atlantronic_can_motor_mip_init(struct atlantronic_can_motor_mip* motor, int id, float outputGain, float offset, struct atlantronic_can_bus* can)
{
	motor->id = id;
	motor->state = CAN_MIP_MOTOR_STATE_POSITION_UNKNOWN;
	motor->raw_pos = 0;
	motor->raw_v = 0;
	motor->pos = 0;
	motor->v = 0;
	motor->speedCmd = 0;
	motor->outputGain = outputGain;
	motor->positionOffset = offset;
	motor->can = can;
}


void atlantronic_can_motor_mip_update(struct atlantronic_can_motor_mip* motor, float dt)
{
	motor->raw_v = (MIP_MOTOR_ENCODER_RESOLUTION * motor->speedCmd) / 60;
	motor->raw_pos += motor->raw_v * dt;
	motor->dtSync += dt;
	motor->pos = motor->positionOffset + motor->outputGain * motor->raw_pos;
	motor->v = motor->outputGain * motor->raw_v;
	if( motor->raw_v )
	{
		motor->state |= CAN_MIP_MOTOR_STATE_IN_MOTION;
	}
	else
	{
		motor->state &= ~CAN_MIP_MOTOR_STATE_IN_MOTION;
	}
}

static void atlantronic_can_motor_mip_send_status(struct atlantronic_can_motor_mip* motor)
{
	struct can_msg tx_msg;
	int32_t pos = (int32_t) motor->raw_pos;
	tx_msg.id = 0x79;
	tx_msg.size = 8;

	tx_msg.data[0] = motor->state;
	tx_msg.data[1] = motor->id;
	tx_msg.data[2] = pos & 0xff;
	tx_msg.data[3] = (pos >> 8) & 0xff;
	tx_msg.data[4] = (pos >> 16) & 0xff;
	tx_msg.data[5] = (pos >> 24) & 0xff;
	tx_msg.data[6] = 0;
	tx_msg.data[7] = 0;

	atlantronic_can_write_bus(motor->can, tx_msg);
}

void atlantronic_can_motor_mip_rx(struct atlantronic_can_motor_mip* motor, struct can_msg msg)
{
	int adresse = msg.id >> 3;
	if( adresse != 0 && motor->id != adresse)
	{
		return;
	}

	int cmd = msg.data[0];

	switch(cmd)
	{
		case CAN_MIP_CMD_SYNC:
			atlantronic_can_motor_mip_send_status(motor);
			break;
		case CAN_MIP_CMD_SPEED:
			if( msg.data[3] )
			{
				motor->speedCmd = (int32_t)(msg.data[1] + (msg.data[2] << 8));
			}
			else
			{
				motor->speedCmd = -(int32_t)(msg.data[1] + (msg.data[2] << 8));
			}
			break;
		case CAN_MIP_CMD_RAZ:
			motor->state = CAN_MIP_MOTOR_STATE_POWERED;
			break;
		case CAN_MIP_CMD_DISABLE:
			motor->state &= ~CAN_MIP_MOTOR_STATE_POWERED;
			break;
		case CAN_MIP_CMD_ENABLE:
			motor->state |= CAN_MIP_MOTOR_STATE_POWERED;
			break;
	}
}
