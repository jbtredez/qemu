#include <stdlib.h>
#include <math.h>
#include "hw/sysbus.h"
#include "hw/boards.h"
#include "hw/arm/arm.h"
#include "atlantronic_cpu.h"
#include "atlantronic_dynamixel.h"

#define DYNAMIXEL_INSTRUCTION_PING             0x01
#define DYNAMIXEL_INSTRUCTION_READ_DATA        0x02
#define DYNAMIXEL_INSTRUCTION_WRITE_DATA       0x03
#define DYNAMIXEL_INSTRUCTION_REG_WRITE        0x04
#define DYNAMIXEL_INSTRUCTION_ACTION           0x05
#define DYNAMIXEL_INSTRUCTION_RESET            0x06
#define DYNAMIXEL_INSTRUCTION_SYNC_WRITE       0x83

enum
{
	DYNAMIXEL_MODEL_NUMBER_L = 0,
	DYNAMIXEL_MODEL_NUMBER_H,
	DYNAMIXEL_FIRMWARE_VERSION,
	DYNAMIXEL_ID,
	DYNAMIXEL_BAUD_RATE,
	DYNAMIXEL_RETURN_DELAY_TIME,
	DYNAMIXEL_CW_ANGLE_LIMIT_L,
	DYNAMIXEL_CW_ANGLE_LIMIT_H,
	DYNAMIXEL_CCW_ANGLE_LIMIT_L,
	DYNAMIXEL_CCW_ANGLE_LIMIT_H,
	DYNAMIXEL_RESERVED1,
	DYNAMIXEL_HIGHEST_LIMIT_TEMPERATURE,
	DYNAMIXEL_LOWEST_LIMIT_VOLTAGE,
	DYNAMIXEL_HIGHEST_LIMIT_VOLTAGE,
	DYNAMIXEL_MAX_TORQUE_L,
	DYNAMIXEL_MAX_TORQUE_H,
	DYNAMIXEL_STATUS_RETURN_LEVEL,
	DYNAMIXEL_ALARM_LED,
	DYNAMIXEL_ALARM_SHUTDOWN,
	DYNAMIXEL_RESERVED2,
	DYNAMIXEL_DOWN_CALIBRATION_L,
	DYNAMIXEL_DOWN_CALIBRATION_H,
	DYNAMIXEL_UP_CALIBRATION_L,
	DYNAMIXEL_UP_CALIBRATION_H,
	DYNAMIXEL_TORQUE_ENABLE,
	DYNAMIXEL_LED,
	DYNAMIXEL_CW_COMPLIANCE_MARGIN,
	DYNAMIXEL_CCW_COMPLIANCE_MARGIN,
	DYNAMIXEL_CW_COMPLIANCE_SLOPE,
	DYNAMIXEL_CCW_COMPLIANCE_SLOPE,
	DYNAMIXEL_GOAL_POSITION_L,
	DYNAMIXEL_GOAL_POSITION_H,
	DYNAMIXEL_MOVING_SPEED_L,
	DYNAMIXEL_MOVING_SPEED_H,
	DYNAMIXEL_TORQUE_LIMIT_L,
	DYNAMIXEL_TORQUE_LIMIT_H,
	DYNAMIXEL_PRESENT_POSITION_L,
	DYNAMIXEL_PRESENT_POSITION_H,
	DYNAMIXEL_PRESENT_SPEED_L,
	DYNAMIXEL_PRESENT_SPEED_H,
	DYNAMIXEL_PRESENT_LOAD_L,
	DYNAMIXEL_PRESENT_LOAD_H,
	DYNAMIXEL_PRESENT_VOLTAGE,
	DYNAMIXEL_PRESENT_TEMPERATURE,
	DYNAMIXEL_REGISTRED_INSTRUCTION,
	DYNAMIXEL_RESERVED3,
	DYNAMIXEL_MOVING,
	DYNAMIXEL_LOCK,
	DYNAMIXEL_PUNCH_L,
	DYNAMIXEL_PUNCH_H
};

#define DYNAMIXEL_POS_TO_RD          (150 * M_PI / (0x1ff * 180.0f))
#define DYNAMIXEL_RD_TO_POS          (0x1ff * 180 / (150 * M_PI))

#define DYNAMIXEL_MAX_MOVING_SPEED_RD      11.938f       // 114 rpm

static void atlantronic_dynamixel_send_buffer(struct atlantronic_dynamixel_state* s)
{
	int i = 0;
	for(i = 0; i < s->tx_size; i++)
	{
		qemu_set_irq(*s->irq_tx, s->tx_buffer[i]);
	}
}

static uint8_t atlantronic_dynamixel_checksum(uint8_t* buffer, uint8_t size)
{
	uint8_t i = 2;
	uint8_t checksum = 0;

	for(; i< size - 1 ; i++)
	{
		checksum += buffer[i];
	}
	checksum = ~checksum;

	return checksum;
}

void atlantronic_dynamixel_in_recv_usart(struct atlantronic_dynamixel_state *s, unsigned char data)
{
	int msgSize = -1;
	int i = 0;

	if( s->disconnected )
	{
		// dynamixel deconnecte
		return;
	}

	s->rx_buffer[s->rx_size] = data;
	s->rx_size++;

	if(s->rx_size == 1 || s->rx_size == 2)
	{
		// les 2 premiers octets doivent etre 0xff
		if(data != 0xff)
		{
			s->rx_size = 0;
			return;
		}
	}
	else if( s->rx_size == 3)
	{
		// ce n'est pas le bon id, on ne regarde pas le message
		if( s->rx_buffer[2] != s->control_table[DYNAMIXEL_ID] && s->rx_buffer[2] != 0xfe)
		{
			s->rx_size = 0;
			return;
		}
	}

	if( s->rx_size >= 4)
	{
		// taille message
		msgSize = 4 + s->rx_buffer[3];
	}

	if( s->rx_size == msgSize)
	{
		s->tx_size = 0;
		if( s->rx_buffer[2] != 0xfe)
		{
			if( s->rx_buffer[4] != DYNAMIXEL_INSTRUCTION_READ_DATA)
			{
				s->tx_size += 6;
			}
			else
			{
				s->tx_size += 6 + s->rx_buffer[6];
			}

			s->tx_buffer[0] = 0xff;
			s->tx_buffer[1] = 0xff;
			s->tx_buffer[2] = s->control_table[DYNAMIXEL_ID];
			s->tx_buffer[3] = s->tx_size - 4;
			s->tx_buffer[4] = 0; // pas d'erreur
		}

		switch(s->rx_buffer[4])
		{
			case DYNAMIXEL_INSTRUCTION_PING:
				break;
			case DYNAMIXEL_INSTRUCTION_READ_DATA:
				for(i = 0; i < s->rx_buffer[6]; i++)
				{
					int addr = s->rx_buffer[5] + i;
					if(addr > 0 && addr < sizeof(s->control_table))
					{
						s->tx_buffer[5+i] = s->control_table[addr];
					}
				}
				break;
			case DYNAMIXEL_INSTRUCTION_WRITE_DATA:
				for(i = 0; i < s->rx_size - 7; i++)
				{
					int addr = s->rx_buffer[5] + i;
					if(addr > 0 && addr < sizeof(s->control_table))
					{
						s->control_table[addr] = s->rx_buffer[6+i];
					}
				}
				break;
			case DYNAMIXEL_INSTRUCTION_REG_WRITE:
				// TODO
				break;
			case DYNAMIXEL_INSTRUCTION_ACTION:
				// TODO
				break;
			case DYNAMIXEL_INSTRUCTION_RESET:
				// TODO reset control_table
				break;
			case DYNAMIXEL_INSTRUCTION_SYNC_WRITE:
				// TODO
				break;
			default:
				printf("dynamixel : unknown command : %d", s->rx_buffer[4]);
				s->rx_size = 0;
				break;
		}
		if( s->tx_size > 0)
		{
			s->tx_buffer[s->tx_size-1] = atlantronic_dynamixel_checksum(s->tx_buffer, s->tx_size);
			atlantronic_dynamixel_send_buffer(s);
			s->tx_size = 0;
		}

		s->rx_size = 0;
	}

	s->rx_size = s->rx_size % sizeof(s->rx_buffer);
}

int atlantronic_dynamixel_init(struct atlantronic_dynamixel_state *s, qemu_irq* irq_tx, unsigned char id, enum atlantronic_dynamixel_type type)
{
	if( irq_tx == NULL)
	{
		return -1;
	}

	if( id == 0 || id >= 0xfe)
	{
		return -1;
	}

	s->irq_tx = irq_tx;
	s->rx_size = 0;
	s->disconnected = 0;

	s->control_table[DYNAMIXEL_MODEL_NUMBER_L]            = type;
	s->control_table[DYNAMIXEL_MODEL_NUMBER_H]            = 0;
	s->control_table[DYNAMIXEL_FIRMWARE_VERSION]          = 0;
	s->control_table[DYNAMIXEL_ID]                        = id;
	s->control_table[DYNAMIXEL_BAUD_RATE]                 = 0x01;
	s->control_table[DYNAMIXEL_RETURN_DELAY_TIME]         = 0xfa;
	s->control_table[DYNAMIXEL_CW_ANGLE_LIMIT_L]          = 0;
	s->control_table[DYNAMIXEL_CW_ANGLE_LIMIT_H]          = 0;
	s->control_table[DYNAMIXEL_CCW_ANGLE_LIMIT_L]         = 0xff;
	s->control_table[DYNAMIXEL_CCW_ANGLE_LIMIT_H]         = 0x03;
	s->control_table[DYNAMIXEL_RESERVED1]                 = 0;
	if(type == 12)
	{
		s->control_table[DYNAMIXEL_HIGHEST_LIMIT_TEMPERATURE] = 0x55;
	}
	else
	{
		s->control_table[DYNAMIXEL_HIGHEST_LIMIT_TEMPERATURE] = 0x50;
	}
	s->control_table[DYNAMIXEL_LOWEST_LIMIT_VOLTAGE]      = 0x3c;
	s->control_table[DYNAMIXEL_HIGHEST_LIMIT_VOLTAGE]     = 0xbe;
	s->control_table[DYNAMIXEL_MAX_TORQUE_L]              = 0xff;
	s->control_table[DYNAMIXEL_MAX_TORQUE_H]              = 0x03;
	s->control_table[DYNAMIXEL_STATUS_RETURN_LEVEL]       = 0x02;
	if(type == 12)
	{
		s->control_table[DYNAMIXEL_ALARM_LED]             = 0x04;
		s->control_table[DYNAMIXEL_ALARM_SHUTDOWN]        = 0x04;
	}
	else
	{
		s->control_table[DYNAMIXEL_ALARM_LED]             = 0x24;
		s->control_table[DYNAMIXEL_ALARM_SHUTDOWN]        = 0x24;
	}
	s->control_table[DYNAMIXEL_RESERVED2]                 = 0;
	s->control_table[DYNAMIXEL_DOWN_CALIBRATION_L]        = 0;
	s->control_table[DYNAMIXEL_DOWN_CALIBRATION_H]        = 0;
	s->control_table[DYNAMIXEL_UP_CALIBRATION_L]          = 0;
	s->control_table[DYNAMIXEL_UP_CALIBRATION_H]          = 0;
	s->control_table[DYNAMIXEL_TORQUE_ENABLE]             = 0;
	s->control_table[DYNAMIXEL_LED]                       = 0;
	if(type == 12)
	{
		s->control_table[DYNAMIXEL_CW_COMPLIANCE_MARGIN]  = 0;
		s->control_table[DYNAMIXEL_CCW_COMPLIANCE_MARGIN] = 0;
	}
	else
	{
		s->control_table[DYNAMIXEL_CW_COMPLIANCE_MARGIN]  = 0x01;
		s->control_table[DYNAMIXEL_CCW_COMPLIANCE_MARGIN] = 0x01;
	}
	s->control_table[DYNAMIXEL_CW_COMPLIANCE_SLOPE]       = 0x20;
	s->control_table[DYNAMIXEL_CCW_COMPLIANCE_SLOPE]      = 0x20;
	s->control_table[DYNAMIXEL_GOAL_POSITION_L]           = 0xff;
	s->control_table[DYNAMIXEL_GOAL_POSITION_H]           = 1;
	s->control_table[DYNAMIXEL_MOVING_SPEED_L]            = 0;
	s->control_table[DYNAMIXEL_MOVING_SPEED_H]            = 0;
	s->control_table[DYNAMIXEL_TORQUE_LIMIT_L]            = s->control_table[DYNAMIXEL_MAX_TORQUE_L];
	s->control_table[DYNAMIXEL_TORQUE_LIMIT_H]            = s->control_table[DYNAMIXEL_MAX_TORQUE_H];
	s->control_table[DYNAMIXEL_PRESENT_POSITION_L]        = 0xff;
	s->control_table[DYNAMIXEL_PRESENT_POSITION_H]        = 1;
	s->control_table[DYNAMIXEL_PRESENT_SPEED_L]           = 0;
	s->control_table[DYNAMIXEL_PRESENT_SPEED_H]           = 0;
	s->control_table[DYNAMIXEL_PRESENT_LOAD_L]            = 0;
	s->control_table[DYNAMIXEL_PRESENT_LOAD_H]            = 0;
	s->control_table[DYNAMIXEL_PRESENT_VOLTAGE]           = 0;
	s->control_table[DYNAMIXEL_PRESENT_TEMPERATURE]       = 0;
	s->control_table[DYNAMIXEL_REGISTRED_INSTRUCTION]     = 0;
	s->control_table[DYNAMIXEL_RESERVED3]                 = 0;
	s->control_table[DYNAMIXEL_MOVING]                    = 0;
	s->control_table[DYNAMIXEL_LOCK]                      = 0;
	s->control_table[DYNAMIXEL_PUNCH_L]                   = 0x20;
	s->control_table[DYNAMIXEL_PUNCH_H]                   = 0;

    return 0;
}

void atlantronic_dynamixel_update(struct atlantronic_dynamixel_state *s, double dt)
{
	uint16_t goal_pos = (s->control_table[DYNAMIXEL_GOAL_POSITION_H] << 8) + s->control_table[DYNAMIXEL_GOAL_POSITION_L];
	uint16_t speed = (s->control_table[DYNAMIXEL_MOVING_SPEED_H] << 8) + s->control_table[DYNAMIXEL_MOVING_SPEED_L];
	uint16_t torque = (s->control_table[DYNAMIXEL_TORQUE_LIMIT_H] << 8) +  s->control_table[DYNAMIXEL_TORQUE_LIMIT_L];
	// vmax si c'est 0 ou plus grand que vmax
	if(speed == 0 || speed > 0x3ff)
	{
		speed = 0x3ff;
	}
	float vmax = speed * DYNAMIXEL_MAX_MOVING_SPEED_RD / 1023.0f;
	if( torque > 0x3ff)
	{
		torque = 0x3ff;
	}

	float deltaMax = vmax * dt * (torque / 1023.0f);
	float error = DYNAMIXEL_POS_TO_RD * goal_pos - s->theta;

	if( error > deltaMax)
	{
		error = deltaMax;
	}
	else if( error < -deltaMax)
	{
		error = -deltaMax;
	}

	s->theta += error;
	uint16_t pos = DYNAMIXEL_RD_TO_POS * s->theta;

	s->control_table[DYNAMIXEL_PRESENT_POSITION_L] = pos & 0xff;
	s->control_table[DYNAMIXEL_PRESENT_POSITION_H] = (pos >> 8) & 0xff;
}
