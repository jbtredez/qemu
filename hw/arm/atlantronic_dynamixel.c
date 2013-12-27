#include <stdlib.h>
#include "hw/sysbus.h"
#include "hw/boards.h"
#include "hw/arm/arm.h"
#include "atlantronic_cpu.h"
#include "kernel/robot_parameters.h"
#include "atlantronic_dynamixel.h"

#define DYNAMIXEL_PERIOD_TICK         7200000

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

struct atlantronic_dynamixel_state
{
	SysBusDevice busdev;
	MemoryRegion iomem;
	qemu_irq irq[DYNAMIXEL_IRQ_OUT_NUM];
//	QEMUTimer* timer; // TODO timer pour mise a jour cinematique dynamixel
	unsigned char rx_buffer[1024];
	unsigned int rx_size;
	unsigned char tx_buffer[1024];
	unsigned int tx_size;
//	uint64_t timer_count;
	float theta;
//	int clock_scale;
	int disconnected; //!< deconnexion du bus (defaillance)
	unsigned char control_table[50]; //!< table de controle, donnes en eeprom et en ram
};

static void atlantronic_dynamixel_send_buffer(struct atlantronic_dynamixel_state* s)
{
	int i = 0;
	for(i = 0; i < s->tx_size; i++)
	{
		qemu_set_irq(s->irq[DYNAMIXEL_IRQ_OUT_USART_TX], s->tx_buffer[i]);
	}
}
#if 0
// mise a jour des mesures
static void atlantronic_dynamixel_update(struct atlantronic_dynamixel_state* s)
{
	(void) s;
}

static void atlantronic_timer_cb(void* arg)
{
	struct atlantronic_dynamixel_state *s = arg;

	s->timer_count += DYNAMIXEL_PERIOD_TICK;
	qemu_mod_timer(s->timer, s->timer_count);
	if( s->clock_scale >= system_clock_scale)
	{
		s->clock_scale = 0;
	}
	s->clock_scale++;
}
#endif

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

static void atlantronic_dynamixel_in_recv_usart(struct atlantronic_dynamixel_state *s, int level)
{
	int msgSize = -1;
	int i = 0;

	if( s->disconnected )
	{
		// dynamixel deconnecte
		return;
	}

	level &= 0xff;

	s->rx_buffer[s->rx_size] = level;
	s->rx_size++;

	if(s->rx_size == 1 || s->rx_size == 2)
	{
		// les 2 premiers octets doivent etre 0xff
		if(level != 0xff)
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

static void atlantronic_dynamixel_in_recv(void * opaque, int numPin, int level)
{
	struct atlantronic_dynamixel_state *s = opaque;

	switch(numPin)
	{
		case DYNAMIXEL_IRQ_IN_ID:
			s->control_table[DYNAMIXEL_ID] = (level & 0xff);
			break;
		case DYNAMIXEL_IRQ_IN_USART_DATA:
			atlantronic_dynamixel_in_recv_usart(s, level);
			break;
		case DYNAMIXEL_IRQ_IN_DISCONNECT:
			// TODO ne plus repondre sur l'usart
			break;
	}

	// on transmet le message sur le reste du bus
	qemu_set_irq(s->irq[DYNAMIXEL_IRQ_OUT_USART_DATA], level);
}

static int atlantronic_dynamixel_init(SysBusDevice * dev, struct atlantronic_dynamixel_state *s, int type)
{
	sysbus_init_mmio(dev, &s->iomem);

	qdev_init_gpio_out(DEVICE(dev), s->irq, DYNAMIXEL_IRQ_OUT_NUM);
	qdev_init_gpio_in(DEVICE(dev), atlantronic_dynamixel_in_recv, DYNAMIXEL_IRQ_IN_NUM);

//	s->timer_count = 0;
//	s->timer = qemu_new_timer(vm_clock, 1, atlantronic_timer_cb, s);
	s->rx_size = 0;
//	s->clock_scale = 1;
	s->disconnected = 0;

	s->control_table[DYNAMIXEL_MODEL_NUMBER_L]            = type;
	s->control_table[DYNAMIXEL_MODEL_NUMBER_H]            = 0;
	s->control_table[DYNAMIXEL_FIRMWARE_VERSION]          = 0;
	s->control_table[DYNAMIXEL_ID]                        = 0x01;
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
	s->control_table[DYNAMIXEL_GOAL_POSITION_L]           = 0;
	s->control_table[DYNAMIXEL_GOAL_POSITION_H]           = 0;
	s->control_table[DYNAMIXEL_MOVING_SPEED_L]            = 0;
	s->control_table[DYNAMIXEL_MOVING_SPEED_H]            = 0;
	s->control_table[DYNAMIXEL_TORQUE_LIMIT_L]            = 0;
	s->control_table[DYNAMIXEL_TORQUE_LIMIT_H]            = 0;
	s->control_table[DYNAMIXEL_PRESENT_POSITION_L]        = 0;
	s->control_table[DYNAMIXEL_PRESENT_POSITION_H]        = 0;
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

static int atlantronic_ax12_init(SysBusDevice * dev)
{
	struct atlantronic_dynamixel_state *s = OBJECT_CHECK(struct atlantronic_dynamixel_state, dev, "atlantronic-ax12");
	return atlantronic_dynamixel_init(dev, s, 12);
}

static int atlantronic_rx24f_init(SysBusDevice * dev)
{
	struct atlantronic_dynamixel_state *s = OBJECT_CHECK(struct atlantronic_dynamixel_state, dev, "atlantronic-rx24f");
	return atlantronic_dynamixel_init(dev, s, 24);
}

static void atlantronic_ax12_class_init(ObjectClass *klass, void *data)
{
	SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);
	sdc->init = atlantronic_ax12_init;
}

static void atlantronic_rx24f_class_init(ObjectClass *klass, void *data)
{
	SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);
	sdc->init = atlantronic_rx24f_init;
}

static TypeInfo atlantronic_ax12_info =
{
	.name          = "atlantronic-ax12",
	.parent        = TYPE_SYS_BUS_DEVICE,
	.instance_size = sizeof(struct atlantronic_dynamixel_state),
	.class_init    = atlantronic_ax12_class_init,
};

static TypeInfo atlantronic_rx24f_info =
{
	.name          = "atlantronic-rx24f",
	.parent        = TYPE_SYS_BUS_DEVICE,
	.instance_size = sizeof(struct atlantronic_dynamixel_state),
	.class_init    = atlantronic_rx24f_class_init,
};

static void atlantronic_dynamixel_register_types(void)
{
	type_register_static(&atlantronic_ax12_info);
	type_register_static(&atlantronic_rx24f_info);
}

type_init(atlantronic_dynamixel_register_types);
