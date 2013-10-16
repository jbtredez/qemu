#include "hw/sysbus.h"
#include "hw/boards.h"
#include "hw/arm/arm.h"
#include "atlantronic_cpu.h"
#include "atlantronic_can.h"
#include "atlantronic_canopen.h"

#define EVENT_NUM                  1024

enum
{
	ATLANTRONIC_CAN_IRQ_TX0,
	ATLANTRONIC_CAN_IRQ_RX0,
	ATLANTRONIC_CAN_IRQ_MAX,
};

struct atlantronic_can_state
{
	SysBusDevice busdev;
	MemoryRegion iomem;
	CAN_TypeDef can;
	QemuMutex event_mutex;
	QemuCond event_cond;
	QemuCond event_rx_complete_cond;
	QemuThread rx_thread_id;
	uint32_t event_start;
	uint32_t event_end;
	struct can_msg event[EVENT_NUM];
	qemu_irq irq[ATLANTRONIC_CAN_IRQ_MAX];
};

static void atlantronic_can_write_tx_mailbox(struct atlantronic_can_state* s, hwaddr offset, uint64_t val)
{
	int i = offset / sizeof(CAN_TxMailBox_TypeDef);

	if(i >= 3)
	{
		printf("Error : CAN forbiden sTxMailBox write acces offset %lx (=> i = %d)\n", offset, i);
		return;
	}

	switch(offset % sizeof(CAN_TxMailBox_TypeDef))
	{
		case offsetof(CAN_TxMailBox_TypeDef, TIR):
			if( val & CAN_TI0R_TXRQ)
			{
				// envoi d'un message
				struct can_msg msg;
				if(val & 0x04)
				{
					// CAN_EXTENDED_FORMAT
					msg.id = val >> 3;
				}
				else
				{
					// CAN_STANDARD_FORMAT
					msg.id = val >> 21;
				}
				msg.size = s->can.sTxMailBox[i].TDTR & CAN_TDT0R_DLC;
				msg._data.low = s->can.sTxMailBox[i].TDLR;
				msg._data.high = s->can.sTxMailBox[i].TDHR;

				// transmission ok
				// TODO on suppose i == 0
				s->can.TSR |= CAN_TSR_RQCP0 | CAN_TSR_TXOK0;
				qemu_set_irq(s->irq[ATLANTRONIC_CAN_IRQ_TX0], 1);

				atlantronic_canopen_tx(s, msg);
			}
			s->can.sTxMailBox[i].TIR = val;
			break;
		W_ACCESS(CAN_TxMailBox_TypeDef, s->can.sTxMailBox[i], TDTR, val);
		W_ACCESS(CAN_TxMailBox_TypeDef, s->can.sTxMailBox[i], TDLR, val);
		W_ACCESS(CAN_TxMailBox_TypeDef, s->can.sTxMailBox[i], TDHR, val);
		default:
			printf("Error : CAN forbiden sTxMailBox write acces offset %lx (=> i = %d)\n", offset, i);
			return;
	}
}

static uint32_t atlantronic_can_read_rx_fifo(struct atlantronic_can_state* s, hwaddr offset)
{
	uint32_t val = 0;
	int i = offset / sizeof(CAN_FIFOMailBox_TypeDef);

	if(i >= 3)
	{
		printf("Error : CAN forbiden sFIFOMailBox read acces offset %lx (=> i = %d)\n", offset, i);
		return val;
	}

	switch(offset % sizeof(CAN_FIFOMailBox_TypeDef))
	{
		R_ACCESS(CAN_FIFOMailBox_TypeDef, s->can.sFIFOMailBox[i], RIR, val);
		R_ACCESS(CAN_FIFOMailBox_TypeDef, s->can.sFIFOMailBox[i], RDTR, val);
		R_ACCESS(CAN_FIFOMailBox_TypeDef, s->can.sFIFOMailBox[i], RDLR, val);
		R_ACCESS(CAN_FIFOMailBox_TypeDef, s->can.sFIFOMailBox[i], RDHR, val);
		default:
			printf("Error : CAN forbiden sFIFOMailBox read acces offset %lx (=> i = %d)\n", offset, i);
	}

	return val;
}

static uint32_t atlantronic_can_read_tx_mailbox(struct atlantronic_can_state* s, hwaddr offset)
{
	uint32_t val = 0;
	int i = offset / sizeof(CAN_TxMailBox_TypeDef);

	if(i >= 3)
	{
		printf("Error : CAN forbiden sTxMailBox read acces offset %lx (=> i = %d)\n", offset, i);
		return val;
	}

	switch(offset % sizeof(CAN_TxMailBox_TypeDef))
	{
		R_ACCESS(CAN_TxMailBox_TypeDef, s->can.sTxMailBox[i], TIR, val);
		R_ACCESS(CAN_TxMailBox_TypeDef, s->can.sTxMailBox[i], TDTR, val);
		R_ACCESS(CAN_TxMailBox_TypeDef, s->can.sTxMailBox[i], TDLR, val);
		R_ACCESS(CAN_TxMailBox_TypeDef, s->can.sTxMailBox[i], TDHR, val);
		default:
			printf("Error : CAN forbiden sTxMailBox read acces offset %lx (=> i = %d)\n", offset, i);
	}

	return val;
}

static void atlantronic_can_write(void *opaque, hwaddr offset, uint64_t val, unsigned size)
{
	struct atlantronic_can_state* s = opaque;

	switch(offset)
	{
		W_ACCESS(CAN_TypeDef, s->can, MCR, val);
		W_ACCESS(CAN_TypeDef, s->can, MSR, val);
		case offsetof(CAN_TypeDef, TSR):
			if(val & CAN_TSR_RQCP0)
			{
				s->can.TSR &= ~CAN_TSR_RQCP0;
				qemu_set_irq(s->irq[ATLANTRONIC_CAN_IRQ_TX0], 0);
			}
			break;
		case offsetof(CAN_TypeDef, RF0R):
			if(val & CAN_RF0R_RFOM0)
			{
				s->can.TSR &= ~CAN_RF0R_RFOM0;
				qemu_set_irq(s->irq[ATLANTRONIC_CAN_IRQ_RX0], 0);
				qemu_mutex_lock(&s->event_mutex);
				qemu_cond_signal(&s->event_rx_complete_cond);
				qemu_mutex_unlock(&s->event_mutex);
			}
			break;
		W_ACCESS(CAN_TypeDef, s->can, RF1R, val);
		W_ACCESS(CAN_TypeDef, s->can, IER, val);
		W_ACCESS(CAN_TypeDef, s->can, ESR, val);
		W_ACCESS(CAN_TypeDef, s->can, BTR, val);
		W_ACCESS(CAN_TypeDef, s->can, FMR, val);
		W_ACCESS(CAN_TypeDef, s->can, FM1R, val);
		W_ACCESS(CAN_TypeDef, s->can, FS1R, val);
		W_ACCESS(CAN_TypeDef, s->can, FFA1R, val);
		W_ACCESS(CAN_TypeDef, s->can, FA1R, val);
		default:
			if(offset >= offsetof(CAN_TypeDef, sTxMailBox[0]) && offset < offsetof(CAN_TypeDef, sTxMailBox[0]) + sizeof(s->can.sTxMailBox))
			{
				// tx mailbox
				atlantronic_can_write_tx_mailbox(s, offset - offsetof(CAN_TypeDef, sTxMailBox[0]), val);
			}
			else if(offset >= offsetof(CAN_TypeDef, sFIFOMailBox[0]) && offset < offsetof(CAN_TypeDef, sFIFOMailBox[0]) + sizeof(s->can.sFIFOMailBox))
			{
				// TODO rx fifo mailbox
			}
			else if(offset >= offsetof(CAN_TypeDef, sFilterRegister[0]) && offset < offsetof(CAN_TypeDef, sFilterRegister[0]) + sizeof(s->can.sFilterRegister))
			{
				// TODO filtres
			}
			else
			{
				printf("Error : CAN forbiden write acces offset %lx, val %lx\n", offset, val);
			}
			break;
	}
}

static uint64_t atlantronic_can_read(void *opaque, hwaddr offset, unsigned size)
{
	uint64_t res = 0;
	struct atlantronic_can_state* s = opaque;

	switch(offset)
	{
		R_ACCESS(CAN_TypeDef, s->can, MCR, res);
		R_ACCESS(CAN_TypeDef, s->can, MSR, res);
		R_ACCESS(CAN_TypeDef, s->can, TSR, res);
		R_ACCESS(CAN_TypeDef, s->can, RF0R, res);
		R_ACCESS(CAN_TypeDef, s->can, RF1R, res);
		R_ACCESS(CAN_TypeDef, s->can, IER, res);
		R_ACCESS(CAN_TypeDef, s->can, ESR, res);
		R_ACCESS(CAN_TypeDef, s->can, BTR, res);
		R_ACCESS(CAN_TypeDef, s->can, FMR, res);
		R_ACCESS(CAN_TypeDef, s->can, FM1R, res);
		R_ACCESS(CAN_TypeDef, s->can, FS1R, res);
		R_ACCESS(CAN_TypeDef, s->can, FFA1R, res);
		R_ACCESS(CAN_TypeDef, s->can, FA1R, res);
		default:
			if(offset >= offsetof(CAN_TypeDef, sTxMailBox[0]) && offset < offsetof(CAN_TypeDef, sTxMailBox[0]) + sizeof(s->can.sTxMailBox))
			{
				// TODO tx mailbox
				res = atlantronic_can_read_tx_mailbox(s, offset - offsetof(CAN_TypeDef, sTxMailBox[0]));
			}
			else if(offset >= offsetof(CAN_TypeDef, sFIFOMailBox[0]) && offset < offsetof(CAN_TypeDef, sFIFOMailBox[0]) + sizeof(s->can.sFIFOMailBox))
			{
				// rx fifo mailbox
				res = atlantronic_can_read_rx_fifo(s, offset - offsetof(CAN_TypeDef, sFIFOMailBox[0]));
			}
			else if(offset >= offsetof(CAN_TypeDef, sFilterRegister[0]) && offset < offsetof(CAN_TypeDef, sFilterRegister[0]) + sizeof(s->can.sFilterRegister))
			{
				// TODO filtres
			}
			else
			{
				printf("Error : CAN forbiden read acces offset %lx\n", offset);
			}
			break;
	}

	return res;	
}

void atlantronic_can_rx(void *opaque, struct can_msg msg)
{
	struct atlantronic_can_state* s = opaque;
	qemu_mutex_lock(&s->event_mutex);
	if((s->event_end + 1) % EVENT_NUM != s->event_start)
	{
		// il reste de la place
		s->event[s->event_end] = msg;
		s->event_end = (s->event_end + 1) % EVENT_NUM;
	}
	qemu_cond_signal(&s->event_cond);
	qemu_mutex_unlock(&s->event_mutex);
}

static void* atlantronic_can_rx_thread(void* arg)
{
	struct atlantronic_can_state* s = arg;
	struct can_msg msg;

	while(1)
	{
		qemu_mutex_lock(&s->event_mutex);

		if(s->event_start == s->event_end)
		{
			qemu_cond_wait(&s->event_cond, &s->event_mutex);
		}
		msg = s->event[s->event_start];
		s->event_start = (s->event_start + 1) % EVENT_NUM;
		qemu_mutex_unlock(&s->event_mutex);

		s->can.sFIFOMailBox[0].RIR = (unsigned int)(msg.id << 21);

		s->can.sFIFOMailBox[0].RDLR = msg._data.low;
		s->can.sFIFOMailBox[0].RDHR = msg._data.high;

		s->can.sFIFOMailBox[0].RDTR = msg.size & 0x0f;
		s->can.RF0R |= CAN_RF0R_FMP0;
		qemu_set_irq(s->irq[ATLANTRONIC_CAN_IRQ_RX0], 1);

		qemu_mutex_lock(&s->event_mutex);
		qemu_cond_wait(&s->event_rx_complete_cond, &s->event_mutex);
		qemu_mutex_unlock(&s->event_mutex);
	}

	return 0;
}

static void atlantronic_can_reset(CAN_TypeDef* can)
{
	int i = 0;
	can->MCR  = 0x00010002;
	can->MSR  = 0x00000C02;
	can->TSR  = 0x1C000000;
	can->RF0R = 0x00;
	can->RF1R = 0x00;
	can->IER  = 0x00;
	can->ESR  = 0x00;
	can->BTR  = 0x01230000;

	for(i = 0; i < sizeof(can->sTxMailBox) / sizeof(can->sTxMailBox[0]) ; i++)
	{
		can->sTxMailBox[i].TIR  = 0x00;
		can->sTxMailBox[i].TDTR = 0x00;
		can->sTxMailBox[i].TDLR = 0x00;
		can->sTxMailBox[i].TDHR = 0x00;
	}

	for(i = 0; i < sizeof(can->sFIFOMailBox) / sizeof(can->sFIFOMailBox[0]) ; i++)
	{
		can->sFIFOMailBox[i].RIR  = 0x00;
		can->sFIFOMailBox[i].RDTR = 0x00;
		can->sFIFOMailBox[i].RDLR = 0x00;
		can->sFIFOMailBox[i].RDHR = 0x00;
	}

	can->FMR   = 0x2A1C0E01;
	can->FM1R  = 0x00;
	can->FS1R  = 0x00;
	can->FFA1R = 0x00;
	can->FA1R  = 0x00;

	for(i = 0; i < sizeof(can->sFilterRegister) / sizeof(can->sFilterRegister[0]) ; i++)
	{
		can->sFilterRegister[i].FR1  = 0x00;
		can->sFilterRegister[i].FR2 = 0x00;
	}
}

static const MemoryRegionOps atlantronic_can_ops =
{
	.read = atlantronic_can_read,
	.write = atlantronic_can_write,
	.endianness = DEVICE_NATIVE_ENDIAN,
};

static int atlantronic_can_init(SysBusDevice * dev)
{
	struct atlantronic_can_state *s = OBJECT_CHECK(struct atlantronic_can_state, dev, "atlantronic-can");

	memory_region_init_io(&s->iomem, OBJECT(s), &atlantronic_can_ops, s, "atlantronic_can", 0x400);
	sysbus_init_mmio(dev, &s->iomem);
	sysbus_init_irq(dev, &s->irq[ATLANTRONIC_CAN_IRQ_TX0]);
	sysbus_init_irq(dev, &s->irq[ATLANTRONIC_CAN_IRQ_RX0]);

	s->event_start = 0;
	s->event_end = 0;
	qemu_mutex_init(&s->event_mutex);
	qemu_cond_init(&s->event_cond);
	qemu_cond_init(&s->event_rx_complete_cond);
	atlantronic_can_reset(&s->can);
	qemu_thread_create(&s->rx_thread_id, atlantronic_can_rx_thread, s, QEMU_THREAD_JOINABLE);

    return 0;
}

static void atlantronic_can_class_init(ObjectClass *klass, void *data)
{
	SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);

	sdc->init = atlantronic_can_init;
}

static TypeInfo atlantronic_can_info =
{
	.name          = "atlantronic-can",
	.parent        = TYPE_SYS_BUS_DEVICE,
	.instance_size = sizeof(struct atlantronic_can_state),
	.class_init    = atlantronic_can_class_init,
};

static void atlantronic_can_register_types(void)
{
	type_register_static(&atlantronic_can_info);
}

type_init(atlantronic_can_register_types);
