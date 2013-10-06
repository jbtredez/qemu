#include "hw/sysbus.h"
#include "hw/boards.h"
#include "hw/arm/arm.h"
#include "atlantronic_cpu.h"

enum
{
	// TODO irq can
	ATLANTRONIC_CAN_IRQ_MAX,
};

struct atlantronic_can_state
{
	SysBusDevice busdev;
	MemoryRegion iomem;
	CAN_TypeDef can;
	qemu_irq irq[ATLANTRONIC_CAN_IRQ_MAX];
};

static void atlantronic_can_write(void *opaque, hwaddr offset, uint64_t val, unsigned size)
{
	struct atlantronic_can_state* s = opaque;

	switch(offset)
	{
		case offsetof(CAN_TypeDef, MCR):
			s->can.MCR = val;
			break;
		case offsetof(CAN_TypeDef, MSR):
			s->can.MCR = val;
			break;
		case offsetof(CAN_TypeDef, TSR):
			s->can.TSR = val;
			break;
		case offsetof(CAN_TypeDef, RF0R):
			s->can.RF0R = val;
			break;
		case offsetof(CAN_TypeDef, RF1R):
			s->can.RF1R = val;
			break;
		case offsetof(CAN_TypeDef, IER):
			s->can.IER = val;
			break;
		case offsetof(CAN_TypeDef, ESR):
			s->can.ESR = val;
			break;
		case offsetof(CAN_TypeDef, BTR):
			s->can.BTR = val;
			break;
		case offsetof(CAN_TypeDef, FMR):
			s->can.FMR = val;
			break;
		case offsetof(CAN_TypeDef, FM1R):
			s->can.FM1R = val;
			break;
		case offsetof(CAN_TypeDef, FS1R):
			s->can.FS1R = val;
			break;
		case offsetof(CAN_TypeDef, FFA1R):
			s->can.FFA1R = val;
			break;
		case offsetof(CAN_TypeDef, FA1R):
			s->can.FA1R = val;
			break;
		default:
			if(offset >= offsetof(CAN_TypeDef, sTxMailBox[0]) && offset < offsetof(CAN_TypeDef, sTxMailBox[0]) + sizeof(s->can.sTxMailBox))
			{
				// TODO tx mailbox
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
		case offsetof(CAN_TypeDef, MCR):
			res = s->can.MCR;
			break;
		case offsetof(CAN_TypeDef, MSR):
			res = s->can.MSR;
			break;
		case offsetof(CAN_TypeDef, TSR):
			res = s->can.TSR;
			break;
		case offsetof(CAN_TypeDef, RF0R):
			res = s->can.RF0R;
			break;
		case offsetof(CAN_TypeDef, RF1R):
			res = s->can.RF1R;
			break;
		case offsetof(CAN_TypeDef, IER):
			res = s->can.IER;
			break;
		case offsetof(CAN_TypeDef, ESR):
			res = s->can.ESR;
			break;
		case offsetof(CAN_TypeDef, BTR):
			res = s->can.BTR;
			break;
		case offsetof(CAN_TypeDef, FMR):
			res = s->can.FMR;
			break;
		case offsetof(CAN_TypeDef, FM1R):
			res = s->can.FM1R;
			break;
		case offsetof(CAN_TypeDef, FS1R):
			res = s->can.FS1R;
			break;
		case offsetof(CAN_TypeDef, FFA1R):
			res = s->can.FFA1R;
			break;
		case offsetof(CAN_TypeDef, FA1R):
			res = s->can.FA1R;
			break;
		default:
			if(offset >= offsetof(CAN_TypeDef, sTxMailBox[0]) && offset < offsetof(CAN_TypeDef, sTxMailBox[0]) + sizeof(s->can.sTxMailBox))
			{
				// TODO tx mailbox
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
				printf("Error : CAN forbiden read acces offset %lx\n", offset);
			}
			break;
	}

	return res;	
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

	atlantronic_can_reset(&s->can);

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
