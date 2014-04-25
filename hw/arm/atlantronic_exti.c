#include "hw/sysbus.h"
#include "hw/boards.h"
#include "hw/arm/arm.h"
#include "atlantronic_cpu.h"
#include "atlantronic_exti.h"

#define EXTI_EXTI_LINE      7
#define EXTI_GPIO_NUM       6

struct atlantronic_exti_state
{
	SysBusDevice busdev;
	MemoryRegion iomem;
	EXTI_TypeDef exti;
	qemu_irq irq[EXTI_EXTI_LINE];
};

static void atlantronic_exti_write(void *opaque, hwaddr offset, uint64_t val, unsigned size)
{
	struct atlantronic_exti_state* s = opaque;

	switch(offset)
	{
		W_ACCESS(EXTI_TypeDef, s->exti, IMR, val);
		W_ACCESS(EXTI_TypeDef, s->exti, EMR, val);
		W_ACCESS(EXTI_TypeDef, s->exti, RTSR, val);
		W_ACCESS(EXTI_TypeDef, s->exti, FTSR, val);
		W_ACCESS(EXTI_TypeDef, s->exti, SWIER, val);
		case offsetof(EXTI_TypeDef, PR):
			{
				uint32_t old_pr = s->exti.PR;
				s->exti.PR &= ~(val & 0x7ffff);
				if((old_pr & 0x01) && !(s->exti.PR & 0x01))
				{
					qemu_set_irq(s->irq[0], 0);
				}
				if((old_pr & 0x02) && !(s->exti.PR & 0x02))
				{
					qemu_set_irq(s->irq[1], 0);
				}
				if((old_pr & 0x04) && !(s->exti.PR & 0x04))
				{
					qemu_set_irq(s->irq[2], 0);
				}
				if((old_pr & 0x08) && !(s->exti.PR & 0x08))
				{
					qemu_set_irq(s->irq[3], 0);
				}
				if((old_pr & 0x10) && !(s->exti.PR & 0x10))
				{
					qemu_set_irq(s->irq[4], 0);
				}
				if((old_pr & 0x3e0) && !(s->exti.PR & 0x3e0))
				{
					qemu_set_irq(s->irq[5], 0);
				}
				if((old_pr & 0xfc00) && !(s->exti.PR & 0xfc00))
				{
					qemu_set_irq(s->irq[6], 0);
				}
			}
			break;
		default:
			printf("Error : EXTI forbiden write acces offset %lx, val %lx\n", offset, val);
			break;
	}
}

static uint64_t atlantronic_exti_read(void *opaque, hwaddr offset, unsigned size)
{
	uint64_t res = 0;
	struct atlantronic_exti_state* s = opaque;

	switch(offset)
	{
		R_ACCESS(EXTI_TypeDef, s->exti, IMR, res);
		R_ACCESS(EXTI_TypeDef, s->exti, EMR, res);
		R_ACCESS(EXTI_TypeDef, s->exti, RTSR, res);
		R_ACCESS(EXTI_TypeDef, s->exti, FTSR, res);
		R_ACCESS(EXTI_TypeDef, s->exti, SWIER, res);
		R_ACCESS(EXTI_TypeDef, s->exti, PR, res);
		default:
			printf("Error : EXTI forbiden read acces offset %lx\n", offset);
			break;
	}

	return res;
}

static void atlantronic_exti_reset(EXTI_TypeDef* exti)
{
	exti->IMR = 0x00;
	exti->EMR = 0x00;
	exti->RTSR = 0x00;
	exti->FTSR = 0x00;
	exti->SWIER = 0x00;
	exti->PR = 0x00;
}

static const MemoryRegionOps atlantronic_exti_ops =
{
	.read = atlantronic_exti_read,
	.write = atlantronic_exti_write,
	.endianness = DEVICE_NATIVE_ENDIAN,
};

static void atlantronic_exti_in_recv(void * opaque, int numPin, int level)
{
	struct atlantronic_exti_state *s = opaque;

	int pinId = level & 0x0f;
	int raise = level>>4;
	uint32_t mask = 1 << pinId;
	int irqId = pinId;
	if( pinId >= 5 && pinId <= 9)
	{
		irqId = 5;
	}
	if( pinId >= 10 && pinId <= 15)
	{
		irqId = 6;
	}

	// si ligne active et detection front montant et front montant
	if( (s->exti.IMR & s->exti.RTSR & mask) && raise )
	{
		// IT front montant
		//printf("TOTO FM %d irq %d\n", pinId, irqId);
		s->exti.PR |= mask;
		qemu_set_irq(s->irq[irqId], 1);
	}

	if( (s->exti.IMR & s->exti.FTSR & mask) && ! raise )
	{
		// IT front descendant
		//printf("TOTO FD %d irq %d\n", pinId, irqId);
		s->exti.PR |= mask;
		qemu_set_irq(s->irq[irqId], 1);
	}
}

static int atlantronic_exti_init(SysBusDevice * dev)
{
	struct atlantronic_exti_state *s = OBJECT_CHECK(struct atlantronic_exti_state, dev, "atlantronic-exti");

	memory_region_init_io(&s->iomem, OBJECT(s), &atlantronic_exti_ops, s, "atlantronic_exti", 0x400);
	sysbus_init_mmio(dev, &s->iomem);

	qdev_init_gpio_out(DEVICE(dev), s->irq, EXTI_EXTI_LINE);
	qdev_init_gpio_in(DEVICE(dev), atlantronic_exti_in_recv, EXTI_IRQ_IN_NUM);

	atlantronic_exti_reset(&s->exti);

	return 0;
}

static void atlantronic_exti_class_init(ObjectClass *klass, void *data)
{
	SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);

	sdc->init = atlantronic_exti_init;
}

static TypeInfo atlantronic_exti_info =
{
	.name          = "atlantronic-exti",
	.parent        = TYPE_SYS_BUS_DEVICE,
	.instance_size = sizeof(struct atlantronic_exti_state),
	.class_init    = atlantronic_exti_class_init,
};

static void atlantronic_exti_register_types(void)
{
	type_register_static(&atlantronic_exti_info);
}

type_init(atlantronic_exti_register_types);
