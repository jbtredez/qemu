#include "hw/sysbus.h"
#include "hw/boards.h"
#include "hw/arm/arm.h"
#include "atlantronic_cpu.h"
#include "atlantronic_syscfg.h"

struct atlantronic_syscfg_state
{
	SysBusDevice busdev;
	MemoryRegion iomem;
	SYSCFG_TypeDef syscfg;
	qemu_irq irq[SYSCFG_IRQ_OUT_NUM];
};

static void atlantronic_syscfg_write(void *opaque, hwaddr offset, uint64_t val, unsigned size)
{
	struct atlantronic_syscfg_state* s = opaque;

	switch(offset)
	{
		W_ACCESS(SYSCFG_TypeDef, s->syscfg, MEMRMP, val);
		W_ACCESS(SYSCFG_TypeDef, s->syscfg, PMC, val);
		W_ACCESS(SYSCFG_TypeDef, s->syscfg, EXTICR[0], val);
		W_ACCESS(SYSCFG_TypeDef, s->syscfg, EXTICR[1], val);
		W_ACCESS(SYSCFG_TypeDef, s->syscfg, EXTICR[2], val);
		W_ACCESS(SYSCFG_TypeDef, s->syscfg, EXTICR[3], val);
		W_ACCESS(SYSCFG_TypeDef, s->syscfg, CMPCR, val);
		default:
			printf("Error : SYSCFG forbiden write acces offset %lx, val %lx\n", offset, val);
			break;
	}
}

static uint64_t atlantronic_syscfg_read(void *opaque, hwaddr offset, unsigned size)
{
	uint64_t res = 0;
	struct atlantronic_syscfg_state* s = opaque;

	switch(offset)
	{
		R_ACCESS(SYSCFG_TypeDef, s->syscfg, MEMRMP, res);
		R_ACCESS(SYSCFG_TypeDef, s->syscfg, PMC, res);
		R_ACCESS(SYSCFG_TypeDef, s->syscfg, EXTICR[0], res);
		R_ACCESS(SYSCFG_TypeDef, s->syscfg, EXTICR[1], res);
		R_ACCESS(SYSCFG_TypeDef, s->syscfg, EXTICR[2], res);
		R_ACCESS(SYSCFG_TypeDef, s->syscfg, EXTICR[3], res);
		R_ACCESS(SYSCFG_TypeDef, s->syscfg, CMPCR, res);
		default:
			printf("Error : SYSCFG forbiden read acces offset %lx\n", offset);
			break;
	}

	return res;
}

static void atlantronic_syscfg_reset(SYSCFG_TypeDef* syscfg)
{
	syscfg->MEMRMP = 0x00;
	syscfg->PMC = 0x00;
	syscfg->EXTICR[0] = 0x00;
	syscfg->EXTICR[1] = 0x00;
	syscfg->EXTICR[2] = 0x00;
	syscfg->EXTICR[3] = 0x00;
	syscfg->CMPCR = 0x00;
}

static const MemoryRegionOps atlantronic_syscfg_ops =
{
	.read = atlantronic_syscfg_read,
	.write = atlantronic_syscfg_write,
	.endianness = DEVICE_NATIVE_ENDIAN,
};

static void atlantronic_syscfg_in_recv(void * opaque, int gpioPort, int level)
{
	struct atlantronic_syscfg_state *s = opaque;

	int pinId = level & 0x0f;

	int exticrId = pinId / 4;
	int shift = 4*(pinId - 4 * exticrId);

	if( ((s->syscfg.EXTICR[exticrId] >> shift) & 0x0f) == gpioPort)
	{
		qemu_set_irq(s->irq[SYSCFG_IRQ_OUT_EXTI], level);
	}
}

static int atlantronic_syscfg_init(SysBusDevice * dev)
{
	struct atlantronic_syscfg_state *s = OBJECT_CHECK(struct atlantronic_syscfg_state, dev, "atlantronic-syscfg");

	memory_region_init_io(&s->iomem, OBJECT(s), &atlantronic_syscfg_ops, s, "atlantronic_syscfg", 0x400);
	sysbus_init_mmio(dev, &s->iomem);

	qdev_init_gpio_out(DEVICE(dev), s->irq, SYSCFG_IRQ_OUT_NUM);
	qdev_init_gpio_in(DEVICE(dev), atlantronic_syscfg_in_recv, SYSCFG_IRQ_IN_NUM);

	atlantronic_syscfg_reset(&s->syscfg);

	return 0;
}

static void atlantronic_syscfg_class_init(ObjectClass *klass, void *data)
{
	SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);

	sdc->init = atlantronic_syscfg_init;
}

static TypeInfo atlantronic_syscfg_info =
{
	.name          = "atlantronic-syscfg",
	.parent        = TYPE_SYS_BUS_DEVICE,
	.instance_size = sizeof(struct atlantronic_syscfg_state),
	.class_init    = atlantronic_syscfg_class_init,
};

static void atlantronic_syscfg_register_types(void)
{
	type_register_static(&atlantronic_syscfg_info);
}

type_init(atlantronic_syscfg_register_types);
