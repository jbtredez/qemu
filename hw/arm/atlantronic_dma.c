#include "hw/sysbus.h"
#include "hw/boards.h"
#include "hw/arm/arm.h"
#include "atlantronic_cpu.h"

#define DMA_STREAM_NUMBER                8

struct atlantronic_dma_state
{
	SysBusDevice busdev;
	MemoryRegion iomem;
	DMA_TypeDef dma;
	qemu_irq irq[DMA_STREAM_NUMBER];
};

static void atlantronic_dma_write_ifcr(volatile uint32_t* xISR, uint32_t val, int bitNum, qemu_irq* irq)
{
	uint32_t v = (val >> bitNum) & 0x3f;
	*xISR &= ~(v << bitNum );
	if( v & 0x30 )
	{
		// TCIF ou HTIF
		// => declencher it
		*xISR &= ~(0x3f << bitNum );
		qemu_set_irq(*irq, 0);
	}
}

static void atlantronic_dma_write(void *opaque, hwaddr offset, uint64_t val, unsigned size)
{
	struct atlantronic_dma_state* s = opaque;

	switch(offset)
	{
		case offsetof(DMA_TypeDef, LIFCR):
			atlantronic_dma_write_ifcr(&s->dma.LISR, val,  0, &s->irq[0]);
			atlantronic_dma_write_ifcr(&s->dma.LISR, val,  6, &s->irq[1]);
			atlantronic_dma_write_ifcr(&s->dma.LISR, val, 16, &s->irq[2]);
			atlantronic_dma_write_ifcr(&s->dma.LISR, val, 22, &s->irq[3]);
			break;
		case offsetof(DMA_TypeDef, HIFCR):
			atlantronic_dma_write_ifcr(&s->dma.HISR, val,  0, &s->irq[4]);
			atlantronic_dma_write_ifcr(&s->dma.HISR, val,  6, &s->irq[5]);
			atlantronic_dma_write_ifcr(&s->dma.HISR, val, 16, &s->irq[6]);
			atlantronic_dma_write_ifcr(&s->dma.HISR, val, 22, &s->irq[7]);
			break;
		case offsetof(DMA_TypeDef, LISR):
		case offsetof(DMA_TypeDef, HISR):
		default:
			printf("Error : DMA forbiden write acces offset %lx, val %lx\n", offset, val);
			break;
	}
}

static uint64_t atlantronic_dma_read(void *opaque, hwaddr offset, unsigned size)
{
	uint64_t res = 0;
	struct atlantronic_dma_state* s = opaque;

	switch(offset)
	{
		case offsetof(DMA_TypeDef, LISR):
			res = s->dma.LISR;
			break;
		case offsetof(DMA_TypeDef, LIFCR):
			res = s->dma.LIFCR;
			break;
		case offsetof(DMA_TypeDef, HISR):
			res = s->dma.HISR;
			break;
		case offsetof(DMA_TypeDef, HIFCR):
			res = s->dma.HIFCR;
			break;
		default:
			printf("Error : DMA forbiden read acces offset %lx\n", offset);
			break;
	}

	return res;	
}

static void atlantronic_dma_reset(DMA_TypeDef* dma)
{
	dma->LISR = 0x00;
	dma->LIFCR = 0x00;
	dma->HISR = 0x00;
	dma->HIFCR = 0x00;
}

static const MemoryRegionOps atlantronic_dma_ops =
{
	.read = atlantronic_dma_read,
	.write = atlantronic_dma_write,
	.endianness = DEVICE_NATIVE_ENDIAN,
};

static void atlantronic_dma_in_recv(void * opaque, int numPin, int level)
{
	struct atlantronic_dma_state *s = opaque;

	level &= 0x03f;
	uint32_t old_val;
	int bitShift = 0;

	volatile uint32_t* xISR = &s->dma.LISR;
	if(numPin >= 4)
	{
		xISR = &s->dma.HISR;
	}

	switch(numPin)
	{
		case 0:
			bitShift = 0;
			break;
		case 1:
			bitShift = 6;
			break;
		case 2:
			bitShift = 16;
			break;
		case 3:
			bitShift = 22;
			break;
		case 4:
			bitShift = 0;
			break;
		case 5:
			bitShift = 6;
			break;
		case 6:
			bitShift = 16;
			break;
		case 7:
			bitShift = 22;
			break;
		default:
			return;
	}


	old_val = (*xISR >> bitShift) & 0x03f;
	*xISR |= level << bitShift;
	uint32_t diff_raise = (old_val ^ level) & level;
	// TODO voir / autres irq (erreur, half transfert)
	if( diff_raise & 0x20 )
	{
		qemu_set_irq(s->irq[numPin], 1);
	}
}

static int atlantronic_dma_init(SysBusDevice * dev)
{
	struct atlantronic_dma_state *s = OBJECT_CHECK(struct atlantronic_dma_state, dev, "atlantronic-dma");

	memory_region_init_io(&s->iomem, OBJECT(s), &atlantronic_dma_ops, s, "atlantronic_dma", sizeof(DMA_TypeDef));
	sysbus_init_mmio(dev, &s->iomem);

	qdev_init_gpio_out(DEVICE(dev), s->irq, DMA_STREAM_NUMBER);
	qdev_init_gpio_in(DEVICE(dev), atlantronic_dma_in_recv, DMA_STREAM_NUMBER);

	atlantronic_dma_reset(&s->dma);

    return 0;
}

static void atlantronic_dma_class_init(ObjectClass *klass, void *data)
{
	SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);

	sdc->init = atlantronic_dma_init;
}

static TypeInfo atlantronic_dma_info =
{
	.name          = "atlantronic-dma",
	.parent        = TYPE_SYS_BUS_DEVICE,
	.instance_size = sizeof(struct atlantronic_dma_state),
	.class_init    = atlantronic_dma_class_init,
};

static void atlantronic_dma_register_types(void)
{
	type_register_static(&atlantronic_dma_info);
}

type_init(atlantronic_dma_register_types);
